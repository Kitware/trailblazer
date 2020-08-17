/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#include <tbutil/reader.h>
#include <tbutil/util.h>

#include <vital/types/geodesy.h>

#include <vital/math_constants.h>

#include <vital/range/iota.h>

#include <valhalla/proto/options.pb.h>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/loki/worker.h>
#include <valhalla/midgard/encoded.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/thor/bidirectional_astar.h>
#include <valhalla/thor/triplegbuilder.h>

#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <iostream>

namespace tb = trailblazer;

namespace kv = kwiver::vital;
namespace kvr = kwiver::vital::range;

namespace vb = valhalla::baldr;
namespace vl = valhalla::loki;
namespace vm = valhalla::midgard;
namespace vs = valhalla::sif;
namespace vt = valhalla::thor;

namespace Arguments
{
  enum
  {
    ExecutableName,
    OsmData,
    Config,
    StartLat,
    StartLon,
    StopLat,
    StopLon,
    ExpectedCount,
  };

  char const* names[ExpectedCount - 1] = {
    "osm",
    "config",
    "start-lat",
    "start-lon",
    "stop-lat",
    "stop-lon",
  };
}

// ----------------------------------------------------------------------------
struct Leg
{
  tb::id_t way;
  double bearing;
  std::vector<tb::location_t> points;
};

// ----------------------------------------------------------------------------
struct Segment
{
  tb::id_t way;
  std::vector<size_t> indices;
  bool forward;
};

// ----------------------------------------------------------------------------
std::vector<Leg> route(
  tb::location_t const& start, tb::location_t const& stop,
  boost::property_tree::ptree const& config, int crs)
{
  // Get something we can use to fetch tiles
  vb::GraphReader reader{config.get_child("mjolnir")};

  // Create the routing request
  auto request = valhalla::Api{};
  auto& options = *request.mutable_options();
  auto& locations = *options.mutable_locations();

  auto* const startPoint = locations.Add();
  startPoint->mutable_ll()->set_lat(static_cast<float>(start.y()));
  startPoint->mutable_ll()->set_lng(static_cast<float>(start.x()));
  startPoint->set_type(valhalla::Location::kVia);
  startPoint->set_original_index(0);

  auto* const stopPoint = locations.Add();
  stopPoint->mutable_ll()->set_lat(static_cast<float>(stop.y()));
  stopPoint->mutable_ll()->set_lng(static_cast<float>(stop.x()));
  stopPoint->set_type(valhalla::Location::kVia);
  startPoint->set_original_index(1);

  options.set_action(valhalla::Options::route);
  options.set_costing(valhalla::auto_);
  vs::ParseAutoCostOptions({}, {}, options.add_costing_options());
  options.set_alternates(0);

  // Set up the costing function
  vs::CostFactory<vs::DynamicCost> factory;
  factory.RegisterStandardCostingModels();
  std::shared_ptr<vs::DynamicCost> costing[] = {
    factory.Create(options),
    nullptr,
    nullptr,
    nullptr,
  };

  // Correlate the start/stop with actual nodes
  auto lokiWorker = vl::loki_worker_t{config};
  lokiWorker.route(request);

  // Generate a trip path
  auto pathAlgorithm = vt::BidirectionalAStar{};
  auto const& paths =
    pathAlgorithm.GetBestPath(*startPoint, *stopPoint, reader, costing,
                              vs::TravelMode::kDrive, options);
  if (paths.empty())
  {
    return {};
  }

  auto const& path = paths.front();
  auto& route = *request.mutable_trip()->mutable_routes()->Add();
  auto& leg = *route.mutable_legs()->Add();

  auto controller = vt::AttributesController{};
  vt::TripLegBuilder::Build(
    controller, reader, costing, path.begin(), path.end(),
    *startPoint, *stopPoint, {}, leg, {});

  // Extract the location points for the leg
  auto const& llPoints = vm::decode<std::vector<vm::PointLL>>(leg.shape());
  auto points = std::vector<tb::location_t>{};

  points.reserve(llPoints.size());
  for (auto const& p : llPoints)
  {
    auto const ll = tb::location_t{p.lng(), p.lat()};
    points.push_back(kv::geo_conv(ll, kv::SRID::lat_lon_WGS84, crs));
  }

  // Convert to internal format
  auto out = std::vector<Leg>{};
  for (auto const& node : leg.node())
  {
    if (!node.has_edge())
    {
      continue;
    }

    auto const& edge = node.edge();
    auto const bearing = static_cast<double>(edge.begin_heading());
    auto const way = static_cast<tb::id_t>(edge.way_id());

    auto l = Leg{way, bearing, {}};

    auto const offset = edge.begin_shape_index();
    for (auto const i : kvr::iota(edge.end_shape_index() + 1 - offset))
    {
      l.points.push_back(points[i + offset]);
    }

    // Recompute bearing; Valhalla's seems to tend to be off, sometimes by a
    // non-trivial amount (especially at the start)
    if (l.points.size() > 1)
    {
      auto const& l0 = l.points[0];
      auto const& l1 = l.points[1];
      auto const d = (l1 - l0).normalized();
      auto const c = std::acos(d.y()) / kv::deg_to_rad;
      l.bearing = (d.x() > 0 ? c : 360.0 - c);
    }

    out.push_back(std::move(l));
  }

  // Return trip parts
  return out;
}

// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  if (argc < (Arguments::ExpectedCount))
  {
    std::cerr << "Usage: " << argv[0];
    for (auto arg : Arguments::names)
    {
      std::cerr << ' ' << '<' << arg << '>';
    }
    std::cerr << std::endl;
    return 1;
  }

  tb::init();

  // Read the OSM data and construct a graph
  tb::Reader graph{argv[Arguments::OsmData]};
  if (graph)
  {
    graph.exec();
  }
  else
  {
    std::cerr << "Failed to read OSM data" << std::endl;
    return 2;
  }

  // Read the Valhalla configuration
  auto config = boost::property_tree::ptree{};
  rapidjson::read_json(argv[Arguments::Config], config);

  // Get start and stop points
  auto const startLL = tb::location_t{
    std::strtod(argv[Arguments::StartLon], nullptr),
    std::strtod(argv[Arguments::StartLat], nullptr)};

  auto const stopLL = tb::location_t{
    std::strtod(argv[Arguments::StopLon], nullptr),
    std::strtod(argv[Arguments::StopLat], nullptr)};

  // Get trip from Valhalla
  auto const trip = route(startLL, stopLL, config, graph.crs());
  if (trip.empty())
  {
    std::cerr << "Failed to generate path" << std::endl;
    return 2;
  }

  // Determine segments
  auto segments = std::vector<Segment>{};
  segments.reserve(trip.size());

  // Map legs to segments
  for (auto const& leg : trip)
  {
    // Match leg shape to OSM nodes
    auto nodes = std::vector<tb::id_t>{};
    for (auto const& p : leg.points)
    {
      if (auto* const node = graph.locate(leg.way, p))
      {
        nodes.push_back(node->id);
      }
    }

    // Determine way node indices and create segment
    auto const nodeCount = nodes.size();
    if (nodeCount > 1)
    {
      // Get end bearing (for certain pathological cases of self-intersecting
      // ways with turn restrictions)
      auto const& l0 = graph.node(nodes[nodeCount - 1])->location;
      auto const& l1 = graph.node(nodes[nodeCount - 2])->location;
      auto const d = (l1 - l0).normalized();
      auto const c = std::acos(d.y()) / kv::deg_to_rad;
      auto const finalBearing = (d.x() > 0 ? c : 360.0 - c);

      // Determine start and stop way node indices
      auto const& hStart = graph.locate(leg.way, nodes.front(), leg.bearing);
      auto const& hStop = graph.locate(leg.way, nodes.back(), finalBearing);

      // Create segment
      auto s = Segment{leg.way, {}, hStart.forward};

      // Build index list for segment
      auto const offset = hStart.node;
      if (hStart.forward)
      {
        for (auto const i : kvr::iota(hStop.node - offset + 1))
        {
          s.indices.push_back(offset + i);
        }
      }
      else
      {
        for (auto const i : kvr::iota(offset + 1 - hStop.node))
        {
          s.indices.push_back(offset - i);
        }
      }

      // Add segment
      segments.push_back(std::move(s));
    }
  }

  // Generate edges from ways
  std::unordered_map<tb::id_t, tb::segmentation_t> segmentations;
  for (auto const& s : segments)
  {
    auto* const dir = (s.forward ? "" : "-");

    // Get way segmentation and determine segment id, if relevant
    auto i = segmentations.find(s.way);
    if (i == segmentations.end())
    {
      i = segmentations.emplace(s.way, graph.segment(s.way, true)).first;
    }

    // Check against way segmentation
    auto const& segmentation = i->second;
    if (auto const sc = segmentation.size())
    {
      auto const l = std::min(s.indices.front(), s.indices.back());
      auto const u = std::max(s.indices.front(), s.indices.back());
      for (auto const si : kvr::iota(sc))
      {
        auto const& ss = segmentation[si];
        if (l < ss.second && u > ss.first)
        {
          std::cout << dir << s.way << '#' << si << std::endl;
        }
      }
    }
    else
    {
      std::cout << dir << s.way << std::endl;
    }
  }

  return 0;
}
