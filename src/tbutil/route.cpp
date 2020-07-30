/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#include "route.h"

#include <tbutil/graph.h>

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

#include <cmath>

namespace tb = trailblazer;

namespace kv = kwiver::vital;
namespace kvr = kwiver::vital::range;

namespace vb = valhalla::baldr;
namespace vl = valhalla::loki;
namespace vm = valhalla::midgard;
namespace vs = valhalla::sif;
namespace vt = valhalla::thor;

namespace trailblazer
{

// ----------------------------------------------------------------------------
routing_config_t readConfig(char const* path)
{
  auto config = boost::property_tree::ptree{};
  rapidjson::read_json(path, config);
  return config;
}

// ----------------------------------------------------------------------------
double computeBearing(location_t origin, location_t target)
{
  auto const d = (target - origin).normalized();
  auto const c = std::acos(d.y()) / kv::deg_to_rad;
  return (d.x() > 0 ? c : 360.0 - c);
}

// ----------------------------------------------------------------------------
std::vector<Leg> route(
  tb::location_t const& start, tb::location_t const& stop,
  boost::property_tree::ptree const& config, tb::Graph const& graph)
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
  auto const crs = graph.crs();

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

    auto l = Leg{way, bearing, {}, {}};

    auto const offset = edge.begin_shape_index();
    for (auto const i : kvr::iota(edge.end_shape_index() + 1 - offset))
    {
      auto const& p = points[i + offset];
      l.points.push_back(p);

      // Match shape point to OSM node
      if (auto* const node = graph.locate(way, p))
      {
        l.nodes.push_back(node->id);
      }
    }

    // Recompute bearing; Valhalla's seems to tend to be off, sometimes by a
    // non-trivial amount (especially at the start)
    if (l.points.size() > 1)
    {
      l.bearing = computeBearing(l.points[0], l.points[1]);
    }

    out.push_back(std::move(l));
  }

  // Return trip parts
  return out;
}

} // namespace trailblazer
