/* Distributed under the Apache License, Version 2.0.
   See accompanying NOTICE file for details.*/

#include "tbutil/route.h"
#include "tbutil/graph.h"
#include "tbutil/util.h"

#include <vital/types/geodesy.h>

#include <vital/range/iota.h>

#include <valhalla/proto/options.pb.h>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/loki/search.h>
#include <valhalla/loki/worker.h>
#include <valhalla/midgard/encoded.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/thor/bidirectional_astar.h>
#include <valhalla/thor/triplegbuilder.h>

#include <limits>

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
class RoutingEngine::Private
{
public:
  Private(routing_config_t const& config)
    : reader{config.get_child("mjolnir")},
      worker{config}
  {
    // Set up the costing function
    valhalla::Options options;
    options.set_costing(valhalla::auto_);
    vs::ParseAutoCostOptions({}, {}, options.add_costing_options());
    vs::CostFactory<vs::DynamicCost> factory;
    factory.RegisterStandardCostingModels();
    costing[costMode] = factory.Create(options);
  }

  std::vector<std::vector<vt::PathInfo>>
  getPaths(valhalla::Location& startPoint,
           valhalla::Location& stopPoint,
           valhalla::Options const& options)
  {
    auto pathAlgorithm = vt::BidirectionalAStar{};
    try
    {
      return pathAlgorithm.GetBestPath(
        startPoint, stopPoint, reader, costing,
        vs::TravelMode::kDrive, options);
    }
    catch (...)
    {
      return {};
    }
  }

  vb::GraphReader reader;
  vl::loki_worker_t worker;
  std::shared_ptr<vs::DynamicCost> costing[4];

  static constexpr auto costMode = static_cast<size_t>(vs::TravelMode::kDrive);
};

// ----------------------------------------------------------------------------
RoutingEngine::RoutingEngine(routing_config_t const& config)
  : m_p{new Private{config}}
{
}

// ----------------------------------------------------------------------------
RoutingEngine::RoutingEngine(RoutingEngine&& other)
  : m_p{std::move(other.m_p)}
{
}

// ----------------------------------------------------------------------------
RoutingEngine::~RoutingEngine()
{
}

// ----------------------------------------------------------------------------
location_t RoutingEngine::locate(location_t const& in) const
{
  auto locations = std::vector<vb::Location>{{{in.x(), in.y()}}};
  auto const& result = vl::Search(locations, m_p->reader,
                                  m_p->costing[m_p->costMode]);

  auto const invalid = std::numeric_limits<double>::quiet_NaN();
  auto p = location_t{invalid, invalid};
  auto best = std::numeric_limits<double>::infinity();

  for (auto const& match : result)
  {
    for (auto const& edge : match.second.edges)
    {
      auto const& c = location_t{edge.projected.lng(), edge.projected.lat()};
      auto const dist = (in - c).squaredNorm();
      if (dist < best)
      {
        p = c;
        best = dist;
      }
    }
  }

  return p;
}

// ----------------------------------------------------------------------------
std::vector<Leg> RoutingEngine::route(
  location_t const& start, location_t const& stop, Graph const& graph) const
{
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

  // Correlate the start/stop with actual nodes
  m_p->worker.route(request);

  // Generate a trip path
  auto const& paths = m_p->getPaths(*startPoint, *stopPoint, options);
  if (paths.empty())
  {
    return {};
  }

  auto const& path = paths.front();
  auto& route = *request.mutable_trip()->mutable_routes()->Add();
  auto& leg = *route.mutable_legs()->Add();

  auto controller = vt::AttributesController{};
  vt::TripLegBuilder::Build(
    controller, m_p->reader, m_p->costing, path.begin(), path.end(),
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

      // Ignore duplicate points
      constexpr auto epsilon = 1e-4;
      if (!l.points.empty() && (l.points.back() - p).squaredNorm() < epsilon)
      {
        continue;
      }

      // Add point to leg
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
