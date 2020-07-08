/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#include "reader.h"
#include "util.h"

#include <vital/types/geodesy.h>

#include <valhalla/proto/options.pb.h>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/loki/worker.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/thor/bidirectional_astar.h>
#include <valhalla/thor/triplegbuilder.h>

#include <boost/property_tree/ptree.hpp>

#include <iostream>

namespace tb = trailblazer;

namespace kv = kwiver::vital;

namespace baldr = valhalla::baldr;
namespace loki = valhalla::loki;
namespace sif = valhalla::sif;
namespace thor = valhalla::thor;

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
  id_t way;
  unsigned heading;
  double length;
};

// ----------------------------------------------------------------------------
std::vector<Leg> route(
  tb::location_t const& start, tb::location_t const& stop,
  boost::property_tree::ptree const& config)
{
  // Get something we can use to fetch tiles
  baldr::GraphReader reader{config.get_child("mjolnir")};

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
  sif::ParseAutoCostOptions({}, {}, options.add_costing_options());
  options.set_alternates(0);

  // Set up the costing function
  sif::CostFactory<sif::DynamicCost> factory;
  factory.RegisterStandardCostingModels();
  std::shared_ptr<sif::DynamicCost> costing[] = {
    factory.Create(options),
    nullptr,
    nullptr,
    nullptr,
  };

  // Correlate the start/stop with actual nodes
  auto lokiWorker = loki::loki_worker_t{config};
  lokiWorker.route(request);

  // Generate a trip path
  auto pathAlgorithm = thor::BidirectionalAStar{};
  auto const& paths =
    pathAlgorithm.GetBestPath(*startPoint, *stopPoint, reader, costing,
                              sif::TravelMode::kDrive, options);
  if (paths.empty())
  {
    return {};
  }

  auto const& path = paths.front();
  auto& route = *request.mutable_trip()->mutable_routes()->Add();
  auto& leg = *route.mutable_legs()->Add();

  auto controller = thor::AttributesController{};
  thor::TripLegBuilder::Build(
    controller, reader, costing, path.begin(), path.end(),
    *startPoint, *stopPoint, {}, leg, {});

  // Convert to internal format
  auto out = std::vector<Leg>{};
  auto last = tb::id_t{-1};
  auto distance = 0.0;
  for (auto const& node : leg.node())
  {
    if (!node.has_edge())
    {
      continue;
    }

    auto const& edge = node.edge();
    distance += edge.length() * 1e3; // km -> m

    if (edge.has_way_id())
    {
      if (auto const way = static_cast<id_t>(edge.way_id()))
      {
        if (last != way)
        {
          auto const heading = edge.begin_heading();
          out.push_back(Leg{way, heading, distance});
          last = way;
          distance = 0.0;
        }
      }
    }
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
  auto const trip = route(startLL, stopLL, config);
  if (trip.empty())
  {
    std::cerr << "Failed to generate path" << std::endl;
    return 2;
  }

  // Locate initial node
  auto* const node = graph.locate(
    trip.front().way,
    kv::geo_conv(startLL, kv::SRID::lat_lon_WGS84, graph.crs()));
  if (!node)
  {
    std::cerr << "Failed to locate starting node";
    return 2;
  }

  std::cout << "Starting node: " << node->id << std::endl;

  // Map legs to segments
  for (auto const& leg : trip)
  {
    std::cout << "way " << leg.way
              << " (bearing " << leg.heading
              << " for " << leg.length << " m)"
              << std::endl;
  }

  return 0;
}
