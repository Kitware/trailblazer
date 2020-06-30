#include <valhalla/proto/options.pb.h>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/loki/worker.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/thor/bidirectional_astar.h>
#include <valhalla/thor/triplegbuilder.h>

#include <boost/property_tree/ptree.hpp>

#include <iostream>

namespace baldr = valhalla::baldr;
namespace loki = valhalla::loki;
namespace sif = valhalla::sif;
namespace thor = valhalla::thor;

namespace Arguments
{
  enum
  {
    ExecutableName,
    Config,
    StartLat,
    StartLon,
    EndLat,
    EndLon,
    ExpectedCount,
  };

  char const* names[ExpectedCount] = {
    "config",
    "start-lat",
    "start-lon",
    "end-lat",
    "end-lon",
  };
}

// ----------------------------------------------------------------------------
std::string toString(rapidjson::Document const& doc)
{
  auto buffer = rapidjson::StringBuffer{};
  auto writer = rapidjson::Writer<rapidjson::StringBuffer>{buffer};

  doc.Accept(writer);
  return buffer.GetString();
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

  // Read the configuration
  auto config = boost::property_tree::ptree{};
  rapidjson::read_json(argv[Arguments::Config], config);

  // Get something we can use to fetch tiles
  baldr::GraphReader reader{config.get_child("mjolnir")};

  // Create the routing request
  auto request = valhalla::Api{};
  auto& options = *request.mutable_options();
  auto& locations = *options.mutable_locations();

  auto* const startPoint = locations.Add();
  startPoint->mutable_ll()->set_lat(
    std::strtof(argv[Arguments::StartLat], nullptr));
  startPoint->mutable_ll()->set_lng(
    std::strtof(argv[Arguments::StartLon], nullptr));
  startPoint->set_type(valhalla::Location::kVia);
  startPoint->set_original_index(0);

  auto* const endPoint = locations.Add();
  endPoint->mutable_ll()->set_lat(
    std::strtof(argv[Arguments::EndLat], nullptr));
  endPoint->mutable_ll()->set_lng(
    std::strtof(argv[Arguments::EndLon], nullptr));
  endPoint->set_type(valhalla::Location::kVia);
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

  // Correlate the start/end with actual nodes
  auto lokiWorker = loki::loki_worker_t{config};
  lokiWorker.route(request);

  // Generate a trip path
  auto pathAlgorithm = thor::BidirectionalAStar{};
  auto const& paths =
    pathAlgorithm.GetBestPath(*startPoint, *endPoint, reader, costing,
                              sif::TravelMode::kDrive, options);
  if (paths.empty())
  {
    std::cerr << "Failed to generate path" << std::endl;
    return 2;
  }

  auto const& path = paths.front();
  auto& route = *request.mutable_trip()->mutable_routes()->Add();
  auto& leg = *route.mutable_legs()->Add();

  auto controller = thor::AttributesController{};
  thor::TripLegBuilder::Build(
    controller, reader, costing, path.begin(), path.end(),
    *startPoint, *endPoint, {}, leg, {});

  auto last = std::uint64_t{0};
  for (auto const& node : leg.node())
  {
    if (!node.has_edge())
    {
      continue;
    }

    auto const& edge = node.edge();
    if (edge.has_way_id())
    {
      if (auto const i = edge.way_id())
      {
        if (last != i)
        {
          std::cout << "way_id: " << edge.way_id();
          if (edge.name_size())
          {
            std::cout << ' ' << '(';
            for (auto const& name : edge.name())
            {
              std::cout << name.value();
            }
            std::cout << ')';
          }
          std::cout << std::endl;
          last = i;
        }
      }
    }
  }

  return 0;
}
