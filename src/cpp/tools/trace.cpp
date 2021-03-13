/* Distributed under the Apache License, Version 2.0.
   See accompanying NOTICE file for details.*/

#include <tbutil/reader.h>
#include <tbutil/route.h>
#include <tbutil/segmentation.h>
#include <tbutil/util.h>

#include <algorithm>
#include <iostream>

namespace tb = trailblazer;

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
template <typename T>
void append(std::vector<T>& out, std::vector<T>&& in)
{
  if (out.empty())
  {
    out = std::move(in);
  }
  else
  {
    out.insert(
      out.end(),
      std::make_move_iterator(in.begin()),
      std::make_move_iterator(in.end()));
  }
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

  // Read the Valhalla configuration and construct a routing engine
  auto config = tb::readConfig(argv[Arguments::Config]);
  auto engine = tb::RoutingEngine{config};

  // Get start and stop points
  auto const startLL = tb::location_t{
    std::strtod(argv[Arguments::StartLon], nullptr),
    std::strtod(argv[Arguments::StartLat], nullptr)};

  auto const stopLL = tb::location_t{
    std::strtod(argv[Arguments::StopLon], nullptr),
    std::strtod(argv[Arguments::StopLat], nullptr)};

  // Get trip from Valhalla
  auto const trip = engine.route(startLL, stopLL, graph);
  if (trip.empty())
  {
    std::cerr << "Failed to generate path" << std::endl;
    return 2;
  }

  // Map legs to edges
  auto segmentation = tb::Segmentation{&graph};
  auto edges = std::vector<tb::Edge>{};
  for (auto const& leg : trip)
  {
    std::cout << "way " << leg.way << std::endl;
    for (auto const n : leg.nodes)
    {
      std::cout << "  node " << n << std::endl;
    }

    append(edges, segmentation.edges(leg));
  }

  std::cout << std::endl;

  for (auto const& e : edges)
  {
    std::cout << (e.forward ? "" : "-") << e.way;
    if (e.segment >= 0)
    {
      std::cout << '#' << e.segment;
    }
    std::cout << std::endl;
  }

  return 0;
}
