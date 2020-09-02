/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#include <tbutil/csv.h>
#include <tbutil/reader.h>
#include <tbutil/route.h>
#include <tbutil/segmentation.h>
#include <tbutil/util.h>

#include <vital/types/geodesy.h>

#include <vital/range/iota.h>
#include <vital/range/sliding.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace tb = trailblazer;

namespace kv = kwiver::vital;
namespace kvr = kwiver::vital::range;

namespace Arguments
{
  enum
  {
    ExecutableName,
    OsmData,
    Config,
    WaypointsPath,
    OutputPath,
    ExpectedCount,
  };

  char const* names[ExpectedCount - 1] = {
    "osm",
    "config",
    "waypoints",
    "output",
  };
}

// ----------------------------------------------------------------------------
struct Waypoint
{
  double time;
  tb::location_t llPoint;
  tb::location_t utmPoint;
};
using Mover = std::vector<Waypoint>;
using MoverMap = std::unordered_map<std::string, Mover>;

// ----------------------------------------------------------------------------
struct Trip
{
  double start;
  double cooldown;
  std::vector<Waypoint> waypoints;
};

// ----------------------------------------------------------------------------
struct Stop
{
  std::string edge;
  double duration;
};

// ----------------------------------------------------------------------------
bool isValid(tb::location_t const& in)
{
  return std::isfinite(in.x()) && std::isfinite(in.y());
}

// ----------------------------------------------------------------------------
bool compareTripStart(std::vector<Trip> const& a, std::vector<Trip> const& b)
{
  if (a.empty())
  {
    return !b.empty();
  }

  if (b.empty())
  {
    return false;
  }

  return a.front().start < b.front().start;
}

// ----------------------------------------------------------------------------
std::vector<Trip> buildTrips(Mover const& mover)
{
  auto trips = std::vector<Trip>{};

  // Loop over adjacent points looking for stops
  for (auto const& wp : mover | kvr::sliding<2>)
  {
    auto const d = (wp[1].utmPoint - wp[0].utmPoint).squaredNorm();
    if (d < 50.0)
    {
      // Ignore duplicate waypoints at start
      if (trips.empty())
      {
        continue;
      }

      // Ignore very short stops that are probably due to traffic
      auto const cooldown = wp[1].time - wp[0].time;
      if (cooldown > 30.0)
      {
        // Add cooldown and start new leg
        trips.back().cooldown = cooldown;
        trips.push_back(Trip{wp[1].time, 0.0, {wp[1]}});
        continue;
      }
    }

    // Append second point to trip
    if (trips.empty())
    {
      trips.push_back(Trip{wp[0].time, 0.0, {wp[0]}});
    }
    trips.back().waypoints.push_back(wp[1]);
  }

  // Remove points that are "too close together"
  constexpr auto threshold = 1e3; // about 30 m
  for (auto& t : trips)
  {
    while (t.waypoints.size() > 2)
    {
      // Find the pair of points that are closest together
      auto shortestIndex = decltype(t.waypoints.size()){0};
      auto shortestDistance = std::numeric_limits<double>::infinity();
      for (auto const i : kvr::iota(t.waypoints.size() - 1))
      {
        auto const& w0 = t.waypoints[i + 0].utmPoint;
        auto const& w1 = t.waypoints[i + 1].utmPoint;
        auto const d = (w1 - w0).squaredNorm();
        if (d < shortestDistance)
        {
          shortestDistance = d;
          if (i == 0)
          {
            shortestIndex = i + 1;
          }
          else if (i + 2 == t.waypoints.size())
          {
            shortestIndex = i;
          }
          else
          {
            auto const& wa = t.waypoints[i - 1].utmPoint;
            auto const& wb = t.waypoints[i + 2].utmPoint;
            auto const da = (w1 - wa).squaredNorm();
            auto const db = (wb - w0).squaredNorm();
            shortestIndex = (da > db ? i + 1 : i);
          }
        }
      }

      // If the pair is within the threshold...
      if (shortestDistance < threshold)
      {
        // ...then remove it
        auto i = t.waypoints.begin();
        std::advance(i, shortestIndex);
        t.waypoints.erase(i);
      }
      else
      {
        // ...otherwise move on to the next trip
        break;
      }
    }
  }

  // Remove trips that are "too short" or degenerate
  for (auto i = trips.begin(); i != trips.end();)
  {
    auto remove = (i->waypoints.size() < 2);
    if (i->waypoints.size() == 2)
    {
      auto const& w0 = i->waypoints.front().utmPoint;
      auto const& w1 = i->waypoints.back().utmPoint;
      auto const d = (w1 - w0).squaredNorm();
      remove = (d < threshold);
    }

    if (remove)
    {
      if (i != trips.begin())
      {
        auto const next = (i + 1);
        if (next != trips.end())
        {
          next->cooldown += i->cooldown;
        }
      }
      i = trips.erase(i);
    }
    else
    {
      ++i;
    }
  }

  return trips;
}

// ----------------------------------------------------------------------------
std::vector<std::string> route(
  tb::location_t const& start, tb::location_t const& stop,
  tb::RoutingEngine const& engine, tb::Graph const& graph,
  tb::Segmentation& segmentation)
{
  auto const trip = engine.route(start, stop, graph);

  auto result = std::vector<std::string>{};
  for (auto const& leg : trip)
  {
    for (auto const& edge : segmentation.edges(leg))
    {
      auto s = std::to_string(edge.way);
      if (!edge.forward)
      {
        s = "-" + s;
      }
      if (edge.segment >= 0)
      {
        s = s + "#" + std::to_string(edge.segment);
      }
      result.push_back(std::move(s));
    }
  }

  return result;
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
  auto const crs = graph.crs();

  // Read the Valhalla configuration and construct a routing engine
  auto config = tb::readConfig(argv[Arguments::Config]);
  auto engine = tb::RoutingEngine{config};

  // Open the waypoints file
  auto in = tb::CsvStream{argv[Arguments::WaypointsPath]};

  // Read waypoints
  auto movers = MoverMap{};
  auto epoch = std::numeric_limits<double>::infinity();
  while (in.nextRecord())
  {
    auto const& id = in.nextValue<std::string>();
    auto const& ts = in.nextValue<std::string>(1);
    auto const& lat = in.nextValue<double>();
    auto const& lon = in.nextValue<double>();

    if (!id || !ts || !lat || !lon)
    {
      std::cerr << "Error parsing waypoint: " << in.record() << std::endl;
      return 1;
    }

    auto const time = tb::parseTime(*ts);
    if (!std::isfinite(time))
    {
      std::cerr << "Error parsing waypoint: " << in.record() << std::endl;
      return 1;
    }

    auto const ll = tb::location_t{*lon, *lat};
    auto const& utm = kv::geo_conv(ll, kv::SRID::lat_lon_WGS84, crs);
    movers[*id].push_back(Waypoint{time, ll, utm});
    epoch = std::min(time, epoch);
  }

  // Convert waypoints to trip plans
  std::cout << "Parsing " << movers.size() << " movers\n";
  auto trips = std::vector<std::vector<Trip>>{};
  for (auto& mover : movers)
  {
    auto& points = mover.second;

    // First, erase points for which we do not have map data
    for (auto i = points.begin(); i != points.end();)
    {
      constexpr auto epsilon = 1e4;
      auto const& ll = engine.locate(i->llPoint);
      auto const& utm = kv::geo_conv(ll, kv::SRID::lat_lon_WGS84, crs);

      if ((utm - i->utmPoint).squaredNorm() > epsilon)
      {
        i = points.erase(i);
      }
      else
      {
        ++i;
      }
    }

    // Convert whatever is left (if anything) into a trip plan
    if (points.size() > 1)
    {
      trips.push_back(buildTrips(points));
    }
  }
  std::cout << "Generated " << trips.size() << " trips\n";

  // Sort trips by start time (needed to make SUMO happy)
  std::sort(trips.begin(), trips.end(), compareTripStart);

  // Write XML header
  auto output = std::ofstream{argv[Arguments::OutputPath]};
  if (!output.is_open())
  {
    std::cerr << "Failed to open output file" << std::endl;
    return 1;
  }
  output << R"(<?xml version="1.0" encoding="UTF-8"?>)" << std::endl
         << R"(<routes)"
         << R"( xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance")"
         << R"( xsi:noNamespaceSchemaLocation)"
         << R"(="http://sumo.dlr.de/xsd/routes_file.xsd">)"
         << std::endl;

  // Build routes from trips in range
  auto tripCount = 0;
  auto segmentation = tb::Segmentation{&graph};
  for (auto const& trip : trips)
  {
    // Gather all edges for route
    auto allEdges = std::vector<std::string>{};
    auto stops = std::vector<Stop>{};
    for (auto const& leg : trip)
    {
      for (auto const& wp : leg.waypoints | kvr::sliding<2>)
      {
        auto edges = route(wp[0].llPoint, wp[1].llPoint,
                           engine, graph, segmentation);

        // Add to accumulated list of edges for the route
        if (allEdges.empty())
        {
          allEdges = std::move(edges);
        }
        else
        {
          // Skip duplicate edges
          auto i = edges.begin();
          while (i != edges.end() && *i == allEdges.back())
          {
            ++i;
          }
          allEdges.insert(
            allEdges.end(),
            std::make_move_iterator(i),
            std::make_move_iterator(edges.end()));
        }
      }

      // Extract stop
      if (!allEdges.empty())
      {
        stops.push_back(Stop{allEdges.back(), leg.cooldown});
      }
    }

    // Write route and vehicle
    if (allEdges.size() > 1)
    {
      // Write route header
      std::cout << "Trip " << tripCount
                << " has " << allEdges.size() << " edges\n";
      output << R"(  <route id="route_)" << tripCount
             << R"(" color="yellow" edges=")";

      // Write route edges
      auto first = true;
      for (auto const& edge : allEdges)
      {
        if (!first)
        {
          output << ' ';
        }
        output << edge;
        first = false;
      }

      if (stops.size() > 1)
      {
        output << R"(">)" << std::endl;

        // Write stops
        for (auto const n : kvr::iota(stops.size() - 1))
        {
          auto const& stop = stops[n];
          output << R"(    <stop lane=")" << stop.edge
                 << R"(_0" duration=")" << stop.duration
                 << R"(" parking="true"/>)" << std::endl;
        }
        output << R"(  </route>)" << std::endl;
      }
      else
      {
        // Write end of route (no stops)
        output << R"("/>)" << std::endl;
      }

      // Write vehicle for route
      output << R"(  <vehicle id="vehicle_)" << tripCount
             << R"(" depart=")" << trip.front().start - epoch
             << R"(" route="route_)" << tripCount
             << R"("/>)" << std::endl;
      ++tripCount;
    }
  }

  // Write XML footer
  output << R"(</routes>)" << std::endl;

  return 0;
}
