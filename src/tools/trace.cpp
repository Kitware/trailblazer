/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#include <tbutil/reader.h>
#include <tbutil/route.h>
#include <tbutil/util.h>

#include <vital/range/iota.h>

#include <algorithm>
#include <iostream>

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
struct Segment
{
  tb::id_t way;
  std::vector<size_t> indices;
  bool forward;
};

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
  auto config = tb::readConfig(argv[Arguments::Config]);

  // Get start and stop points
  auto const startLL = tb::location_t{
    std::strtod(argv[Arguments::StartLon], nullptr),
    std::strtod(argv[Arguments::StartLat], nullptr)};

  auto const stopLL = tb::location_t{
    std::strtod(argv[Arguments::StopLon], nullptr),
    std::strtod(argv[Arguments::StopLat], nullptr)};

  // Get trip from Valhalla
  auto const trip = route(startLL, stopLL, config, graph);
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
    std::cout << "way " << leg.way << std::endl;
    for (auto const n : leg.nodes)
    {
      std::cout << "  node " << n << std::endl;
    }

    // Determine way node indices and create segment
    auto const nodeCount = leg.nodes.size();
    if (nodeCount > 1)
    {
      // Get end bearing (for certain pathological cases of self-intersecting
      // ways with turn restrictions)
      auto const& l0 = graph.node(leg.nodes[nodeCount - 1])->location;
      auto const& l1 = graph.node(leg.nodes[nodeCount - 2])->location;
      auto const finalBearing = tb::computeBearing(l0, l1);

      // Determine start and stop way node indices
      auto const& hStart =
        graph.locate(leg.way, leg.nodes.front(), leg.bearing);
      auto const& hStop =
        graph.locate(leg.way, leg.nodes.back(), finalBearing);

      std::cout << "  bearing " << leg.bearing
                << " to " << finalBearing << std::endl;

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

  std::cout << std::endl;

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
