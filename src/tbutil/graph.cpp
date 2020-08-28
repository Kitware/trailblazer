/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#include "graph.h"

#include <vital/types/geodesy.h>

#include <vital/math_constants.h>

#include <vital/range/iota.h>

#include <limits>

#include <cmath>

namespace kv = kwiver::vital;
namespace kvr = kwiver::vital::range;

namespace trailblazer
{

// ----------------------------------------------------------------------------
int Graph::crs() const
{
  return m_crs;
}

// ----------------------------------------------------------------------------
Node const* Graph::node(id_t which) const
{
  auto const& iter = m_nodes.find(which);
  return (iter == m_nodes.end() ? nullptr : std::addressof(iter->second));
}

// ----------------------------------------------------------------------------
Way const* Graph::way(id_t which) const
{
  auto const& iter = m_ways.find(which);
  return (iter == m_ways.end() ? nullptr : std::addressof(iter->second));
}

// ----------------------------------------------------------------------------
Node const* Graph::locate(id_t wi, location_t const& location) const
{
  Node const* result = nullptr;
  auto best = std::numeric_limits<double>::infinity();

  if (auto* const wp = this->way(wi))
  {
    for (auto const ni : wp->nodes)
    {
      if (auto* const np = this->node(ni))
      {
        auto const delta = location - np->location;
        auto const d = delta.squaredNorm();
        if (d < best)
        {
          result = np;
          best = d;
        }
      }
    }
  }

  return result;
}

// ----------------------------------------------------------------------------
Heading Graph::locate(id_t wi, id_t ni, double bDeg) const
{
  if (auto* const wp = this->way(wi))
  {
    auto const bRad = bDeg * kv::deg_to_rad;
    auto const vec = kv::vector_2d{std::sin(bRad), std::cos(bRad)};
    auto const k = wp->nodes.size();

    // Initial best heading
    auto bestAngle = -1.0;
    auto bestIndex = k;
    auto bestDir = true;

    // Helper to test a pair of nodes
    auto test = [&](size_t i, size_t j)
    {
      if (auto* const n0 = this->node(wp->nodes[i]))
      {
        if (auto* const n1 = this->node(wp->nodes[j]))
        {
          // Get direction vector and angle-cosine with desired bearing
          auto const& l0 = n0->location;
          auto const& l1 = n1->location;
          auto const v = (l1 - l0).normalized();
          auto const q = v.dot(vec);

          // Test against current best
          if (q > bestAngle)
          {
            bestAngle = q;
            bestIndex = i;
            bestDir = (j > i);
          }
        }
      }
    };

    // Loop over all nodes in the way
    for (auto const i : kvr::iota(k))
    {
      // Only test at the requested node
      if (wp->nodes[i] == ni)
      {
        // Test both directions (except at ends of the way)
        if (i > 0)
        {
          test(i, i - 1);
        }
        if (i + 1 < k)
        {
          test(i, i + 1);
        }
      }
    }

    // Return result, if any
    if (bestIndex < k)
    {
      return {wi, bestIndex, bestDir};
    }
  }

  return {-1, 0, true};
}

// ----------------------------------------------------------------------------
segmentation_t Graph::segment(id_t wi, bool splitLoops) const
{
  if (auto* const wp = this->way(wi))
  {
    auto const testLoop = [](Way const* way, size_t n0, size_t n1)
    {
      return ((n1 - n0) > 1 && way->nodes[n0] == way->nodes[n1]);
    };

    auto const count = wp->nodes.size();
    if (count > 2)
    {
      // Get intersections along way (don't count the ends; they are always
      // intersections, but don't split the way)
      auto intersections = std::vector<size_t>{};
      for (auto const i : kvr::iota(count - 2))
      {
        auto const ni = wp->nodes[i + 1];
        if (auto const np = this->node(ni))
        {
          // A node used by the middle of a way has two edges of the way, but
          // only records the way once; since the start and end are always
          // intersections anyway, and interior node with more than one way
          // must also be in intersection
          if (np->ways.size() > 1)
          {
            intersections.push_back(i + 1);
          }
          // Additionally, we need to split ways at traffic control devices,
          // because these affect flow and so need to be treated as separate
          // edges
          else if (np->controlled)
          {
            intersections.push_back(i + 1);
          }
        }
      }

      if (!intersections.empty())
      {
        // Prepare result
        auto result = segmentation_t{};
        result.reserve(intersections.size() + 1);

        // Generate segments from indices that are intersections
        auto last = size_t{0};
        for (auto const i : intersections)
        {
          if (splitLoops && testLoop(wp, last, i))
          {
            auto const mid = (last + i) / 2;
            result.emplace_back(last, mid);
            result.emplace_back(mid, i);
          }
          else
          {
            result.emplace_back(last, i);
          }
          last = i;
        }
        result.emplace_back(last, count - 1);

        return result;
      }
    }

    if (splitLoops && testLoop(wp, 0, count - 1))
    {
      auto const mid = count / 2;
      return {{0, mid}, {mid, count - 1}};
    }
  }

  return {};
}

// ----------------------------------------------------------------------------
void Graph::build()
{
  for (auto const& wi : m_ways)
  {
    for (auto const& n : wi.second.nodes)
    {
      auto ni = m_nodes.find(n);
      if (ni != m_nodes.end())
      {
        ni->second.ways.push_back(wi.first);
      }
    }
  }
}

// ----------------------------------------------------------------------------
void Graph::convertFrom(int from)
{
  for (auto& n : m_nodes)
  {
    auto& location = n.second.location;
    location = kv::geo_conv(location, from, m_crs);
  }
}

} // namespace trailblazer
