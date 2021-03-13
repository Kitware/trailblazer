/* Distributed under the Apache License, Version 2.0.
   See accompanying NOTICE file for details.*/

#include "tbutil/segmentation.h"
#include "tbutil/graph.h"
#include "tbutil/util.h"

#include <vital/range/iota.h>

namespace kvr = kwiver::vital::range;

namespace trailblazer
{

// ----------------------------------------------------------------------------
Segmentation::Segmentation(Graph const* graph)
  : m_graph{graph}
{
}

// ----------------------------------------------------------------------------
std::vector<Edge> Segmentation::edges(Leg const& leg)
{
  auto const way = leg.way;
  auto const nodeCount = leg.nodes.size();
  if (!m_graph || nodeCount < 2)
  {
    return {};
  }

  auto out = std::vector<Edge>{};
  std::vector<size_t> indices;

  // Get way segmentation and determine segment id, if relevant
  auto i = m_segmentations.find(way);
  if (i == m_segmentations.end())
  {
    i = m_segmentations.emplace(way, m_graph->segment(way, true)).first;
  }

  // Get end bearing (for certain pathological cases of self-intersecting
  // ways with turn restrictions)
  auto const& l0 = m_graph->node(leg.nodes[nodeCount - 1])->location;
  auto const& l1 = m_graph->node(leg.nodes[nodeCount - 2])->location;
  auto const finalBearing = computeBearing(l0, l1);

  // Determine start and stop way node indices
  auto const& hStart =
    m_graph->locate(leg.way, leg.nodes.front(), leg.bearing);
  auto const& hStop =
    m_graph->locate(leg.way, leg.nodes.back(), finalBearing);

  // Build index list for leg
  auto const offset = hStart.node;
  if (hStart.forward)
  {
    for (auto const i : kvr::iota(hStop.node - offset + 1))
    {
      indices.push_back(offset + i);
    }
  }
  else
  {
    for (auto const i : kvr::iota(offset + 1 - hStop.node))
    {
      indices.push_back(offset - i);
    }
  }

  // Check against way segmentation
  auto const& segmentation = i->second;
  if (auto const sc = segmentation.size())
  {
    auto const l = std::min(indices.front(), indices.back());
    auto const u = std::max(indices.front(), indices.back());
    for (auto const si : kvr::iota(sc))
    {
      auto const& ss = segmentation[si];
      if (l < ss.second && u > ss.first)
      {
        out.push_back(Edge{way, static_cast<int>(si), hStart.forward});
      }
    }
  }
  else
  {
    out.push_back(Edge{way, -1, hStart.forward});
  }

  if (!hStart.forward)
  {
    std::reverse(out.begin(), out.end());
  }

  return out;
}

} // namespace trailblazer
