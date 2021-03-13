/* Distributed under the Apache License, Version 2.0.
   See accompanying NOTICE file for details.*/

#pragma once

#include "tbutil/types.h"
#include "tbutil/tbutil_export.h"

#include <unordered_map>

namespace trailblazer
{

class Graph;

// ----------------------------------------------------------------------------
class TBUTIL_EXPORT Segmentation
{
public:
  Segmentation(Graph const* graph);

  /// Compute edges from a routing leg
  std::vector<Edge> edges(Leg const& leg);

protected:
  Graph const* m_graph;
  std::unordered_map<id_t, segmentation_t> m_segmentations;
};

} // namespace trailblazer

