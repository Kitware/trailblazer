/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#ifndef _TRAILBLAZER_SEGMENTATION_H
#define _TRAILBLAZER_SEGMENTATION_H

#include <tbutil/types.h>

#include <tbutil/tbutil_export.h>

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

#endif
