/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#ifndef _TRAILBLAZER_GRAPH_H
#define _TRAILBLAZER_GRAPH_H

#include <tbutil/types.h>

#include <tbutil/tbutil_export.h>

#include <unordered_map>
#include <vector>

namespace trailblazer
{

// ----------------------------------------------------------------------------
class TBUTIL_EXPORT Graph
{
public:
  int crs() const;

  Node const* node(id_t which) const;
  Way const* way(id_t which) const;

  /// Return the node on \p way which is closest to \p location
  Node const* locate(id_t way, location_t const& location) const;

  /// Determine a ::Heading for a way at a specific node
  ///
  /// \param way The way being traversed
  /// \param node The node at which to make a determination
  /// \param bearing The bearing in degrees to match (0 = North, 90 = East)
  Heading locate(id_t way, id_t node, double bearing) const;

  /// Break a way into segments
  ///
  /// This method divides a way at intersections (nodes which have more than
  /// three edges, or which are used by the same way more than once) and
  /// returns the list of resulting segments according to the way indices which
  /// comprise each segment.
  ///
  /// If \p splitLoops is \c true, segments which form a loop (i.e. start and
  /// end at the same node) will be split at their midpoint into two segments.
  ///
  /// If the way has only one segment, an empty vector is returned.
  segmentation_t segment(id_t way, bool splitLoops) const;

protected:
  void build();
  void convertFrom(int from);

  std::unordered_map<id_t, Node> m_nodes;
  std::unordered_map<id_t, Way> m_ways;
  int m_crs = -1;
};

} // namespace trailblazer

#endif
