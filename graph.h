/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#ifndef _TRAILBLAZER_GRAPH_H
#define _TRAILBLAZER_GRAPH_H

#include "types.h"

#include "tbutil_export.h"

#include <unordered_map>
#include <vector>

namespace trailblazer
{

using id_t = long long;

// ----------------------------------------------------------------------------
class TBUTIL_EXPORT Graph
{
public:
  int crs() const;

  Node const* node(id_t which) const;
  Way const* way(id_t which) const;

  /// Return the node on \p way which is closest to \p location
  Node const* locate(id_t way, location_t const& location) const;

protected:
  void build();
  void convertFrom(int from);

  std::unordered_map<id_t, Node> m_nodes;
  std::unordered_map<id_t, Way> m_ways;
  int m_crs = -1;
};

} // namespace trailblazer

#endif
