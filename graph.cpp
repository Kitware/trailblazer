/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#include "graph.h"

#include <vital/types/geodesy.h>

namespace kv = kwiver::vital;

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
