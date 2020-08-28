/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#include "reader.h"

#include <vital/types/geodesy.h>

#include <vital/range/iota.h>

#include <readosm.h>

namespace kv = kwiver::vital;
namespace kvr = kwiver::vital::range;

namespace trailblazer
{

// ----------------------------------------------------------------------------
Reader::Reader(char const* path)
{
  m_status = readosm_open(path, &m_handle);
};

// ----------------------------------------------------------------------------
Reader::~Reader()
{
  readosm_close(m_handle);
}

// ----------------------------------------------------------------------------
Reader::operator bool() const
{
  return m_status == READOSM_OK;
}

// ----------------------------------------------------------------------------
void Reader::exec()
{
  readosm_node_callback parseNode =
    [](void const* p, readosm_node const* node) -> int
    {
      auto* const self = reinterpret_cast<Reader*>(const_cast<void*>(p));

      auto const lat = node->latitude;
      auto const lon = node->longitude;
      auto const raw = kv::vector_2d{lon, lat};
      auto const zone = kv::utm_ups_zone(raw);

      auto controlled = false;
      for (auto const t : kvr::iota(node->tag_count))
      {
        auto const tag = node->tags[t];
        if (strcmp(tag.key, "highway") == 0 ||
            strcmp(tag.key, "crossing") == 0)
        {
          if (strcmp(tag.value, "traffic_signals") == 0)
          {
            controlled = true;
            break;
          }
        }
      }

      self->m_nodes.emplace(node->id, Node{node->id, raw, {}, controlled});

      auto const zi = zone.number * (zone.north ? +1 : -1);
      ++self->m_crs_use[zi];

      return READOSM_OK;
    };

  readosm_way_callback parseWay =
    [](void const* p, readosm_way const* way) -> int
    {
      auto* const self = reinterpret_cast<Reader*>(const_cast<void*>(p));

      auto const count = static_cast<size_t>(way->node_ref_count);

      auto nodes = std::vector<id_t>{};
      nodes.reserve(count);

      for (auto const i : kvr::iota(count))
      {
        nodes.push_back(way->node_refs[i]);
      }

      if (nodes.size() > 1)
      {
        self->m_ways.emplace(way->id, Way{way->id, nodes});
      }

      return READOSM_OK;
    };

  m_status = readosm_parse(m_handle, this, parseNode, parseWay, nullptr);

  int zone = 0;
  long long count = 0;
  for (auto const& i : m_crs_use)
  {
    if (i.second > count)
    {
      count = i.second;
      zone = i.first;
    }
  }

  if (count > 0)
  {
    if (zone > 0)
    {
      m_crs = kv::SRID::UTM_WGS84_north + zone;
    }
    else
    {
      m_crs = kv::SRID::UTM_WGS84_south - zone;
    }
  }
  else
  {
    m_crs = -1;
  }

  this->convertFrom(kv::SRID::lat_lon_WGS84);
  this->build();
}

} // namespace trailblazer
