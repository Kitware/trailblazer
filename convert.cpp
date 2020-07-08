/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#include <vital/plugin_loader/plugin_manager.h>

#include <vital/types/geo_point.h>
#include <vital/types/geodesy.h>

#include <vital/range/iota.h>

#include <readosm.h>

#include <iostream>
#include <unordered_map>
#include <vector>

#include <cmath>
#include <cstdio>

namespace kv = kwiver::vital;
namespace kvr = kwiver::vital::range;

namespace trailblazer
{

using id_t = long long;

// ----------------------------------------------------------------------------
class Reader
{
public:
  Reader(char const* path);
  ~Reader();

  operator bool() const { return m_status == READOSM_OK; }

  void exec();

private:
  struct Node
  {
    id_t id;
    kv::geo_point::geo_2d_point_t location;
    std::vector<id_t> ways;
  };

  struct Way
  {
    id_t id;
    std::vector<id_t> nodes;
  };

  void const* m_handle;
  int m_status;

  std::unordered_map<id_t, Node> m_nodes;
  std::unordered_map<id_t, Way> m_ways;
  std::unordered_map<int, long long> m_crs_use;
  int m_crs;
};

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
void Reader::exec()
{
  std::cout << "Parsing OSM" << std::endl;

  readosm_node_callback parseNode =
    [](void const* p, readosm_node const* node) -> int
    {
      auto* const self = reinterpret_cast<Reader*>(const_cast<void*>(p));

      auto const lat = node->latitude;
      auto const lon = node->longitude;
      auto const raw = kv::vector_2d{lon, lat};
      auto const zone = kv::utm_ups_zone(raw);

      self->m_nodes.emplace(node->id, Node{node->id, raw});

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

  std::cout << "Determining UTM zone... ";

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
      std::cout << +zone << " North (";
      m_crs = kv::SRID::UTM_WGS84_north + zone;
    }
    else
    {
      std::cout << +zone << " South (";
      m_crs = kv::SRID::UTM_WGS84_south - zone;
    }
    std::cout << m_crs << ')' << std::endl;
  }
  else
  {
    std::cout << "no data?" << std::endl;
  }

  std::cout << "Converting node locations" << std::endl;
  for (auto& n : m_nodes)
  {
    constexpr auto WGS = kv::SRID::lat_lon_WGS84;
    auto& location = n.second.location;
    location = kv::geo_conv(location, WGS, m_crs);
  }

  std::cout << "Building graph" << std::endl;
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

} // namespace trailblazer

// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <in>" << std::endl;
    return 1;
  }

  kv::plugin_manager::instance().load_all_plugins();

  trailblazer::Reader reader{argv[1]};
  if (reader)
  {
    reader.exec();
  }

  return 0;
}
