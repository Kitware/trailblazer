/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#ifndef _TRAILBLAZER_TYPES_H
#define _TRAILBLAZER_TYPES_H

#include "types.h"

#include <vital/types/geo_point.h>

#include <vector>

namespace trailblazer
{

/// Data type of an OSM identifier (nodes, ways, etc.)
using id_t = long long;

// ----------------------------------------------------------------------------
/// Simplified representation of an OSM node
struct Node
{
  id_t id;
  kwiver::vital::geo_point::geo_2d_point_t location;
  std::vector<id_t> ways;
};

// ----------------------------------------------------------------------------
/// Simplified representation of an OSM way
struct Way
{
  id_t id;
  std::vector<id_t> nodes;
};

} // namespace trailblazer

#endif
