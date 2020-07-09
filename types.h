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

/// Data type of a geodetic location
using location_t = kwiver::vital::geo_point::geo_2d_point_t;

/// Segmentation of a way
///
/// \sa Graph::segment
using segmentation_t = std::vector<std::pair<size_t, size_t>>;

// ----------------------------------------------------------------------------
/// Simplified representation of an OSM node
struct Node
{
  id_t id;
  location_t location;
  std::vector<id_t> ways;
};

// ----------------------------------------------------------------------------
/// Simplified representation of an OSM way
struct Way
{
  id_t id;
  std::vector<id_t> nodes;
};

// ----------------------------------------------------------------------------
/// Description of a heading
struct Heading
{
  /// Identifier of the way being traversed
  id_t way;

  /// Index of the node at which the way is being traversed
  size_t node;

  /// True if the way is being traversed forwards (node indices increasing)
  bool forward;
};

} // namespace trailblazer

#endif
