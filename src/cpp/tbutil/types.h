/* Distributed under the Apache License, Version 2.0.
   See accompanying NOTICE file for details.*/

#pragma once

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
  bool controlled;
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

// ----------------------------------------------------------------------------
/// Description of a routing leg
struct Leg
{
  /// Identifier of the way being traversed
  id_t way;

  /// Initial bearing (0-360; 0 = North, 90 = East)
  double bearing;

  /// List of OSM nodes along this leg
  std::vector<id_t> nodes;

  /// List of locations along this leg
  std::vector<location_t> points;
};

// ----------------------------------------------------------------------------
/// Description of a route edge
struct Edge
{
  /// Identifier of the way being traversed
  id_t way;

  /// Index of the way segment being traversed
  /// (negative if there is only one segment)
  int segment;

  /// True if the way is being traversed forwards (node indices increasing)
  bool forward;
};

} // namespace trailblazer

