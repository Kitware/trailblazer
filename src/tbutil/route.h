/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#ifndef _TRAILBLAZER_ROUTE_H
#define _TRAILBLAZER_ROUTE_H

#include <tbutil/types.h>

#include <tbutil/tbutil_export.h>

#include <boost/property_tree/ptree.hpp>

namespace trailblazer
{

using routing_config_t = boost::property_tree::ptree;

class Graph;

// ----------------------------------------------------------------------------
/// Read a Valhalla configuration from the specified path.
///
/// This function reads a Valhalla configuration from the JSON file named by
/// \p path. The configuration is suitable for passing to other functions which
/// require a Valhalla configuration.
///
/// \sa trailblazer::route
routing_config_t TBUTIL_EXPORT readConfig(char const* path);

// ----------------------------------------------------------------------------
/// Calculate bearing from \p origin to \p target.
///
/// This function computes the bearing (in degrees, 0-360) from \p origin to
/// \p target, with 0 = North, 90 = East, etc. The inputs must be in UTM
/// coordinates.
///
/// \sa trailblazer::Leg, trailblazer::Graph::locate
double TBUTIL_EXPORT computeBearing(location_t origin, location_t target);

// ----------------------------------------------------------------------------
/// Calculate a route between two waypoints
///
/// This function calculates a route between two waypoints, using Valhalla.
/// The route is returned as a list of trip legs with locations converted into
/// the CRS used by the specified \p graph.
///
/// \param start Starting waypoint for the route to be computed.
/// \param stop Stopping waypoint for the route to be computed.
/// \param config Valhalla configuration to use for computing the route.
/// \param graph
///   OSM graph to use to rectify the routing information returned by Valhalla
///   into OSM nodes.
///
/// \sa trailblazer::Leg
std::vector<Leg> TBUTIL_EXPORT route(
  location_t const& start, location_t const& stop,
  routing_config_t const& config, Graph const& graph);

} // namespace trailblazer

#endif