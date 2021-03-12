/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#ifndef _TRAILBLAZER_ROUTE_H
#define _TRAILBLAZER_ROUTE_H

#include <tbutil/types.h>

#include <tbutil/tbutil_export.h>

#include <boost/property_tree/ptree.hpp>

#include <memory>

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
/// Utility class used to calculate routes
class TBUTIL_EXPORT RoutingEngine
{
public:
  /// Constructor.
  ///
  /// \param config Valhalla configuration to use for computing routes.
  RoutingEngine(routing_config_t const& config);

  RoutingEngine(RoutingEngine&&);
  ~RoutingEngine();

  /// Locate a point in the available routing data
  ///
  /// This function attempts to locate the specified point in the available
  /// routing data. This can be used to validate waypoints before using them to
  /// compute routes.
  location_t locate(location_t const& in) const;

  /// Calculate a route between two waypoints
  ///
  /// This function calculates a route between two waypoints, using Valhalla.
  /// The route is returned as a list of trip legs with locations converted
  /// into the CRS used by the specified \p graph.
  ///
  /// \param start Starting waypoint for the route to be computed.
  /// \param stop Stopping waypoint for the route to be computed.
  /// \param graph
  ///   OSM graph to use to rectify the routing information returned by
  ///   Valhalla into OSM nodes.
  ///
  /// \sa trailblazer::Leg
  std::vector<Leg> route(
    location_t const& start, location_t const& stop, Graph const& graph) const;

private:
  RoutingEngine(RoutingEngine const&) = delete;

  class Private;
  std::unique_ptr<Private> m_p;
};

} // namespace trailblazer

#endif
