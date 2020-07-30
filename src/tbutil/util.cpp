/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#include "util.h"

#include <vital/plugin_loader/plugin_manager.h>

#include <vital/math_constants.h>

#include <cmath>

namespace kv = kwiver::vital;

namespace trailblazer
{

// ----------------------------------------------------------------------------
void init()
{
  kwiver::vital::plugin_manager::instance().load_all_plugins();
}
// ----------------------------------------------------------------------------
double computeBearing(location_t origin, location_t target)
{
  auto const d = (target - origin).normalized();
  auto const c = std::acos(d.y()) / kv::deg_to_rad;
  return (d.x() > 0 ? c : 360.0 - c);
}

} // namespace trailblazer
