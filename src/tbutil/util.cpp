/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#include "util.h"

#include <vital/plugin_loader/plugin_manager.h>

#include <vital/math_constants.h>

#include <limits>
#include <regex>

#include <cmath>
#include <ctime>

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

// ----------------------------------------------------------------------------
double parseTime(std::string const& in)
{
  constexpr static auto format =
    R"((\d+)-(\d?\d)-(\d?\d)T([012]?\d):([0-6]\d):([0-6]\d)(\.\d+)?Z)";
  static auto const re = std::regex(format);

  auto m = std::smatch{};
  if (std::regex_match(in, m, re))
  {
    tm time;
    time.tm_year = std::stoi(m[1]) - 1900;
    time.tm_mon = std::stoi(m[2]) - 1;
    time.tm_mday = std::stoi(m[3]);
    time.tm_hour = std::stoi(m[4]);
    time.tm_min = std::stoi(m[5]);
    time.tm_sec = std::stoi(m[6]);
    time.tm_gmtoff = 0;
    time.tm_isdst = 0;

    auto const& fpart = m[7];
    auto const frac = (fpart.matched ? std::stod("0" + fpart.str()) : 0.0);

#ifdef _WIN32
    auto* const toUTC = &_mkgmtime;
#else
    auto* const toUTC = &timegm;
#endif
    return static_cast<double>(toUTC(&time)) + frac;
  }

  return std::numeric_limits<double>::quiet_NaN();
}

} // namespace trailblazer
