/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#ifndef _TRAILBLAZER_UTIL_H
#define _TRAILBLAZER_UTIL_H

#include <tbutil/types.h>

#include <tbutil/tbutil_export.h>

namespace trailblazer
{

// ----------------------------------------------------------------------------
/// Initialize library.
///
/// This function prepares the library for use, and should be called by users
/// prior to using any other functionality of the library to avoid potential
/// misbehavior or abnormal program termination.
///
/// It is safe to call this function more than once.
void TBUTIL_EXPORT init();

// ----------------------------------------------------------------------------
/// Calculate bearing from \p origin to \p target.
///
/// This function computes the bearing (in degrees, 0-360) from \p origin to
/// \p target, with 0 = North, 90 = East, etc. The inputs must be in UTM
/// coordinates.
///
/// \sa trailblazer::Leg, trailblazer::Graph::locate
double TBUTIL_EXPORT computeBearing(location_t origin, location_t target);

} // namespace trailblazer

#endif
