/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#include "util.h"

#include <vital/plugin_loader/plugin_manager.h>

// ----------------------------------------------------------------------------
void trailblazer::init()
{
  kwiver::vital::plugin_manager::instance().load_all_plugins();
}
