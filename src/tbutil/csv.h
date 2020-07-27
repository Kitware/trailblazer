/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#ifndef _TRAILBLAZER_CSV_H
#define _TRAILBLAZER_CSV_H

#include "types.h"

#include "tbutil_export.h"

#include <unordered_map>
#include <vector>

namespace trailblazer
{
  using CSV = std::vector<std::pair<std::string, std::vector<float>>>;

  TBUTIL_EXPORT bool ReadCSV(std::string const& filename, CSV& data);
  TBUTIL_EXPORT bool WriteCSV(std::string const& filename, CSV const& data);

} // namespace trailblazer

#endif
