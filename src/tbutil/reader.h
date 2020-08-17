/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#ifndef _TRAILBLAZER_READER_H
#define _TRAILBLAZER_READER_H

#include <tbutil/graph.h>

namespace trailblazer
{

// ----------------------------------------------------------------------------
class TBUTIL_EXPORT Reader : public Graph
{
public:
  Reader(char const* path);
  ~Reader();

  operator bool() const;

  void exec();

private:
  void const* m_handle;
  int m_status;

  std::unordered_map<int, long long> m_crs_use;
};

} // namespace trailblazer

#endif
