/* Distributed under the Apache License, Version 2.0.
   See accompanying NOTICE file for details.*/
   
#pragma once

#include "tbutil/graph.h"

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

