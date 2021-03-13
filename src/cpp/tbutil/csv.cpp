/* Distributed under the Apache License, Version 2.0.
   See accompanying NOTICE file for details.*/

#include "tbutil/csv.h"

#include <algorithm>
#include <fstream>
#include <sstream>

#include <cctype>

using kwiver::vital::optional;

namespace // anonymous
{

// ----------------------------------------------------------------------------
std::string trim(std::string const& in)
{
  if (in.empty())
  {
    return {};
  }

  int (*isspace)(int) = &std::isspace;
  auto const begin = std::find_if_not(in.begin(), in.end(), isspace);
  if (begin == in.end())
  {
    return {};
  }

  auto end = std::find_if_not(in.rbegin(), in.rend(), isspace);
  return {begin, end.base()};
}

} // namespace <anonymous>

namespace trailblazer
{

// ----------------------------------------------------------------------------
class CsvStream::Private
{
public:
  Private(std::string const& path)
    : stream{path}
  {
  }

  std::ifstream stream;
  std::stringstream record;
};

// ----------------------------------------------------------------------------
CsvStream::CsvStream(std::string const& path)
  : m_p{new Private{path}}
{
}

// ----------------------------------------------------------------------------
CsvStream::CsvStream(CsvStream&& other)
  : m_p{std::move(other.m_p)}
{
}

// ----------------------------------------------------------------------------
CsvStream::~CsvStream()
{
}

// ----------------------------------------------------------------------------
bool CsvStream::nextRecord()
{
  auto buffer = std::string{};
  for (;;)
  {
    // Read next line from stream
    if (!std::getline(m_p->stream, buffer))
    {
      return false;
    }
    buffer = trim(buffer);

    // Skip blank lines and comments
    if (buffer.empty() || buffer[0] == '#')
    {
      continue;
    }

    // Reset record stream
    m_p->record = std::stringstream{buffer};
    return true;
  }
}

// ----------------------------------------------------------------------------
template <>
optional<std::string> CsvStream::nextValue(unsigned skip)
{

  for (;;)
  {
    auto out = std::string{};
    if (std::getline(m_p->record, out, ','))
    {
      if (skip)
      {
        --skip;
        continue;
      }
      return trim(out);
    }
  }

  return {};
}


// ----------------------------------------------------------------------------
template <>
optional<double> CsvStream::nextValue(unsigned skip)
{
  auto const& v = this->nextValue<std::string>(skip);
  if (v && !v->empty())
  {
    auto p = static_cast<char*>(nullptr);
    auto const out = std::strtod(v->data(), &p);
    if (*p == 0) // All characters were consumed
    {
      return out;
    }
  }
  return {};
}

// ----------------------------------------------------------------------------
std::string CsvStream::record() const
{
  return m_p->record.str();
}

} // namespace trailblazer
