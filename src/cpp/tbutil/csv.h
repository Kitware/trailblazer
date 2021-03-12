/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#ifndef _TRAILBLAZER_CSV_H
#define _TRAILBLAZER_CSV_H

#include <tbutil/tbutil_export.h>

#include <vital/optional.h>

#include <memory>
#include <string>

namespace trailblazer
{

// ----------------------------------------------------------------------------
/// Utility class to read a CSV file
class TBUTIL_EXPORT CsvStream
{
public:
  /// Constructor.
  ///
  /// \param config Path to CSV file to read.
  CsvStream(std::string const& path);

  CsvStream(CsvStream&&);
  ~CsvStream();

  bool nextRecord();

  template <typename T>
  kwiver::vital::optional<T> nextValue(unsigned skip = 0);

  std::string record() const;

private:
  CsvStream(CsvStream const&) = delete;

  class Private;
  std::unique_ptr<Private> m_p;
};

template <>
kwiver::vital::optional<std::string> CsvStream::nextValue(unsigned skip);

template <>
kwiver::vital::optional<double> CsvStream::nextValue(unsigned skip);

} // namespace trailblazer

#endif
