/* This file is part of Trailblazer, and is distributed under the OSI-approved
 * BSD 3-Clause License. See top-level LICENSE file or
 * https://github.com/Kitware/trailblazer/blob/master/LICENSE for details. */

#include "csv.h"

#include <fstream>

namespace trailblazer
{

  // ----------------------------------------------------------------------------
  // trim from start (in place)
  static inline void ltrim(std::string& s)
  {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch)
    {
      return !std::isspace(ch);
    }));
  }

  // ----------------------------------------------------------------------------
  // trim from end (in place)
  static inline void rtrim(std::string& s)
  {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch)
    {
      return !std::isspace(ch);
    }).base(), s.end());
  }

  // ----------------------------------------------------------------------------
  // trim from both ends (in place)
  static inline void trim(std::string& s)
  {
    ltrim(s);
    rtrim(s);
  }

  bool WriteCSV(std::string const& filename, CSV const& data)
  {
    std::ofstream csv;
    csv.open(filename, std::ofstream::out | std::ofstream::trunc);
    if (!csv.good())
      return false;

    size_t num_rows = 0;
    size_t num_columns = data.size();
    // Write out the headers
    for (auto columns : data)
    {
      num_columns--;
      csv << columns.first;
      if (num_columns > 0)
        csv << ", ";
      num_rows = columns.second.size();
    }
    csv << "\n";

    // Write out the data
    for (size_t idx = 0; idx < num_rows; idx++)
    {
      // Write out values
      num_columns = data.size();
      for (auto columns : data)
      {
        num_columns--;
        auto& v = columns.second;
        csv << v[idx];
        if (num_columns > 0)
          csv << ", ";
      }
      csv << "\n";
    }
    csv << "\n";
    csv.close();
    return true;
  }

  bool ReadCSV(std::string const& filename, CSV& data)
  {
    data.clear();
    std::ifstream csv;
    csv.open(filename, std::ofstream::in);
    if (!csv.good())
      return false;

    int line_num = 1;
    int idx;
    bool get_names = true;
    std::string line;
    while (!csv.eof()) // To get you all the lines.
    {
      idx = 0;
      std::getline(csv, line); // Get the line.
      if (line.empty())
        break;
      std::stringstream s_stream(line); // Create stringstream from the line
      while (s_stream.good())
      {
        std::string value;
        std::getline(s_stream, value, ','); //get first string delimited by comma
        if (get_names)
        {
          trim(value);
          data.push_back(std::pair<std::string, std::vector<float>>(value, std::vector<float>()));
        }
        else
        {
          try {
            data[idx++].second.push_back(std::stof(value));
          }
          catch (...) { return false; }
        }
      }
      line_num++;
      get_names = false;
    }

    return true;
  }

} // namespace trailblazer
