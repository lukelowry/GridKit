#include <array>
#include <cmath>
#include <complex>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
  struct CsvTable
  {
    std::map<std::string, std::size_t> column_indices;
    std::vector<std::vector<double>>   rows;
  };

  std::vector<std::string> splitCsvLine(const std::string& line)
  {
    std::vector<std::string> fields;
    std::stringstream        input(line);
    std::string              field;
    while (std::getline(input, field, ','))
    {
      if (!field.empty() && field.back() == '\r')
      {
        field.pop_back();
      }
      fields.push_back(field);
    }
    return fields;
  }

  double parseFiniteValue(const std::string& text)
  {
    std::size_t parsed{0};
    const auto  value = std::stod(text, &parsed);
    if (parsed != text.size() || !std::isfinite(value))
    {
      throw std::runtime_error("EMT monitor CSV contains a non-finite or invalid value");
    }
    return value;
  }

  CsvTable readCsv(const std::string& file_name)
  {
    std::ifstream input(file_name);
    if (!input)
    {
      throw std::runtime_error("Could not open EMT monitor CSV: " + file_name);
    }

    std::string header_line;
    if (!std::getline(input, header_line))
    {
      throw std::runtime_error("EMT monitor CSV is empty");
    }

    CsvTable   table;
    const auto headers = splitCsvLine(header_line);
    if (headers.empty() || headers.front() != "t")
    {
      throw std::runtime_error("EMT monitor CSV must begin with a t column");
    }
    for (std::size_t column = 0; column < headers.size(); ++column)
    {
      if (!table.column_indices.emplace(headers[column], column).second)
      {
        throw std::runtime_error("EMT monitor CSV contains a duplicate column: "
                                 + headers[column]);
      }
    }

    std::string line;
    while (std::getline(input, line))
    {
      if (line.empty())
      {
        continue;
      }
      const auto fields = splitCsvLine(line);
      if (fields.size() != headers.size())
      {
        throw std::runtime_error("EMT monitor CSV row has the wrong number of columns");
      }

      std::vector<double> row;
      row.reserve(fields.size());
      for (const auto& field : fields)
      {
        row.push_back(parseFiniteValue(field));
      }
      table.rows.push_back(std::move(row));
    }

    if (table.rows.empty())
    {
      throw std::runtime_error("EMT monitor CSV has no data rows");
    }
    return table;
  }

  std::size_t requireColumn(const CsvTable& table, const std::string& name)
  {
    const auto entry = table.column_indices.find(name);
    if (entry == table.column_indices.end())
    {
      throw std::runtime_error("EMT monitor CSV is missing column: " + name);
    }
    return entry->second;
  }

  void validateTimes(const CsvTable& table, double final_time)
  {
    const auto time_column = requireColumn(table, "t");
    auto       previous    = table.rows.front()[time_column];
    for (std::size_t row = 1; row < table.rows.size(); ++row)
    {
      const auto time = table.rows[row][time_column];
      if (!(time > previous))
      {
        throw std::runtime_error("EMT monitor timestamps are not strictly increasing");
      }
      previous = time;
    }

    constexpr double final_time_tolerance = 1.0e-10;
    if (std::abs(previous - final_time) > final_time_tolerance)
    {
      throw std::runtime_error("EMT monitor CSV does not end at the requested final time");
    }
  }

  double rms(const CsvTable& table,
             std::size_t     value_column,
             double          window_begin,
             double          window_end,
             bool            include_window_end)
  {
    const auto  time_column = requireColumn(table, "t");
    double      sum_squares{0.0};
    std::size_t sample_count{0};
    for (const auto& row : table.rows)
    {
      const auto time       = row[time_column];
      const bool before_end = include_window_end ? time <= window_end
                                                 : time < window_end;
      if (time >= window_begin && before_end)
      {
        sum_squares += row[value_column] * row[value_column];
        ++sample_count;
      }
    }
    if (sample_count < 100)
    {
      throw std::runtime_error("EMT monitor CSV has too few samples in an RMS window");
    }
    return std::sqrt(sum_squares / static_cast<double>(sample_count));
  }

  double threePhaseRms(const CsvTable&                   table,
                       const std::array<std::size_t, 3>& columns,
                       double                            window_begin,
                       double                            window_end)
  {
    const auto  time_column = requireColumn(table, "t");
    double      sum_squares{0.0};
    std::size_t sample_count{0};
    for (const auto& row : table.rows)
    {
      const auto time = row[time_column];
      if (time >= window_begin && time < window_end)
      {
        sum_squares  += row[columns[0]] * row[columns[0]];
        sum_squares  += row[columns[1]] * row[columns[1]];
        sum_squares  += row[columns[2]] * row[columns[2]];
        sample_count += 3;
      }
    }
    if (sample_count < 300)
    {
      throw std::runtime_error("EMT monitor CSV has too few source-current samples");
    }
    return std::sqrt(sum_squares / static_cast<double>(sample_count));
  }

  double interpolate(const CsvTable& table,
                     std::size_t     value_column,
                     double          target_time)
  {
    const auto time_column = requireColumn(table, "t");
    if (target_time < table.rows.front()[time_column]
        || target_time > table.rows.back()[time_column])
    {
      throw std::runtime_error("EMT interpolation time is outside the monitor range");
    }

    for (std::size_t row = 0; row < table.rows.size(); ++row)
    {
      const auto time = table.rows[row][time_column];
      if (time == target_time || row == 0)
      {
        if (time == target_time)
        {
          return table.rows[row][value_column];
        }
        continue;
      }
      if (time > target_time)
      {
        const auto previous_time  = table.rows[row - 1][time_column];
        const auto previous_value = table.rows[row - 1][value_column];
        const auto fraction       = (target_time - previous_time)
                              / (time - previous_time);
        return previous_value
               + fraction * (table.rows[row][value_column] - previous_value);
      }
    }
    return table.rows.back()[value_column];
  }

  std::complex<double> fundamentalPhasor(const CsvTable& table,
                                         std::size_t     value_column,
                                         double          window_begin,
                                         double          period)
  {
    const auto   time_column = requireColumn(table, "t");
    const double window_end  = window_begin + period;
    const double omega       = 2.0 * std::acos(-1.0) / period;
    double       cos_integral{0.0};
    double       sin_integral{0.0};
    double       previous_time  = window_begin;
    double       previous_value = interpolate(table, value_column, window_begin);

    const auto accumulate = [&](double time, double value)
    {
      const auto step  = time - previous_time;
      cos_integral    += 0.5 * step
                      * (previous_value * std::cos(omega * previous_time)
                         + value * std::cos(omega * time));
      sin_integral += 0.5 * step
                      * (previous_value * std::sin(omega * previous_time)
                         + value * std::sin(omega * time));
      previous_time  = time;
      previous_value = value;
    };

    for (const auto& row : table.rows)
    {
      const auto time = row[time_column];
      if (time <= window_begin)
      {
        continue;
      }
      if (time >= window_end)
      {
        break;
      }
      accumulate(time, row[value_column]);
    }
    accumulate(window_end, interpolate(table, value_column, window_end));

    const auto scale = std::sqrt(2.0) / period;
    return {scale * cos_integral, -scale * sin_integral};
  }

  void validatePrefaultPeriodicSteadyState(const CsvTable& table)
  {
    constexpr double                  cycle = 1.0 / 60.0;
    const auto                        first = table.rows.front()[requireColumn(table, "t")];
    const std::array<std::string, 12> columns{
        "Bus_650_va", "Bus_650_vb", "Bus_650_vc", "Bus_632_va", "Bus_632_vb", "Bus_632_vc", "Bus_670_va", "Bus_670_vb", "Bus_670_vc", "VoltageSource_source_650_ia", "VoltageSource_source_650_ib", "VoltageSource_source_650_ic"};

    double maximum_relative_change{0.0};
    for (const auto& column : columns)
    {
      const auto index           = requireColumn(table, column);
      const auto phasor1         = fundamentalPhasor(table, index, first, cycle);
      const auto phasor2         = fundamentalPhasor(table, index, first + cycle, cycle);
      const auto relative_change = std::abs(phasor2 - phasor1)
                                   / std::max(1.0, std::abs(phasor1));
      maximum_relative_change = std::max(maximum_relative_change,
                                         relative_change);
    }

    if (maximum_relative_change > 1.0e-3)
    {
      throw std::runtime_error(
          "EMT prefault state is not cycle-to-cycle periodic");
    }
    std::cout << "Maximum prefault phasor change: "
              << maximum_relative_change << '\n';
  }

  void validateFaultResponse(const CsvTable& table)
  {
    constexpr double cycle      = 1.0 / 60.0;
    constexpr double fault_on   = 0.050;
    constexpr double fault_off  = 0.100;
    constexpr double final_time = 0.200;

    const std::array<std::size_t, 3> bus_voltage_columns{
        requireColumn(table, "Bus_632_va"),
        requireColumn(table, "Bus_632_vb"),
        requireColumn(table, "Bus_632_vc")};
    const std::array<std::size_t, 3> source_current_columns{
        requireColumn(table, "VoltageSource_source_650_ia"),
        requireColumn(table, "VoltageSource_source_650_ib"),
        requireColumn(table, "VoltageSource_source_650_ic")};

    for (std::size_t phase = 0; phase < bus_voltage_columns.size(); ++phase)
    {
      const auto prefault  = rms(table,
                                bus_voltage_columns[phase],
                                fault_on - cycle,
                                fault_on,
                                false);
      const auto fault     = rms(table,
                             bus_voltage_columns[phase],
                             fault_off - cycle,
                             fault_off,
                             false);
      const auto recovered = rms(table,
                                 bus_voltage_columns[phase],
                                 final_time - cycle,
                                 final_time,
                                 true);

      if (!(fault < 0.8 * prefault))
      {
        throw std::runtime_error("Bus 632 phase voltage did not sag below 80 percent");
      }
      if (std::abs(recovered - prefault) > 0.05 * prefault)
      {
        throw std::runtime_error("Bus 632 phase voltage did not recover within 5 percent");
      }

      std::cout << "Bus 632 phase " << phase
                << " RMS: prefault=" << prefault
                << ", fault=" << fault
                << ", recovered=" << recovered << '\n';
    }

    const auto prefault_current = threePhaseRms(table,
                                                source_current_columns,
                                                fault_on - cycle,
                                                fault_on);
    const auto fault_current    = threePhaseRms(table,
                                             source_current_columns,
                                             fault_off - cycle,
                                             fault_off);
    if (!(fault_current > 2.0 * prefault_current))
    {
      throw std::runtime_error("Source-current RMS did not double during the fault");
    }

    std::cout << "Source current RMS: prefault=" << prefault_current
              << ", fault=" << fault_current << '\n';
  }
} // namespace

int main(int argc, const char* argv[])
{
  if (argc != 2)
  {
    std::cerr << "Usage: test_emt_three_bus_validator <monitor-csv>\n";
    return 1;
  }

  try
  {
    const auto table = readCsv(argv[1]);
    validateTimes(table, 0.200);
    validatePrefaultPeriodicSteadyState(table);
    validateFaultResponse(table);
  }
  catch (const std::exception& error)
  {
    std::cerr << "EMT three-bus validation failed: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
