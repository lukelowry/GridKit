#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdlib>
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
    // Algebraic currents settle on subnormal magnitudes while a switch is
    // open. Those are valid finite values, so the parse must not reject the
    // underflow that std::stod reports as an out-of-range error.
    char*      end   = nullptr;
    const auto value = std::strtod(text.c_str(), &end);
    if (end != text.c_str() + text.size() || !std::isfinite(value))
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

  void validateMatchingTimes(const CsvTable& reference,
                             const CsvTable& candidate)
  {
    if (reference.rows.size() != candidate.rows.size())
    {
      throw std::runtime_error(
          "EMT integrator outputs have different sample counts");
    }

    const auto       reference_time = requireColumn(reference, "t");
    const auto       candidate_time = requireColumn(candidate, "t");
    constexpr double time_tolerance = 1.0e-13;
    for (std::size_t row = 0; row < reference.rows.size(); ++row)
    {
      if (std::abs(reference.rows[row][reference_time]
                   - candidate.rows[row][candidate_time])
          > time_tolerance)
      {
        throw std::runtime_error(
            "EMT integrator outputs have different timestamps");
      }
    }
  }

  double maximumPhaseNormalizedL2Error(
      const CsvTable&                   reference,
      const CsvTable&                   candidate,
      const std::array<std::string, 3>& columns,
      double                            window_begin,
      double                            window_end)
  {
    const auto time_column = requireColumn(reference, "t");
    double     maximum_error{0.0};
    for (const auto& column : columns)
    {
      const auto  reference_column = requireColumn(reference, column);
      const auto  candidate_column = requireColumn(candidate, column);
      double      squared_error{0.0};
      double      squared_reference{0.0};
      std::size_t sample_count{0};

      for (std::size_t row = 0; row < reference.rows.size(); ++row)
      {
        const auto time = reference.rows[row][time_column];
        if (time <= window_begin || time > window_end)
        {
          continue;
        }

        const auto reference_value =
            reference.rows[row][reference_column];
        const auto candidate_value =
            candidate.rows[row][candidate_column];
        const auto difference  = candidate_value - reference_value;
        squared_error         += difference * difference;
        squared_reference     += reference_value * reference_value;
        ++sample_count;
      }
      if (sample_count == 0)
      {
        throw std::runtime_error(
            "EMT integrator comparison window has no usable samples");
      }
      // A branch behind an open switch is quiescent for whole windows, so an
      // empty window is measured against the channel's own working range
      // rather than against itself.
      double normalization = squared_reference;
      if (normalization == 0.0)
      {
        for (const auto& row : reference.rows)
        {
          normalization += row[reference_column] * row[reference_column];
        }
      }
      if (normalization == 0.0)
      {
        continue;
      }
      maximum_error = std::max(
          maximum_error,
          std::sqrt(squared_error / normalization));
    }
    return maximum_error;
  }

  double maximumPhaseRelativePeakError(
      const CsvTable&                   reference,
      const CsvTable&                   candidate,
      const std::array<std::string, 3>& columns,
      double                            window_begin,
      double                            window_end)
  {
    const auto time_column = requireColumn(reference, "t");
    double     maximum_error{0.0};
    for (const auto& column : columns)
    {
      const auto reference_column = requireColumn(reference, column);
      const auto candidate_column = requireColumn(candidate, column);
      double     reference_peak{0.0};
      double     candidate_peak{0.0};

      for (std::size_t row = 0; row < reference.rows.size(); ++row)
      {
        const auto time = reference.rows[row][time_column];
        if (time <= window_begin || time > window_end)
        {
          continue;
        }

        reference_peak = std::max(
            reference_peak,
            std::abs(reference.rows[row][reference_column]));
        candidate_peak = std::max(
            candidate_peak,
            std::abs(candidate.rows[row][candidate_column]));
      }
      // A branch behind an open switch has no peak of its own in a quiescent
      // window, so the difference is scaled by its peak over the whole record.
      double normalization = reference_peak;
      if (normalization == 0.0)
      {
        for (const auto& row : reference.rows)
        {
          normalization = std::max(normalization,
                                   std::abs(row[reference_column]));
        }
      }
      if (normalization == 0.0)
      {
        continue;
      }
      maximum_error = std::max(
          maximum_error,
          std::abs(candidate_peak - reference_peak) / normalization);
    }
    return maximum_error;
  }

  void validateIntegratorAgreement(const CsvTable& adaptive,
                                   const CsvTable& fixed)
  {
    constexpr double fault_on   = 0.0001;
    constexpr double fault_off  = 0.0002;
    constexpr double final_time = 0.0003;

    validateTimes(adaptive, final_time);
    validateTimes(fixed, final_time);
    validateMatchingTimes(adaptive, fixed);

    const std::array<std::string, 3> bus_voltage_columns{
        "Bus_632_va", "Bus_632_vb", "Bus_632_vc"};
    const std::array<std::string, 3> source_current_columns{
        "VoltageSource_source_650_ia",
        "VoltageSource_source_650_ib",
        "VoltageSource_source_650_ic"};
    const std::array<std::string, 3> fault_current_columns{
        "LoadZ_fault_632_ia", "LoadZ_fault_632_ib", "LoadZ_fault_632_ic"};

    struct Window
    {
      const char* name;
      double      begin;
      double      end;
      double      voltage_limit;
      double      current_limit;
    };

    const std::array<Window, 3> windows{{
        {"prefault", 0.0, fault_on, 1.0e-5, 1.0e-5},
        {"fault", fault_on, fault_off, 1.0e-2, 2.5e-3},
        {"post-clear", fault_off, final_time, 2.5e-2, 7.5e-3},
    }};

    for (const auto& window : windows)
    {
      const auto voltage_error = maximumPhaseNormalizedL2Error(
          adaptive,
          fixed,
          bus_voltage_columns,
          window.begin,
          window.end);
      const auto source_error = maximumPhaseNormalizedL2Error(
          adaptive,
          fixed,
          source_current_columns,
          window.begin,
          window.end);
      const auto fault_error = maximumPhaseNormalizedL2Error(
          adaptive,
          fixed,
          fault_current_columns,
          window.begin,
          window.end);
      if (voltage_error > window.voltage_limit
          || source_error > window.current_limit
          || fault_error > window.current_limit)
      {
        throw std::runtime_error(
            std::string{"EMT adaptive/fixed L2 mismatch in "} + window.name);
      }

      std::cout << window.name << " maximum phase normalized L2: voltage="
                << voltage_error << ", source_current=" << source_error
                << ", fault_current=" << fault_error << '\n';
    }

    constexpr double peak_limit = 7.5e-3;
    for (const auto& window : {windows[1], windows[2]})
    {
      const auto voltage_peak = maximumPhaseRelativePeakError(
          adaptive,
          fixed,
          bus_voltage_columns,
          window.begin,
          window.end);
      const auto source_peak = maximumPhaseRelativePeakError(
          adaptive,
          fixed,
          source_current_columns,
          window.begin,
          window.end);
      const auto fault_peak = maximumPhaseRelativePeakError(
          adaptive,
          fixed,
          fault_current_columns,
          window.begin,
          window.end);
      if (voltage_peak > peak_limit || source_peak > peak_limit
          || fault_peak > peak_limit)
      {
        throw std::runtime_error(
            std::string{"EMT adaptive/fixed peak mismatch in "} + window.name);
      }

      std::cout << window.name << " maximum phase relative peak error: voltage="
                << voltage_peak << ", source_current=" << source_peak
                << ", fault_current=" << fault_peak << '\n';
    }
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

  double sampledRms(const CsvTable&    table,
                    const std::string& column,
                    double             window_begin,
                    double             window_end,
                    bool               include_window_end)
  {
    const auto  time_column  = requireColumn(table, "t");
    const auto  value_column = requireColumn(table, column);
    double      sum_squares{0.0};
    std::size_t sample_count{0};
    for (const auto& row : table.rows)
    {
      const auto time       = row[time_column];
      const bool before_end = include_window_end ? time <= window_end
                                                 : time < window_end;
      if (time < window_begin || !before_end)
      {
        continue;
      }
      sum_squares += row[value_column] * row[value_column];
      ++sample_count;
    }
    if (sample_count == 0)
    {
      throw std::runtime_error("EMT response window has no samples");
    }
    return std::sqrt(sum_squares / static_cast<double>(sample_count));
  }

  void validateSwitchedFaultResponse(const CsvTable& table)
  {
    constexpr double                 cycle      = 1.0 / 60.0;
    constexpr double                 fault_on   = 0.050;
    constexpr double                 fault_off  = 0.100;
    constexpr double                 final_time = 0.300;
    const std::array<std::string, 3> bus_voltage_columns{
        "Bus_632_va", "Bus_632_vb", "Bus_632_vc"};
    const std::array<std::string, 3> source_current_columns{
        "VoltageSource_source_650_ia",
        "VoltageSource_source_650_ib",
        "VoltageSource_source_650_ic"};
    const std::array<std::string, 3> fault_current_columns{
        "LoadZ_fault_632_ia", "LoadZ_fault_632_ib", "LoadZ_fault_632_ic"};

    for (std::size_t phase = 0; phase < bus_voltage_columns.size(); ++phase)
    {
      const auto prefault_voltage  = sampledRms(table,
                                               bus_voltage_columns[phase],
                                               fault_on - cycle,
                                               fault_on,
                                               false);
      const auto fault_voltage     = sampledRms(table,
                                            bus_voltage_columns[phase],
                                            fault_off - cycle,
                                            fault_off,
                                            true);
      const auto recovered_voltage = sampledRms(table,
                                                bus_voltage_columns[phase],
                                                final_time - cycle,
                                                final_time,
                                                true);
      const auto prefault_source   = sampledRms(table,
                                              source_current_columns[phase],
                                              fault_on - cycle,
                                              fault_on,
                                              false);
      const auto fault_source      = sampledRms(table,
                                           source_current_columns[phase],
                                           fault_off - cycle,
                                           fault_off,
                                           true);
      const auto post_clear_source = sampledRms(table,
                                                source_current_columns[phase],
                                                final_time - cycle,
                                                final_time,
                                                true);

      const auto prefault_fault_current = sampledRms(table,
                                                     fault_current_columns[phase],
                                                     fault_on - cycle,
                                                     fault_on,
                                                     false);
      const auto cleared_fault_current  = sampledRms(table,
                                                    fault_current_columns[phase],
                                                    final_time - cycle,
                                                    final_time,
                                                    true);
      const auto faulted_fault_current  = sampledRms(table,
                                                    fault_current_columns[phase],
                                                    fault_off - cycle,
                                                    fault_off,
                                                    true);

      // An open switch determines the branch current, so the fault branch
      // carries nothing at all before the fault and after clearing.
      if (!(prefault_fault_current < 1.0e-9 * faulted_fault_current)
          || !(cleared_fault_current < 1.0e-9 * faulted_fault_current))
      {
        throw std::runtime_error(
            "EMT open fault switch did not hold the branch current at zero");
      }
      if (!(fault_voltage < 0.5 * prefault_voltage))
      {
        throw std::runtime_error(
            "EMT switched fault did not depress the faulted bus voltage");
      }
      if (!(fault_source > 20.0 * prefault_source))
      {
        throw std::runtime_error(
            "EMT switched fault did not produce a source-current transient");
      }
      if (std::abs(recovered_voltage - prefault_voltage)
              > 0.05 * prefault_voltage
          || std::abs(post_clear_source - prefault_source)
                 > 0.05 * prefault_source)
      {
        throw std::runtime_error(
            "EMT voltage or source current did not recover after clearing");
      }

      std::cout << "Phase " << phase
                << " response RMS: prefault_voltage=" << prefault_voltage
                << ", fault_voltage=" << fault_voltage
                << ", recovered_voltage=" << recovered_voltage
                << ", prefault_source_current=" << prefault_source
                << ", fault_source_current=" << fault_source
                << ", post_clear_source_current=" << post_clear_source
                << '\n';
    }
  }

} // namespace

int main(int argc, const char* argv[])
{
  if (argc == 3 && std::string{argv[1]} == "--periodic")
  {
    try
    {
      const auto table = readCsv(argv[2]);
      validateTimes(table, 0.200);
      validatePrefaultPeriodicSteadyState(table);
      return 0;
    }
    catch (const std::exception& error)
    {
      std::cerr << "EMT periodic-state validation failed: " << error.what()
                << '\n';
      return 1;
    }
  }

  if (argc == 4 && std::string{argv[1]} == "--compare")
  {
    try
    {
      const auto adaptive = readCsv(argv[2]);
      const auto fixed    = readCsv(argv[3]);
      validateIntegratorAgreement(adaptive, fixed);
      return 0;
    }
    catch (const std::exception& error)
    {
      std::cerr << "EMT integrator comparison failed: " << error.what()
                << '\n';
      return 1;
    }
  }

  if (argc != 2)
  {
    std::cerr << "Usage: test_emt_three_bus_validator <monitor-csv>\n"
              << "   or: test_emt_three_bus_validator --periodic "
                 "<monitor-csv>\n"
              << "   or: test_emt_three_bus_validator --compare "
                 "<adaptive-csv> <fixed-csv>\n";
    return 1;
  }

  try
  {
    const auto table = readCsv(argv[1]);
    validateTimes(table, 0.300);
    validatePrefaultPeriodicSteadyState(table);
    validateSwitchedFaultResponse(table);
  }
  catch (const std::exception& error)
  {
    std::cerr << "EMT three-bus validation failed: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
