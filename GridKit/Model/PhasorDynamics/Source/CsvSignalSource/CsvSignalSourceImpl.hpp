/**
 * @file CsvSignalSourceImpl.hpp
 * @brief Definition of a CSV-backed signal source
 */

#pragma once

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Model/PhasorDynamics/Source/CsvSignalSource/CsvSignalSource.hpp>
#include <GridKit/Model/PhasorDynamics/Source/CsvSignalSource/CsvSignalSourceData.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Source
    {
      using Log = ::GridKit::Utilities::Logger;

      template <class ScalarT, typename IdxT>
      CsvSignalSource<ScalarT, IdxT>::CsvSignalSource()
      {
        size_  = 1;
        time_  = 0.0;
        alpha_ = 0.0;
      }

      template <class ScalarT, typename IdxT>
      CsvSignalSource<ScalarT, IdxT>::CsvSignalSource(signal_type*           output,
                                                      const model_data_type& data)
      {
        signals_.template assignSignalNode<CsvSignalSourceInternalVariables::VALUE>(output);
        size_  = 1;
        time_  = 0.0;
        alpha_ = 0.0;
        initializeParameters(data);
      }

      template <class ScalarT, typename IdxT>
      CsvSignalSource<ScalarT, IdxT>::CsvSignalSource(const model_data_type& data)
      {
        size_  = 1;
        time_  = 0.0;
        alpha_ = 0.0;
        initializeParameters(data);
      }

      template <class ScalarT, typename IdxT>
      void CsvSignalSource<ScalarT, IdxT>::initializeParameters(const model_data_type& data)
      {
        file_         = data.file;
        time_column_  = data.time_column;
        value_column_ = data.value_column;
        value_scale_  = data.value_scale;
        value_offset_ = data.value_offset;
      }

      template <class ScalarT, typename IdxT>
      int CsvSignalSource<ScalarT, IdxT>::setGridKitComponentID(IdxT component_id)
      {
        gridkit_component_id_ = component_id;
        return 0;
      }

      template <class ScalarT, typename IdxT>
      int CsvSignalSource<ScalarT, IdxT>::allocate()
      {
        auto size = static_cast<size_t>(size_);
        f_.resize(size);
        y_.resize(size);
        yp_.resize(size);
        tag_.resize(size);
        variable_indices_.resize(size);
        residual_indices_.resize(size);

        this->setVariableIndex(static_cast<IdxT>(VALUE_INDEX), static_cast<IdxT>(VALUE_INDEX));
        this->setResidualIndex(static_cast<IdxT>(VALUE_INDEX), static_cast<IdxT>(VALUE_INDEX));

        if (signals_.template isAssigned<CsvSignalSourceInternalVariables::VALUE>())
        {
          signals_.template getSignalNode<CsvSignalSourceInternalVariables::VALUE>()->set(
              &y_[VALUE_INDEX], &(this->getVariableIndex(static_cast<IdxT>(VALUE_INDEX))));
        }

        csv_error_count_ = loadCsv();

        return 0;
      }

      template <class ScalarT, typename IdxT>
      int CsvSignalSource<ScalarT, IdxT>::verify() const
      {
        int ret = 0;

        if (!signals_.template isAssigned<CsvSignalSourceInternalVariables::VALUE>())
        {
          Log::error() << "CsvSignalSource: required output signal VALUE is not assigned\n";
          ret += 1;
        }

        if (time_samples_.size() < 2)
        {
          Log::error() << "CsvSignalSource: expected at least two CSV samples\n";
          ret += 1;
        }

        if (csv_error_count_ > 0)
        {
          Log::error() << "CsvSignalSource: CSV load reported " << csv_error_count_ << " error(s)\n";
          ret += csv_error_count_;
        }

        if (time_samples_.size() != value_samples_.size())
        {
          Log::error() << "CsvSignalSource: time and value sample counts must match\n";
          ret += 1;
        }

        for (size_t i = 1; i < time_samples_.size(); ++i)
        {
          if (time_samples_[i] <= time_samples_[i - 1])
          {
            Log::error() << "CsvSignalSource: time samples must be strictly increasing\n";
            ret += 1;
            break;
          }
        }

        return ret;
      }

      template <class ScalarT, typename IdxT>
      int CsvSignalSource<ScalarT, IdxT>::initialize()
      {
        y_[VALUE_INDEX]  = interpolate(time_);
        yp_[VALUE_INDEX] = 0.0;
        return 0;
      }

      template <class ScalarT, typename IdxT>
      int CsvSignalSource<ScalarT, IdxT>::tagDifferentiable()
      {
        tag_[VALUE_INDEX] = false;
        return 0;
      }

      template <class ScalarT, typename IdxT>
      int CsvSignalSource<ScalarT, IdxT>::evaluateResidual()
      {
        f_[VALUE_INDEX] = y_[VALUE_INDEX] - interpolate(time_);
        return 0;
      }

      template <class ScalarT, typename IdxT>
      int CsvSignalSource<ScalarT, IdxT>::loadCsv()
      {
        time_samples_.clear();
        value_samples_.clear();

        if (file_.empty())
        {
          Log::error() << "CsvSignalSource: CSV file path is empty\n";
          return 1;
        }

        std::ifstream input(file_);
        if (!input)
        {
          Log::error() << "CsvSignalSource: unable to open CSV file " << file_ << "\n";
          return 1;
        }

        std::string header_line;
        if (!std::getline(input, header_line))
        {
          Log::error() << "CsvSignalSource: CSV file is empty " << file_ << "\n";
          return 1;
        }

        auto header       = splitCsvLine(header_line);
        auto time_column  = header.size();
        auto value_column = header.size();

        for (size_t i = 0; i < header.size(); ++i)
        {
          if (header[i] == time_column_)
          {
            time_column = i;
          }
          if (header[i] == value_column_)
          {
            value_column = i;
          }
        }

        if (time_column == header.size())
        {
          Log::error() << "CsvSignalSource: missing time column " << time_column_ << "\n";
          return 1;
        }
        if (value_column == header.size())
        {
          Log::error() << "CsvSignalSource: missing value column " << value_column_ << "\n";
          return 1;
        }

        int         ret = 0;
        size_t      line_number{1};
        std::string line;
        while (std::getline(input, line))
        {
          ++line_number;
          if (trim(line).empty())
          {
            continue;
          }

          auto columns = splitCsvLine(line);
          if (columns.size() <= std::max(time_column, value_column))
          {
            Log::error() << "CsvSignalSource: too few columns on line " << line_number << "\n";
            ret += 1;
            continue;
          }

          try
          {
            RealT t     = static_cast<RealT>(std::stod(columns[time_column]));
            RealT value = static_cast<RealT>(std::stod(columns[value_column]));

            time_samples_.push_back(t);
            value_samples_.push_back(value_scale_ * value + value_offset_);
          }
          catch (const std::exception&)
          {
            Log::error() << "CsvSignalSource: invalid numeric value on line " << line_number << "\n";
            ret += 1;
          }
        }

        return ret;
      }

      template <class ScalarT, typename IdxT>
      typename CsvSignalSource<ScalarT, IdxT>::RealT CsvSignalSource<ScalarT, IdxT>::interpolate(RealT t) const
      {
        if (time_samples_.empty())
        {
          return 0.0;
        }

        if (t <= time_samples_.front())
        {
          return value_samples_.front();
        }

        if (t >= time_samples_.back())
        {
          return value_samples_.back();
        }

        auto hi = std::upper_bound(time_samples_.begin(), time_samples_.end(), t);
        auto i  = static_cast<size_t>(hi - time_samples_.begin() - 1);

        RealT theta = (t - time_samples_[i]) / (time_samples_[i + 1] - time_samples_[i]);
        return (1.0 - theta) * value_samples_[i] + theta * value_samples_[i + 1];
      }

      template <class ScalarT, typename IdxT>
      auto CsvSignalSource<ScalarT, IdxT>::trim(const std::string& value) -> std::string
      {
        auto first = std::find_if(value.begin(), value.end(), [](unsigned char c)
                                  { return !std::isspace(c); });
        auto last  = std::find_if(value.rbegin(), value.rend(), [](unsigned char c)
                                 { return !std::isspace(c); })
                        .base();

        if (first >= last)
        {
          return {};
        }

        return std::string(first, last);
      }

      template <class ScalarT, typename IdxT>
      auto CsvSignalSource<ScalarT, IdxT>::splitCsvLine(const std::string& line) -> std::vector<std::string>
      {
        std::vector<std::string> columns;
        std::stringstream        stream(line);
        std::string              column;

        while (std::getline(stream, column, ','))
        {
          columns.push_back(trim(column));
        }

        return columns;
      }

    } // namespace Source
  } // namespace PhasorDynamics
} // namespace GridKit
