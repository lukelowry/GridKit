#pragma once

#include <cstddef>
#include <functional>
#include <iomanip>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include <GridKit/Model/VariableMonitor.hpp>
#include <GridKit/ScalarTraits.hpp>

namespace GridKit
{
  namespace Model
  {
    template <class ScalarT>
    class CallbackVariableMonitor : public VariableMonitorBase
    {
    public:
      using RealT  = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using Getter = std::function<RealT()>;

      explicit CallbackVariableMonitor(std::string label)
        : label_(std::move(label))
      {
      }

      template <class FuncT>
      void add(std::string variable, FuncT getter)
      {
        entries_.push_back({std::move(variable),
                            [getter = std::move(getter)]()
                            {
                              return static_cast<RealT>(getter());
                            }});
      }

      bool empty() const override
      {
        return entries_.empty();
      }

    private:
      void printHeader(std::ostream& os, Csv csv) const override
      {
        for (const auto& entry : entries_)
        {
          os << csv.delim << label_ << '_' << entry.variable;
        }
      }

      void print(std::ostream& os, Csv csv) const override
      {
        for (const auto& entry : entries_)
        {
          os << csv.delim << entry.getter();
        }
      }

      void print(std::ostream& os, Json) const override
      {
        if (empty())
        {
          return;
        }

        os << "    " << std::quoted(label_) << ": {\n";
        for (size_t i = 0; i < entries_.size(); ++i)
        {
          const auto& entry = entries_[i];
          os << "      " << std::quoted(entry.variable) << ": " << entry.getter();
          if (i + 1 < entries_.size())
          {
            os << ',';
          }
          os << '\n';
        }
        os << "    }";
      }

      void print(std::ostream& os, Yaml) const override
      {
        if (empty())
        {
          return;
        }

        os << "    " << label_ << ":\n";
        for (const auto& entry : entries_)
        {
          os << "      " << entry.variable << ": " << entry.getter() << '\n';
        }
      }

      struct Entry
      {
        std::string variable;
        Getter      getter;
      };

      std::string        label_;
      std::vector<Entry> entries_;
    };
  } // namespace Model
} // namespace GridKit
