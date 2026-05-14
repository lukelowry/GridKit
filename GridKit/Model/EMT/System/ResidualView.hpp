#pragma once

#include <array>
#include <span>

#include <GridKit/Model/EMT/System/Layout.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class ResidualView
    {
    public:
      ResidualView(ScalarT*              f,
                   const Layout<IdxT>&   layout,
                   ComponentLayout<IdxT> component,
                   std::span<const IdxT> terminal_bus_ids)
        : f_{f},
          layout_{layout},
          component_{component},
          terminal_bus_ids_{terminal_bus_ids}
      {
      }

      void equation(IdxT local, ScalarT value)
      {
        f_[component_.equation_offset + local] = value;
      }

      void addEquation(IdxT local, ScalarT value)
      {
        f_[component_.equation_offset + local] += value;
      }

      template <size_t N>
      void equations(IdxT first, const std::array<ScalarT, N>& values)
      {
        for (IdxT i = 0; i < static_cast<IdxT>(N); ++i)
        {
          equation(first + i, values[i]);
        }
      }

      void inject(IdxT terminal, const std::array<ScalarT, 3>& current)
      {
        const IdxT bus = terminal_bus_ids_[terminal];
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          f_[layout_.busEquation(bus, phase)] += current[phase];
        }
      }

    private:
      ScalarT*              f_{nullptr};
      const Layout<IdxT>&   layout_;
      ComponentLayout<IdxT> component_;
      std::span<const IdxT> terminal_bus_ids_;
    };
  } // namespace EMT
} // namespace GridKit
