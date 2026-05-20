#pragma once

#include <cstddef>
#include <cstdint>

namespace GridKit
{
  namespace Model
  {
    namespace Events
    {
      class PhaseMask
      {
      public:
        constexpr PhaseMask() = default;

        static constexpr PhaseMask none()
        {
          return PhaseMask{0b000};
        }

        static constexpr PhaseMask a()
        {
          return PhaseMask{0b001};
        }

        static constexpr PhaseMask b()
        {
          return PhaseMask{0b010};
        }

        static constexpr PhaseMask c()
        {
          return PhaseMask{0b100};
        }

        static constexpr PhaseMask abc()
        {
          return PhaseMask{0b111};
        }

        static constexpr PhaseMask fromBits(std::uint8_t bits)
        {
          return PhaseMask{static_cast<std::uint8_t>(bits & abc().bits_)};
        }

        constexpr bool includes(std::size_t phase) const
        {
          return phase < 3 && (bits_ & static_cast<std::uint8_t>(1u << phase)) != 0u;
        }

        constexpr bool empty() const
        {
          return bits_ == 0u;
        }

        constexpr std::uint8_t bits() const
        {
          return bits_;
        }

        constexpr PhaseMask with(PhaseMask phases) const
        {
          return fromBits(static_cast<std::uint8_t>(bits_ | phases.bits_));
        }

        constexpr PhaseMask without(PhaseMask phases) const
        {
          return fromBits(static_cast<std::uint8_t>(bits_ & static_cast<std::uint8_t>(~phases.bits_)));
        }

        friend constexpr bool operator==(PhaseMask lhs, PhaseMask rhs)
        {
          return lhs.bits_ == rhs.bits_;
        }

        friend constexpr bool operator!=(PhaseMask lhs, PhaseMask rhs)
        {
          return !(lhs == rhs);
        }

      private:
        explicit constexpr PhaseMask(std::uint8_t bits)
          : bits_{bits}
        {
        }

        std::uint8_t bits_{0b111};
      };
    } // namespace Events
  } // namespace Model
} // namespace GridKit
