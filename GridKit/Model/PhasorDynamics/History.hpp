/**
 * @file History.hpp
 * @brief Time history helper
 */

#pragma once

#include <algorithm>
#include <deque>
#include <limits>

namespace GridKit
{
  namespace PhasorDynamics
  {
    /**
     * @brief Time history buffer with bounded retention.
     *
     * This helper is intentionally independent of Component, SignalNode, and
     * solver types. Components own the sampled value type and decide when to
     * record values, usually from their `stepAccepted` implementation.
     */
    template <typename RealT, typename ValueT>
    class History
    {
    public:
      struct Sample
      {
        RealT  time;
        ValueT value;
      };

      History() = default;

      History(RealT window, RealT max_step_size = 0.0)
      {
        configure(window, max_step_size);
      }

      void configure(RealT window, RealT max_step_size = 0.0)
      {
        window_        = window;
        max_step_size_ = max_step_size;
      }

      void clear()
      {
        history_.clear();
        has_anchor_         = false;
        has_accepted_time_  = false;
        last_accepted_time_ = 0.0;
      }

      void record(RealT t, const ValueT& value)
      {
        if (window_ <= 0.0)
        {
          clear();
          return;
        }

        if (has_accepted_time_ && t < last_accepted_time_)
        {
          history_.clear();
          has_anchor_ = false;
        }

        if (!history_.empty() && t == history_.back().time)
        {
          history_.back().value = value;
        }
        else
        {
          history_.push_back({t, value});
        }

        last_accepted_time_ = t;
        has_accepted_time_  = true;

        prune(t);
      }

      void setMaxStepSize(RealT& hmax) const
      {
        if (window_ <= 0.0)
        {
          hmax = std::numeric_limits<RealT>::infinity();
          return;
        }

        hmax = window_;
        if (max_step_size_ > 0.0 && max_step_size_ < hmax)
        {
          hmax = max_step_size_;
        }
      }

      ValueT heldValue(RealT lookup_time, const ValueT& initial_value) const
      {
        if (history_.empty() || lookup_time < history_.front().time)
        {
          return has_anchor_ ? anchor_.value : initial_value;
        }

        auto it = std::upper_bound(history_.begin(),
                                   history_.end(),
                                   lookup_time,
                                   [](RealT t, const Sample& sample)
                                   {
                                     return t < sample.time;
                                   });

        if (it == history_.begin())
        {
          return has_anchor_ ? anchor_.value : initial_value;
        }

        --it;
        return it->value;
      }

      template <typename InterpolateT>
      ValueT interpolatedValue(RealT         lookup_time,
                               const ValueT& initial_value,
                               InterpolateT  interpolate) const
      {
        if (history_.empty())
        {
          return has_anchor_ ? anchor_.value : initial_value;
        }

        if (lookup_time < history_.front().time)
        {
          if (!has_anchor_)
          {
            return initial_value;
          }

          return interpolate(anchor_.value,
                             history_.front().value,
                             interpolationFraction(anchor_, history_.front(), lookup_time));
        }

        auto upper = std::upper_bound(history_.begin(),
                                      history_.end(),
                                      lookup_time,
                                      [](RealT t, const Sample& sample)
                                      {
                                        return t < sample.time;
                                      });

        if (upper == history_.end())
        {
          return history_.back().value;
        }

        auto lower = upper;
        --lower;

        return interpolate(lower->value,
                           upper->value,
                           interpolationFraction(*lower, *upper, lookup_time));
      }

      const std::deque<Sample>& samples() const
      {
        return history_;
      }

    private:
      static RealT interpolationFraction(const Sample& lower, const Sample& upper, RealT t)
      {
        const auto dt = upper.time - lower.time;
        if (dt == 0.0)
        {
          return 1.0;
        }

        return (t - lower.time) / dt;
      }

      void prune(RealT t)
      {
        const auto cutoff = t - window_;

        while (!history_.empty() && history_.front().time < cutoff)
        {
          anchor_     = history_.front();
          has_anchor_ = true;
          history_.pop_front();
        }
      }

    private:
      RealT window_{0.0};
      RealT max_step_size_{0.0};

      std::deque<Sample> history_;

      Sample anchor_{};
      bool   has_anchor_{false};

      RealT last_accepted_time_{0.0};
      bool  has_accepted_time_{false};
    };

  } // namespace PhasorDynamics
} // namespace GridKit
