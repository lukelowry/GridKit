/**
 * @file CsvSignalSource.hpp
 * @brief Declaration of a CSV-backed signal source
 */

#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include <GridKit/Model/PhasorDynamics/Component.hpp>
#include <GridKit/Model/PhasorDynamics/ComponentSignals.hpp>
#include <GridKit/Model/PhasorDynamics/Source/CsvSignalSource/CsvSignalSourceData.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    template <class ScalarT, typename IdxT>
    class SignalNode;

    namespace Source
    {
      template <typename RealT, typename IdxT>
      struct CsvSignalSourceData;
    } // namespace Source
  } // namespace PhasorDynamics
} // namespace GridKit

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Source
    {
      /// Internal variables of a `CsvSignalSource`
      enum class CsvSignalSourceInternalVariables : size_t
      {
        VALUE, ///< Signal value
        MAXIMUM,
      };

      /// External variables of a `CsvSignalSource`
      enum class CsvSignalSourceExternalVariables : size_t
      {
        MAXIMUM,
      };

      /**
       * @brief Signal source driven by CSV samples.
       *
       * The inherited `y_` vector stores one algebraic output variable,
       * which is assigned to a `SignalNode` for use by other components.
       */
      template <class ScalarT, typename IdxT>
      class CsvSignalSource : public Component<ScalarT, IdxT>
      {
        using Component<ScalarT, IdxT>::gridkit_component_id_;
        using Component<ScalarT, IdxT>::alpha_;
        using Component<ScalarT, IdxT>::f_;
        using Component<ScalarT, IdxT>::size_;
        using Component<ScalarT, IdxT>::tag_;
        using Component<ScalarT, IdxT>::time_;
        using Component<ScalarT, IdxT>::y_;
        using Component<ScalarT, IdxT>::yp_;
        using Component<ScalarT, IdxT>::J_;
        using Component<ScalarT, IdxT>::J_rows_buffer_;
        using Component<ScalarT, IdxT>::J_cols_buffer_;
        using Component<ScalarT, IdxT>::J_vals_buffer_;
        using Component<ScalarT, IdxT>::variable_indices_;
        using Component<ScalarT, IdxT>::residual_indices_;

      public:
        using RealT           = typename Component<ScalarT, IdxT>::RealT;
        using model_data_type = CsvSignalSourceData<RealT, IdxT>;
        using signal_type     = SignalNode<ScalarT, IdxT>;

        CsvSignalSource();
        CsvSignalSource(signal_type* output, const model_data_type& data);
        CsvSignalSource(const model_data_type& data);
        ~CsvSignalSource() = default;

        int setGridKitComponentID(IdxT) override final;
        int allocate() override final;
        int verify() const override final;
        int initialize() override final;
        int tagDifferentiable() override final;
        int evaluateResidual() override final;
        int evaluateJacobian() override final;

        /// Get the `ComponentSignals` from this `CsvSignalSource`
        auto getSignals()
            -> ComponentSignals<ScalarT,
                                IdxT,
                                CsvSignalSourceInternalVariables,
                                CsvSignalSourceExternalVariables>&
        {
          return signals_;
        }

      private:
        void  initializeParameters(const model_data_type& data);
        int   loadCsv();
        RealT interpolate(RealT t) const;

        static auto trim(const std::string& value) -> std::string;
        static auto splitCsvLine(const std::string& line) -> std::vector<std::string>;

      private:
        static constexpr size_t VALUE_INDEX = 0;

        std::filesystem::path file_;
        std::string           time_column_{"t"};
        std::string           value_column_{"u"};
        RealT                 value_scale_{1.0};
        RealT                 value_offset_{0.0};

        std::vector<RealT> time_samples_;
        std::vector<RealT> value_samples_;
        int                csv_error_count_{0};

        ComponentSignals<ScalarT,
                         IdxT,
                         CsvSignalSourceInternalVariables,
                         CsvSignalSourceExternalVariables>
            signals_;
      };

    } // namespace Source
  } // namespace PhasorDynamics
} // namespace GridKit
