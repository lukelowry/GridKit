/**
 * @file ModelWrappers.hpp
 * @author Nicholson Koukpaizan (koukpaizannk@ornl.gov)
 *
 */

#pragma once

#include <vector>

namespace GridKit
{
  namespace Enzyme
  {
    namespace Sparse
    {
      /**
       * @brief Model member function parameter keys
       *
       */
      enum class MemberFunctions
      {
        InternalResidual,
        InternalResidualWithSignal,
        InternalResidualWithSignalDerivative,
        InternalResidualWithBusDerivative,
        BusResidual,
        BusResidualWithBusDerivative,
        BusResidual11,                  //< Special case for branches that are connected to two buses
        BusResidual12,                  //< Special case for branches that are connected to two buses
        BusResidual21,                  //< Special case for branches that are connected to two buses
        BusResidual22,                  //< Special case for branches that are connected to two buses
        BusResidual11WithBusDerivative, //< Special case for branches that are connected to two buses
        BusResidual12WithBusDerivative, //< Special case for branches that are connected to two buses
        BusResidual21WithBusDerivative, //< Special case for branches that are connected to two buses
        BusResidual22WithBusDerivative  //< Special case for branches that are connected to two buses
      };

      /**
       * @brief Template definition for wrapper around residual methods inside model classes
       *
       * @tparam ModelT - model type
       * @tparam MemberFunctions - member function parameter key
       * @tparam ScalarT - scalar data type
       *
       */
      template <typename ModelT, MemberFunctions function, typename ScalarT>
      struct ModelWrapper
      {
      };

      /**
       * @brief Residual wrapper partial template specialization for InternalResidual
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::InternalResidual, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[out] f - Internal residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* f)
        {
          model->evaluateInternalResidual(y, yp, wb, f);
        }
      };

      /**
       * @brief Residual wrapper partial template specialization for InternalResidualWithSignal
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::InternalResidualWithSignal, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[in] ws - Signal variables
         * @param[out] f - Internal residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* ws, ScalarT* f)
        {
          model->evaluateInternalResidual(y, yp, wb, ws, f);
        }
      };

      /**
       * @brief Residual wrapper partial template specialization for InternalResidualWithSignalDerivative
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::InternalResidualWithSignalDerivative, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[in] ws - Signal variables
         * @param[in] wsp - Signal variable derivatives
         * @param[out] f - Internal residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* ws, ScalarT* wsp, ScalarT* f)
        {
          model->evaluateInternalResidual(y, yp, wb, ws, wsp, f);
        }
      };

      /**
       * @brief Residual wrapper partial template specialization for InternalResidualWithBusDerivative
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::InternalResidualWithBusDerivative, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[in] wbp - Bus variable derivatives
         * @param[out] f - Internal residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* wbp, ScalarT* f)
        {
          model->evaluateInternalResidual(y, yp, wb, wbp, f);
        }
      };

      /**
       * @brief Residual wrapper partial template specialization for BusResidual
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::BusResidual, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[out] h - Bus residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* h)
        {
          model->evaluateBusResidual(y, yp, wb, h);
        }
      };

      /**
       * @brief Residual wrapper partial template specialization for BusResidualWithBusDerivative
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::BusResidualWithBusDerivative, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[in] wbp - Bus variable derivatives
         * @param[out] h - Bus residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* wbp, ScalarT* h)
        {
          model->evaluateBusResidual(y, yp, wb, wbp, h);
        }
      };

      /**
       * @brief Residual wrapper partial template specialization for BusResidual11 (branch member function)
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::BusResidual11, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[out] h - Bus residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* h)
        {
          model->evaluateBusResidual11(y, yp, wb, h);
        }
      };

      /**
       * @brief Residual wrapper partial template specialization for BusResidual12 (branch member function)
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::BusResidual12, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[out] h - Bus residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* h)
        {
          model->evaluateBusResidual12(y, yp, wb, h);
        }
      };

      /**
       * @brief Residual wrapper partial template specialization for BusResidual21 (branch member function)
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::BusResidual21, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
              __enzyme_fwddiff<void>((void*) ModelWrapper<ModelT, function, ScalarT>::eval,
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[out] h - Bus residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* h)
        {
          model->evaluateBusResidual21(y, yp, wb, h);
        }
      };

      /**
       * @brief Residual wrapper partial template specialization for BusResidual22 (branch member function)
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::BusResidual22, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[out] h - Bus residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* h)
        {
          model->evaluateBusResidual22(y, yp, wb, h);
        }
      };

      /**
       * @brief Residual wrapper partial template specialization for BusResidual11WithBusDerivative
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::BusResidual11WithBusDerivative, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[in] wbp - Bus variable derivatives
         * @param[out] h - Bus residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* wbp, ScalarT* h)
        {
          model->evaluateBusResidual11(y, yp, wb, wbp, h);
        }
      };

      /**
       * @brief Residual wrapper partial template specialization for BusResidual12WithBusDerivative
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::BusResidual12WithBusDerivative, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[in] wbp - Bus variable derivatives
         * @param[out] h - Bus residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* wbp, ScalarT* h)
        {
          model->evaluateBusResidual12(y, yp, wb, wbp, h);
        }
      };

      /**
       * @brief Residual wrapper partial template specialization for BusResidual21WithBusDerivative
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::BusResidual21WithBusDerivative, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[in] wbp - Bus variable derivatives
         * @param[out] h - Bus residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* wbp, ScalarT* h)
        {
          model->evaluateBusResidual21(y, yp, wb, wbp, h);
        }
      };

      /**
       * @brief Residual wrapper partial template specialization for BusResidual22WithBusDerivative
       *
       */
      template <typename ModelT, typename ScalarT>
      struct ModelWrapper<ModelT, MemberFunctions::BusResidual22WithBusDerivative, ScalarT>
      {
        /**
         * @param[in] model - Pointer to the model to be differentiated
         * @param[in] y - Internal variables
         * @param[in] yp - Internal variable derivatives
         * @param[in] wb - Bus variables
         * @param[in] wbp - Bus variable derivatives
         * @param[out] h - Bus residual
         */
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* wbp, ScalarT* h)
        {
          model->evaluateBusResidual22(y, yp, wb, wbp, h);
        }
      };
    } // namespace Sparse
  } // namespace Enzyme
} // namespace GridKit
