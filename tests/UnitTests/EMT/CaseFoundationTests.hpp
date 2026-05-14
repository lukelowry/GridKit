#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Case.hpp>
#include <GridKit/Model/EMT/IO/JsonSupport.hpp>
#include <GridKit/Model/EMT/IO/ParamReader.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    namespace Detail
    {
      template <class Tuple, std::size_t... Is>
      consteval bool allDescribable(std::index_sequence<Is...>)
      {
        return (EMT::Describable<std::tuple_element_t<Is, Tuple>> && ...);
      }

      template <class Tuple>
      consteval bool allDescribable()
      {
        return allDescribable<Tuple>(std::make_index_sequence<std::tuple_size_v<Tuple>>{});
      }
    } // namespace Detail

    template <class RealT, typename IdxT>
    class EMTCaseFoundationTests
    {
    public:
      using Json        = nlohmann::json;
      using Load        = EMT::LoadRL<RealT, IdxT>;
      using LoadData    = EMT::LoadRLData<RealT, IdxT>;
      using Source      = EMT::VoltageSource<RealT, IdxT>;
      using SourceData  = EMT::VoltageSourceData<RealT, IdxT>;
      using Branch      = EMT::BranchLumpedConstant<RealT, IdxT>;
      using BranchData  = EMT::BranchLumpedConstantData<RealT, IdxT>;
      using Breaker     = EMT::Breaker<RealT, IdxT>;
      using BreakerData = EMT::BreakerData<RealT, IdxT>;
      using Data        = EMT::CaseData<RealT, IdxT>;
      using PhaseMask   = GridKit::Model::Events::PhaseMask;

      static_assert(EMT::Describable<Load>);
      static_assert(EMT::Describable<Source>);
      static_assert(EMT::Describable<Branch>);
      static_assert(EMT::Describable<Breaker>);
      static_assert(Detail::allDescribable<typename Data::component_types>());

      TestOutcome jsonSupport()
      {
        TestStatus success = true;

        const Json obj{{"scalar", RealT{4.5}},
                       {"vector", vectorJson(RealT{1.0}, RealT{2.0}, RealT{3.0})},
                       {"matrix", diagonalMatrixJson(RealT{0.5})},
                       {"phases", "ac"}};

        const auto scalar = EMT::require<RealT>(obj, "scalar", "root");
        const auto vector = EMT::require<EMT::PhaseVector<RealT>>(obj, "vector", "root");
        const auto matrix = EMT::require<EMT::PhaseMatrix<RealT>>(obj, "matrix", "root");
        const auto mask   = EMT::require<PhaseMask>(obj, "phases", "root");

        success *= isEqual(scalar, RealT{4.5});
        success *= isEqual(vector[0], RealT{1.0});
        success *= isEqual(vector[2], RealT{3.0});
        success *= isEqual(matrix[0][0], RealT{0.5});
        success *= isEqual(matrix[1][1], RealT{0.5});
        success *= mask.includes(0);
        success *= !mask.includes(1);
        success *= mask.includes(2);

        success *= doesNotThrowCaseError(
            [&]()
            {
              EMT::rejectUnknownKeys(obj, {"scalar", "vector", "matrix", "phases"}, "root");
            });
        success *= caseErrorContains(
            [&]()
            {
              EMT::rejectUnknownKeys(obj, {"scalar", "vector"}, "root");
            },
            "unknown key 'matrix'");
        success *= caseErrorContains(
            [&]()
            {
              (void) EMT::require<RealT>(obj, "missing", "root");
            },
            "root.missing");
        success *= caseErrorContains(
            [&]()
            {
              const Json bad{{"vector", Json::array({1.0, 2.0})}};
              (void) EMT::require<EMT::PhaseVector<RealT>>(bad, "vector", "root");
            },
            "exactly 3");
        success *= caseErrorContains(
            [&]()
            {
              const Json bad{{"matrix", Json::array({Json::array({1.0, 2.0, 3.0})})}};
              (void) EMT::require<EMT::PhaseMatrix<RealT>>(bad, "matrix", "root");
            },
            "exactly 3 rows");
        success *= caseErrorContains(
            [&]()
            {
              const Json bad{{"scalar", "not-a-number"}};
              (void) EMT::require<RealT>(bad, "scalar", "root");
            },
            "root.scalar");

        return success.report(__func__);
      }

      TestOutcome phaseMaskReader()
      {
        TestStatus success = true;

        const auto a   = EMT::require<PhaseMask>(Json{{"phases", "a"}}, "phases", "root");
        const auto ab  = EMT::require<PhaseMask>(Json{{"phases", "ab"}}, "phases", "root");
        const auto abc = EMT::require<PhaseMask>(Json{{"phases", "abc"}}, "phases", "root");
        const auto ac  = EMT::require<PhaseMask>(Json{{"phases", Json::array({"a", "c"})}}, "phases", "root");

        success *= a.includes(0);
        success *= !a.includes(1);
        success *= ab.includes(0);
        success *= ab.includes(1);
        success *= !ab.includes(2);
        success *= abc.includes(0);
        success *= abc.includes(1);
        success *= abc.includes(2);
        success *= ac.includes(0);
        success *= !ac.includes(1);
        success *= ac.includes(2);

        success *= caseErrorContains(
            [&]()
            {
              (void) EMT::require<PhaseMask>(Json{{"phases", ""}}, "phases", "root");
            },
            "must not be empty");
        success *= caseErrorContains(
            [&]()
            {
              (void) EMT::require<PhaseMask>(Json{{"phases", "ad"}}, "phases", "root");
            },
            "only 'a', 'b', and 'c'");
        success *= caseErrorContains(
            [&]()
            {
              (void) EMT::require<PhaseMask>(Json{{"phases", "aa"}}, "phases", "root");
            },
            "duplicate");
        success *= caseErrorContains(
            [&]()
            {
              (void) EMT::require<PhaseMask>(Json{{"phases", Json::array({"a", "a"})}}, "phases", "root");
            },
            "duplicate");
        success *= caseErrorContains(
            [&]()
            {
              (void) EMT::require<PhaseMask>(Json{{"phases", Json::array({"ab"})}}, "phases", "root");
            },
            "single phases");

        return success.report(__func__);
      }

      TestOutcome paramReader()
      {
        TestStatus success = true;

        const Json branchParams{{"r", diagonalMatrixJson(RealT{0.10})},
                                {"l", diagonalMatrixJson(RealT{1.0e-3})},
                                {"length", RealT{2.0}}};
        const auto branch = EMT::parseStruct<BranchData>(branchParams,
                                                         EMT::ComponentDescriptor<Branch>::params,
                                                         "component 'line': params");

        success *= isEqual(branch.r[0][0], RealT{0.10});
        success *= isEqual(branch.l[2][2], RealT{1.0e-3});
        success *= isEqual(branch.length, RealT{2.0});
        success *= isEqual(branch.g[0][0], RealT{0.0});
        success *= isEqual(branch.c[2][2], RealT{0.0});

        const Json  sourceParams{{"e", vectorJson(RealT{120.0}, RealT{121.0}, RealT{122.0})},
                                 {"phi", vectorJson(RealT{0.0}, RealT{-2.0}, RealT{2.0})},
                                 {"r", vectorJson(RealT{0.1}, RealT{0.1}, RealT{0.1})},
                                 {"frequency", RealT{60.0}}};
        const auto  source  = EMT::parseStruct<SourceData>(sourceParams,
                                                         EMT::ComponentDescriptor<Source>::params,
                                                         "component 'source': params");
        const RealT pi      = std::acos(RealT{-1.0});
        success            *= isEqual(source.omega0, RealT{2.0} * pi * RealT{60.0}, RealT{1.0e-12});

        const auto default_breaker  = EMT::parseStruct<BreakerData>(Json::object(),
                                                                   EMT::ComponentDescriptor<Breaker>::params,
                                                                   "component 'breaker': params");
        success                    *= (default_breaker.closed == PhaseMask::abc());

        const Json breakerParams{{"closed", "ac"}};
        const auto breaker  = EMT::parseStruct<BreakerData>(breakerParams,
                                                           EMT::ComponentDescriptor<Breaker>::params,
                                                           "component 'breaker': params");
        success            *= breaker.closed.includes(0);
        success            *= !breaker.closed.includes(1);
        success            *= breaker.closed.includes(2);

        Json unknown      = branchParams;
        unknown["bogus"]  = RealT{1.0};
        success          *= caseErrorContains(
            [&]()
            {
              (void) EMT::parseStruct<BranchData>(unknown,
                                                  EMT::ComponentDescriptor<Branch>::params,
                                                  "component 'line': params");
            },
            "unknown key 'bogus'");

        Json missing = branchParams;
        missing.erase("length");
        success *= caseErrorContains(
            [&]()
            {
              (void) EMT::parseStruct<BranchData>(missing,
                                                  EMT::ComponentDescriptor<Branch>::params,
                                                  "component 'line': params");
            },
            "length");

        return success.report(__func__);
      }

      TestOutcome descriptorCoverage()
      {
        TestStatus success = true;

        success *= (EMT::ComponentDescriptor<Load>::class_name == "LoadRL");
        success *= (EMT::ComponentDescriptor<Load>::terminals.size() == 1u);
        success *= (EMT::ComponentDescriptor<Load>::terminals[0] == "ac");
        success *= (EMT::ComponentDescriptor<Source>::class_name == "VoltageSource");
        success *= (EMT::ComponentDescriptor<Source>::terminals[0] == "ac");
        success *= (EMT::ComponentDescriptor<Branch>::class_name == "BranchLumpedConstant");
        success *= (EMT::ComponentDescriptor<Branch>::terminals[0] == "from");
        success *= (EMT::ComponentDescriptor<Branch>::terminals[1] == "to");
        success *= EMT::ComponentDescriptor<Branch>::inputs.empty();
        success *= EMT::ComponentDescriptor<Branch>::outputs.empty();
        success *= (EMT::ComponentDescriptor<Breaker>::class_name == "Breaker");
        success *= (EMT::ComponentDescriptor<Breaker>::terminals[0] == "from");
        success *= (EMT::ComponentDescriptor<Breaker>::terminals[1] == "to");
        success *= EMT::ComponentDescriptor<Breaker>::inputs.empty();
        success *= EMT::ComponentDescriptor<Breaker>::outputs.empty();

        return success.report(__func__);
      }

      TestOutcome monitorResolution()
      {
        TestStatus success = true;

        const auto va  = EMT::resolveBusMonitorVariable("va");
        const auto dc  = EMT::resolveBusMonitorVariable("dvc");
        success       *= va.has_value();
        success       *= (*va == EMT::BusMonitorVariable::va);
        success       *= dc.has_value();
        success       *= (*dc == EMT::BusMonitorVariable::dvc);
        success       *= !EMT::resolveBusMonitorVariable("vx").has_value();

        const auto loadIa  = EMT::ComponentMonitorTraits<Load>::resolve("ia");
        const auto loadDi  = EMT::ComponentMonitorTraits<Load>::resolve("dic");
        success           *= loadIa.has_value();
        success           *= (*loadIa == static_cast<std::size_t>(EMT::LoadRLMonitorVariable::ia));
        success           *= loadDi.has_value();
        success           *= (*loadDi == static_cast<std::size_t>(EMT::LoadRLMonitorVariable::dic));
        success           *= !EMT::ComponentMonitorTraits<Load>::resolve("v").has_value();

        const auto branchIb  = EMT::ComponentMonitorTraits<Branch>::resolve("ib");
        success             *= branchIb.has_value();
        success             *= (*branchIb == static_cast<std::size_t>(EMT::BranchLumpedConstantMonitorVariable::ib));

        const auto sourceIc  = EMT::ComponentMonitorTraits<Source>::resolve("ic");
        success             *= sourceIc.has_value();
        success             *= (*sourceIc == static_cast<std::size_t>(EMT::VoltageSourceMonitorVariable::ic));
        success             *= !EMT::ComponentMonitorTraits<Source>::resolve("dia").has_value();

        const auto breakerDia  = EMT::ComponentMonitorTraits<Breaker>::resolve("dia");
        success               *= breakerDia.has_value();
        success               *= (*breakerDia == static_cast<std::size_t>(EMT::BreakerMonitorVariable::dia));
        success               *= !EMT::ComponentMonitorTraits<Breaker>::resolve("v").has_value();

        return success.report(__func__);
      }

      TestOutcome constructorRethrowing()
      {
        TestStatus success = true;

        const Json badLoad{{"r", vectorJson(RealT{-1.0}, RealT{1.0}, RealT{1.0})},
                           {"l", vectorJson(RealT{0.1}, RealT{0.1}, RealT{0.1})}};
        success *= caseErrorContains(
            [&]()
            {
              (void) EMT::Detail::constructFromParams<Load>(badLoad, "component 'load'");
            },
            "component 'load'",
            "resistance");

        const Json badSource{{"e", vectorJson(RealT{120.0}, RealT{120.0}, RealT{120.0})},
                             {"phi", vectorJson(RealT{0.0}, RealT{0.0}, RealT{0.0})},
                             {"r", vectorJson(RealT{0.1}, RealT{0.0}, RealT{0.1})},
                             {"frequency", RealT{60.0}}};
        success *= caseErrorContains(
            [&]()
            {
              (void) EMT::Detail::constructFromParams<Source>(badSource, "component 'source'");
            },
            "component 'source'",
            "terminal resistance");

        return success.report(__func__);
      }

    private:
      static Json vectorJson(RealT a, RealT b, RealT c)
      {
        return Json::array({a, b, c});
      }

      static Json diagonalMatrixJson(RealT value)
      {
        return Json::array({Json::array({value, RealT{0.0}, RealT{0.0}}),
                            Json::array({RealT{0.0}, value, RealT{0.0}}),
                            Json::array({RealT{0.0}, RealT{0.0}, value})});
      }

      template <class Fn>
      static bool doesNotThrowCaseError(Fn&& fn)
      {
        try
        {
          fn();
          return true;
        }
        catch (const EMT::CaseError&)
        {
          return false;
        }
      }

      template <class Fn>
      static bool caseErrorContains(Fn&& fn, std::string_view text)
      {
        return caseErrorContains(std::forward<Fn>(fn), text, {});
      }

      template <class Fn>
      static bool caseErrorContains(Fn&& fn, std::string_view first, std::string_view second)
      {
        try
        {
          fn();
        }
        catch (const EMT::CaseError& ex)
        {
          const std::string message = ex.what();
          return message.find(first) != std::string::npos
                 && (second.empty() || message.find(second) != std::string::npos);
        }
        catch (...)
        {
          return false;
        }
        return false;
      }
    };
  } // namespace Testing
} // namespace GridKit
