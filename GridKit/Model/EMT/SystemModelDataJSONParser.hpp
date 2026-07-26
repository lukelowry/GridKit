#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/ABCUtils.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Model/PhasorDynamics/SignalSource/ConstantSignalSourceData.hpp>

namespace GridKit::EMT
{
  using json = nlohmann::json;

  namespace JSONDetail
  {
    inline void validateKeys(const json&                  object,
                             const std::set<std::string>& allowed,
                             const std::set<std::string>& required,
                             const std::string&           context)
    {
      if (!object.is_object())
      {
        throw std::runtime_error(context + " must be a JSON object");
      }
      for (const auto& [key, value] : object.items())
      {
        static_cast<void>(value);
        if (!allowed.contains(key))
        {
          throw std::runtime_error("Unknown " + context + " key: " + key);
        }
      }
      for (const auto& key : required)
      {
        if (!object.contains(key))
        {
          throw std::runtime_error("Missing " + context + " key: " + key);
        }
      }
    }

    template <typename RealT>
    RealT realValue(const json& value, const std::string& context)
    {
      if (!value.is_number())
      {
        throw std::runtime_error(context + " must be numeric");
      }
      const auto result = value.template get<RealT>();
      if (!std::isfinite(result))
      {
        throw std::runtime_error(context + " must be finite");
      }
      return result;
    }

    inline unsigned short unsignedShortValue(const json&        value,
                                             const std::string& context)
    {
      const auto result = realValue<double>(value, context);
      if (result < 0.0
          || result > static_cast<double>(
                 std::numeric_limits<unsigned short>::max())
          || std::floor(result) != result)
      {
        throw std::runtime_error(
            context + " must be a nonnegative integer representable as unsigned short");
      }
      return static_cast<unsigned short>(result);
    }

    template <typename RealT>
    ABCVector<RealT> realVector(const json&        value,
                                const std::string& context)
    {
      if (!value.is_array() || value.size() != 3)
      {
        throw std::runtime_error(context + " must contain exactly 3 values");
      }
      return {realValue<RealT>(value[0], context),
              realValue<RealT>(value[1], context),
              realValue<RealT>(value[2], context)};
    }

    template <typename RealT>
    ABCMatrix<RealT> realMatrix(const json&        value,
                                const std::string& context)
    {
      if (!value.is_array() || value.size() != 3)
      {
        throw std::runtime_error(context + " must contain exactly 3 rows");
      }
      return {realVector<RealT>(value[0], context + " row a"),
              realVector<RealT>(value[1], context + " row b"),
              realVector<RealT>(value[2], context + " row c")};
    }

    template <typename RealT>
    std::complex<RealT> complexValue(const json&        value,
                                     const std::string& context)
    {
      if (!value.is_array() || value.size() != 2)
      {
        throw std::runtime_error(context + " must be [real,imag]");
      }
      return {realValue<RealT>(value[0], context + " real"),
              realValue<RealT>(value[1], context + " imag")};
    }

    template <typename RealT>
    ABCMatrix<std::complex<RealT>> complexMatrix(
        const json&        value,
        const std::string& context)
    {
      if (!value.is_array() || value.size() != 3)
      {
        throw std::runtime_error(context + " must contain exactly 3 rows");
      }
      ABCMatrix<std::complex<RealT>> result{};
      for (std::size_t row = 0; row < 3; ++row)
      {
        if (!value[row].is_array() || value[row].size() != 3)
        {
          throw std::runtime_error(context + " rows must contain 3 entries");
        }
        for (std::size_t column = 0; column < 3; ++column)
        {
          result[row][column] = complexValue<RealT>(
              value[row][column], context + " entry");
        }
      }
      return result;
    }

    inline std::string lower(std::string value)
    {
      std::ranges::transform(value, value.begin(), [](unsigned char c)
                             { return static_cast<char>(std::tolower(c)); });
      return value;
    }

    inline void validateDeviceEnvelope(const json&        device,
                                       const std::string& expected_class)
    {
      validateKeys(device,
                   {"class", "id", "params", "ports", "mon"},
                   {"class", "id", "params", "ports"},
                   expected_class + " device");
      if (device.at("class").get<std::string>() != expected_class)
      {
        throw std::runtime_error("Unexpected device class");
      }
      if (!device.at("id").is_string()
          || device.at("id").get<std::string>().empty())
      {
        throw std::runtime_error(expected_class + " id must be nonempty");
      }
    }

    template <typename DataT>
    void populateEnvelope(const json&        device,
                          const std::string& expected_class,
                          DataT&             data)
    {
      validateDeviceEnvelope(device, expected_class);
      data.device_class          = expected_class;
      data.disambiguation_string = device.at("id").get<std::string>();
    }

    inline std::vector<std::string> monitorNames(const json& device)
    {
      std::vector<std::string> result;
      if (!device.contains("mon"))
      {
        return result;
      }
      if (!device.at("mon").is_array())
      {
        throw std::runtime_error("mon must be an array");
      }
      for (const auto& monitor : device.at("mon"))
      {
        if (!monitor.is_string())
        {
          throw std::runtime_error("monitor names must be strings");
        }
        result.push_back(monitor.get<std::string>());
      }
      return result;
    }

    template <typename RealT>
    bool approximatelyEqual(RealT lhs, RealT rhs)
    {
      const RealT scale = std::max({RealT{1.0}, std::abs(lhs), std::abs(rhs)});
      return std::abs(lhs - rhs)
             <= RealT{128.0} * std::numeric_limits<RealT>::epsilon()
                    * scale;
    }

    template <typename RealT>
    bool approximatelyConjugate(const std::complex<RealT>& lhs,
                                const std::complex<RealT>& rhs)
    {
      return approximatelyEqual(lhs.real(), rhs.real())
             && approximatelyEqual(lhs.imag(), -rhs.imag());
    }

    template <typename RealT>
    bool realMatrix(const ABCMatrix<std::complex<RealT>>& matrix)
    {
      return approximatelyEqual(matrix[0][0].imag(), RealT{0.0})
             && approximatelyEqual(matrix[0][1].imag(), RealT{0.0})
             && approximatelyEqual(matrix[0][2].imag(), RealT{0.0})
             && approximatelyEqual(matrix[1][0].imag(), RealT{0.0})
             && approximatelyEqual(matrix[1][1].imag(), RealT{0.0})
             && approximatelyEqual(matrix[1][2].imag(), RealT{0.0})
             && approximatelyEqual(matrix[2][0].imag(), RealT{0.0})
             && approximatelyEqual(matrix[2][1].imag(), RealT{0.0})
             && approximatelyEqual(matrix[2][2].imag(), RealT{0.0});
    }

    template <typename RealT>
    bool conjugateMatrices(
        const ABCMatrix<std::complex<RealT>>& lhs,
        const ABCMatrix<std::complex<RealT>>& rhs)
    {
      return approximatelyConjugate(lhs[0][0], rhs[0][0])
             && approximatelyConjugate(lhs[0][1], rhs[0][1])
             && approximatelyConjugate(lhs[0][2], rhs[0][2])
             && approximatelyConjugate(lhs[1][0], rhs[1][0])
             && approximatelyConjugate(lhs[1][1], rhs[1][1])
             && approximatelyConjugate(lhs[1][2], rhs[1][2])
             && approximatelyConjugate(lhs[2][0], rhs[2][0])
             && approximatelyConjugate(lhs[2][1], rhs[2][1])
             && approximatelyConjugate(lhs[2][2], rhs[2][2]);
    }

    template <typename RealT>
    void validateVectorFitPairs(
        const std::vector<std::complex<RealT>>&            poles,
        const std::vector<ABCMatrix<std::complex<RealT>>>& residues)
    {
      if (poles.size() != residues.size())
      {
        throw std::runtime_error(
            "VectorFit requires one residue matrix per pole");
      }

      for (std::size_t q = 0; q < poles.size(); ++q)
      {
        if (approximatelyEqual(poles[q].imag(), RealT{0.0}))
        {
          if (!realMatrix(residues[q]))
          {
            throw std::runtime_error(
                "VectorFit real poles require real residues");
          }
          continue;
        }

        if (q + 1 >= poles.size()
            || !approximatelyConjugate(poles[q], poles[q + 1])
            || !conjugateMatrices(residues[q], residues[q + 1]))
        {
          throw std::runtime_error(
              "VectorFit nonreal poles and residues must be adjacent conjugates");
        }
        ++q;
      }
    }
  } // namespace JSONDetail

  template <typename RealT, typename IdxT>
  void from_json(const json& input, SystemModelData<RealT, IdxT>& model)
  {
    using namespace JSONDetail;

    validateKeys(input,
                 {"header", "monitors", "buses", "signals", "devices"},
                 {"header", "buses", "signals", "devices"},
                 "EMT case");

    if (!input.at("buses").is_array()
        || !input.at("devices").is_array())
    {
      throw std::runtime_error("EMT buses and devices must be arrays");
    }

    const auto& header = input.at("header");
    validateKeys(header,
                 {"format_version", "format_revision", "case_name", "case_description", "case_comments"},
                 {"case_name"},
                 "EMT header");
    model.case_name        = header.at("case_name").get<std::string>();
    model.case_description = header.value("case_description", std::string{});
    model.case_comments    = header.value("case_comments", std::string{});
    if (header.contains("format_version"))
    {
      model.format_version = unsignedShortValue(
          header.at("format_version"), "format_version");
    }
    if (header.contains("format_revision"))
    {
      model.format_revision = unsignedShortValue(
          header.at("format_revision"), "format_revision");
    }

    if (input.contains("monitors"))
    {
      if (!input.at("monitors").is_array())
      {
        throw std::runtime_error("EMT monitors must be an array");
      }
      for (const auto& monitor : input.at("monitors"))
      {
        validateKeys(monitor,
                     {"file_name", "format", "delim"},
                     {"format"},
                     "monitor sink");
        const auto                   format = lower(monitor.at("format").get<std::string>());
        Model::VariableMonitorFormat parsed_format;
        if (format == "csv")
        {
          parsed_format = Model::VariableMonitorFormat::CSV;
        }
        else if (format == "json")
        {
          parsed_format = Model::VariableMonitorFormat::JSON;
        }
        else if (format == "yaml")
        {
          parsed_format = Model::VariableMonitorFormat::YAML;
        }
        else
        {
          throw std::runtime_error("Unknown monitor format: " + format);
        }
        model.monitor_sink.push_back({parsed_format,
                                      monitor.value("file_name", std::string{}),
                                      monitor.value("delim", std::string(","))});
      }
    }

    std::set<IdxT> bus_ids;
    for (const auto& raw_bus : input.at("buses"))
    {
      validateKeys(raw_bus,
                   {"number", "class", "name", "mon"},
                   {"number", "class", "name"},
                   "bus");
      if (raw_bus.at("class").get<std::string>() != "bus")
      {
        throw std::runtime_error("EMT buses must use class bus");
      }
      typename SystemModelData<RealT, IdxT>::BusDataT bus;
      raw_bus.at("number").get_to(bus.bus_id);
      raw_bus.at("name").get_to(bus.name);
      if (!bus_ids.insert(bus.bus_id).second)
      {
        throw std::runtime_error("Duplicate EMT bus id");
      }
      if (raw_bus.contains("mon"))
      {
        if (!raw_bus.at("mon").is_array())
        {
          throw std::runtime_error("bus mon must be an array");
        }
        for (const auto& raw_monitor : raw_bus.at("mon"))
        {
          const auto monitor = raw_monitor.get<std::string>();
          if (monitor == "v")
          {
            bus.monitored_variables.insert(BusMonitorableVariables::va);
            bus.monitored_variables.insert(BusMonitorableVariables::vb);
            bus.monitored_variables.insert(BusMonitorableVariables::vc);
          }
          else if (monitor == "va")
          {
            bus.monitored_variables.insert(BusMonitorableVariables::va);
          }
          else if (monitor == "vb")
          {
            bus.monitored_variables.insert(BusMonitorableVariables::vb);
          }
          else if (monitor == "vc")
          {
            bus.monitored_variables.insert(BusMonitorableVariables::vc);
          }
          else
          {
            throw std::runtime_error("Unknown bus monitor: " + monitor);
          }
        }
      }
      model.bus.push_back(bus);
    }

    std::set<IdxT> signal_ids;
    if (input.contains("signals"))
    {
      if (!input.at("signals").is_array())
      {
        throw std::runtime_error("EMT signals must be an array");
      }
      for (const auto& raw_signal : input.at("signals"))
      {
        validateKeys(raw_signal,
                     {"signal_id", "name"},
                     {"signal_id", "name"},
                     "signal");
        typename SystemModelData<RealT, IdxT>::SignalDataT signal;
        raw_signal.at("signal_id").get_to(signal.signal_id);
        raw_signal.at("name").get_to(signal.name);
        if (!signal_ids.insert(signal.signal_id).second)
        {
          throw std::runtime_error("Duplicate EMT signal id");
        }
        model.signal.push_back(signal);
      }
    }

    std::set<std::string> device_ids;
    for (const auto& device : input.at("devices"))
    {
      const auto kind = device.at("class").get<std::string>();
      const auto id   = device.at("id").get<std::string>();
      if (!device_ids.insert(id).second)
      {
        throw std::runtime_error("Duplicate EMT device id: " + id);
      }

      const auto& params = device.at("params");
      const auto& ports  = device.at("ports");

      if (kind == "LineLumped")
      {
        typename SystemModelData<RealT, IdxT>::LineLumpedDataT data;
        populateEnvelope(device, kind, data);
        validateKeys(params, {"dx", "Rp", "Lp", "Gp", "Cp"}, {"dx", "Rp", "Lp", "Gp", "Cp"}, kind + " params");
        validateKeys(ports, {"bus1", "bus2"}, {"bus1", "bus2"}, kind + " ports");
        const auto dx = realValue<RealT>(params.at("dx"),
                                         "LineLumped dx");
        const auto Rp = realMatrix<RealT>(params.at("Rp"),
                                          "LineLumped Rp");
        const auto Lp = realMatrix<RealT>(params.at("Lp"),
                                          "LineLumped Lp");
        const auto Gp = realMatrix<RealT>(params.at("Gp"),
                                          "LineLumped Gp");
        const auto Cp = realMatrix<RealT>(params.at("Cp"),
                                          "LineLumped Cp");
        if (!(dx > RealT{0.0}) || !Detail::positiveSemidefinite(Rp)
            || !Detail::positiveDefinite(Lp)
            || !Detail::positiveSemidefinite(Gp)
            || !Detail::positiveSemidefinite(Cp))
        {
          throw std::runtime_error(
              "LineLumped requires positive dx, positive-definite Lp, "
              "and positive-semidefinite Rp, Gp, and Cp");
        }
        data.parameters[LineLumpedParameters::dx] = dx;
        data.parameters[LineLumpedParameters::Rp] = Rp;
        data.parameters[LineLumpedParameters::Lp] = Lp;
        data.parameters[LineLumpedParameters::Gp] = Gp;
        data.parameters[LineLumpedParameters::Cp] = Cp;
        ports.at("bus1").get_to(data.buses[LineLumpedBuses::bus1]);
        ports.at("bus2").get_to(data.buses[LineLumpedBuses::bus2]);
        for (const auto& monitor : monitorNames(device))
        {
          if (monitor == "i12")
          {
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i12a);
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i12b);
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i12c);
          }
          else if (monitor == "i_sh1")
          {
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i_sh1a);
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i_sh1b);
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i_sh1c);
          }
          else if (monitor == "i_sh2")
          {
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i_sh2a);
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i_sh2b);
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i_sh2c);
          }
          else if (monitor == "i12a")
          {
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i12a);
          }
          else if (monitor == "i12b")
          {
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i12b);
          }
          else if (monitor == "i12c")
          {
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i12c);
          }
          else if (monitor == "i_sh1a")
          {
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i_sh1a);
          }
          else if (monitor == "i_sh1b")
          {
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i_sh1b);
          }
          else if (monitor == "i_sh1c")
          {
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i_sh1c);
          }
          else if (monitor == "i_sh2a")
          {
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i_sh2a);
          }
          else if (monitor == "i_sh2b")
          {
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i_sh2b);
          }
          else if (monitor == "i_sh2c")
          {
            data.monitored_variables.insert(LineLumpedMonitorableVariables::i_sh2c);
          }
          else
          {
            throw std::runtime_error("Unknown LineLumped monitor: " + monitor);
          }
        }
        model.line_lumped.push_back(data);
      }
      else if (kind == "LoadZ")
      {
        typename SystemModelData<RealT, IdxT>::LoadZDataT data;
        populateEnvelope(device, kind, data);
        validateKeys(params, {"R", "L"}, {"R", "L"}, kind + " params");
        validateKeys(ports, {"bus", "enable"}, {"bus", "enable"}, kind + " ports");
        const auto R = realMatrix<RealT>(params.at("R"), "LoadZ R");
        const auto L = realMatrix<RealT>(params.at("L"), "LoadZ L");
        if (!Detail::positiveSemidefinite(R)
            || !Detail::positiveDefinite(L))
        {
          throw std::runtime_error(
              "LoadZ requires positive-semidefinite R and positive-definite L");
        }
        data.parameters[LoadZParameters::R] = R;
        data.parameters[LoadZParameters::L] = L;
        ports.at("bus").get_to(data.buses[LoadZBuses::bus]);
        ports.at("enable").get_to(
            data.signal_inputs[LoadZSignalInputs::enable]);
        for (const auto& monitor : monitorNames(device))
        {
          if (monitor == "i")
          {
            data.monitored_variables.insert(LoadZMonitorableVariables::ia);
            data.monitored_variables.insert(LoadZMonitorableVariables::ib);
            data.monitored_variables.insert(LoadZMonitorableVariables::ic);
          }
          else if (monitor == "ia")
          {
            data.monitored_variables.insert(LoadZMonitorableVariables::ia);
          }
          else if (monitor == "ib")
          {
            data.monitored_variables.insert(LoadZMonitorableVariables::ib);
          }
          else if (monitor == "ic")
          {
            data.monitored_variables.insert(LoadZMonitorableVariables::ic);
          }
          else
          {
            throw std::runtime_error("Unknown LoadZ monitor: " + monitor);
          }
        }
        model.loadz.push_back(data);
      }
      else if (kind == "VoltageSource")
      {
        typename SystemModelData<RealT, IdxT>::VoltageSourceDataT data;
        populateEnvelope(device, kind, data);
        validateKeys(params, {"E", "phi", "omega", "Rs", "Ls"}, {"E", "phi", "omega", "Rs", "Ls"}, kind + " params");
        validateKeys(ports, {"bus"}, {"bus"}, kind + " ports");
        const auto E     = realVector<RealT>(params.at("E"),
                                         "VoltageSource E");
        const auto phi   = realVector<RealT>(params.at("phi"),
                                           "VoltageSource phi");
        const auto omega = realValue<RealT>(params.at("omega"),
                                            "VoltageSource omega");
        const auto Rs    = realMatrix<RealT>(params.at("Rs"),
                                          "VoltageSource Rs");
        const auto Ls    = realMatrix<RealT>(params.at("Ls"),
                                          "VoltageSource Ls");
        if (E[0] < RealT{0.0} || E[1] < RealT{0.0}
            || E[2] < RealT{0.0} || !(omega > RealT{0.0})
            || !Detail::positiveSemidefinite(Rs)
            || !Detail::positiveDefinite(Ls))
        {
          throw std::runtime_error(
              "VoltageSource requires nonnegative E, positive omega, "
              "positive-semidefinite Rs, and positive-definite Ls");
        }
        data.parameters[VoltageSourceParameters::E]     = E;
        data.parameters[VoltageSourceParameters::phi]   = phi;
        data.parameters[VoltageSourceParameters::omega] = omega;
        data.parameters[VoltageSourceParameters::Rs]    = Rs;
        data.parameters[VoltageSourceParameters::Ls]    = Ls;
        ports.at("bus").get_to(data.buses[VoltageSourceBuses::bus]);
        for (const auto& monitor : monitorNames(device))
        {
          if (monitor == "e")
          {
            data.monitored_variables.insert(VoltageSourceMonitorableVariables::ea);
            data.monitored_variables.insert(VoltageSourceMonitorableVariables::eb);
            data.monitored_variables.insert(VoltageSourceMonitorableVariables::ec);
          }
          else if (monitor == "i")
          {
            data.monitored_variables.insert(VoltageSourceMonitorableVariables::ia);
            data.monitored_variables.insert(VoltageSourceMonitorableVariables::ib);
            data.monitored_variables.insert(VoltageSourceMonitorableVariables::ic);
          }
          else if (monitor == "ea")
          {
            data.monitored_variables.insert(VoltageSourceMonitorableVariables::ea);
          }
          else if (monitor == "eb")
          {
            data.monitored_variables.insert(VoltageSourceMonitorableVariables::eb);
          }
          else if (monitor == "ec")
          {
            data.monitored_variables.insert(VoltageSourceMonitorableVariables::ec);
          }
          else if (monitor == "ia")
          {
            data.monitored_variables.insert(VoltageSourceMonitorableVariables::ia);
          }
          else if (monitor == "ib")
          {
            data.monitored_variables.insert(VoltageSourceMonitorableVariables::ib);
          }
          else if (monitor == "ic")
          {
            data.monitored_variables.insert(VoltageSourceMonitorableVariables::ic);
          }
          else
          {
            throw std::runtime_error("Unknown VoltageSource monitor: " + monitor);
          }
        }
        model.voltage_source.push_back(data);
      }
      else if (kind == "VectorFit")
      {
        typename SystemModelData<RealT, IdxT>::VectorFitDataT data;
        populateEnvelope(device, kind, data);
        validateKeys(params, {"D", "E", "poles", "residues"}, {"D", "E", "poles", "residues"}, kind + " params");
        validateKeys(ports,
                     {"input_a", "input_b", "input_c", "out_a", "out_b", "out_c"},
                     {"input_a", "input_b", "input_c", "out_a", "out_b", "out_c"},
                     kind + " ports");
        data.parameters[VectorFitParameters::D] =
            realMatrix<RealT>(params.at("D"), "VectorFit D");
        data.parameters[VectorFitParameters::E] =
            realMatrix<RealT>(params.at("E"), "VectorFit E");
        if (!params.at("poles").is_array()
            || !params.at("residues").is_array())
        {
          throw std::runtime_error(
              "VectorFit poles and residues must be arrays");
        }
        std::vector<std::complex<RealT>> poles;
        for (const auto& pole : params.at("poles"))
        {
          poles.push_back(complexValue<RealT>(pole, "VectorFit pole"));
        }
        std::vector<ABCMatrix<std::complex<RealT>>> residues;
        for (const auto& residue : params.at("residues"))
        {
          residues.push_back(complexMatrix<RealT>(residue,
                                                  "VectorFit residue"));
        }
        validateVectorFitPairs(poles, residues);
        data.parameters[VectorFitParameters::poles]    = poles;
        data.parameters[VectorFitParameters::residues] = residues;
        ports.at("input_a").get_to(
            data.signal_inputs[VectorFitSignalInputs::input_a]);
        ports.at("input_b").get_to(
            data.signal_inputs[VectorFitSignalInputs::input_b]);
        ports.at("input_c").get_to(
            data.signal_inputs[VectorFitSignalInputs::input_c]);
        ports.at("out_a").get_to(
            data.signal_outputs[VectorFitSignalOutputs::out_a]);
        ports.at("out_b").get_to(
            data.signal_outputs[VectorFitSignalOutputs::out_b]);
        ports.at("out_c").get_to(
            data.signal_outputs[VectorFitSignalOutputs::out_c]);
        if (!monitorNames(device).empty())
        {
          throw std::runtime_error("VectorFit has no monitors");
        }
        model.vector_fit.push_back(data);
      }
      else if (kind == "ConstantSignalSource")
      {
        using namespace PhasorDynamics;
        typename SystemModelData<RealT, IdxT>::ConstantSourceDataT data;
        validateDeviceEnvelope(device, kind);
        data.device_class          = kind;
        data.disambiguation_string = id;
        validateKeys(params, {"Sr", "Si"}, {"Sr"}, kind + " params");
        validateKeys(ports, {"sr", "si"}, {"sr"}, kind + " ports");
        if (params.contains("Si") != ports.contains("si"))
        {
          throw std::runtime_error(
              "ConstantSignalSource Si and si must be provided together");
        }
        if (params.contains("Sr"))
        {
          data.parameters[ConstantSignalSourceParameters::Sr] =
              realValue<RealT>(params.at("Sr"), "ConstantSignalSource Sr");
        }
        if (params.contains("Si"))
        {
          data.parameters[ConstantSignalSourceParameters::Si] =
              realValue<RealT>(params.at("Si"), "ConstantSignalSource Si");
        }
        if (ports.contains("sr"))
        {
          ports.at("sr").get_to(
              data.signal_outputs[ConstantSignalSourceSignalOutputs::sr]);
        }
        if (ports.contains("si"))
        {
          ports.at("si").get_to(
              data.signal_outputs[ConstantSignalSourceSignalOutputs::si]);
        }
        if (!monitorNames(device).empty())
        {
          throw std::runtime_error(
              "ConstantSignalSource has no monitors");
        }
        model.constant_source.push_back(data);
      }
      else
      {
        throw std::runtime_error("Unknown EMT device class: " + kind);
      }
    }

    const auto requireBus = [&bus_ids](IdxT               bus_id,
                                       const std::string& context)
    {
      if (!bus_ids.contains(bus_id))
      {
        throw std::runtime_error(context + " references an unknown bus");
      }
    };
    const auto requireSignal = [&signal_ids](IdxT               signal_id,
                                             const std::string& context)
    {
      if (!signal_ids.contains(signal_id))
      {
        throw std::runtime_error(context + " references an unknown signal");
      }
    };

    for (const auto& line : model.line_lumped)
    {
      const auto bus1 = line.buses.at(LineLumpedBuses::bus1);
      const auto bus2 = line.buses.at(LineLumpedBuses::bus2);
      requireBus(bus1, line.disambiguation_string);
      requireBus(bus2, line.disambiguation_string);
      if (bus1 == bus2)
      {
        throw std::runtime_error(
            line.disambiguation_string + " must connect distinct buses");
      }
    }
    for (const auto& load : model.loadz)
    {
      requireBus(load.buses.at(LoadZBuses::bus),
                 load.disambiguation_string);
      requireSignal(load.signal_inputs.at(LoadZSignalInputs::enable),
                    load.disambiguation_string);
    }
    for (const auto& source : model.voltage_source)
    {
      requireBus(source.buses.at(VoltageSourceBuses::bus),
                 source.disambiguation_string);
    }

    std::set<IdxT> signal_owners;
    for (const auto& source : model.constant_source)
    {
      for (const auto& [port, signal_id] : source.signal_outputs)
      {
        static_cast<void>(port);
        requireSignal(signal_id, source.disambiguation_string);
        if (!signal_owners.insert(signal_id).second)
        {
          throw std::runtime_error("An EMT signal has multiple owners");
        }
      }
    }
    for (const auto& vector_fit : model.vector_fit)
    {
      for (const auto& [port, signal_id] : vector_fit.signal_inputs)
      {
        static_cast<void>(port);
        requireSignal(signal_id, vector_fit.disambiguation_string);
      }
      for (const auto& [port, signal_id] : vector_fit.signal_outputs)
      {
        static_cast<void>(port);
        requireSignal(signal_id, vector_fit.disambiguation_string);
        if (!signal_owners.insert(signal_id).second)
        {
          throw std::runtime_error("An EMT signal has multiple owners");
        }
      }
    }

    for (const auto& load : model.loadz)
    {
      const auto signal_id = load.signal_inputs.at(
          LoadZSignalInputs::enable);
      if (!signal_owners.contains(signal_id))
      {
        throw std::runtime_error(
            load.disambiguation_string
            + " enable signal has no component owner");
      }
    }
    for (const auto& vector_fit : model.vector_fit)
    {
      for (const auto& [port, signal_id] : vector_fit.signal_inputs)
      {
        static_cast<void>(port);
        if (!signal_owners.contains(signal_id))
        {
          throw std::runtime_error(
              vector_fit.disambiguation_string
              + " input signal has no component owner");
        }
      }
    }
  }
} // namespace GridKit::EMT
