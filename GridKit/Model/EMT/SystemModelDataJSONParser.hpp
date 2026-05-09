/**
 * @file SystemModelDataJSONParser.hpp
 * @brief EMT adapter for the shared GridKit JSON case schema.
 */

#pragma once

#include <array>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include <GridKit/Model/Case/CaseData.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Detail
    {
      using Json     = Model::Case::Json;
      using FieldSet = std::set<std::string_view>;

      inline std::runtime_error parserError(const std::string& path, const std::string& message)
      {
        return std::runtime_error("EMT case parser: " + path + ": " + message);
      }

      inline void requireObject(const Json& input, const std::string& path)
      {
        if (!input.is_object())
        {
          throw parserError(path, "expected an object");
        }
      }

      inline void requireArray(const Json& input, const std::string& path)
      {
        if (!input.is_array())
        {
          throw parserError(path, "expected an array");
        }
      }

      inline void checkKnownFields(const Json&        input,
                                   const FieldSet&    fields,
                                   const std::string& path)
      {
        requireObject(input, path);
        for (const auto& item : input.items())
        {
          if (!fields.contains(std::string_view(item.key())))
          {
            throw parserError(path, "unknown field '" + item.key() + "'");
          }
        }
      }

      inline const Json& requireField(const Json&        input,
                                      const std::string& field,
                                      const std::string& path)
      {
        if (!input.contains(field))
        {
          throw parserError(path, "missing required field '" + field + "'");
        }
        return input.at(field);
      }

      template <typename RealT>
      RealT numberField(const Json& input, const std::string& field, const std::string& path)
      {
        return requireField(input, field, path).get<RealT>();
      }

      template <typename IdxT>
      IdxT indexField(const Json& input, const std::string& field, const std::string& path)
      {
        const auto value = requireField(input, field, path).get<long long>();
        if (value <= 0)
        {
          throw parserError(path + "." + field, "bus number must be positive");
        }
        return static_cast<IdxT>(value);
      }

      template <typename RealT>
      std::array<RealT, 3> readVector3(const Json& input, const std::string& path)
      {
        requireArray(input, path);
        if (input.size() != 3)
        {
          throw parserError(path, "expected a vector of length 3");
        }

        std::array<RealT, 3> values{};
        for (size_t i = 0; i < values.size(); ++i)
        {
          values[i] = input.at(i).get<RealT>();
        }
        return values;
      }

      template <typename RealT>
      std::vector<RealT> readVector(const Json& input, size_t dimension, const std::string& path)
      {
        requireArray(input, path);
        if (input.size() != dimension)
        {
          throw parserError(path, "expected a vector of length " + std::to_string(dimension));
        }

        std::vector<RealT> values(dimension);
        for (size_t i = 0; i < dimension; ++i)
        {
          values[i] = input.at(i).get<RealT>();
        }
        return values;
      }

      template <typename RealT>
      std::array<RealT, 9> readMatrix3(const Json& input, const std::string& path)
      {
        requireArray(input, path);
        std::array<RealT, 9> values{};

        if (input.size() == 9 && !input.at(0).is_array())
        {
          for (size_t i = 0; i < values.size(); ++i)
          {
            values[i] = input.at(i).get<RealT>();
          }
          return values;
        }

        if (input.size() != 3)
        {
          throw parserError(path, "expected a 3 x 3 matrix");
        }
        for (size_t row = 0; row < 3; ++row)
        {
          requireArray(input.at(row), path + "[" + std::to_string(row) + "]");
          if (input.at(row).size() != 3)
          {
            throw parserError(path, "expected a 3 x 3 matrix");
          }
          for (size_t col = 0; col < 3; ++col)
          {
            values[row * 3 + col] = input.at(row).at(col).get<RealT>();
          }
        }
        return values;
      }

      template <typename RealT>
      std::vector<RealT> readMatrix(const Json& input, size_t dimension, const std::string& path)
      {
        requireArray(input, path);
        std::vector<RealT> values(dimension * dimension);

        if (input.size() == dimension * dimension && !input.at(0).is_array())
        {
          for (size_t i = 0; i < values.size(); ++i)
          {
            values[i] = input.at(i).get<RealT>();
          }
          return values;
        }

        if (input.size() != dimension)
        {
          throw parserError(path, "expected a " + std::to_string(dimension) + " x " + std::to_string(dimension) + " matrix");
        }
        for (size_t row = 0; row < dimension; ++row)
        {
          requireArray(input.at(row), path + "[" + std::to_string(row) + "]");
          if (input.at(row).size() != dimension)
          {
            throw parserError(path, "expected a " + std::to_string(dimension) + " x " + std::to_string(dimension) + " matrix");
          }
          for (size_t col = 0; col < dimension; ++col)
          {
            values[row * dimension + col] = input.at(row).at(col).get<RealT>();
          }
        }
        return values;
      }

      template <typename RealT, typename IdxT>
      RationalApproxData<RealT, IdxT> readYc(const Json& input, const std::string& path)
      {
        static const FieldSet fields{"id",
                                     "dimension",
                                     "d",
                                     "e",
                                     "real_poles",
                                     "complex_pole_pairs",
                                     "extension"};
        checkKnownFields(input, fields, path);

        RationalApproxData<RealT, IdxT> data;
        data.dimension = requireField(input, "dimension", path).get<size_t>();
        if (data.dimension != 3)
        {
          throw parserError(path + ".dimension", "EMT line characteristic admittance dimension "
                                                 "must be 3");
        }

        data.d = readMatrix<RealT>(requireField(input, "d", path), data.dimension, path + ".d");
        data.e = readMatrix<RealT>(requireField(input, "e", path), data.dimension, path + ".e");

        if (input.contains("real_poles"))
        {
          const auto& poles = input.at("real_poles");
          requireArray(poles, path + ".real_poles");
          for (size_t i = 0; i < poles.size(); ++i)
          {
            const auto            pole_path = path + ".real_poles[" + std::to_string(i) + "]";
            static const FieldSet pole_fields{"pole", "b", "c", "extension"};
            checkKnownFields(poles.at(i), pole_fields, pole_path);

            const auto pole = numberField<RealT>(poles.at(i), "pole", pole_path);
            if (pole >= static_cast<RealT>(0.0))
            {
              throw parserError(pole_path + ".pole", "real pole must be negative");
            }
            data.p.push_back(pole);

            const auto b =
                readVector<RealT>(requireField(poles.at(i), "b", pole_path), data.dimension, pole_path + ".b");
            const auto c =
                readVector<RealT>(requireField(poles.at(i), "c", pole_path), data.dimension, pole_path + ".c");
            data.b.insert(data.b.end(), b.begin(), b.end());
            data.c.insert(data.c.end(), c.begin(), c.end());
          }
        }

        if (input.contains("complex_pole_pairs"))
        {
          const auto& pairs = input.at("complex_pole_pairs");
          requireArray(pairs, path + ".complex_pole_pairs");
          for (size_t i = 0; i < pairs.size(); ++i)
          {
            const auto            pair_path = path + ".complex_pole_pairs[" + std::to_string(i) + "]";
            static const FieldSet pair_fields{"pole", "b", "c", "extension"};
            static const FieldSet complex_fields{"real", "imag"};
            checkKnownFields(pairs.at(i), pair_fields, pair_path);
            checkKnownFields(requireField(pairs.at(i), "pole", pair_path), complex_fields, pair_path + ".pole");
            checkKnownFields(requireField(pairs.at(i), "b", pair_path), complex_fields, pair_path + ".b");
            checkKnownFields(requireField(pairs.at(i), "c", pair_path), complex_fields, pair_path + ".c");

            const auto p_real =
                numberField<RealT>(pairs.at(i).at("pole"), "real", pair_path + ".pole");
            const auto p_imag =
                numberField<RealT>(pairs.at(i).at("pole"), "imag", pair_path + ".pole");
            if (p_real >= static_cast<RealT>(0.0))
            {
              throw parserError(pair_path + ".pole.real", "complex pole real part must be "
                                                          "negative");
            }
            if (p_imag <= static_cast<RealT>(0.0))
            {
              throw parserError(pair_path + ".pole.imag", "complex pole imaginary part must be "
                                                          "positive");
            }

            data.complex_p_real.push_back(p_real);
            data.complex_p_imag.push_back(p_imag);

            const auto b_real =
                readVector<RealT>(requireField(pairs.at(i).at("b"), "real", pair_path + ".b"),
                                  data.dimension,
                                  pair_path + ".b.real");
            const auto b_imag =
                readVector<RealT>(requireField(pairs.at(i).at("b"), "imag", pair_path + ".b"),
                                  data.dimension,
                                  pair_path + ".b.imag");
            const auto c_real =
                readVector<RealT>(requireField(pairs.at(i).at("c"), "real", pair_path + ".c"),
                                  data.dimension,
                                  pair_path + ".c.real");
            const auto c_imag =
                readVector<RealT>(requireField(pairs.at(i).at("c"), "imag", pair_path + ".c"),
                                  data.dimension,
                                  pair_path + ".c.imag");

            data.complex_b_real.insert(data.complex_b_real.end(), b_real.begin(), b_real.end());
            data.complex_b_imag.insert(data.complex_b_imag.end(), b_imag.begin(), b_imag.end());
            data.complex_c_real.insert(data.complex_c_real.end(), c_real.begin(), c_real.end());
            data.complex_c_imag.insert(data.complex_c_imag.end(), c_imag.begin(), c_imag.end());
          }
        }

        return data;
      }

      template <typename RealT, typename IdxT>
      std::map<std::string, RationalApproxData<RealT, IdxT>>
      readYcModels(const Model::Case::CaseData& input)
      {
        std::map<std::string, RationalApproxData<RealT, IdxT>> models;
        if (!input.models.contains("Yc"))
        {
          return models;
        }

        const auto& entries = input.models.at("Yc");
        requireArray(entries, "models.Yc");
        for (size_t i = 0; i < entries.size(); ++i)
        {
          const auto path = "models.Yc[" + std::to_string(i) + "]";
          const auto id   = requireField(entries.at(i), "id", path).get<std::string>();
          models.emplace(id, readYc<RealT, IdxT>(entries.at(i), path));
        }
        return models;
      }

      template <typename IdxT>
      void requireKnownBus(const std::set<IdxT>& buses, IdxT bus, const std::string& path)
      {
        if (!buses.contains(bus))
        {
          throw parserError(path, "unknown bus number " + std::to_string(bus));
        }
      }

      template <typename RealT, typename IdxT>
      typename SystemModelData<RealT, IdxT>::BusSpec
      readBus(const Model::Case::ComponentData& input)
      {
        if (input.component_class != "emt_bus")
        {
          throw parserError("buses", "unsupported bus class '" + input.component_class + "'");
        }

        typename SystemModelData<RealT, IdxT>::BusSpec bus;
        if (input.number <= 0)
        {
          throw parserError("buses", "bus number must be positive");
        }
        bus.number     = static_cast<IdxT>(input.number);
        bus.data.v0[0] = numberField<RealT>(input.init, "Va", "bus.init");
        bus.data.v0[1] = numberField<RealT>(input.init, "Vb", "bus.init");
        bus.data.v0[2] = numberField<RealT>(input.init, "Vc", "bus.init");
        if (input.init.contains("dVa"))
        {
          bus.data.vp0[0] = input.init.at("dVa").get<RealT>();
        }
        if (input.init.contains("dVb"))
        {
          bus.data.vp0[1] = input.init.at("dVb").get<RealT>();
        }
        if (input.init.contains("dVc"))
        {
          bus.data.vp0[2] = input.init.at("dVc").get<RealT>();
        }
        return bus;
      }

      template <typename RealT, typename IdxT>
      typename SystemModelData<RealT, IdxT>::LineSpec
      readLine(const Model::Case::ComponentData&                             input,
               const std::set<IdxT>&                                         buses,
               const std::map<std::string, RationalApproxData<RealT, IdxT>>& admittance_models)
      {
        if (input.component_class != "Line")
        {
          throw parserError("branches." + input.id,
                            "unsupported branch class '" + input.component_class + "'");
        }

        typename SystemModelData<RealT, IdxT>::LineSpec line;
        line.id   = input.id;
        line.bus1 = indexField<IdxT>(input.ports, "bus1", "branches." + input.id + ".ports");
        line.bus2 = indexField<IdxT>(input.ports, "bus2", "branches." + input.id + ".ports");
        requireKnownBus(buses, line.bus1, "branches." + input.id + ".ports.bus1");
        requireKnownBus(buses, line.bus2, "branches." + input.id + ".ports.bus2");

        const auto admittance_id =
            requireField(input.params, "Yc", "branches." + input.id + ".params")
                .get<std::string>();
        auto it = admittance_models.find(admittance_id);
        if (it == admittance_models.end())
        {
          throw parserError("branches." + input.id + ".params.Yc",
                            "unknown characteristic admittance model '" + admittance_id + "'");
        }
        line.data.characteristic_admittance = it->second;
        return line;
      }

      template <typename RealT, typename IdxT>
      typename SystemModelData<RealT, IdxT>::VoltageSourceSpec
      readVoltageSource(const Model::Case::ComponentData& input, const std::set<IdxT>& buses)
      {
        typename SystemModelData<RealT, IdxT>::VoltageSourceSpec source;
        source.id  = input.id;
        source.bus = indexField<IdxT>(input.ports, "bus", "devices." + input.id + ".ports");
        requireKnownBus(buses, source.bus, "devices." + input.id + ".ports.bus");

        source.data.amplitude =
            numberField<RealT>(input.params, "amplitude", "devices." + input.id + ".params");
        source.data.frequency =
            numberField<RealT>(input.params, "frequency", "devices." + input.id + ".params");
        source.data.phase =
            numberField<RealT>(input.params, "phase", "devices." + input.id + ".params");
        if (input.init.contains("current"))
        {
          source.data.current0 =
              readVector3<RealT>(input.init.at("current"), "devices." + input.id + ".init.current");
        }
        return source;
      }

      template <typename RealT, typename IdxT>
      typename SystemModelData<RealT, IdxT>::ShuntLoadSpec
      readShuntLoad(const Model::Case::ComponentData& input, const std::set<IdxT>& buses)
      {
        typename SystemModelData<RealT, IdxT>::ShuntLoadSpec load;
        load.id  = input.id;
        load.bus = indexField<IdxT>(input.ports, "bus", "devices." + input.id + ".ports");
        requireKnownBus(buses, load.bus, "devices." + input.id + ".ports.bus");

        load.data.conductance = readMatrix3<RealT>(
            requireField(input.params, "conductance", "devices." + input.id + ".params"),
            "devices." + input.id + ".params.conductance");
        if (input.init.contains("closed"))
        {
          load.data.closed = input.init.at("closed").get<bool>();
        }
        return load;
      }

    } // namespace Detail

    template <typename RealT, typename IdxT>
    SystemModelData<RealT, IdxT> systemModelDataFromCase(const Model::Case::CaseData& input)
    {
      SystemModelData<RealT, IdxT> data;
      data.format_version   = input.header.format_version;
      data.format_revision  = input.header.format_revision;
      data.case_name        = input.header.case_name;
      data.case_date_time   = input.header.case_date_time;
      data.case_description = input.header.case_description;
      data.case_comments    = input.header.case_comments;
      data.freq_base        = static_cast<RealT>(input.header.freq_base);
      data.va_base          = static_cast<RealT>(input.header.va_base);

      std::set<IdxT> bus_numbers;
      data.bus.reserve(input.buses.size());
      for (const auto& bus_input : input.buses)
      {
        auto bus = Detail::readBus<RealT, IdxT>(bus_input);
        bus_numbers.insert(bus.number);
        data.bus.push_back(bus);
      }

      const auto admittance_models = Detail::readYcModels<RealT, IdxT>(input);

      data.line.reserve(input.branches.size());
      for (const auto& branch_input : input.branches)
      {
        data.line.push_back(
            Detail::readLine<RealT, IdxT>(branch_input, bus_numbers, admittance_models));
      }

      for (const auto& device_input : input.devices)
      {
        if (device_input.component_class == "VoltageSource")
        {
          data.voltage_source.push_back(
              Detail::readVoltageSource<RealT, IdxT>(device_input, bus_numbers));
        }
        else if (device_input.component_class == "ShuntLoad")
        {
          data.shunt_load.push_back(Detail::readShuntLoad<RealT, IdxT>(device_input, bus_numbers));
        }
        else
        {
          throw Detail::parserError("devices." + device_input.id,
                                    "unsupported device class '"
                                        + device_input.component_class + "'");
        }
      }

      return data;
    }

  } // namespace EMT
} // namespace GridKit
