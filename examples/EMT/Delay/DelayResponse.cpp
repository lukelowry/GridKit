#include <algorithm>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <GridKit/Model/EMT/Operators/Shift/Delay/Delay.hpp>
#include <GridKit/Solver/Dynamic/ArkStep.hpp>

namespace
{
  constexpr double pi = 3.141592653589793238462643383279502884;

  using Delay                = GridKit::EMT::Delay<double, size_t>;
  using Function             = std::function<double(double)>;
  using InitialStateFunction = Delay::InitialStateFunction;
  using MethodType           = AnalysisManager::Sundials::ArkStep<double, size_t>::Type;

  enum class StepReference
  {
    SectionTime,
    DelayTime,
  };

  struct Method
  {
    std::string   label;
    MethodType    type;
    int           order;
    bool          fixed_step;
    double        fixed_step_factor;
    StepReference step_reference;
  };

  struct SignalCase
  {
    std::string          category;
    std::string          slug;
    std::string          label;
    double               tau;
    double               fmax;
    double               t_final;
    int                  adaptive_outputs;
    Function             input;
    Function             derivative;
    InitialStateFunction initial_state{};
    std::vector<Method>  case_methods{};
    double               input_frequency{0.0};
  };

  double smoothStep(double t, double center, double width)
  {
    return 0.5 * (1.0 + std::tanh((t - center) / width));
  }

  double smoothStepDerivative(double t, double center, double width)
  {
    const double z     = (t - center) / width;
    const double tanhz = std::tanh(z);
    return 0.5 * (1.0 - tanhz * tanhz) / width;
  }

  double pulse(double t, double start, double stop, double width)
  {
    return smoothStep(t, start, width) - smoothStep(t, stop, width);
  }

  double pulseDerivative(double t, double start, double stop, double width)
  {
    return smoothStepDerivative(t, start, width) - smoothStepDerivative(t, stop, width);
  }

  InitialStateFunction sineSteadyState(double frequency, double amplitude)
  {
    return [frequency, amplitude](size_t section, double time, double section_time, double& y, double& yp)
    {
      const double               omega = 2.0 * pi * frequency;
      const std::complex<double> j{0.0, 1.0};
      const std::complex<double> H = 1.0 / (1.0 + j * omega * section_time);

      std::complex<double> response{1.0, 0.0};
      for (size_t i = 0; i <= section; ++i)
        response *= H;

      const std::complex<double> state = amplitude * std::exp(j * omega * time) * response;
      y                                = std::imag(state);
      yp                               = std::imag(j * omega * state);
    };
  }

  SignalCase sineCase(const std::string&  category,
                      const std::string&  slug,
                      const std::string&  label,
                      double              tau,
                      double              fmax,
                      double              t_final,
                      int                 adaptive_outputs,
                      double              frequency,
                      double              amplitude    = 1.0,
                      std::vector<Method> case_methods = {})
  {
    return {
        category,
        slug,
        label,
        tau,
        fmax,
        t_final,
        adaptive_outputs,
        [frequency, amplitude](double t)
        {
          return amplitude * std::sin(2.0 * pi * frequency * t);
        },
        [frequency, amplitude](double t)
        {
          return amplitude * 2.0 * pi * frequency * std::cos(2.0 * pi * frequency * t);
        },
        sineSteadyState(frequency, amplitude),
        case_methods,
        frequency,
    };
  }

  std::vector<Method> methods()
  {
    return {
        {"ArkStep implicit adaptive", MethodType::Implicit, 4, false, 0.0, StepReference::SectionTime},
        {"ArkStep explicit adaptive", MethodType::Explicit, 4, false, 0.0, StepReference::SectionTime},
        {"ArkStep forward Euler fixed h=0.5T", MethodType::Explicit, 1, true, 0.5, StepReference::SectionTime},
        {"ArkStep forward Euler fixed h=T", MethodType::Explicit, 1, true, 1.0, StepReference::SectionTime},
        {"ArkStep forward Euler fixed h=1.5T", MethodType::Explicit, 1, true, 1.5, StepReference::SectionTime},
        {"ArkStep forward Euler fixed h=2.1T", MethodType::Explicit, 1, true, 2.1, StepReference::SectionTime},
    };
  }

  Method phaseMethod(double h_over_T)
  {
    return {"ArkStep forward Euler phase-map fixed h", MethodType::Explicit, 1, true, h_over_T, StepReference::SectionTime};
  }

  std::vector<SignalCase> signalCases()
  {
    return {
        sineCase("phase_diagram", "phase_good_damped", "Low-band N=10 FE", 10.0 / 80.0, 80.0, 0.36, 900, 8.8, 1.0, {phaseMethod(1.0)}),
        sineCase("phase_diagram", "phase_good_sample_shift", "Low-band N=3 FE", 3.0 / 80.0, 80.0, 0.36, 900, 12.0, 1.0, {phaseMethod(1.0)}),
        sineCase("phase_diagram", "phase_good_near_band", "High-band N=1 FE", 1.0 / 80.0, 80.0, 0.36, 900, 35.2, 1.0, {phaseMethod(1.0)}),
        sineCase("phase_diagram", "phase_good_coarse_n1", "Mid-band N=2 FE", 2.0 / 80.0, 80.0, 0.36, 900, 23.2, 1.0, {phaseMethod(1.0)}),
        sineCase("phase_diagram", "phase_boundary_in_band", "Mid-band N=7 FE", 7.0 / 80.0, 80.0, 0.36, 900, 20.8, 1.0, {phaseMethod(1.0)}),
        sineCase("phase_diagram", "phase_out_of_band_stable", "High-band N=5 FE", 5.0 / 80.0, 80.0, 0.36, 900, 32.8, 1.0, {phaseMethod(1.0)}),

        sineCase("section_count", "n1_low_frequency", "N=1 low-frequency sine", 0.005, 20.0, 1.0, 700, 2.0),
        sineCase("section_count", "n1_above_fmax", "N=1 sine above f_max", 0.005, 20.0, 0.12, 800, 80.0),
        sineCase("section_count", "n2_fractional_ceiling", "N=2 from ceil(f_max tau)", 0.02, 75.0, 0.25, 800, 20.0),
        sineCase("section_count", "n4_exact_product", "N=4 exact f_max tau product", 0.04, 100.0, 0.25, 900, 25.0),
        sineCase("section_count", "n24_many_sections", "N=24 many-section low-frequency sine", 0.12, 200.0, 1.0, 700, 3.0),

        sineCase("frequency_sweep", "sine_10hz_far_below_fmax", "10 Hz sine far below f_max", 0.04, 80.0, 0.30, 700, 10.0),
        sineCase("frequency_sweep", "sine_20hz_below_inv_tau", "20 Hz sine below 1/tau", 0.04, 80.0, 0.25, 900, 20.0),
        sineCase("frequency_sweep", "sine_25hz_at_inv_tau", "25 Hz sine at 1/tau", 0.04, 80.0, 0.25, 900, 25.0),
        sineCase("frequency_sweep", "sine_30hz_above_inv_tau_below_fmax", "30 Hz sine above 1/tau and below f_max", 0.04, 80.0, 0.25, 1000, 30.0),
        sineCase("frequency_sweep", "sine_40hz_half_fmax", "40 Hz sine at half f_max", 0.04, 80.0, 0.25, 900, 40.0),
        sineCase("frequency_sweep", "sine_75hz_near_fmax", "75 Hz sine near f_max", 0.04, 80.0, 0.20, 1000, 75.0),
        sineCase("frequency_sweep", "sine_80hz_at_fmax", "80 Hz sine at f_max", 0.04, 80.0, 0.20, 1000, 80.0),
        sineCase("frequency_sweep", "sine_90hz_just_above_fmax", "90 Hz sine just above f_max", 0.04, 80.0, 0.18, 1100, 90.0),
        sineCase("frequency_sweep", "sine_160hz_twice_fmax", "160 Hz sine at twice f_max", 0.04, 80.0, 0.15, 1200, 160.0),
        sineCase("frequency_sweep", "sine_320hz_four_times_fmax", "320 Hz sine at four times f_max", 0.04, 80.0, 0.10, 1400, 320.0),

        sineCase("delay_sweep", "short_tau_n1", "Short tau with N=1", 0.005, 80.0, 0.20, 800, 20.0),
        sineCase("delay_sweep", "medium_tau_n4", "Medium tau with N=4", 0.04, 80.0, 0.30, 900, 20.0),
        sineCase("delay_sweep", "long_tau_n8", "Long tau with N=8", 0.10, 80.0, 0.45, 1000, 20.0),
        sineCase("delay_sweep", "very_long_tau_n16", "Very long tau with N=16", 0.20, 80.0, 0.70, 1100, 10.0),

        {
            "composite",
            "smooth_blend_step",
            "Smooth low-frequency blend with step",
            0.12,
            200.0,
            1.0,
            700,
            [](double t)
            {
              return 0.60 * std::sin(2.0 * pi * 1.5 * t)
                     + 0.25 * std::sin(2.0 * pi * 3.0 * t + 0.2)
                     + 0.35 * smoothStep(t, 0.25, 0.05);
            },
            [](double t)
            {
              return 0.60 * 2.0 * pi * 1.5 * std::cos(2.0 * pi * 1.5 * t)
                     + 0.25 * 2.0 * pi * 3.0 * std::cos(2.0 * pi * 3.0 * t + 0.2)
                     + 0.35 * smoothStepDerivative(t, 0.25, 0.05);
            },
            {},
        },
        {
            "composite",
            "two_tone_cross_fmax",
            "20 Hz plus 140 Hz two-tone",
            0.04,
            80.0,
            0.25,
            1100,
            [](double t)
            {
              return 0.70 * std::sin(2.0 * pi * 20.0 * t)
                     + 0.30 * std::sin(2.0 * pi * 140.0 * t + 0.4);
            },
            [](double t)
            {
              return 0.70 * 2.0 * pi * 20.0 * std::cos(2.0 * pi * 20.0 * t)
                     + 0.30 * 2.0 * pi * 140.0 * std::cos(2.0 * pi * 140.0 * t + 0.4);
            },
            {},
        },
        {
            "composite",
            "chirp_5hz_to_160hz",
            "Chirp from 5 Hz to 160 Hz",
            0.04,
            80.0,
            0.30,
            1300,
            [](double t)
            {
              constexpr double f0 = 5.0;
              constexpr double f1 = 160.0;
              constexpr double tf = 0.30;
              const double     k  = (f1 - f0) / tf;
              return std::sin(2.0 * pi * (f0 * t + 0.5 * k * t * t));
            },
            [](double t)
            {
              constexpr double f0    = 5.0;
              constexpr double f1    = 160.0;
              constexpr double tf    = 0.30;
              const double     k     = (f1 - f0) / tf;
              const double     phase = 2.0 * pi * (f0 * t + 0.5 * k * t * t);
              return 2.0 * pi * (f0 + k * t) * std::cos(phase);
            },
            {},
        },
        {
            "transients",
            "smooth_step",
            "Smooth step input",
            0.04,
            80.0,
            0.25,
            700,
            [](double t)
            {
              return smoothStep(t, 0.05, 0.006);
            },
            [](double t)
            {
              return smoothStepDerivative(t, 0.05, 0.006);
            },
            {},
        },
        {
            "transients",
            "smooth_pulse_pair",
            "Smooth pulse pair",
            0.075,
            80.0,
            0.45,
            900,
            [](double t)
            {
              return 0.90 * pulse(t, 0.06, 0.13, 0.006)
                     - 0.55 * pulse(t, 0.22, 0.30, 0.010);
            },
            [](double t)
            {
              return 0.90 * pulseDerivative(t, 0.06, 0.13, 0.006)
                     - 0.55 * pulseDerivative(t, 0.22, 0.30, 0.010);
            },
            {},
        },
        {
            "transients",
            "narrow_pulse_above_band",
            "Narrow pulse with above-band content",
            0.04,
            80.0,
            0.20,
            1000,
            [](double t)
            {
              return pulse(t, 0.045, 0.060, 0.0015);
            },
            [](double t)
            {
              return pulseDerivative(t, 0.045, 0.060, 0.0015);
            },
            {},
        },
    };
  }

  size_t sectionCount(const SignalCase& signal)
  {
    return std::max<size_t>(1, static_cast<size_t>(std::ceil(signal.fmax * signal.tau)));
  }

  double sectionTime(const SignalCase& signal)
  {
    return signal.tau / static_cast<double>(sectionCount(signal));
  }

  double fixedStep(const SignalCase& signal, const Method& method)
  {
    return method.fixed_step_factor * (method.step_reference == StepReference::DelayTime ? signal.tau : sectionTime(signal));
  }

  void writeRow(std::ofstream&    out,
                const SignalCase& signal,
                const Method&     method,
                double            t,
                double            output)
  {
    const double in    = signal.input(t);
    const double ideal = signal.input(t - signal.tau);
    out << signal.category << ','
        << signal.slug << ','
        << signal.label << ','
        << std::setprecision(16) << signal.tau << ','
        << signal.fmax << ',';
    if (signal.input_frequency > 0.0)
      out << signal.input_frequency;
    out << ','
        << sectionCount(signal) << ','
        << sectionTime(signal) << ','
        << method.label << ',';

    if (method.fixed_step)
    {
      const double h = fixedStep(signal, method);
      out << h << ','
          << h / sectionTime(signal) << ','
          << h / signal.tau << ',';
    }
    else
    {
      out << ",,,";
    }

    out << t << ','
        << in << ','
        << ideal << ','
        << output << ','
        << output - ideal << '\n';
  }

  void writeInputHistory(std::ofstream& out, const SignalCase& signal)
  {
    constexpr int history_samples = 200;
    for (int i = 0; i < history_samples; ++i)
    {
      const double t = -signal.tau + signal.tau * static_cast<double>(i) / static_cast<double>(history_samples);
      out << signal.category << ','
          << signal.slug << ','
          << signal.label << ','
          << std::setprecision(16) << signal.tau << ','
          << signal.fmax << ',';
      if (signal.input_frequency > 0.0)
        out << signal.input_frequency;
      out << ','
          << sectionCount(signal) << ','
          << sectionTime(signal) << ",input_history,,,,"
          << t << ','
          << signal.input(t) << ",,,\n";
    }
  }

  void runMethod(std::ofstream& out, const SignalCase& signal, const Method& method)
  {
    Delay                                              delay(signal.tau, signal.fmax, signal.input, signal.derivative, signal.initial_state);
    AnalysisManager::Sundials::ArkStep<double, size_t> solver(&delay, method.type);

    const double h       = fixedStep(signal, method);
    const int    nout    = method.fixed_step
                               ? std::max(1, static_cast<int>(std::ceil(signal.t_final / h - 1.0e-12)))
                               : signal.adaptive_outputs;
    const double t_final = method.fixed_step ? static_cast<double>(nout) * h : signal.t_final;

    solver.setOrder(method.order);
    solver.setTolerance(1.0e-8, 1.0e-10);
    solver.setMaxSteps(100000);
    if (method.fixed_step)
      solver.setFixedStep(h);

    solver.configureSimulation();
    solver.initializeSimulation(0.0);
    writeRow(out, signal, method, 0.0, delay.output());

    auto callback = [&](double t)
    {
      writeRow(out, signal, method, t, delay.output());
    };
    solver.runSimulation(t_final, nout, callback);
  }
} // namespace

int main(int argc, char** argv)
{
  std::filesystem::path output = "delay_response.csv";
  if (argc > 1)
    output = argv[1];

  if (output.has_parent_path())
    std::filesystem::create_directories(output.parent_path());

  std::ofstream out(output);
  if (!out)
  {
    std::cerr << "Could not open " << output << " for writing.\n";
    return 1;
  }

  out << "category,case,case_label,tau,fmax,frequency,N,T,method,h,h_over_T,h_over_tau,t,input,ideal_delay,output,error\n";

  const auto all_methods = methods();
  for (const auto& signal : signalCases())
  {
    writeInputHistory(out, signal);
    const auto& signal_methods = signal.case_methods.empty() ? all_methods : signal.case_methods;
    for (const auto& method : signal_methods)
      runMethod(out, signal, method);
  }

  return 0;
}
