find_package(
  Python3
  COMPONENTS Interpreter
  REQUIRED)

add_executable(emt_delay_response DelayResponse.cpp)
target_link_libraries(
  emt_delay_response GridKit::emt_delay GridKit::solvers_dyn)

set(_delay_output_dir ${CMAKE_CURRENT_SOURCE_DIR}/output)
set(_delay_data_dir ${_delay_output_dir}/data)
set(_delay_response_csv ${_delay_data_dir}/delay_response.csv)
set(_delay_response_plot_stamp ${_delay_output_dir}/delay_response_plots.stamp)
set(_delay_response_pngs
    ${_delay_output_dir}/delay_response.png
    ${_delay_output_dir}/phase_diagram/delay_response_phase_good_damped.png
    ${_delay_output_dir}/phase_diagram/delay_response_phase_good_sample_shift.png
    ${_delay_output_dir}/phase_diagram/delay_response_phase_good_near_band.png
    ${_delay_output_dir}/phase_diagram/delay_response_phase_good_coarse_n1.png
    ${_delay_output_dir}/phase_diagram/delay_response_phase_boundary_in_band.png
    ${_delay_output_dir}/phase_diagram/delay_response_phase_out_of_band_stable.png
    ${_delay_output_dir}/section_count/delay_response_n1_low_frequency.png
    ${_delay_output_dir}/section_count/delay_response_n1_above_fmax.png
    ${_delay_output_dir}/section_count/delay_response_n2_fractional_ceiling.png
    ${_delay_output_dir}/section_count/delay_response_n4_exact_product.png
    ${_delay_output_dir}/section_count/delay_response_n24_many_sections.png
    ${_delay_output_dir}/frequency_sweep/delay_response_sine_10hz_far_below_fmax.png
    ${_delay_output_dir}/frequency_sweep/delay_response_sine_20hz_below_inv_tau.png
    ${_delay_output_dir}/frequency_sweep/delay_response_sine_25hz_at_inv_tau.png
    ${_delay_output_dir}/frequency_sweep/delay_response_sine_30hz_above_inv_tau_below_fmax.png
    ${_delay_output_dir}/frequency_sweep/delay_response_sine_40hz_half_fmax.png
    ${_delay_output_dir}/frequency_sweep/delay_response_sine_75hz_near_fmax.png
    ${_delay_output_dir}/frequency_sweep/delay_response_sine_80hz_at_fmax.png
    ${_delay_output_dir}/frequency_sweep/delay_response_sine_90hz_just_above_fmax.png
    ${_delay_output_dir}/frequency_sweep/delay_response_sine_160hz_twice_fmax.png
    ${_delay_output_dir}/frequency_sweep/delay_response_sine_320hz_four_times_fmax.png
    ${_delay_output_dir}/delay_sweep/delay_response_short_tau_n1.png
    ${_delay_output_dir}/delay_sweep/delay_response_medium_tau_n4.png
    ${_delay_output_dir}/delay_sweep/delay_response_long_tau_n8.png
    ${_delay_output_dir}/delay_sweep/delay_response_very_long_tau_n16.png
    ${_delay_output_dir}/composite/delay_response_smooth_blend_step.png
    ${_delay_output_dir}/composite/delay_response_two_tone_cross_fmax.png
    ${_delay_output_dir}/composite/delay_response_chirp_5hz_to_160hz.png
    ${_delay_output_dir}/transients/delay_response_smooth_step.png
    ${_delay_output_dir}/transients/delay_response_smooth_pulse_pair.png
    ${_delay_output_dir}/transients/delay_response_narrow_pulse_above_band.png
    ${_delay_output_dir}/phase_diagram/delay_phase_diagram.png)

add_custom_command(
  OUTPUT ${_delay_response_csv}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${_delay_data_dir}
  COMMAND $<TARGET_FILE:emt_delay_response> ${_delay_response_csv}
  DEPENDS emt_delay_response)

add_custom_command(
  OUTPUT ${_delay_response_plot_stamp}
  BYPRODUCTS ${_delay_response_pngs}
  COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/plot_delay_response.py ${_delay_response_csv}
    ${_delay_output_dir}
  COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/plot_delay_phase_diagram.py
    ${_delay_response_csv} ${_delay_output_dir}/phase_diagram/delay_phase_diagram.png
  COMMAND ${CMAKE_COMMAND} -E touch ${_delay_response_plot_stamp}
  DEPENDS ${_delay_response_csv} ${CMAKE_CURRENT_SOURCE_DIR}/plot_delay_response.py
          ${CMAKE_CURRENT_SOURCE_DIR}/plot_delay_phase_diagram.py)

add_custom_target(plot_emt_delay_response DEPENDS ${_delay_response_plot_stamp})

install(TARGETS emt_delay_response RUNTIME DESTINATION bin)
gridkit_example_add_file(plot_delay_response.py)
gridkit_example_add_file(plot_delay_phase_diagram.py)
