#

# [[
#  Author(s):
#    - Cameron Rutherford <cameron.rutherford@pnnl.gov>
#]]

add_subdirectory(PhasorDynamics)
add_subdirectory(EMT)
add_subdirectory(PowerFlow)
add_subdirectory(PowerElectronics)

install(FILES
  Events.hpp
  Evaluator.hpp
  VariableMonitor.hpp
  VariableMonitorController.hpp
  DESTINATION include/GridKit/Model)
