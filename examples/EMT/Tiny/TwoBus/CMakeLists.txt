gridkit_example_current_install_path(_install_path)

add_executable(EMTTinyTwoBus TwoBus.cpp)
target_link_libraries(EMTTinyTwoBus GridKit::emt)

install(TARGETS EMTTinyTwoBus RUNTIME DESTINATION ${_install_path})

add_test(NAME EMTTinyTwoBus COMMAND EMTTinyTwoBus)
