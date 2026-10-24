cmake_minimum_required(VERSION 3.16)
project(ocean_station_systems VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(ocean-station-mooring-system SHARED src/StationMooring.cc)
target_compile_features(ocean-station-mooring-system PRIVATE cxx_std_17)
target_compile_options(ocean-station-mooring-system PRIVATE
  -Wall -Wextra -Wpedantic)
target_include_directories(ocean-station-mooring-system PRIVATE src)
target_link_libraries(ocean-station-mooring-system PRIVATE
  gz-sim8::gz-sim8 gz-plugin2::gz-plugin2)

add_library(ocean-replay-contact-system SHARED src/ReplayContact.cc)
target_compile_features(ocean-replay-contact-system PRIVATE cxx_std_17)
target_compile_options(ocean-replay-contact-system PRIVATE
  -Wall -Wextra -Wpedantic)
target_link_libraries(ocean-replay-contact-system PRIVATE
  gz-sim8::gz-sim8 gz-plugin2::gz-plugin2
  gz-transport13::core gz-msgs10::gz-msgs10)

install(TARGETS ocean-station-mooring-system ocean-replay-contact-system
  LIBRARY DESTINATION lib)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(mooring_math_test test/mooring_math_test.cc)
  target_compile_features(mooring_math_test PRIVATE cxx_std_17)
  target_include_directories(mooring_math_test PRIVATE src)
  add_test(NAME mooring_math_test COMMAND mooring_math_test)
  add_test(NAME contact_reset_test
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/contact_reset_test.py
      --plugin-dir $<TARGET_FILE_DIR:ocean-replay-contact-system>
      --output ${CMAKE_CURRENT_BINARY_DIR}/contact-reset-report.json)
  set_tests_properties(contact_reset_test PROPERTIES TIMEOUT 60)
endif()

ament_package()
