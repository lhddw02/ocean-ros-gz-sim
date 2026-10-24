cmake_minimum_required(VERSION 3.16)
project(ocean_landing_gui VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)
find_package(gz-gui8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

qt5_add_resources(replay_resources src/replay.qrc)
add_library(LandingReplay SHARED
  src/LandingReplay.cc src/LandingReplay.hh ${replay_resources})
target_compile_features(LandingReplay PRIVATE cxx_std_17)
target_compile_options(LandingReplay PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(LandingReplay PRIVATE
  gz-gui8::gz-gui8 gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13 gz-msgs10::gz-msgs10
  Qt5::Core Qt5::Quick Qt5::QuickControls2)
install(TARGETS LandingReplay LIBRARY DESTINATION lib)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  find_package(Qt5 REQUIRED COMPONENTS Test)
  add_executable(replay_button_test test/replay_button_test.cc)
  target_compile_features(replay_button_test PRIVATE cxx_std_17)
  target_include_directories(replay_button_test PRIVATE src)
  target_link_libraries(replay_button_test PRIVATE LandingReplay
    gz-gui8::gz-gui8 gz-transport13::gz-transport13 gz-msgs10::gz-msgs10
    Qt5::Core Qt5::Quick Qt5::QuickControls2 Qt5::Test)
  add_test(NAME replay_button_test COMMAND replay_button_test)
  set_tests_properties(replay_button_test PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;QT_QUICK_BACKEND=software"
    TIMEOUT 30)
endif()

ament_package()
