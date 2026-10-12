cmake_minimum_required(VERSION 3.20)

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" ODRFORGE_VERSION)
string(STRIP "${ODRFORGE_VERSION}" ODRFORGE_VERSION)
project(odrforge VERSION ${ODRFORGE_VERSION} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(ODRFORGE_BUILD_TESTS "Build unit tests" ON)
option(ODRFORGE_BUILD_CLI "Build the osm2xodr command line tool" ON)
option(ODRFORGE_BUILD_PYTHON "Build the Python module (needs pybind11)" OFF)

include(FetchContent)
FetchContent_Declare(pugixml
  URL https://github.com/zeux/pugixml/releases/download/v1.14/pugixml-1.14.tar.gz
  URL_HASH SHA256=2f10e276870c64b1db6809050a75e11a897a8d7456c4be5c6b2e35a11168a015)
FetchContent_Declare(nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d)
set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(pugixml nlohmann_json)

# viewer template embedded as a C string (regenerated when viewer.html changes)
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/viz_template.cpp
  COMMAND ${CMAKE_COMMAND} -DIN=${CMAKE_CURRENT_SOURCE_DIR}/src/viz/viewer.html
          -DOUT=${CMAKE_CURRENT_BINARY_DIR}/viz_template.cpp -DNAME=kVizTemplate -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed.cmake
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/viz/viewer.html ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed.cmake
  COMMENT "Embedding viewer.html")

configure_file(include/odrforge/version.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/include/odrforge/version.hpp @ONLY)

add_library(odrforge
  src/geometry.cpp
  src/clothoid_fit.cpp
  src/projection.cpp
  src/osm.cpp
  src/odr_writer.cpp
  src/odr_query.cpp
  src/rules.cpp
  src/lanes.cpp
  src/network.cpp
  src/network_junctions.cpp
  src/validate.cpp
  src/odr_reader.cpp
  src/terrain.cpp
  src/elevation.cpp
  src/overlap.cpp
  src/snap.cpp
  src/gore.cpp
  src/sidewalk.cpp
  src/viz.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/viz_template.cpp
)
target_include_directories(odrforge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(odrforge PUBLIC pugixml::pugixml nlohmann_json::nlohmann_json)
target_compile_definitions(odrforge PUBLIC ODRFORGE_RULES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/rules")
if(MSVC)
  target_compile_options(odrforge PRIVATE /W4 /permissive- /utf-8 /wd4100)
  target_compile_definitions(odrforge PUBLIC _USE_MATH_DEFINES NOMINMAX)
else()
  target_compile_options(odrforge PRIVATE -Wall -Wextra)
endif()

if(ODRFORGE_BUILD_CLI)
  add_executable(osm2xodr tools/osm2xodr.cpp)
  target_link_libraries(osm2xodr PRIVATE odrforge)
  add_executable(xodrviz tools/xodrviz.cpp)
  target_link_libraries(xodrviz PRIVATE odrforge)
  install(TARGETS osm2xodr xodrviz RUNTIME DESTINATION bin)
endif()

if(ODRFORGE_BUILD_TESTS)
  enable_testing()
  add_executable(odrforge_tests tests/test_main.cpp tests/test_geometry.cpp tests/test_projection.cpp
    tests/test_spec.cpp tests/test_network.cpp tests/test_elevation.cpp tests/test_topology.cpp)
  target_link_libraries(odrforge_tests PRIVATE odrforge)
  set(ODRFORGE_TEST_OUT ${CMAKE_CURRENT_BINARY_DIR}/test_out)
  file(MAKE_DIRECTORY ${ODRFORGE_TEST_OUT})
  add_test(NAME odrforge_tests COMMAND odrforge_tests)
  set_tests_properties(odrforge_tests PROPERTIES ENVIRONMENT "ODRFORGE_TEST_OUT=${ODRFORGE_TEST_OUT}"
                       FIXTURES_SETUP scenario_files)
  # official ASAM OpenDRIVE 1.8.1 XSD (XSD 1.1) validation of every scenario, if Python + xmlschema exist
  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_FOUND)
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import xmlschema" RESULT_VARIABLE _xs OUTPUT_QUIET ERROR_QUIET)
    if(_xs EQUAL 0)
      add_test(NAME xsd_conformance COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_CURRENT_SOURCE_DIR}/tests/xsd_validate.py
               ${CMAKE_CURRENT_SOURCE_DIR}/tests/xsd/opendrive_1_8_1 ${ODRFORGE_TEST_OUT})
      set_tests_properties(xsd_conformance PROPERTIES FIXTURES_REQUIRED scenario_files)
      add_test(NAME xsd_conformance_1_4 COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_CURRENT_SOURCE_DIR}/tests/xsd_validate.py --allow-lht-rule
               ${CMAKE_CURRENT_SOURCE_DIR}/tests/xsd/opendrive_1_4/OpenDRIVE_1.4H.xsd ${ODRFORGE_TEST_OUT}/rev4)
      set_tests_properties(xsd_conformance_1_4 PROPERTIES FIXTURES_REQUIRED scenario_files)
      add_test(NAME xsd_rejects_invalid COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_CURRENT_SOURCE_DIR}/tests/xsd_validate.py
               ${CMAKE_CURRENT_SOURCE_DIR}/tests/xsd/opendrive_1_8_1 ${CMAKE_CURRENT_SOURCE_DIR}/tests/data/invalid_roadmark_height.xodr)
      set_tests_properties(xsd_rejects_invalid PROPERTIES WILL_FAIL TRUE)
    else()
      message(STATUS "xmlschema not available: XSD conformance test disabled (pip install xmlschema)")
    endif()
  endif()
endif()

install(TARGETS odrforge ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/odrforge DESTINATION include FILES_MATCHING PATTERN "*.hpp")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/odrforge/version.hpp DESTINATION include/odrforge)
install(DIRECTORY rules DESTINATION share/odrforge)
