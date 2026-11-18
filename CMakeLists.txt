cmake_minimum_required(VERSION 3.16)
project(booster_landing LANGUAGES CXX)

# enable_testing() has to happen in the top-level list file as well, otherwise
# ctest run from this build tree never descends into external/pdg.
enable_testing()

# The guidance/dynamics core. Its own tests stay on so `ctest` covers them too.
add_subdirectory(external/pdg)

# Bridge: solves the descent, flies it closed loop, and writes the telemetry
# the Rerun viewer replays.
add_executable(pdg_flight bridge/pdg_flight.cpp)
target_link_libraries(pdg_flight PRIVATE pdg::pdg)
# pdg sets CMAKE_CXX_STANDARD in its own directory scope only, so ask here too
# (<filesystem> needs it).
target_compile_features(pdg_flight PRIVATE cxx_std_17)

if(MSVC)
  target_compile_options(pdg_flight PRIVATE /W4)
  target_compile_definitions(pdg_flight PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(pdg_flight PRIVATE -Wall -Wextra)
endif()
