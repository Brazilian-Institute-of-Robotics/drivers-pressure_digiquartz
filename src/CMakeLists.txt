rock_library(digiquartz_pressure
    SOURCES Driver.cpp
    HEADERS Driver.hpp
    DEPS_PKGCONFIG base-types iodrivers_base)

rock_executable(digiquartz_pressure_bin Main.cpp
    DEPS digiquartz_pressure)

