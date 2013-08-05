rock_library(digiquartz_pressure
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(digiquartz_pressure_bin Main.cpp
    DEPS digiquartz_pressure)

