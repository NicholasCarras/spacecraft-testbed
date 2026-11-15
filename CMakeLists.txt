cmake_minimum_required(VERSION 3.20)

project(SpacecraftTestbed LANGUAGES CXX)

add_executable(flight main.cpp FlightLogic.cpp FlightTests.cpp SensorProtocol.cpp)

target_compile_features(flight PRIVATE cxx_std_17)

if(UNIX)
    add_executable(serial-check
        SerialCheck.cpp
        SerialPort.cpp
        SensorProtocol.cpp
        FlightLogic.cpp
    )
    target_compile_features(serial-check PRIVATE cxx_std_17)
endif()