cmake_minimum_required(VERSION 3.20)

project(SpacecraftTestbed LANGUAGES CXX)

add_executable(flight main.cpp FlightLogic.cpp FlightTests.cpp SensorProtocol.cpp SerialPort.cpp)

target_compile_features(flight PRIVATE cxx_std_17)