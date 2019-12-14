cmake_minimum_required(VERSION 2.8)
project(observation)

find_package(qibuild)

# Create a executable named observation
# with the source file: main.cpp
qi_create_bin(observation "main.cpp")

# Add a simple test:
qi_create_test(test_observation "test.cpp")
qi_use_lib(observation ALCOMMON ALERROR ALPROXIES ALVISION)