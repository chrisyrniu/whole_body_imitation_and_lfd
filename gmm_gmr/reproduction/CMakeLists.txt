cmake_minimum_required(VERSION 2.8)
project(reproduction)

find_package(qibuild)

# Create a executable named reproduction
# with the source file: main.cpp
qi_create_bin(reproduction "main.cpp")

# Add a simple test:
qi_create_test(test_reproduction "test.cpp")
qi_use_lib(reproduction ALCOMMON ALERROR ALPROXIES)