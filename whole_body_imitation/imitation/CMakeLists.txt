cmake_minimum_required(VERSION 2.8)
project(whole_body_imitation)

find_package(qibuild)

# Create a executable named whole_body_imitation
# with the source file: main.cpp
qi_create_bin(whole_body_imitation "main.cpp")

# Add a simple test:
qi_create_test(test_whole_body_imitation "test.cpp")
qi_use_lib(whole_body_imitation ALCOMMON ALERROR ALPROXIES ALVISION) 