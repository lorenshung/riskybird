# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sensor_check)

# Reuse pmw3901_test's driver rather than taking a third copy of it. There are
# already two (pmw3901_test and sensor_bringup) and they can drift apart; one
# more copy makes a fix land in three places instead of two.
target_sources(app PRIVATE src/main.c ../pmw3901_test/src/pmw3901.c)
target_include_directories(app PRIVATE ../pmw3901_test/src)
