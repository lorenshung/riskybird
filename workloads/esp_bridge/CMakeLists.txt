# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(esp_bridge)

target_sources(app PRIVATE src/main.c src/load_link.c src/persist.c src/esp_safety.c)

# The motor duty-frame wire format is a contract shared with the FPGA flight
# controller and with workloads/esp_motors, so it lives in integration/ and every
# end compiles the same header rather than each keeping a copy that can drift.
#
# Added unconditionally even though only the `motors` build includes the header:
# the include path is harmless in the default build (nothing includes it, and
# HAVE_MOTORS compiles the whole motor half out), and making it conditional would
# mean the two builds disagree about their include paths, which is exactly the
# kind of difference that turns "works in the bridge build" into a link error in
# the one that drives motors.
set(RB_MOTOR_LINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lib/motor_link/include)
if(NOT EXISTS ${RB_MOTOR_LINK_DIR}/riskybird/motor_link.h)
  message(FATAL_ERROR
    "esp_bridge: the shared motor-link header is missing at ${RB_MOTOR_LINK_DIR}. "
    "It comes from workloads/lib/motor_link/ -- the same directory "
    "flight_controller --mode esp-motors passes as ROSE_MOTOR_LINK_DIR.")
endif()
target_include_directories(app PRIVATE ${RB_MOTOR_LINK_DIR})

# The motors build's duty ceiling, reachable from the build line. Same reason as
# workloads/esp_motors/CMakeLists.txt: RB_EXTRA_CFLAGS is dropped on this target
# (fpga_board=None), so a bare #ifndef in src/main.c could only ever be 25 %.
# Declared here, -DESP_LINK_MAX_DUTY_PCT=80 on the CMake line reaches the
# compiler. A cache variable: it persists in the build tree, so build a flight
# image into its own directory or with a pristine build, and read the value back
# from the boot banner ("duty ceiling NN%"). The default matches src/main.c.
set(ESP_LINK_MAX_DUTY_PCT 25 CACHE STRING
    "motors build: collective duty ceiling applied at the gate (percent)")
target_compile_definitions(app PRIVATE ESP_LINK_MAX_DUTY_PCT=${ESP_LINK_MAX_DUTY_PCT})

# Motors build: does the 120 ms silence failsafe also require a disarm -> arm
# edge before ARMED frames drive again (src/esp_safety_core.h)? 0 (default) =
# no, the next ARMED frame resumes (unlimited if the cut was < SS_CUT_COLD_MS,
# else through a cold spin-up) -- in flight the FC never disarms across a
# link gap. 1 = bench policy. Read back from the boot banner ("re-arm gate").
#   RB_CMAKE_ARGS="-DESP_LINK_REARM_AFTER_CUT=1"
set(ESP_LINK_REARM_AFTER_CUT 0 CACHE STRING
    "motors build: failsafe cut needs a re-arm edge (1/0)")
target_compile_definitions(app PRIVATE ESP_LINK_REARM_AFTER_CUT=${ESP_LINK_REARM_AFTER_CUT})

# Motors build, PWM phase interleave and spin-up gate: same mechanism, for
# bench A/B. 1 = on (default), 0 = off; read back from the boot banner
# ("PWM interleave ON/OFF", "spin-up gate ON/OFF") and RBLINK ph= / ss=. Cache
# variables persist, so build each variant pristine or into its own directory:
#   RB_CMAKE_ARGS="-DESP_LINK_PWM_INTERLEAVE=0 -DESP_LINK_SOFTSTART=0"
set(ESP_LINK_PWM_INTERLEAVE 1 CACHE STRING
    "motors build: spread the four LEDC on-edges over the period (1/0)")
set(ESP_LINK_SOFTSTART 1 CACHE STRING
    "motors build: spin-up gate on cold motors (1/0)")
target_compile_definitions(app PRIVATE
  ESP_LINK_PWM_INTERLEAVE=${ESP_LINK_PWM_INTERLEAVE}
  ESP_LINK_SOFTSTART=${ESP_LINK_SOFTSTART})
# The gate's numbers (riskybird/motor_softstart.h), passed only when given,
# e.g. -DSS_IDLE=1200 -DSS_HOLD_MS=60. Unset, the header defaults apply.
# SS_CUT_COLD_MS (300): a failsafe cut shorter than this resumes unlimited;
# one at least this long resumes through a cold spin-up.
foreach(ss_param SS_IDLE SS_HOLD_MS SS_RATE_PER_S SS_STAGGER_MS SS_MAX_MS SS_CUT_COLD_MS)
  if(DEFINED ${ss_param})
    target_compile_definitions(app PRIVATE ${ss_param}=${${ss_param}})
  endif()
endforeach()
