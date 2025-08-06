# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(quantum)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

target_sources(app PRIVATE src/main.c)

target_sources(app PRIVATE
  src/acquisition/adc.c
  src/acquisition/pulses_observer.c
  src/acquisition/controls_observer.c
  src/font/creato_display_quantum_14.c
  src/font/creato_display_quantum_22.c
  src/font/FA5_quantum_15.c
  src/font/font_fallback.c
  src/font/us101_quantum_bold_30.c
  src/font/us101_quantum_bold_53.c
  src/view/theme/dark.c
  src/view/core/bottom_bar.c
  src/view/core/navigation_triggers.c
  src/view/core/status_bar.c
  src/time/clock.c
  src/view/view.c
  src/view/dose.c
  src/view/histogram.c
  src/view/numeric.c
  src/view/gosettings.c
  src/view/settings/alarm.c
  src/view/settings/display.c
  src/view/settings/performance_mode.c
  src/view/settings/display/brightness.c
  src/view/settings/display/sleep.c
  src/view/settings/settings.c
  src/view/settings/unit.c
  src/view/spectrogram.c
  src/utils/calibration.c
  src/utils/miscellaneous.c
  src/utils/numbers.c
  src/utils/physics.c
  src/utils/system.c
  src/settings.c
)
