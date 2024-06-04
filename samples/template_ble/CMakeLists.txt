#
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

include(../common/bootloader_version.cmake)

set(mcuboot_KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/child_image/mcuboot/Kconfig.root)
set(hci_rpmsg_KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/child_image/hci_rpmsg/Kconfig.root)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sidewalk_template)

zephyr_include_directories(include)

target_sources(app PRIVATE
  src/main.c
  src/application_thread.c
  src/board_events.c
  src/sidewalk_callbacks.c
  src/u_adc_turn.c
  #src/u_spi_turn.c
  #src/u_app_timers.c
  #src/u_app_nfc.c
  src/u_app_turn.c
  src/u_buzzer_turn.c
  src/u_uart_turn.c
  src/u_led_turn.c
  src/u_i2c_turn.c
  src/u_esp32_turn.c
)
