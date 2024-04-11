# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  icm42670.c
  icm42670_spi.c
  icm42670_i2c.c
)
zephyr_library_include_directories(.)
zephyr_library_sources(imu/inv_imu_driver.c imu/inv_imu_transport.c)

zephyr_library_sources_ifdef(CONFIG_ICM42670_TRIGGER icm42670_trigger.c)

zephyr_library_sources_ifdef(CONFIG_ICM42670_APEX icm42670_apex.c)
zephyr_library_sources_ifdef(CONFIG_ICM42670_APEX imu/inv_imu_apex.c)

zephyr_library_sources_ifdef(CONFIG_ICM42670S_AML icm42670S_aml.c)
if(CONFIG_ICM42670S_AML)
  zephyr_library_include_directories(./invn.algo.sw-integration.aml-42670S-gcc-arm-none-eabi-cm4-fpu-1.6.0/include)
  zephyr_library_import(InvnAlgoAML_42670S ${CMAKE_CURRENT_SOURCE_DIR}/invn.algo.sw-integration.aml-42670S-gcc-arm-none-eabi-cm4-fpu-1.6.0/release/lib/libInvnAlgoAML_42670S.a)
endif()
