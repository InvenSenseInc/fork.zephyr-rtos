# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(icm42670S.c icm42670S_spi.c icm42670S_i2c.c icm42670S_trigger.c)

zephyr_library_include_directories(.)
zephyr_library_sources(imu/inv_imu_driver.c imu/inv_imu_transport.c)

zephyr_library_sources_ifdef(CONFIG_ICM42670S_APEX icm42670S_apex.c)
zephyr_library_sources_ifdef(CONFIG_ICM42670S_APEX imu/inv_imu_apex.c)