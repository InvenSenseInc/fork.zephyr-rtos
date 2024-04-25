# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  icm42x7x.c
  icm42x7x_spi.c
  icm42x7x_i2c.c
)
zephyr_library_include_directories(.)
zephyr_library_sources(imu/inv_imu_driver.c imu/inv_imu_transport.c)

zephyr_library_sources_ifdef(CONFIG_ICM42X7X_TRIGGER icm42x7x_trigger.c)

zephyr_library_sources_ifdef(CONFIG_ICM42X7X_APEX icm42x7x_apex.c)
zephyr_library_sources_ifdef(CONFIG_ICM42X7X_APEX imu/inv_imu_apex.c)