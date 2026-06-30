# Copyright (c) 2025 Croxel Inc.
# Copyright (c) 2025 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_include_directories(.)
zephyr_library_include_directories(${ZEPHYR_HAL_TDK_MODULE_DIR}/icm566xx)

zephyr_library_sources(
  icm566xx.c
  icm566xx_bus.c
)
zephyr_library_sources_ifdef(CONFIG_SENSOR_ASYNC_API
  icm566xx_decoder.c
)
zephyr_library_sources_ifdef(CONFIG_ICM566XX_TRIGGER
  icm566xx_trigger.c
)
zephyr_library_sources_ifdef(CONFIG_TDK_APEX
  icm566xx_apex.c
)
zephyr_library_sources_ifdef(CONFIG_ICM566XX_STREAM
  icm566xx_stream.c
)

if(CONFIG_DT_HAS_INVENSENSE_ICM56622_ENABLED)
	zephyr_compile_definitions(SENSOR_DATA_TYPE=int16_t)
	zephyr_compile_definitions(FORMAT_SENSOR_DATA=FORMAT_16BIT_REG_DATA)
	zephyr_compile_definitions(INV_IMU_WHOAMI=0xD3)
elseif(CONFIG_DT_HAS_INVENSENSE_ICM56686_ENABLED)
	zephyr_compile_definitions(SENSOR_DATA_TYPE=int32_t)
	zephyr_compile_definitions(FORMAT_SENSOR_DATA=FORMAT_20BIT_REG_DATA)
	zephyr_compile_definitions(INV_IMU_WHOAMI=0x08)
endif()
