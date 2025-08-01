# SPDX-License-Identifier: Apache-2.0
zephyr_library()

zephyr_library_sources(icm456xx.c)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/hal/tdk/icm456xx)
zephyr_library_sources_ifdef(CONFIG_SENSOR_ASYNC_API icm456xx_decoder.c)
zephyr_library_sources_ifdef(CONFIG_ICM456XX_TRIGGER icm456xx_trigger.c)
zephyr_library_sources_ifdef(CONFIG_TDK_APEX icm456xx_apex.c)
zephyr_library_sources_ifdef(CONFIG_ICM456XX_STREAM icm456xx_stream.c)

