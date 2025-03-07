# SPDX-License-Identifier: Apache-2.0
zephyr_library()

zephyr_library_sources(icm456xx.c)

zephyr_library_sources_ifdef(CONFIG_SPI icm456xx_spi.c)
zephyr_library_sources_ifdef(CONFIG_I2C icm456xx_i2c.c)
zephyr_library_sources_ifdef(CONFIG_ICM456XX_TRIGGER icm456xx_trigger.c)
zephyr_library_sources_ifdef(CONFIG_TDK_APEX icm456xx_apex.c)
