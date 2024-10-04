# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(icp201xx_drv.c icp201xx_trigger.c)
zephyr_library_include_directories(.)
