# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(icp201xx.c icp201xx_trigger.c ICP201xx/Icp201xx.c ICP201xx/Icp201xxDriver.c ICP201xx/Icp201xxSerif.c)
zephyr_library_include_directories(. ./ICP201xx)
