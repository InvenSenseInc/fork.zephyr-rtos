#
# Copyright (c) 2024 TDK Invensense
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(6dof_motion_drdy)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources} gaf/release/lib/libInvnAlgoGAFOpen.a.obj)

set(gaf_src_dir   ${CMAKE_CURRENT_SOURCE_DIR}/gaf)
set(GAF_LIB_DIR     ${gaf_src_dir}/release/lib)
set(GAF_INCLUDE_DIR ${gaf_src_dir}/include)

target_include_directories(app PUBLIC ${GAF_INCLUDE_DIR})

# Relocate library code
zephyr_code_relocate(FILES src/invn_algo.c LOCATION EXTFLASH_TEXT NOCOPY)
zephyr_code_relocate(FILES src/invn_algo.c LOCATION RAM_DATA)

zephyr_code_relocate(FILES gaf/release/lib/libInvnAlgoGAFOpen.a.obj LOCATION EXTFLASH_TEXT NOCOPY)
zephyr_code_relocate(FILES gaf/release/lib/libInvnAlgoGAFOpen.a.obj LOCATION RAM_DATA)


