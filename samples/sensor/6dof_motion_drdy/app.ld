#include <zephyr/linker/sections.h>
#include <zephyr/devicetree.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/linker/linker-tool.h>

MEMORY
{
    /* This maps in mcuboot_primary_2 partition defined in pm_static.yaml
    * components for ORIGIN calculation:
    *  - 0x12000000: offset of QSPI external memory in SoC memory mapping.
    *  - 0x120000: mcuboot_primary_2 offset in QSPI external memory
    *  - 0x200: image header size.
    * The size of this region is size of mcuboot_primary_2 reduced by the
    * image header size.
    */
    EXTFLASH (wx) : ORIGIN = 0x10000000, LENGTH = 0x40000
}

#include <zephyr/arch/arm/cortex_m/scripts/linker.ld>