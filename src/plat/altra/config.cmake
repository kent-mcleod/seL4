#
# Copyright 2025, Kry10 Limited.
#
# SPDX-License-Identifier: GPL-2.0-only
#

declare_platform(altra KernelPlatformAltra PLAT_ALTRA KernelArchARM)

if(KernelPlatformAltra)
    declare_seL4_arch(aarch64)
    # TODO update this to neoverse when needed
    set(KernelArmNeoverseN1 ON)
    set(KernelArchArmV8a ON)
    set(KernelArmGicV3 ON)
    config_set(KernelARMPlatform ARM_PLAT ${KernelPlatform})
    list(APPEND KernelDTSList "tools/dts/${KernelPlatform}.dts")
    list(APPEND KernelDTSList "src/plat/altra/overlay-${KernelPlatform}.dts")
    declare_default_headers(
        TIMER_FREQUENCY 25000000
        TIMER drivers/timer/arm_generic.h
        TIMER_OVERHEAD_TICKS 1
        NUM_PPI 32
        MAX_IRQ 16383
        INTERRUPT_CONTROLLER arch/machine/gic_v3.h
        KERNEL_WCET 10u
    )
endif()

add_sources(
    DEP "KernelPlatformAltra"
    CFILES src/arch/arm/machine/gic_v3.c src/arch/arm/machine/l2c_nop.c
)
