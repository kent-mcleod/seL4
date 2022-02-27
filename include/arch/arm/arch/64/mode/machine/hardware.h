/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <config.h>
#include <sel4/sel4_arch/constants.h>

#define PAGE_BITS seL4_PageBits

/* Extract the n-level PT index from a virtual address.
 * There are either 3 or 4 page table levels depending on whether the address range
 * being translated is 48bits, 44 bits or 40 bits.
 */
#ifdef AARCH64_VSPACE_S2_START_L1
#define PT_LEVELS 3
#else
#define PT_LEVELS 4
#endif
#define GET_PT_INDEX(addr, n)  (((addr) >> (((PT_INDEX_BITS) * (((PT_LEVELS) - 1) - (n))) + seL4_PageBits)) & MASK(PT_INDEX_BITS))
#define GET_LVL_PGSIZE_BITS(n) (((PT_INDEX_BITS) * (((PT_LEVELS) - 1) - (n))) + seL4_PageBits)
#define GET_LVL_PGSIZE(n)      BIT(GET_LVL_PGSIZE_BITS((n)))

/* Control register fields */
#define CONTROL_M         0  /* MMU enable */
#define CONTROL_A         1  /* Alignment check enable */
#define CONTROL_C         2  /* Cacheability control, for data caching */
#define CONTROL_SA0       4  /* Stack Alignment Check Enable for EL0 */
#define CONTROL_SA        3  /* Stack Alignment Check for EL1 */
#define CONTROL_I         12 /* Instruction access Cacheability control */
#define CONTROL_E0E       24 /* Endianness of data accesses at EL0 */
#define CONTROL_EE        25 /* Endianness of data accesses at EL1 */

#ifndef __ASSEMBLER__

#include <arch/types.h>

enum vm_page_size {
    ARMSmallPage,
    ARMLargePage,
    ARMHugePage
};
typedef word_t vm_page_size_t;

enum frameSizeConstants {
    ARMSmallPageBits    = seL4_PageBits,
    ARMLargePageBits    = seL4_LargePageBits,
    ARMHugePageBits     = seL4_HugePageBits
};

static inline word_t CONST pageBitsForSize(vm_page_size_t pagesize)
{
    switch (pagesize) {
    case ARMSmallPage:
        return ARMSmallPageBits;

    case ARMLargePage:
        return ARMLargePageBits;

    case ARMHugePage:
        return ARMHugePageBits;

    default:
        fail("Invalid page size");
    }
}

#endif /* __ASSEMBLER__ */

