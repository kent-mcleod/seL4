/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <config.h>
#include <arch/model/statedata.h>
#include <plat/machine.h>

#ifdef CONFIG_KERNEL_MCS
#include <mode/util.h>
#include <arch/kernel/apic.h>

static inline CONST ticks_t getKernelWcetTicks(void)
{
    return  10u;
}

static inline PURE ticks_t getTimerPrecision(void)
{
    return 1u;
}

static inline void ackDeadlineIRQ(void)
{
}

static inline ticks_t getCurrentTime(void)
{
    return x86_rdtsc();
}

static inline void setDeadline(ticks_t deadline)
{
    if (likely(x86KSapicRatio == 0)) {
        x86_wrmsr(IA32_TSC_DEADLINE_MSR, deadline);
    } else {
        /* Must not underflow */
        deadline -= MIN(deadline, getCurrentTime());
        /* Convert deadline from tscKhz to apic khz. Must be at least 1 tick. */
        apic_write_reg(APIC_TIMER_COUNT, MAX(1, div64(deadline, x86KSapicRatio)));
    }
}
#else
static inline void resetTimer(void)
{
    /* nothing to do */
}
#endif /* CONFIG_KERNEL_MCS */


BOOT_CODE uint32_t tsc_init(void);

