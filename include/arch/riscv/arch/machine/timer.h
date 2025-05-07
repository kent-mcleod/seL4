/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#ifdef CONFIG_KERNEL_MCS

#include <types.h>
#include <mode/util.h>
#include <arch/sbi.h>
#include <arch/machine/hardware.h>

static inline CONST ticks_t getKernelWcetTicks(void)
{
    /* Copied from x86_64. Hopefully it's an overestimate here. */
    return  10u;
}

static inline PURE ticks_t getTimerPrecision(void)
{
    return 1;
}

/* Get the max. ticks_t value that can be expressed in time_t (time in us). This
 * is the max. value ticksToUs() can be passed without overflowing.
 */
static inline CONST ticks_t getMaxTicksToUs(void)
{
    return UINT64_MAX;
}

/* Read the current time from the timer. */
static inline ticks_t getCurrentTime(void)
{
    return riscv_read_time();
}

/* set the next deadline irq - deadline is absolute */
static inline void setDeadline(ticks_t deadline)
{
    /* Setting the timer acknowledges any existing IRQs */
    sbi_set_timer(deadline);
}

/* ack previous deadline irq */
static inline void ackDeadlineIRQ(void)
{
}

#endif /* CONFIG_KERNEL_MCS */
