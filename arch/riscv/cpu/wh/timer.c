/* SPDX-License-Identifier:	GPL-2.0+ */

#include <common.h>
#include <asm/io.h>
#include <asm/csr.h>
#include <asm/encoding.h>

volatile u64 starttime = 0UL;

int timer_init(void)
{
    starttime = csr_read(mcycleh);
    starttime =  csr_read(mcycle)| starttime<<32;
    return 0;
}

/*
 * return difference between timer ticks and base
 * return milliseconds
 */

ulong get_timer(ulong base)
{
    u64 now = csr_read(mcycleh);
    now = csr_read(mcycle) | now<<32;
    return (now - starttime)*1000/CONFIG_SYS_CLK_FREQ - base;
}


/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On RISC-V it just returns the timer value (uSec since boot)
 */
u64 get_ticks(void)
{
    u64 now = csr_read(mcycleh);
    now = csr_read(mcycle) | now<<32;
    return now;
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On RISC-V it returns the number of timer ticks per second.
 */
ulong get_tbclk(void)
{
    return CONFIG_SYS_CLK_FREQ;
}
