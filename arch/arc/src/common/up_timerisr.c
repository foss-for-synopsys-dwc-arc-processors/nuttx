/****************************************************************************
 * arch/arc/src/common/up_timerisr.c
 *
 *   Copyright (C) 2020 Synopsys. All rights reserved.
 *   Author: Wayne Ren <wei.ren@synopsys.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "clock/clock.h"
#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "arc/arc_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * The Clock Source: The System Tick Timer's clock source is always the core
 * clock
 */

#define SYSTICK_RELOAD ((BOARD_CORECLK_FREQ / CLK_TCK) - 1)


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  kinetis_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int arc_timerisr(int irq, uint32_t *regs, FAR void *arg)
{
  /* Process timer interrupt */
  timer_int_clear(CONFIG_SYS_TICK_ID);
  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{

  up_disable_irq(CONFIG_SYS_TICK_IRQ); /* disable os timer interrupt */
  timer_stop(CONFIG_SYS_TICK_ID);
  timer_start(CONFIG_SYS_TICK_ID, TIMER_CTRL_IE | TIMER_CTRL_NH, SYSTICK_RELOAD);

  (void)irq_attach(CONFIG_SYS_TICK_IRQ, (xcpt_t)arc_timerisr, NULL);
  up_prioritize_irq(CONFIG_SYS_TICK_IRQ, 1);
  up_enable_irq(CONFIG_SYS_TICK_IRQ);

}
