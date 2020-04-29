/****************************************************************************
 * arch/arc/include/irq.h
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARC_INCLUDE_IRQ_H
#define __ARCH_ARC_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include NuttX-specific IRQ definitions */

#include <nuttx/irq.h>

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/
#ifndef __ASSEMBLY__


/* sys timer tick irq no */
#ifndef CONFIG_SYS_TICK_IRQ
#define CONFIG_SYS_TICK_IRQ    16
#endif

#ifndef CONFIG_SYS_TICK_ID
#define CONFIG_SYS_TICK_ID     0
#endif


/* xcptcontext is the architecture specific part of thread tcb */

struct xcptcontext
{
	uint32_t sp; /* thread stack pointer */
};

static inline irqstate_t up_irq_save(void)
{
	irqstate_t v;

	__asm__ __volatile__("clri %0" : "=r" (v):: "memory");

	return v;
}

static inline void up_irq_restore(irqstate_t flags)
{
	__asm__ __volatile__("seti %0" : : "ir" (flags) : "memory");
}

#endif
#endif /* __ARCH_ARC_INCLUDE_IRQ_H */

