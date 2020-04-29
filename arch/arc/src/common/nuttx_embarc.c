/****************************************************************************
 * arch/arc/src/common/nuttx_embarc.h
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
#include <nuttx/config.h>
#include <nuttx/init.h>

#include "nuttx_embarc.h"
#include "embARC.h"
#include "embARC_debug.h"

/* the following variables are used for context switch
 * in irq handling
 */
/* counter for irq stack */
uint32_t g_exc_nest_count;
/* flag for context switch in irq handling */
uint32_t g_context_switch_reqflg;
/* pointer to the stack pointer of old thread */
uint32_t *g_old_thread_sp_ptr;
/* new thread stack pointer */
uint32_t g_new_thread_sp;
/* the stack used by idle thread */
#define NUTTX_IDLE_STACKSIZE  (CONFIG_IDLETHREAD_STACKSIZE >> 2)
uint32_t g_idle_thread_stack[NUTTX_IDLE_STACKSIZE];

void board_main(void)
{
	uint32_t *nx_init_stack = &g_idle_thread_stack[NUTTX_IDLE_STACKSIZE];

#if defined(__CCAC__)
/* Metaware toolchain C++ init */
	arc_mwdt_init();
#else
/* ARC GNU toolchain C++ init */
	arc_gnu_do_global_ctors_aux();
	arc_gnu_do_init_array_aux();
#endif

#ifdef CONFIG_ARCH_RAMFUNCS
  /* \todo: initialization of RAM function */
  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in SRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs
   */
#endif
	/* init core level interrupt & exception management */
	exc_int_init();
	/* init cache */
	arc_cache_init();
	/* necessary board level init */
	board_init();
	/* low level serial output setup */
	xprintf_setup();

	/* nx_start needs to be called in the stack of idle thread */
	Asm(
		"mov %%sp, %0\n"
		"jl nx_start\n"
		: : "r"(nx_init_stack)
	   );
 	/* the following codes won't be called */
#if 0
#if defined(__CCAC__)
	arc_mwdt_fini();
#else
	arc_gnu_do_global_dtors_aux();
#endif
#endif
}