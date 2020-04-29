/************************************************************************************
 * arch/arc/src/nsim/nsim_uart.c
 *
 *   Copyright (C) 2020 Synopsys. All rights reserved.
 *   Author: Wayne Ren <wei.ren@synopsys.com>
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
 ************************************************************************************/
#include <stdint.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>

#include "up_internal.h"

/*
 * for nsimdrv, "nsim_mem-dev=uart0,base=0xf0000000,irq=24" is
 * used to simulate a uart.
 *
 * UART Register set (this is not a Standards Compliant IP)
 * Also each reg is Word aligned, but only 8 bits wide
 */
#define NSIM_UART_BASE_REG 0xf0000000
#define R_ID0	0
#define R_ID1	4
#define R_ID2	8
#define R_ID3	12
#define R_DATA	16
#define R_STS	20
#define R_BAUDL	24
#define R_BAUDH	28

/* Bits for UART Status Reg (R/W) */
#define RXIENB  0x04	/* Receive Interrupt Enable */
#define TXIENB  0x40	/* Transmit Interrupt Enable */

#define RXEMPTY 0x20	/* Receive FIFO Empty: No char received */
#define TXEMPTY 0x80	/* Transmit FIFO Empty, thus char can be written into */

#define RXFULL  0x08	/* Receive FIFO full */
#define RXFULL1 0x10	/* Receive FIFO has space for 1 char (tot space=4) */

#define RXFERR  0x01	/* Frame Error: Stop Bit not detected */
#define RXOERR  0x02	/* OverFlow Err: Char recv but RXFULL still set */


#define UART_REG_SET(u, r, v) ((*(uint8_t *)(u + r)) = v)
#define UART_REG_GET(u, r)    (*(uint8_t *)(u + r))

#define UART_REG_OR(u, r, v)  UART_REG_SET(u, r, UART_REG_GET(u, r) | (v))
#define UART_REG_CLR(u, r, v) UART_REG_SET(u, r, UART_REG_GET(u, r) & ~(v))

#define UART_SET_DATA(uart, val)   UART_REG_SET(uart, R_DATA, val)
#define UART_GET_DATA(uart)        UART_REG_GET(uart, R_DATA)

#define UART_CLR_STATUS(uart, val) UART_REG_CLR(uart, R_STS, val)
#define UART_GET_STATUS(uart)      UART_REG_GET(uart, R_STS)


void up_lowputc(char ch)
{
	uint32_t regs = NSIM_UART_BASE_REG;
	/* wait for transmitter to ready to accept a character */

	while (!(UART_GET_STATUS(regs) & TXEMPTY)) {
	}

	UART_SET_DATA(regs, ch);
}