/* Copyright (c) 2007 Axel Wachtler
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

 * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
 * Neither the name of the authors nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE. */

/*
 * ===========================================================================
 * This file contains refactored code from hif_rf230.c,
 * which is part of Atmels software package "IEEE 802.15.4 MAC for AVR Z-Link"
 * ===========================================================================
 */
/*
 * Copyright (c) 2006, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* $Id$ */
/**
 * @file
 * @brief Implementation of the host interface for the AT86RF230
 *
 * Hardware interface implementation for radio-dependant functions of
 * the AT86RF230 radio chip.
 *
 */

/* === Includes ============================================================ */
#include <MxRadio.h>


/* === external functions =================================================== */
void cMxRadio::trx_io_init (int spirate)
{
	m_cs = 1;
	m_spi.format(8, 0);
	m_spi.frequency(spirate);
	irq_pin.rise(this,&cMxRadio::rf_irq_callback);

	/* set the SLPTR and RESET
	TRX_RESET_INIT();
	TRX_SLPTR_INIT();
	SPI_INIT(spirate);
	TRX_IRQ_INIT();
	EI_TRX_IRQ();
	*/
}

void cMxRadio::trx_reg_write(uint8_t addr, trx_regval_t val)
{

	addr = TRX_CMD_RW | (TRX_CMD_RADDR_MASK & addr);

	m_cs = 0;//SPI_SELN_LOW();
	m_spi.write(addr);//SPI_DATA_REG = addr;
	//SPI_WAITFOR();
	m_spi.write(val);//SPI_DATA_REG = val;
	//SPI_WAITFOR();
	m_cs = 1;//SPI_SELN_HIGH();
}

trx_regval_t cMxRadio::trx_reg_read(uint8_t addr)
{

	uint8_t val;

	addr=TRX_CMD_RR | (TRX_CMD_RADDR_MASK & addr);

	// Select transceiver
	m_cs = 0;//SPI_SELN_LOW();

	m_spi.write(addr);//SPI_DATA_REG = addr;
	//SPI_WAITFOR();
	val=m_spi.write(addr);//SPI_DATA_REG = addr;        /* dummy out */
	//SPI_WAITFOR();
	//val = SPI_DATA_REG;

	m_cs=1;//SPI_SELN_HIGH();

	return (trx_regval_t)val;
}


/* EOF */
