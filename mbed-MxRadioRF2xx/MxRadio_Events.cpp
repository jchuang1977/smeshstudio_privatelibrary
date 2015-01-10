/* Copyright (c) 2011 Frank Zhao
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

#include "MxRadio.h"


void (*user_radio_error)(radio_error_t) = 0;
void (*user_radio_irq)(uint8_t) = 0;
uint8_t* (*user_radio_receive_frame)(uint8_t, uint8_t*, uint8_t,int8_t, uint8_t) = 0;
void (*user_radio_tx_done)(radio_tx_done_t) = 0;

void cMxRadio::zr_attach_error(void (*funct)(radio_error_t))
{
	user_radio_error = funct;
}

void cMxRadio::zr_attach_irq(void (*funct)(uint8_t))
{
	user_radio_irq = funct;
}

void cMxRadio::zr_attach_receive_frame(uint8_t* (cMxRadio::*funct)(uint8_t, uint8_t*, uint8_t, int8_t, uint8_t))
{
	user_radio_receive_frame = funct;
}

void cMxRadio::zr_attach_tx_done(void (cMxRadio::*funct)(radio_tx_done_t))
{
	user_radio_tx_done = funct;
}
void cMxRadio::usr_radio_error(radio_error_t err)
{
	if (user_radio_error) user_radio_error(err);
}

void cMxRadio::usr_radio_irq(uint8_t cause)
{
	if (user_radio_irq) user_radio_irq(cause);
}

uint8_t* cMxRadio::usr_radio_receive_frame(uint8_t len, uint8_t *frm, uint8_t lqi, int8_t ed,uint8_t crc_fail)
{
	if (user_radio_receive_frame)
		return (this->*user_radio_receive_frame)(len, frm, lqi,ed, crc_fail);
	else return frm;
}

void cMxRadio::usr_radio_tx_done(radio_tx_done_t status)
{
	if (user_radio_tx_done) (this->*user_radio_tx_done)(status);
}
