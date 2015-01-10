/* THIS FILE IS GENERATED by ds2reg.py FROM INPUT Templates/at86rf212.txt */

/* Copyright (c) 2008 Axel Wachtler
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

/* $Id$ */
/**
 * @file
 * @brief AT86RF212 - 700/868/900MHz IEEE 802.15.4-2006-Transceiver.
 */
#ifndef AT86RF212_H
#define AT86RF212_H (1)

#define _BV(bit) (1 << (bit))
/* === Includes ============================================================== */

/* === Externals ============================================================= */

/* === Types ================================================================= */

typedef uint8_t trx_ramaddr_t;
typedef uint8_t trx_regval_t;
typedef uint8_t trx_regaddr_t;

/* === Macros ================================================================ */
/** Offset for register TRX_STATUS */
#define RG_TRX_STATUS (0x1)
  /** Access parameters for sub-register CCA_DONE in register TRX_STATUS */
  #define SR_CCA_DONE 0x1,0x80,7
  /** Access parameters for sub-register CCA_STATUS in register TRX_STATUS */
  #define SR_CCA_STATUS 0x1,0x40,6
  /** Access parameters for sub-register TRX_STATUS in register TRX_STATUS */
  #define SR_TRX_STATUS 0x1,0x1f,0
//for rfa1 or rfr2 with rf212
#undef P_ON
#undef BUSY_RX
#undef BUSY_TX
#undef RX_ON
#undef TRX_OFF
#undef PLL_ON
#undef TRX_SLEEP
#undef BUSY_RX_AACK
#undef BUSY_TX_ARET
#undef RX_AACK_ON
#undef TX_ARET_ON
#undef RX_ON_NOCLK
#undef RX_AACK_ON_NOCLK
#undef BUSY_RX_AACK_NOCLK

#define P_ON (0)
#define BUSY_RX (1)
#define BUSY_TX (2)
#define RX_ON (6)
#define TRX_OFF (8)
#define PLL_ON (9)
#define TRX_SLEEP (15)
#define BUSY_RX_AACK (17)
#define BUSY_TX_ARET (18)
#define RX_AACK_ON (22)
#define TX_ARET_ON (25)
#define RX_ON_NOCLK (28)
#define RX_AACK_ON_NOCLK (29)
#define BUSY_RX_AACK_NOCLK (30)
/** Offset for register TRX_STATE */
#define RG_TRX_STATE (0x2)
  /** Access parameters for sub-register TRAC_STATUS in register TRX_STATE */
  #define SR_TRAC_STATUS 0x2,0xe0,5

#undef TRAC_SUCCESS
#undef TRAC_SUCCESS_DATA_PENDING
#undef TRAC_SUCCESS_WAIT_FOR_ACK
#undef TRAC_CHANNEL_ACCESS_FAILURE
#undef TRAC_NO_ACK
#undef TRAC_INVALID

#define TRAC_SUCCESS (0)
#define TRAC_SUCCESS_DATA_PENDING (1)
#define TRAC_SUCCESS_WAIT_FOR_ACK (2)
#define TRAC_CHANNEL_ACCESS_FAILURE (3)
#define TRAC_NO_ACK (5)
#define TRAC_INVALID (7)
/** Access parameters for sub-register TRX_CMD in register TRX_STATE */
#define SR_TRX_CMD 0x2,0x1f,0

#undef CMD_NOP
#undef CMD_TX_START
#undef CMD_FORCE_TRX_OFF
#undef CMD_RX_ON
#undef CMD_TRX_OFF
#undef CMD_PLL_ON
#undef CMD_RX_AACK_ON
#undef CMD_TX_ARET_ON

#define CMD_NOP (0)
#define CMD_TX_START (2)
#define CMD_FORCE_TRX_OFF (3)
#define CMD_RX_ON (6)
#define CMD_TRX_OFF (8)
#define CMD_PLL_ON (9)
#define CMD_RX_AACK_ON (22)
#define CMD_TX_ARET_ON (25)
/** Offset for register TRX_CTRL_0 */
#define RG_TRX_CTRL_0 (0x3)
  /** Access parameters for sub-register PAD_IO in register TRX_CTRL_0 */
  #define SR_PAD_IO 0x3,0xc0,6
  /** Access parameters for sub-register PAD_IO_CLKM in register TRX_CTRL_0 */
  #define SR_PAD_IO_CLKM 0x3,0x30,4
    #define CLKM_2mA (0)
    #define CLKM_4mA (1)
    #define CLKM_6mA (2)
    #define CLKM_8mA (3)
  /** Access parameters for sub-register CLKM_SHA_SEL in register TRX_CTRL_0 */
  #define SR_CLKM_SHA_SEL 0x3,0x8,3
  /** Access parameters for sub-register CLKM_CTRL in register TRX_CTRL_0 */
  #define SR_CLKM_CTRL 0x3,0x7,0
    #define CLKM_no_clock (0)
    #define CLKM_1MHz (1)
    #define CLKM_2MHz (2)
    #define CLKM_4MHz (3)
    #define CLKM_8MHz (4)
    #define CLKM_16MHz (5)
/** Offset for register TRX_CTRL_1 */
#define RG_TRX_CTRL_1 (0x4)
  /** Access parameters for sub-register PA_EXT_EN in register TRX_CTRL_1 */
  #define SR_PA_EXT_EN 0x4,0x80,7
  /** Access parameters for sub-register IRQ_2_EXT_EN in register TRX_CTRL_1 */
  #define SR_IRQ_2_EXT_EN 0x4,0x40,6
  /** Access parameters for sub-register TX_AUTO_CRC_ON in register TRX_CTRL_1 */
  #define SR_TX_AUTO_CRC_ON 0x4,0x20,5
  /** Access parameters for sub-register RX_BL_CTRL in register TRX_CTRL_1 */
  #define SR_RX_BL_CTRL 0x4,0x10,4
  /** Access parameters for sub-register SPI_CMD_MODE in register TRX_CTRL_1 */
  #define SR_SPI_CMD_MODE 0x4,0xc,2
  /** Access parameters for sub-register IRQ_POLARITY in register TRX_CTRL_1 */
  #define SR_IRQ_POLARITY 0x4,0x1,0
  /** Access parameters for sub-register IRQ_MASK_MODE in register TRX_CTRL_1 */
  #define SR_IRQ_MASK_MODE 0x4,0x2,1
/** Offset for register PHY_TX_PWR */
#define RG_PHY_TX_PWR (0x5)
  /** Access parameters for sub-register PA_BOOST in register PHY_TX_PWR */
  #define SR_PA_BOOST 0x5,0x80,7
  /** Access parameters for sub-register GC_PA in register PHY_TX_PWR */
  #define SR_GC_PA 0x5,0x60,5
  /** Access parameters for sub-register TX_PWR in register PHY_TX_PWR */
  #define SR_TX_PWR 0x5,0xff,0
/** Offset for register PHY_RSSI */
#define RG_PHY_RSSI (0x6)
  /** Access parameters for sub-register RX_CRC_VALID in register PHY_RSSI */
  #define SR_RX_CRC_VALID 0x6,0x80,7
  /** Access parameters for sub-register RND_VALUE in register PHY_RSSI */
  #define SR_RND_VALUE 0x6,0x60,5
  /** Access parameters for sub-register RSSI in register PHY_RSSI */
  #define SR_RSSI 0x6,0x1f,0
/** Offset for register PHY_ED_LEVEL */
#define RG_PHY_ED_LEVEL (0x7)
  /** Access parameters for sub-register ED_LEVEL in register PHY_ED_LEVEL */
  #define SR_ED_LEVEL 0x7,0xff,0
/** Offset for register PHY_CC_CCA */
#define RG_PHY_CC_CCA (0x8)
  /** Access parameters for sub-register CCA_REQUEST in register PHY_CC_CCA */
  #define SR_CCA_REQUEST 0x8,0x80,7
  /** Access parameters for sub-register CCA_MODE in register PHY_CC_CCA */
  #define SR_CCA_MODE 0x8,0x60,5
  /** Access parameters for sub-register CHANNEL in register PHY_CC_CCA */
  #define SR_CHANNEL 0x8,0x1f,0
/** Offset for register CCA_THRES */
#define RG_CCA_THRES (0x9)
  /** Access parameters for sub-register CCA_ED_THRES in register CCA_THRES */
  #define SR_CCA_ED_THRES 0x9,0xf,0
/** Offset for register SFD_VALUE */
#define RG_SFD_VALUE (0xb)
  /** Access parameters for sub-register SFD_VALUE in register SFD_VALUE */
  #define SR_SFD_VALUE 0xb,0xff,0
/** Offset for register TRX_CTRL_2 */
#define RG_TRX_CTRL_2 (0xc)
  /** Access parameters for sub-register RX_SAFE_MODE in register TRX_CTRL_2 */
  #define SR_RX_SAFE_MODE 0xc,0x80,7
  /** Access parameters for sub-register TRX_OFF_AVDD_EN in register TRX_CTRL_2 */
  #define SR_TRX_OFF_AVDD_EN 0xc,0x40,6
/** Access parameters for sub-register TRX_SUB1_RC_EN in register TRX_CTRL_2 */
  #define SR_OQPSK_SUB1_RC_EN 0xc,0x10,4
  /** Access parameters for sub-register BPSK_OQPSK in register TRX_CTRL_2 */
  #define SR_BPSK_OQPSK 0xc,0x8,3
  /** Access parameters for sub-register SUB_MODE in register TRX_CTRL_2 */
  #define SR_SUB_MODE 0xc,0x4,2
  /** Access parameters for sub-register OQPSK_DATA_RATE in register TRX_CTRL_2 */
  #define SR_OQPSK_DATA_RATE 0xc,0x3,0
/** Offset for register ANT_DIV */
#define RG_ANT_DIV (0xd)
  /** Access parameters for sub-register ANT_SEL in register ANT_DIV */
  #define SR_ANT_SEL 0xd,0x80,7
  /** Access parameters for sub-register ANT_EXT_SW_EN in register ANT_DIV */
  #define SR_ANT_EXT_SW_EN 0xd,0x4,2
  /** Access parameters for sub-register ANT_CTRL in register ANT_DIV */
  #define SR_ANT_CTRL 0xd,0x3,0
/** Offset for register IRQ_MASK */
#define RG_IRQ_MASK (0xe)
  /** Access parameters for sub-register MASK_BAT_LOW in register IRQ_MASK */
  #define SR_MASK_BAT_LOW 0xe,0x80,7
  /** Access parameters for sub-register MASK_TRX_UR in register IRQ_MASK */
  #define SR_MASK_TRX_UR 0xe,0x40,6
  /** Access parameters for sub-register MASK_AMI in register IRQ_MASK */
  #define SR_MASK_AMI 0xe,0x20,5
  /** Access parameters for sub-register MASK_CCA_ED_READY in register IRQ_MASK */
  #define SR_MASK_CCA_ED_READY 0xe,0x10,4
  /** Access parameters for sub-register MASK_TRX_END in register IRQ_MASK */
  #define SR_MASK_TRX_END 0xe,0x8,3
  /** Access parameters for sub-register MASK_RX_START in register IRQ_MASK */
  #define SR_MASK_RX_START 0xe,0x4,2
  /** Access parameters for sub-register MASK_PLL_LOCK in register IRQ_MASK */
  #define SR_MASK_PLL_LOCK 0xe,0x1,0
  /** Access parameters for sub-register MASK_PLL_UNLOCK in register IRQ_MASK */
  #define SR_MASK_PLL_UNLOCK 0xe,0x2,1
/** Offset for register IRQ_STATUS */
#define RG_IRQ_STATUS (0xf)
  /** Access parameters for sub-register BAT_LOW in register IRQ_STATUS */
  #define SR_BAT_LOW 0xf,0x80,7
  /** Access parameters for sub-register TRX_UR in register IRQ_STATUS */
  #define SR_TRX_UR 0xf,0x40,6
  /** Access parameters for sub-register AMI in register IRQ_STATUS */
  #define SR_AMI 0xf,0x20,5
  /** Access parameters for sub-register CCA_ED_READY in register IRQ_STATUS */
  #define SR_CCA_ED_READY 0xf,0x10,4
  /** Access parameters for sub-register TRX_END in register IRQ_STATUS */
  #define SR_TRX_END 0xf,0x8,3
  /** Access parameters for sub-register RX_START in register IRQ_STATUS */
  #define SR_RX_START 0xf,0x4,2
  /** Access parameters for sub-register PLL_LOCK in register IRQ_STATUS */
  #define SR_PLL_LOCK 0xf,0x1,0
  /** Access parameters for sub-register PLL_UNLOCK in register IRQ_STATUS */
  #define SR_PLL_UNLOCK 0xf,0x2,1
/** Offset for register VREG_CTRL */
#define RG_VREG_CTRL (0x10)
  /** Access parameters for sub-register AVREG_EXT in register VREG_CTRL */
  #define SR_AVREG_EXT 0x10,0x80,7
  /** Access parameters for sub-register AVDD_OK in register VREG_CTRL */
  #define SR_AVDD_OK 0x10,0x40,6
  /** Access parameters for sub-register DVREG_EXT in register VREG_CTRL */
  #define SR_DVREG_EXT 0x10,0x8,3
  /** Access parameters for sub-register DVDD_OK in register VREG_CTRL */
  #define SR_DVDD_OK 0x10,0x4,2
/** Offset for register BATMON */
#define RG_BATMON (0x11)
  /** Access parameters for sub-register BATMON_OK in register BATMON */
  #define SR_BATMON_OK 0x11,0x20,5
  /** Access parameters for sub-register BATMON_HR in register BATMON */
  #define SR_BATMON_HR 0x11,0x10,4
  /** Access parameters for sub-register BATMON_VTH in register BATMON */
  #define SR_BATMON_VTH 0x11,0xf,0
/** Offset for register XOSC_CTRL */
#define RG_XOSC_CTRL (0x12)
  /** Access parameters for sub-register XTAL_MODE in register XOSC_CTRL */
  #define SR_XTAL_MODE 0x12,0xf0,4
  /** Access parameters for sub-register XTAL_TRIM in register XOSC_CTRL */
  #define SR_XTAL_TRIM 0x12,0xf,0
/** Offset for register CC_CTRL_0 */
#define RG_CC_CTRL_0 (0x13)
  /** Access parameters for sub-register CC_NUMBER in register CC_CTRL_0 */
  #define SR_CC_NUMBER 0x13,0xff,0
/** Offset for register CC_CTRL_1 */
#define RG_CC_CTRL_1 (0x14)
  /** Access parameters for sub-register CC_BAND in register CC_CTRL_1 */
  #define SR_CC_BAND 0x14,0x4,2
  /** Access parameters for sub-register BAND in register CC_CTRL_1 */
  #define SR_BAND 0x14,0x1,0
  /** Access parameters for sub-register CC_ in register CC_CTRL_1 */
  #define SR_CC_ 0x14,0x2,1
/** Offset for register RX_SYN */
#define RG_RX_SYN (0x15)
  /** Access parameters for sub-register RX_PDT_DIS in register RX_SYN */
  #define SR_RX_PDT_DIS 0x15,0x80,7
  /** Access parameters for sub-register RX_PDT_LEVEL in register RX_SYN */
  #define SR_RX_PDT_LEVEL 0x15,0xf,0
/** Offset for register RF_CTRL_0 */
#define RG_RF_CTRL_0 (0x16)
  /** Access parameters for sub-register PA_LT in register RF_CTRL_0 */
  #define SR_PA_LT 0x16,0xc0,6
  /** Access parameters for sub-register GC_TX_OFFS in register RF_CTRL_0 */
  #define SR_GC_TX_OFFS 0x16,0x3,0
/** Offset for register XAH_CTRL_1 */
#define RG_XAH_CTRL_1 (0x17)
  /** Access parameters for sub-register CSMA_LBT_MODE in register XAH_CTRL_1 */
  #define SR_CSMA_LBT_MODE 0x17,0x80,7
  /** Access parameters for sub-register AACK_FLTR_RES_FT in register XAH_CTRL_1 */
  #define SR_AACK_FLTR_RES_FT 0x17,0x20,5
  /** Access parameters for sub-register AACK_UPLD_RES_FT in register XAH_CTRL_1 */
  #define SR_AACK_UPLD_RES_FT 0x17,0x10,4
  /** Access parameters for sub-register AACK_ACK_TIME in register XAH_CTRL_1 */
  #define SR_AACK_ACK_TIME 0x17,0x4,2
  /** Access parameters for sub-register AACK_PROM_MODE in register XAH_CTRL_1 */
  #define SR_AACK_PROM_MODE 0x17,0x2,1
/** Offset for register FTN_CTRL */
#define RG_FTN_CTRL (0x18)
  /** Access parameters for sub-register FTN_START in register FTN_CTRL */
  #define SR_FTN_START 0x18,0x80,7
/** Offset for register RF_CTRL_1 */
#define RG_RF_CTRL_1 (0x19)
  /** Access parameters for sub-register RF_MC in register RF_CTRL_1 */
  #define SR_RF_MC 0x19,0xf0,4
/** Offset for register PLL_CF */
#define RG_PLL_CF (0x1a)
  /** Access parameters for sub-register PLL_CF_START in register PLL_CF */
  #define SR_PLL_CF_START 0x1a,0x80,7
/** Offset for register PLL_DCU */
#define RG_PLL_DCU (0x1b)
  /** Access parameters for sub-register PLL_DCU_START in register PLL_DCU */
  #define SR_PLL_DCU_START 0x1b,0x80,7
/** Offset for register PART_NUM */
#define RG_PART_NUM (0x1c)
  /** Access parameters for sub-register PART_NUM in register PART_NUM */
  #define SR_PART_NUM 0x1c,0xff,0
    #define RF212A_PART_NUM (7)
/** Offset for register VERSION_NUM */
#define RG_VERSION_NUM (0x1d)
  /** Access parameters for sub-register VERSION_NUM in register VERSION_NUM */
  #define SR_VERSION_NUM 0x1d,0xff,0
    #define RF212A_VERSION_NUM (1)
/** Offset for register MAN_ID_0 */
#define RG_MAN_ID_0 (0x1e)
  /** Access parameters for sub-register MAN_ID_0 in register MAN_ID_0 */
  #define SR_MAN_ID_0 0x1e,0xff,0
/** Offset for register MAN_ID_1 */
#define RG_MAN_ID_1 (0x1f)
  /** Access parameters for sub-register MAN_ID_1 in register MAN_ID_1 */
  #define SR_MAN_ID_1 0x1f,0xff,0
/** Offset for register SHORT_ADDR_0 */
#define RG_SHORT_ADDR_0 (0x20)
  /** Access parameters for sub-register SHORT_ADDR_0 in register SHORT_ADDR_0 */
  #define SR_SHORT_ADDR_0 0x20,0xff,0
/** Offset for register SHORT_ADDR_1 */
#define RG_SHORT_ADDR_1 (0x21)
  /** Access parameters for sub-register SHORT_ADDR_1 in register SHORT_ADDR_1 */
  #define SR_SHORT_ADDR_1 0x21,0xff,0
/** Offset for register PAN_ID_0 */
#define RG_PAN_ID_0 (0x22)
  /** Access parameters for sub-register PAN_ID_0 in register PAN_ID_0 */
  #define SR_PAN_ID_0 0x22,0xff,0
/** Offset for register PAN_ID_1 */
#define RG_PAN_ID_1 (0x23)
  /** Access parameters for sub-register PAN_ID_1 in register PAN_ID_1 */
  #define SR_PAN_ID_1 0x23,0xff,0
/** Offset for register IEEE_ADDR_0 */
#define RG_IEEE_ADDR_0 (0x24)
  /** Access parameters for sub-register IEEE_ADDR_0 in register IEEE_ADDR_0 */
  #define SR_IEEE_ADDR_0 0x24,0xff,0
/** Offset for register IEEE_ADDR_1 */
#define RG_IEEE_ADDR_1 (0x25)
  /** Access parameters for sub-register IEEE_ADDR_1 in register IEEE_ADDR_1 */
  #define SR_IEEE_ADDR_1 0x25,0xff,0
/** Offset for register IEEE_ADDR_2 */
#define RG_IEEE_ADDR_2 (0x26)
  /** Access parameters for sub-register IEEE_ADDR_2 in register IEEE_ADDR_2 */
  #define SR_IEEE_ADDR_2 0x26,0xff,0
/** Offset for register IEEE_ADDR_3 */
#define RG_IEEE_ADDR_3 (0x27)
  /** Access parameters for sub-register IEEE_ADDR_3 in register IEEE_ADDR_3 */
  #define SR_IEEE_ADDR_3 0x27,0xff,0
/** Offset for register IEEE_ADDR_4 */
#define RG_IEEE_ADDR_4 (0x28)
  /** Access parameters for sub-register IEEE_ADDR_4 in register IEEE_ADDR_4 */
  #define SR_IEEE_ADDR_4 0x28,0xff,0
/** Offset for register IEEE_ADDR_5 */
#define RG_IEEE_ADDR_5 (0x29)
  /** Access parameters for sub-register IEEE_ADDR_5 in register IEEE_ADDR_5 */
  #define SR_IEEE_ADDR_5 0x29,0xff,0
/** Offset for register IEEE_ADDR_6 */
#define RG_IEEE_ADDR_6 (0x2a)
  /** Access parameters for sub-register IEEE_ADDR_6 in register IEEE_ADDR_6 */
  #define SR_IEEE_ADDR_6 0x2a,0xff,0
/** Offset for register IEEE_ADDR_7 */
#define RG_IEEE_ADDR_7 (0x2b)
  /** Access parameters for sub-register IEEE_ADDR_7 in register IEEE_ADDR_7 */
  #define SR_IEEE_ADDR_7 0x2b,0xff,0
/** Offset for register XAH_CTRL_0 */
#define RG_XAH_CTRL_0 (0x2c)
  /** Access parameters for sub-register MAX_FRAME_RETRIES in register XAH_CTRL_0 */
  #define SR_MAX_FRAME_RETRIES 0x2c,0xf0,4
  /** Access parameters for sub-register SLOTTED_OPERATION in register XAH_CTRL_0 */
  #define SR_SLOTTED_OPERATION 0x2c,0x1,0
  /** Access parameters for sub-register MAX_CSMA_RETRIES in register XAH_CTRL_0 */
  #define SR_MAX_CSMA_RETRIES 0x2c,0xe,1
/** Offset for register CSMA_SEED_0 */
#define RG_CSMA_SEED_0 (0x2d)
  /** Access parameters for sub-register CSMA_SEED_0 in register CSMA_SEED_0 */
  #define SR_CSMA_SEED_0 0x2d,0xff,0
/** Offset for register CSMA_SEED_1 */
#define RG_CSMA_SEED_1 (0x2e)
  /** Access parameters for sub-register AACK_FVN_MODE in register CSMA_SEED_1 */
  #define SR_AACK_FVN_MODE 0x2e,0xc0,6
  /** Access parameters for sub-register AACK_SET_PD in register CSMA_SEED_1 */
  #define SR_AACK_SET_PD 0x2e,0x20,5
  /** Access parameters for sub-register AACK_DIS_ACK in register CSMA_SEED_1 */
  #define SR_AACK_DIS_ACK 0x2e,0x10,4
  /** Access parameters for sub-register AACK_I_AM_COORD in register CSMA_SEED_1 */
  #define SR_AACK_I_AM_COORD 0x2e,0x8,3
  /** Access parameters for sub-register CSMA_SEED_1 in register CSMA_SEED_1 */
  #define SR_CSMA_SEED_1 0x2e,0x7,0
/** Offset for register CSMA_BE */
#define RG_CSMA_BE (0x2f)
  /** Access parameters for sub-register MAX_BE in register CSMA_BE */
  #define SR_MAX_BE 0x2f,0xf0,4
  /** Access parameters for sub-register MIN_BE in register CSMA_BE */
  #define SR_MIN_BE 0x2f,0xf,0
/** name string of the radio */
#define RADIO_NAME "AT86RF212"
/** contents of the RG_PART_NUM register */
#define RADIO_PART_NUM (RF212A_PART_NUM)
/** contents of the RG_VERSION_NUM register */
#define RADIO_VERSION_NUM (RF212A_VERSION_NUM)

/** SPI command code for register write */
#define TRX_CMD_RW           (_BV(7) | _BV(6))
/** SPI command code for register read */
#define TRX_CMD_RR           (_BV(7))
/** SPI command code for frame write */
#define TRX_CMD_FW           (_BV(6) | _BV(5))
/** SPI command code for frame read */
#define TRX_CMD_FR           (_BV(5))
/** SPI command code for sram write */
#define TRX_CMD_SW           (_BV(6))
/** SPI command code for sram read */
#define TRX_CMD_SR           (0)

#define TRX_CMD_RADDR_MASK   (0x3f)

/** duration while reset=low is asserted */
#define TRX_RESET_TIME_US    (6)

/** duration transceiver reaches TRX_OFF for the first time */
#define TRX_INIT_TIME_US     (510)

/** maximum duration, which PLL needs to lock */
#define TRX_PLL_LOCK_TIME_US     (180)


/** duration of a CCA measurement */
#define TRX_CCA_TIME_US     (140)

/** Mask for PLL lock interrupt */
#define TRX_IRQ_PLL_LOCK   _BV(0)

/** Mask for PLL unlock interrupt */
#define TRX_IRQ_PLL_UNLOCK _BV(1)

/** Mask for RX Start interrupt */
#define TRX_IRQ_RX_START   _BV(2)

/** Mask for RX/TX end interrupt */
#define TRX_IRQ_TRX_END    _BV(3)

/** Mask for CCA_ED interrupt */
#define TRX_IRQ_CCA_ED     _BV(4)

/** Mask for AMI interrupt */
#define TRX_IRQ_AMI        _BV(5)

/** Mask for RX/TX underrun interrupt */
#define TRX_IRQ_UR         _BV(6)

/** Mask for battery low interrupt */
#define TRX_IRQ_BAT_LOW    _BV(7)

/** TX ARET status for successful transmission */
#define TRAC_SUCCESS (0)
/** TX ARET status for unsuccessful transmission due to no channel access */
#define TRAC_CHANNEL_ACCESS_FAILURE (3)
/** TX ARET status for unsuccessful transmission due no ACK frame was received */
#define TRAC_NO_ACK (5)


/** lowest supported channel number */
#define TRX_MIN_CHANNEL (0)

/** highest supported channel number */
#ifdef CHINABAND
#define TRX_MAX_CHANNEL (3)

/** number of channels */
#define TRX_NB_CHANNELS (4)
#define TRX_SUPPORTED_CHANNELS  (0x000000fUL)
#else
#define TRX_MAX_CHANNEL (10)

/** number of channels */
#define TRX_NB_CHANNELS (11)
#define TRX_SUPPORTED_CHANNELS  (0x00007ffUL)
#endif
/**
 * @brief Mask for supported channels of this radio.
 * The AT86RF212 supports channels 0 ... 10 of IEEE 802.15.4
 * (currently no support for free configurable frequencies here)
 */


/**
 * @brief Mask for supported channel pages (a.k.a. modulation schemes) of this radio.
 * The AT86RF230 supports channel page ???? OQPSK_250
 */
#define TRX_SUPPORTED_PAGES     (42)
#define TRX_SUPPORTS_BAND_700 (1)
#define TRX_SUPPORTS_BAND_800 (1)
#define TRX_SUPPORTS_BAND_900 (1)

/** Rate code for BPSK20, xx kchip/s, yy kbit/s */
#define TRX_BPSK20    (0)

/** Rate code for BPSK40, xx kchip/s, yy kbit/s */
#define TRX_BPSK40    (4)

/** Rate code for OQPSK100, xx kchip/s, yy kbit/s */
#define TRX_OQPSK100  (8)

/** Rate code for OQPSK200, xx kchip/s, yy kbit/s */
#define TRX_OQPSK200  (9)

/** Rate code for OQPSK400, xx kchip/s, yy kbit/s */
#define TRX_OQPSK400  (10)
#define TRX_OQPSK400_1  (11)

/** Rate code for OQPSK250, xx kchip/s, yy kbit/s */
#ifdef CHINABAND
#define TRX_OQPSK250  (28)
#define CCBAND  (4)
#define CCNUMBER  (11)
#else
#define TRX_OQPSK250  (12)
#endif

/** Rate code for OQPSK500, xx kchip/s, yy kbit/s */
#define TRX_OQPSK500  (13)

/** Rate code for OQPSK1000, xx kchip/s, yy kbit/s */
#define TRX_OQPSK1000 (14)
#define TRX_OQPSK1000_1 (15)
/** undefined data rate */
#define TRX_NONE      (255)

#endif /* ifndef AT86RF212_H */
