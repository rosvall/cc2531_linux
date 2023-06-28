// SPDX-FileCopyrightText: 2023 Andreas Sig Rosvall
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef _CC2531_REG__H
#define _CC2531_REG__H

#define FOR_EACH_CC2531_REG(FN)             \
	FN(0x6000, 0, (8*128 - 1), RXFIFO)                                                                                                                                                                         \
	FN(0x6080, 0, (8*128 - 1), TXFIFO) \
	FN(0x6100, 0, (8*96 - 1), SRCTABLE)  /* TODO: Source matching address table */                                                                                                                                                      \
	FN(0x6160, 0, 23, SRCRESMASK)  /* 24-bit mask that indicates source address match for each individual entry in the source address table */                                                                                    \
	FN(0x6163, 0, 7, SRCRESINDEX)  /* The bit index of the least-significant 1 in SRCRESMASK, or 0x3F when there is no source match */                                                                                            \
	FN(0x6164, 0, 23, SRCEXTPENDEN)  /* The 24-bit mask that enables and disables automatic pending for each of the 12 extended addresses. Entry n is mapped to SRCEXTPENDEN[2n].  All SRCEXTPENDEN[2n + 1] bits are don't care. */ \
	FN(0x6167, 0, 23, SRCSHORTPENDEN)  /* The 24-bit mask that enables and disables automatic pending for each of the 24 short addresses */                                                                                           \
	FN(0x616A, 0, 63, EXT_ADD)  /* The IEEE extended address used during destination address filtering (little endian) */                                                                                                      \
	FN(0x6172, 0, 15, PAN_ID)  /* The PAN ID used during destination address filtering (little endian) */                                                                                                                     \
	FN(0x6174, 0, 15, SHORT_ADDR)  /* The short address used during destination address filtering (little endian) */                                                                                                              \
	FN(0x6180, 0, 0, FRAME_FILTER_EN)         \
	FN(0x6180, 0, 7, FRMFILT0)  /* Frame Filtering */                                                                                                                                                                          \
	FN(0x6180, 1, 1, PAN_COORDINATOR)         \
	FN(0x6180, 2, 3, MAX_FRAME_VERSION)       \
	FN(0x6180, 4, 6, FCF_RESERVED_MASK)       \
	FN(0x6181, 0, 0, FRMFILT1_RESERVED)       \
	FN(0x6181, 0, 7, FRMFILT1)  /* Frame Filtering */                                                                                                                                                                          \
	FN(0x6181, 1, 2, MODIFY_FT_FILTER)        \
	FN(0x6181, 3, 3, ACCEPT_FT_0_BEACON)      \
	FN(0x6181, 4, 4, ACCEPT_FT_1_DATA)        \
	FN(0x6181, 5, 5, ACCEPT_FT_2_ACK)         \
	FN(0x6181, 6, 6, ACCEPT_FT_3_MAC_CMD)     \
	FN(0x6181, 7, 7, ACCEPT_FT_4TO7_RESERVED) \
	FN(0x6182, 0, 0, SRC_MATCH_EN)            \
	FN(0x6182, 0, 7, SRCMATCH)  /* Source Address Matching and Pending Bits */                                                                                                                                                 \
	FN(0x6182, 1, 1, AUTOPEND)                \
	FN(0x6182, 2, 2, PEND_DATAREQ_ONLY)       \
	FN(0x6182, 3, 7, SRCMATCH_RESERVED)       \
	FN(0x6183, 0, 23, SRCSHORTEN)  /* Short Address Matching */                                                                                                                                                                   \
	FN(0x6186, 0, 23, SRCEXTEN)  /* Extended Address Matching */                                                                                                                                                                \
	FN(0x6189, 0, 1, TX_MODE)                 \
	FN(0x6189, 0, 7, FRMCTRL0)  /* Frame Handling */                                                                                                                                                                           \
	FN(0x6189, 2, 3, RX_MODE)                 \
	FN(0x6189, 4, 4, ENERGY_SCAN)             \
	FN(0x6189, 5, 5, AUTOACK)                 \
	FN(0x6189, 6, 6, AUTOCRC)                 \
	FN(0x6189, 7, 7, APPEND_DATA_MODE)        \
	FN(0x618A, 0, 0, SET_RXENMASK_ON_TX)      \
	FN(0x618A, 0, 7, FRMCTRL1)  /* Frame Handling */                                                                                                                                                                           \
	FN(0x618A, 1, 1, IGNORE_TX_UNDERF)        \
	FN(0x618A, 2, 2, PENDING_OR)              \
	FN(0x618B, 0, 7, RXENABLE)  /* RX Enabling */                                                                                                                                                                              \
	FN(0x618C, 0, 7, RXMASKSET)  /* RX Enabling */                                                                                                                                                                              \
	FN(0x618D, 0, 7, RXMASKCLR)  /* RX Disabling */                                                                                                                                                                             \
	FN(0x618F, 0, 7, FREQCTRL)  /* Controls the RF Frequency */                                                                                                                                                                \
	FN(0x6190, 0, 7, TXPOWER)  /* Controls the TX Settings */                                                                                                                                                                 \
	FN(0x6191, 0, 1, TXMIX_CURRENT)           \
	FN(0x6191, 0, 7, TXCTRL)  /* Controls the Output Power */                                                                                                                                                                \
	FN(0x6191, 2, 3, DAC_DC)                  \
	FN(0x6191, 4, 6, DAC_CURR)                \
	FN(0x6193, 0, 7, FSMSTAT1)  /* Radio Status Register */                                                                                                                                                                    \
	FN(0x6194, 0, 7, FIFOPCTRL)  /* FIFOP Threshold */                                                                                                                                                                          \
	FN(0x6195, 0, 0, RX2RX_TIME_OFF)          \
	FN(0x6195, 0, 7, FSMCTRL)  /* FSM Options */                                                                                                                                                                              \
	FN(0x6195, 1, 1, SLOTTED_ACK)             \
	FN(0x6196, 0, 7, CCACTRL0)  /* CCA Threshold */                                                                                                                                                                            \
	FN(0x6196, 0, 7, CCA_THR)                 \
	FN(0x6197, 0, 2, CCA_HYST)                \
	FN(0x6197, 0, 7, CCACTRL1)  /* Other CCA Options */                                                                                                                                                                        \
	FN(0x6197, 3, 4, CCA_MODE)                \
	FN(0x6198, 0, 7, RSSI)  /* RSSI Status Register */                                                                                                                                                                     \
	FN(0x6199, 0, 7, RSSISTAT)  /* RSSI Valid Status Register */                                                                                                                                                               \
	FN(0x619A, 0, 7, RXFIRST)  /* First Byte in RXFIFO */                                                                                                                                                                     \
	FN(0x619B, 0, 7, RXFIFOCNT)  /* Number of Bytes in RXFIFO */                                                                                                                                                                \
	FN(0x619C, 0, 7, TXFIFOCNT)  /* Number of Bytes in TXFIFO */                                                                                                                                                                \
	FN(0x619D, 0, 7, RXFIRST_PTR)  /* RXFIFO Pointer */                                                                                                                                                                           \
	FN(0x619E, 0, 7, RXLAST_PTR)  /* RXFIFO Pointer */                                                                                                                                                                           \
	FN(0x619F, 0, 7, RXP1_PTR)  /* RXFIFO Pointer */                                                                                                                                                                           \
	FN(0x61A1, 0, 7, TXFIRST_PTR)  /* TXFIFO Pointer */                                                                                                                                                                           \
	FN(0x61A2, 0, 7, TXLAST_PTR)  /* TXFIFO Pointer */                                                                                                                                                                           \
	FN(0x61A3, 0, 7, RFIRQM0)  /* RF Interrupt Masks */                                                                                                                                                                       \
	FN(0x61A4, 0, 7, RFIRQM1)  /* RF Interrupt Masks */                                                                                                                                                                       \
	FN(0x61A5, 0, 7, RFERRM)  /* RF Error Interrupt Masks */                                                                                                                                                                 \
	FN(0x61A6, 0, 7, OPAMPMC)  /* Operational Amplifier Mode Control */                                                                                                                                                       \
	FN(0x61A7, 0, 7, RFRND)  /* Random Data */                                                                                                                                                                              \
	FN(0x61A8, 0, 7, MDMCTRL0)  /* Controls Modem */                                                                                                                                                                           \
	FN(0x61A9, 0, 7, MDMCTRL1)  /* Controls Modem */                                                                                                                                                                           \
	FN(0x61AA, 0, 7, FREQEST)  /* Estimated RF Frequency Offset */                                                                                                                                                            \
	FN(0x61AB, 0, 1, MIX_CURRENT)             \
	FN(0x61AB, 0, 7, RXCTRL)  /* Tune Receive Section */                                                                                                                                                                     \
	FN(0x61AB, 2, 3, GBIAS_LNA_REF)           \
	FN(0x61AB, 4, 5, GBIAS_LNA2_REF)          \
	FN(0x61AC, 0, 1, LODIV_CURRENT)           \
	FN(0x61AC, 0, 7, FSCTRL)  /* Tune Frequency Synthesizer */                                                                                                                                                               \
	FN(0x61AC, 2, 3, LODIV_BUF_CURRENT_RX)    \
	FN(0x61AC, 4, 5, LODIV_BUF_CURRENT_TX)    \
	FN(0x61AC, 6, 7, PRE_CURRENT)             \
	FN(0x61AE, 0, 1, VCO_CURR)                \
	FN(0x61AE, 0, 7, FSCAL1)  /* Tune Frequency Calibration */                                                                                                                                                               \
	FN(0x61AF, 0, 5, VCO_CAPARR)              \
	FN(0x61AF, 0, 7, FSCAL2)  /* Tune Frequency Calibration */                                                                                                                                                               \
	FN(0x61AF, 6, 6, VCO_CAPARR_OE)           \
	FN(0x61B0, 0, 1, VCO_CAPARR_CAL_CTRL)     \
	FN(0x61B0, 0, 7, FSCAL3)  /* Tune Frequency Calibration */                                                                                                                                                               \
	FN(0x61B0, 2, 5, VCO_VC_DAC)              \
	FN(0x61B0, 6, 6, VCO_DAC_EN_OV)           \
	FN(0x61B1, 0, 5, AGC_DR_XTND_THR)         \
	FN(0x61B1, 0, 7, AGCCTRL0)  /* AGC Dynamic Range Control */                                                                                                                                                                \
	FN(0x61B1, 6, 6, AGC_DR_XTND_EN)          \
	FN(0x61B2, 0, 7, AGCCTRL1)  /* AGC Reference Level */                                                                                                                                                                      \
	FN(0x61B3, 0, 0, LNA_CURRENT_OE)          \
	FN(0x61B3, 0, 7, AGCCTRL2)  /* AGC Gain Override */                                                                                                                                                                        \
	FN(0x61B3, 1, 2, LNA3_CURRENT)            \
	FN(0x61B3, 3, 5, LNA2_CURRENT)            \
	FN(0x61B3, 6, 7, LNA1_CURRENT)            \
	FN(0x61B4, 0, 0, AAF_RP_OE)               \
	FN(0x61B4, 0, 7, AGCCTRL3)  /* AGC Control */                                                                                                                                                                              \
	FN(0x61B4, 1, 2, AAF_RP)                  \
	FN(0x61B4, 3, 4, AGC_WIN_SIZE)            \
	FN(0x61B4, 5, 6, AGC_SETTLE_WAIT)         \
	FN(0x61B5, 0, 0, ADC_DAC2_EN)             \
	FN(0x61B5, 0, 7, ADCTEST0)  /* ADC Tuning */                                                                                                                                                                               \
	FN(0x61B5, 1, 3, ADC_GM_ADJ)              \
	FN(0x61B5, 4, 5, ADC_QUANT_ADJ)           \
	FN(0x61B5, 6, 7, ADC_VREF_ADJ)            \
	FN(0x61B6, 0, 1, ADC_C3_ADJ)              \
	FN(0x61B6, 0, 7, ADCTEST1)  /* ADC Tuning */                                                                                                                                                                               \
	FN(0x61B6, 2, 3, ADC_C2_ADJ)              \
	FN(0x61B6, 4, 7, ADC_TEST_CTRL)           \
	FN(0x61B7, 0, 0, ADC_DAC_ROT)             \
	FN(0x61B7, 0, 7, ADCTEST2)  /* ADC Tuning */                                                                                                                                                                               \
	FN(0x61B7, 1, 2, ADC_FF_ADJ)              \
	FN(0x61B7, 3, 4, AAF_RS)                  \
	FN(0x61B7, 5, 6, ADC_TEST_MODE)           \
	FN(0x61B8, 0, 1, DC_BLOCK_MODE)           \
	FN(0x61B8, 0, 7, MDMTEST0)  /* Test Register for Modem */                                                                                                                                                                  \
	FN(0x61B8, 2, 3, DC_WIN_SIZE)             \
	FN(0x61B8, 4, 7, TX_TONE)                 \
	FN(0x61B9, 0, 7, MDMTEST1)  /* Test Register for Modem */                                                                                                                                                                  \
	FN(0x61B9, 1, 1, MODULATION_MODE)         \
	FN(0x61B9, 2, 2, RFC_SNIFF_EN)            \
	FN(0x61B9, 3, 3, RAMP_AMP)                \
	FN(0x61B9, 4, 4, MOD_IF)                  \
	FN(0x61BA, 0, 7, DACTEST0)  /* DAC Override Value */                                                                                                                                                                       \
	FN(0x61BB, 0, 7, DACTEST1)  /* DAC Override Value */                                                                                                                                                                       \
	FN(0x61BC, 0, 7, DACTEST2)  /* DAC Test Setting */                                                                                                                                                                         \
	FN(0x61BD, 0, 7, ATEST)  /* Analog Test Control */                                                                                                                                                                      \
	FN(0x61BE, 0, 7, PTEST0)  /* Override Power-Down Register */                                                                                                                                                             \
	FN(0x61BF, 0, 7, PTEST1)  /* Override Power-Down Register */                                                                                                                                                             \
	FN(0x61C0, 0, (8*24 - 1), CSPROG)  /* CSP Program (read only) */                                                                                                                                                                  \
	FN(0x61E0, 0, 7, CSPCTRL)  /* CSP Control Bit */                                                                                                                                                                          \
	FN(0x61E1, 0, 7, CSPSTAT)  /* CSP Status Register */                                                                                                                                                                      \
	FN(0x61E2, 0, 7, CSPX)  /* CSP X Register */                                                                                                                                                                           \
	FN(0x61E3, 0, 7, CSPY)  /* CSP Y Register */                                                                                                                                                                           \
	FN(0x61E4, 0, 7, CSPZ)  /* CSP Z Register */                                                                                                                                                                           \
	FN(0x61E5, 0, 7, CSPT)  /* CSP T Register */                                                                                                                                                                           \
	FN(0x61EB, 0, 7, RFC_OBS_CTRL0)  /* RF Observation Mux Control */                                                                                                                                                               \
	FN(0x61EC, 0, 7, RFC_OBS_CTRL1)  /* RF Observation Mux Control */                                                                                                                                                               \
	FN(0x61ED, 0, 7, RFC_OBS_CTRL2)  /* RF Observation Mux Control */                                                                                                                                                               \
	FN(0x61FA, 0, 7, TXFILTCFG)  /* TX Filter Configuration */                                                                                                                                                                  \
	FN(0x61a8, 0, 0, TX_FILTER)               \
	FN(0x61a8, 1, 4, PREAMBLE_LENGTH)         \
	FN(0x61a8, 5, 5, DEMOD_AVG_MODE)          \
	FN(0x61a8, 6, 7, DEM_NUM_ZEROS)           \
	FN(0x61a9, 0, 4, CORR_THR)                \
	FN(0x61a9, 5, 5, CORR_THR_SFD)            \
	FN(0x6243, 0, 7, OBSSEL0)  /* Observation Output Control Register 0 */                                                                                                                                                    \
	FN(0x6244, 0, 7, OBSSEL1)  /* Observation Output Control Register 1 */                                                                                                                                                    \
	FN(0x6245, 0, 7, OBSSEL2)  /* Observation Output Control Register 2 */                                                                                                                                                    \
	FN(0x6246, 0, 7, OBSSEL3)  /* Observation Output Control Register 3 */                                                                                                                                                    \
	FN(0x6247, 0, 7, OBSSEL4)  /* Observation Output Control Register 4 */                                                                                                                                                    \
	FN(0x6248, 0, 7, OBSSEL5)  /* Observation Output Control Register 5 */                                                                                                                                                    \
	FN(0x6249, 0, 7, CHVER)  /* Chip Version */                                                                                                                                                                             \
	FN(0x624A, 0, 7, CHIPID)  /* Chip ID */                                                                                                                                                                                  \
	FN(0x624B, 0, 7, TR0)  /* Test Register 0 */                                                                                                                                                                          \
	FN(0x6265, 0, 1, PA_BIAS_CTRL) \
	FN(0x6265, 2, 2, TXMIX_DC_CTRL)           \
	FN(0x6265, 3, 3, LODIV_BIAS_CTRL)         \
	FN(0x6265, 4, 5, DAC_CURR_CTRL)           \
	FN(0x6276, 0, 7, CHIPINFO0)  /* Chip Information Byte 0 */                                                                                                                                                                  \
	FN(0x6277, 0, 7, CHIPINFO1)  /* Chip Information Byte 1 */                                                                                                                                                                  \
	FN(0x70D9, 0, 7, RFD)  /* RF FIFO */                                                                                                                                                                                  \
	FN(0x70E1, 0, 7, RFST)  /* RF Command Strobe Processor */                                                                                                                                                              \
	FN(0x780C, 0, 63, IEEE_PERM_ADDR)  /* Permanent 64 bit addr in read-only info page */


#define CC2531_REG_ADDR(_name) CC2531_REG_##_name##_ADDR
#undef FN
#define FN(_addr, _lsb, _msb, _name) CC2531_REG_ADDR(_name) = _addr,
enum cc2531_reg_addr {
	FOR_EACH_CC2531_REG(FN)
};

#define CC2531_REG_SHIFT(_name) CC2531_REG_##_name##_SHIFT
#undef FN
#define FN(_addr, _lsb, _msb, _name) CC2531_REG_SHIFT(_name) = _lsb,
enum cc2531_reg_shift {
	FOR_EACH_CC2531_REG(FN)
};

#define CC2531_REG_LSB(_name) CC2531_REG_##_name##_LSB
#undef FN
#define FN(_addr, _lsb, _msb, _name) CC2531_REG_LSB(_name) = _lsb,
enum cc2531_reg_lsb {
	FOR_EACH_CC2531_REG(FN)
};

#define CC2531_REG_MSB(_name) CC2531_REG_##_name##_MSB
#undef FN
#define FN(_addr, _lsb, _msb, _name) CC2531_REG_MSB(_name) = _msb,
enum cc2531_reg_msb {
	FOR_EACH_CC2531_REG(FN)
};

#define CC2531_REG_NBITS(_name) CC2531_REG_##_name##_NBITS
#undef FN
#define FN(_addr, _lsb, _msb, _name) CC2531_REG_NBITS(_name) = 1 + _msb - _lsb,
enum cc2531_reg_nbits {
	FOR_EACH_CC2531_REG(FN)
};

#define CC2531_REG_NBYTES(_name) CC2531_REG_##_name##_NBYTES
#undef FN
#define FN(_addr, _lsb, _msb, _name) CC2531_REG_NBYTES(_name) = (7 + CC2531_REG_NBITS(_name))/8,
enum cc2531_reg_nbytes {
	FOR_EACH_CC2531_REG(FN)
};


#define CC2531_MAX_REG_LEN 128

#endif /* _CC2531_REG__H */
