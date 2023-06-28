// SPDX-FileCopyrightText: 2023 Andreas Sig Rosvall
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef	_CC2531_CSP_H
#define	_CC2531_CSP_H


enum csp_cmd_strobe {
	/* No operation
	 * Operation continues at the next instruction.
	 * PC = PC + 1 */
	CSP_CMD_SNOP = 0x0,
	/* Start program execution
	 * The ISSTART instruction starts the CSP program execution from first instruction written to instruction memory.
	 * Do not issue an ISSTART instruction if CSP is already running.
	 * PC = 0, start execution */
	CSP_CMD_START = 0x1,
	/* Stop program execution
	 * The ISSTOP instruction stops the CSP program execution and the IRQ_CSP_STOP interrupt flag is asserted. */
	CSP_CMD_STOP = 0x2,
	/* Enable and calibrate frequency synthesizer for RX
	 * The SRXON instruction asserts the output FFCTL_SRXON_STRB to enable and
	 * calibrate the frequency synthesizer for RX. The instruction waits for the radio to
	 * acknowledge the command before executing the next instruction. */
	CSP_CMD_RXON = 0x3,
	/* Set bit in RXENABLE
	 * The SRXMASKBITSET instruction sets bit 5 in the RXENABLE register. */
	CSP_CMD_RXMASKBITSET = 0x4,
	/* Clear bit in RXENABLE
	 * The SRXMASKBITCLR instruction clears bit 5 in the RXENABLE register. */
	CSP_CMD_RXMASKBITCLR = 0x5,
	/* Send acknowledge frame with pending field cleared
	 * The SACK instruction sends an acknowledge frame.
	 * The instruction waits for the radio to acknowledge the command before executing the next instruction. */
	CSP_CMD_ACK = 0x6,
	/* Send acknowledge frame with the pending field set
	 * The SACKPEND instruction sends an acknowledge frame with the pending field set.
	 * The instruction waits for the radio to acknowledge the command before executing the next instruction. */
	CSP_CMD_ACKPEND = 0x7,
	/* Abort sending of acknowledge frame
	 * The ISNACK instruction immediately prevents sending of an acknowledge frame to the currently received frame. */
	CSP_CMD_NACK = 0x8,
	/* Enable TX after calibration
	 * The STXON instruction enables TX after calibration. The instruction waits for the radio to
	 * acknowledge the command before executing the next instruction. Sets a bit in RXENABLE if
	 * SET_RXENMASK_ON_TX is set */
	CSP_CMD_TXON = 0x9,
	/* Enable calibration and TX if CCA indicates a clear channel
	 * The STXONCCA instruction enables TX after calibration if CCA indicates a clear channel. */
	CSP_CMD_TXONCCA = 0xA,
	/* Sample the current CCA value to SAMPLED_CCA
	 * The current CCA value is written to SAMPLED_CCA in XREG. */
	CSP_CMD_SAMPLECCA = 0xB,
	/* Flush RXFIFO buffer and reset demodulator
	 * The SFLUSHRX instruction flushes the RXFIFO buffer and resets the demodulator.
	 * The instruction waits for the radio to acknowledge the command before executing
	 * the next instruction. */
	CSP_CMD_FLUSHRX = 0xD,
	/* Flush TXFIFO buffer
	 * The SFLUSHTX instruction flushes the TXFIFO buffer.
	 * The instruction waits for the radio to acknowledge the command before executing the next instruction. */
	CSP_CMD_FLUSHTX = 0xE,
	/* Disable RX or TX and frequency synthesizer.
	 * The SRFOFF instruction disables RX or TX and the frequency synthesizer. */
	CSP_CMD_RFOFF = 0xF,
	/* Clear CSP program memory, reset program counter
	 * The ISCLEAR clears the program memory, resets the program counter, and aborts any running program.
	 * No stop interrupt is generated. The LABEL pointer is cleared.
	 * The ISCLEAR instruction must be issued twice to reset the program counter.
	 * PC = 0, clear program memory */
	CSP_CMD_CLEAR = 0xFF,
};


enum csp_condition {
	CSP_IF_CCA            = 0x0,
	CSP_IF_SFD            = 0x1,
	CSP_IF_CPUCTRL        = 0x2,
	CSP_IF_X_0            = 0x4,
	CSP_IF_Y_0            = 0x5,
	CSP_IF_Z_0            = 0x6,
	CSP_IF_RSSI_VALID     = 0x7,

	CSP_IF_NOT_CCA        = (0x8 | CSP_IF_CCA),
	CSP_IF_NOT_SFD        = (0x8 | CSP_IF_SFD),
	CSP_IF_NOT_CPUCTRL    = (0x8 | CSP_IF_CPUCTRL),
	CSP_IF_X_NOT_0        = (0x8 | CSP_IF_X_0),
	CSP_IF_Y_NOT_0        = (0x8 | CSP_IF_Y_0),
	CSP_IF_Z_NOT_0        = (0x8 | CSP_IF_Z_0),
	CSP_IF_RSSI_NOT_VALID = (0x8 | CSP_IF_RSSI_VALID),
};

/*
 * Command strobe processor instructions.
 * A series of up to 24 instructions can stored in the CSP by writing to the RFST fifo.
 */
enum csp_insn {
	/* Decrement Z
	 * The Z register is decremented by 1. An original value of 0x00 underflows to 0xFF.
	 * Z=Z–1 */
	CSP_INSN_DECZ = 0xC5,

	/* Decrement Y
	 * The Y register is decremented by 1. An original value of 0x00 underflows to 0xFF.
	 * Y=Y–1 */
	CSP_INSN_DECY = 0xC4,

	/* Decrement X
	 * The X register is decremented by 1. An original value of 0x00 underflows to 0xFF.
	 * X=X–1 */
	CSP_INSN_DECX = 0xC3,

	/* Increment Z
	 * The X register is incremented by 1. An original value of 0xFF overflows to 0x00.
	 * Z=Z+1 */
	CSP_INSN_INCZ = 0xC2,

	/* Increment Y
	 * The Y register is incremented by 1. An original value of 0xFF overflows to 0x00.
	 * Y=Y+1 */
	CSP_INSN_INCY = 0xC1,

	/* Increment X
	 * The X register is incremented by 1. An original value of 0xFF overflows to 0x00.
	 * X=X+1 */
	CSP_INSN_INCX = 0xC0,

	/* Increment Y not greater than M. (M=0-7)
	 * The Y register is incremented by 1 if the result is less than M; otherwise, Y register is loaded with value M.
	 * Y = min(Y + 1, M) */
	#define CSP_INSN_INCMAXY(M) (0xC8 | ((M) & 0x7))

	/* Load random value into X.
	 * The [Y] LSBs of the X register are loaded with a random value. Note that if a second
	 * RANDXY instruction is issued immediately (within 13 clock cycles) after the first, the
	 * same random value is used in both cases. If Y equals zero or is greater than 7, then an
	 * 8-bit random value is loaded into X.
	 * X[(Y – 1):0]: = RNG_DOUT[(Y – 1):0], X[7:Y]: = 0 */
	CSP_INSN_RANDXY = 0xBD,

	/* Interrupt
	 * The interrupt IRQ_CSP_INT is asserted when this instruction is executed.
	 * IRQ_CSP_INT = 1 */
	CSP_INSN_INT = 0xBA,

	/* Wait for X MAC Timer overflows
	 * Wait for MAC Timer to overflow [X] times, where [X] is the value of register X.
	 * Each time a MAC Timer overflow is detected, the value in register X is decremented.
	 * Program execution continues as soon as X = 0.
	 * (If X = 0 when instruction is run, no wait is performed and execution continues directly.)
	 * An IRQ_CSP_WAIT interrupt request is generated when execution continues.
	 * Note: The difference compared to WAIT W is that W is a fixed value,
	 * whereas X is a register value (which could potentially be changed,
	 * such that the number of overflows actually does not correspond to the value of X at
	 * the time WAITX instruction is run).
	 * X = X – 1 when MAC Timer overflow = true
	 * PC = PC while X > 0
	 * PC = PC + 1 when X = 0 */
	CSP_INSN_WAITX = 0xBC,

	/* Set the compare value of the MAC Timer to the current timer value.
	 * Csp_mact_setcmp1 = 1 */
	CSP_INSN_SETCMP1 = 0xBE,

	/* Wait for W MAC Timer overflows (W = 0–31)
	 * Wait until MAC Timer overflows a number of times equal to the value of W.
	 * If W = 0, the instruction waits for 32 overflows.
	 * Program execution continues with the next instruction,
	 * and the interrupt flag IRQ_CSP_WT is asserted when the wait condition is true.
	 * PC = PC while number of MAC Timer overflows < W
	 * PC = PC + 1 when number of MAC Timer overflows = W */
	#define CSP_INSN_WAITW(_overflows) (0x80 | ((_overflows) & 0x1f) )

	/* Wait until MAC Timer event 1
	 * Wait until next MAC Timer event.
	 * Program execution continues with the next instruction when the wait condition is true.
	 * PC = PC while MAC Timer compare = false
	 * PC = PC + 1 when MAC Timer compare = true */
	CSP_INSN_WEVENT1 = 0xB8,

	/* Wait until MAC Timer event 2
	 * Wait until next MAC Timer event.
	 * Program execution continues with the next instruction when the wait condition is true.
	 * PC = PC while MAC Timer compare = false
	 * PC = PC + 1 when MAC Timer compare = true */
	CSP_INSN_WEVENT2 = 0xB9,

	/* Set loop label
	 * Sets next instruction as start of loop.
	 * If the current instruction is the last instruction in the instruction memory,
	 * then the current PC is set as start of loop. If several label instructions are executed,
	 * the last label executed is the active label.
	 * Earlier labels are removed, which means that only one level of loops is supported.
	 * LABEL: = PC + 1
	 * Opcode: 0xBB */
	CSP_INSN_LABEL = 0xBB,

	/* Conditional repeat
	 * If condition C is true, then jump to the instruction defined by the last LABEL instruction,
	 * that is, jump to start of loop. If the condition is false or if a LABEL instruction has not
	 * been executed, then execution continues from next instruction. The condition C may be
	 * negated by setting N = 1 and is described in the following table.
	 * PC = LABEL when (C XOR N) = true
	 * PC = PC + 1 when (C XOR N) = false or LABEL = not set */
	#define CSP_INSN_RPT(_condition)  (0xA0 | (_condition))

	/* Conditional skip instruction
	 * Skip S instructions on condition C (where condition C may be negated; N = 1). When
	 * condition (C XOR N) is true, skip the next S instructions, else execute the next
	 * instruction. If S = 0, re-execute the conditional jump (that is, busy loop until condition is
	 * false). Skipping past the last instruction in the command buffer results in an implicit
	 * STOP command.
	 * PC = PC + S + 1 when (C XOR N) = true
	 * PC = PC + 1 when (C XOR N) = false */
	#define CSP_INSN_SKIP(_condition, _skip) (0x00 | ((_skip)<<4) | (_condition))

	#define CSP_INSN_STROBE(_cmd_strobe) (0xD0 + (_cmd_strobe))
};

/*
 * Command strobe processor immediate strobes.
 * When written to the RFST fifo, these are not stored but take effect immediately.
 */
#define CSP_IMM_CMD_STROBE(_cmd_strobe) (0xE0 | (_cmd_strobe))

#endif /* !_CC2531_CSP_H */
