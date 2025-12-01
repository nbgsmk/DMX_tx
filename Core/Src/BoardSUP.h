/*
 * BoardSUPPORT.h
 *
 *  Created on: Sep 27, 2024
 *      Author: peca
 */

#ifndef SRC_BOARDSUP_H_
#define SRC_BOARDSUP_H_



	void boardLedOn(void);
	void boardLedOff(void);
	void boardLedTogl(void);

	void boardLedBlink(uint32_t ticksOn);
	void boardLedBlinkCount(uint32_t count, uint32_t ticksOn, uint32_t ticksOff);
	void boardLedBlinkPeriod(uint32_t count, uint32_t ticksOn, uint32_t ticksOff, uint32_t ticksTotalPeriod);

	bool boardKeyPressed(void);

#endif /* SRC_BOARDSUP_H_ */
