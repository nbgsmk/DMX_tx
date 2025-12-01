/*
 * BoardSUPPORT.cpp
 *
 *  Created on: Sep 27, 2024
 *      Author: peca
 */

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stdbool.h"

#include "BoardSUP.h"



void boardLedOn(void) {
	HAL_GPIO_WritePin(BOARD_LED_0_GPIO_Port, BOARD_LED_0_Pin, GPIO_PIN_RESET);
}

void boardLedOff(void) {
	HAL_GPIO_WritePin(BOARD_LED_0_GPIO_Port, BOARD_LED_0_Pin, GPIO_PIN_SET);
}

void boardLedTogl(void) {
	HAL_GPIO_TogglePin(BOARD_LED_0_GPIO_Port, BOARD_LED_0_Pin);
}

void boardLedBlink(uint32_t ticksOn) {
	boardLedOn();
	osDelay(ticksOn);
	boardLedOff();
}

void boardLedBlinkCount(uint32_t count, uint32_t ticksOn, uint32_t ticksOff) {
	for (uint32_t i = 0; i < count; i++) {
		boardLedBlink(ticksOn);
		osDelay(ticksOff);
	}
}

void boardLedBlinkPeriod(uint32_t count, uint32_t ticksOn, uint32_t ticksOff, uint32_t ticksTotalPeriod) {
	for (uint32_t i = 0; i < count; i++) {
		boardLedBlink(ticksOn);
		osDelay(ticksOff);
	}
	uint32_t treptanje = count * (ticksOn + ticksOff);
	uint32_t ostatak;
	if (treptanje >= ticksTotalPeriod) {
		ostatak = 1;
	} else {
		ostatak = ticksTotalPeriod - (count * (ticksOn + ticksOff));
	}
	osDelay(ostatak);
}

bool boardKeyPressed(void){
	if ( HAL_GPIO_ReadPin(BOARD_KEY_0_GPIO_Port, BOARD_KEY_0_Pin) == GPIO_PIN_RESET ) {
		return true;
	} else {
		return false;
	}
}


