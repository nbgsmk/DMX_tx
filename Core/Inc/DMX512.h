/*
 * DMX512.h
 *
 *  Created on: Nov 29, 2025
 *      Author: peca
 */


#ifndef INC_DMX512_H_
#define INC_DMX512_H_

#include <stdint.h>

#define LO 0x00
#define LO5 LO, LO, LO, LO, LO
#define LO10 LO5, LO5


#define HI 0xff
#define HI5 HI, HI, HI, HI, HI
#define HI10 HI5, HI5

#define dmxHederOffset 37
#define dmxChanSize 11


typedef struct {
	const uint8_t start;
	uint8_t payload[8];
	const uint8_t stop[2];
} DmxChan_t ;


DmxChan_t dmxChan = {
	.start =  LO,
	.payload = { 0, 0, 0, 0, 0, 0, 0, 0 },
	.stop = { HI, HI }
};


typedef struct {
    uint8_t combined[ dmxHederOffset + 512 * dmxChanSize ];
} DmxPkt_t;

DmxPkt_t dmxPkt = {
	.combined = {
		LO10,	LO10,	LO,	LO,	LO,		// "break": 			92uS / 4 = 23 bytes @ 2MHz
		HI,		HI,		HI,				// "mark after break": 	12uS / 4 = 3 bytes @ 2MHz
		LO,								// 1 x low = "start bit"
		LO5,	LO,	LO, LO,				// 8 x low = 8bit "start frame": always zero in DMX lighting
		HI,		HI						// 2 x hi = "stop bits"
    }
};


void setChannel(uint16_t dmxAddr, uint8_t value);


#endif /* INC_DMX512_H_ */
