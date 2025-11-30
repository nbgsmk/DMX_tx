/*
 * DMX512.c
 *
 *  Created on: Nov 29, 2025
 *      Author: peca
 */

#include "main.h"
#include <string.h>
#include "DMX512.h"

void setChannel(uint16_t dmxAddr, uint8_t value){

	for (uint8_t i = 0; i < sizeof(dmxChan.payload); ++i) {
		if (value & (1 << i)) {
			dmxChan.payload[i] = 0xFF;  // bit is 1
		} else {
			dmxChan.payload[i] = 0x00;  // bit is 0
		}
	}

//	uint16_t ofs;
//	ofs = dmxHederOffset + ((dmxAddr-1) * sizeof(DmxChan_t));			// absolute offset into dmx packet 'start bit'
//	dmxPkt.combined[ofs] = dmxChan.start;					// copy 'start bit'
//
//	ofs += sizeof(dmxChan.start);							// move offset behind 'start bit'
//
//	uint16_t pls = sizeof(dmxChan.payload);					// payload size is 8
//	for (uint16_t i = 0; i < pls; ++i) {
//		dmxPkt.combined[ofs + i] = dmxChan.payload[i];		// copy payload
//	}
//
//	ofs += sizeof(dmxChan.payload);							// move offset behind 'payload'
//	uint16_t ss = sizeof(dmxChan.stop);						// 'stop bit' size is 2
//	for (uint16_t i = 0; i < ss; ++i) {
//		dmxPkt.combined[ofs + i] = dmxChan.stop[i];		// copy payload
//	}


	uint16_t ofs = dmxHederOffset + ((dmxAddr-1) * sizeof(DmxChan_t));			// absolute offset into dmx packet 'start bit'
	memcpy(dmxPkt.combined + ofs, &dmxChan.start, sizeof(dmxChan.start));

	ofs += sizeof(dmxChan.start);										// move offset behind 'start bit'
	memcpy(dmxPkt.combined + ofs, &dmxChan.payload, sizeof(dmxChan.payload));

	ofs += sizeof(dmxChan.payload);										// move offset behind 'payload'
	memcpy(dmxPkt.combined + ofs, &dmxChan.stop, sizeof(dmxChan.stop));

	ofs++;
};
