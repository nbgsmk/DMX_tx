/*
 * DMX512.c
 *
 *  Created on: Nov 29, 2025
 *      Author: peca
 */

#include "main.h"
#include <string.h>
#include "cmsis_os.h"
#include "DMX512.h"

extern osMutexId_t dmxLLandChannelMutexHandle;


void setChannel(uint16_t dmxAddress, uint16_t value){

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

	//------------------------
	// directly save user data
	// there is nothing more to it!
	//------------------------

//	uint8_t val8b = (uint8_t)(value & 0x00ff);
	uint8_t val8b = value;

	//+++++++++++++++++++++++
	// CRITICAL SECTION MUTEX
	osMutexAcquire(dmxLLandChannelMutexHandle, portMAX_DELAY);
	dmxAllChannels[dmxAddress-1] = val8b;

	//------------------------
	// low-level preparation
	// construct low-level data in memory for SPI transmission
	//------------------------
	for (uint8_t i = 0; i < sizeof(dmxLLFrame.payload); ++i) {
		if (val8b & (1 << i)) {
			dmxLLFrame.payload[i] = 0xFF;  // bit is 1
		} else {
			dmxLLFrame.payload[i] = 0x00;  // bit is 0
		}
	}

	uint16_t ofs = dmxHeaderOffset + ((dmxAddress-1) * sizeof(DmxLLFrame_t));			// absolute offset into dmx packet 'start bit'
	memcpy(dmxLLPkt.combined + ofs, &dmxLLFrame.start, sizeof(dmxLLFrame.start));

	ofs += sizeof(dmxLLFrame.start);										// move offset behind 'start bit'
	memcpy(dmxLLPkt.combined + ofs, &dmxLLFrame.payload, sizeof(dmxLLFrame.payload));

	ofs += sizeof(dmxLLFrame.payload);										// move offset behind 'payload'
	memcpy(dmxLLPkt.combined + ofs, &dmxLLFrame.stop, sizeof(dmxLLFrame.stop));

	osMutexRelease(dmxLLandChannelMutexHandle);
	// CRITICAL SECTION MUTEX END
	//+++++++++++++++++++++++++++

	__NOP();	// only for breakpoints
};


DmxChannel_t getChannel(uint16_t dmxAddress){
	return dmxAllChannels[dmxAddress];
}


DmxAllChannels_t* getAllChannels(void){
	return &dmxAllChannels;
}
DmxLLPkt_t* getLLPkt(void){
	return &dmxLLPkt.combined;
}


void clearAllChannels(){
	for (int i = 0; i < dmxChannelCount; ++i) {
		setChannel(i, 0);
		setChannel(i, (uint8_t)'_');
	}
}


char* hexToStr(uint8_t hexVal){
	char s[4];
	sprintf(s, 5, "%4X", hexVal);
	return &s;
}
