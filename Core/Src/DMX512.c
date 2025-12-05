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


int setChannel(uint16_t dmxAddress_range_1_to_512, uint16_t value){


	// sanitize
	if ( (dmxAddress_range_1_to_512 < 1) || (dmxAddress_range_1_to_512 > 512) ) {
		// dmx addresses are 1..512. They are NOT zero-based!!
		return -1;
	}
	uint16_t dmxAddr = dmxAddress_range_1_to_512 - 1;	// zero-based addresses for our sanity :-)
	uint8_t chVal = (uint8_t)(value & 0x00ff);			// keep only lower 8 bits


	//+++++++++++++++++++++++
	//+++++++++++++++++++++++
	// CRITICAL SECTION MUTEX
	osMutexAcquire(dmxLLandChannelMutexHandle, portMAX_DELAY);	// TODO timeout here!
	//+++++++++++++++++++++++
	//+++++++++++++++++++++++

	//------------------------
	// simply save channel value as an uint8_t to an array, for debugging purposes
	//------------------------
	dmxAllChannels[dmxAddr-1] = chVal;

	//------------------------
	// prepare low-level data
	// construct low-level data in memory for SPI transmission
	// each dmx payload bit becomes 1 byte in low-level frame
	//------------------------
	for (uint8_t i = 0; i < sizeof(dmxLLFrame.payload); ++i) {
		if (chVal & (1 << i)) {
			dmxLLFrame.payload[i] = 0xFF;  // bit is 1
		} else {
			dmxLLFrame.payload[i] = 0x00;  // bit is 0
		}
	}

	uint16_t ofs = dmxHeaderOffset + ((dmxAddr-1) * sizeof(DmxLLFrame_t));			// absolute offset into dmx packet 'start bit'
	memcpy(dmxLLPkt.combined + ofs, &dmxLLFrame.startBits, sizeof(dmxLLFrame.startBits));

	ofs += sizeof(dmxLLFrame.startBits);		// move offset behind 'start bit'
	memcpy(dmxLLPkt.combined + ofs, &dmxLLFrame.payload, sizeof(dmxLLFrame.payload));

	ofs += sizeof(dmxLLFrame.payload);			// move offset behind 'payload'
	memcpy(dmxLLPkt.combined + ofs, &dmxLLFrame.stopBits, sizeof(dmxLLFrame.stopBits));

	//+++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++
	osMutexRelease(dmxLLandChannelMutexHandle);
	// CRITICAL SECTION MUTEX END
	//+++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++

	return 0;
};


void setChannels(uint16_t fromDmxAddr, uint16_t toDmxAddr, uint16_t value){
	for (uint16_t i = fromDmxAddr; i <= toDmxAddr; ++i) {
		setChannel(i, value);
	}
}

void setAllChannels(uint8_t val){
	for (int i = 1; i <= dmxChannelCount; ++i) {
		setChannel(i, val);
	}
}

void clearAllChannels(){
	setAllChannels(0x00);
}


DmxChannel_t getChannel(uint16_t dmxAddress){
	return dmxAllChannels[dmxAddress];
}


DmxAllChannels_t* getAllChannels(void){
	return &dmxAllChannels;
}
DmxLLPkt_t* getLLPkt(void){
	return &dmxLLPkt.combined;
}


char* hexToStr(uint8_t hexVal){
	char s[4];
	sprintf(s, 5, "%4X", hexVal);
	return &s;
}
