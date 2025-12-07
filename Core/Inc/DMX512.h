/*
 * DMX512.h
 *
 *  Created on: Nov 29, 2025
 *      Author: peca
 */


#ifndef INC_DMX512_H_
#define INC_DMX512_H_

#define LO 0x00
#define LO5 LO, LO, LO, LO, LO
#define LO10 LO5, LO5


#define HI 0xff
#define HI5 HI, HI, HI, HI, HI
#define HI10 HI5, HI5

#define dmxHeaderOffset 37								// dmx channel payloads start 37 'bits' from start of DmxPkt_t
#define dmxFrameSize 11									// dmx frame is: start bit + 8bit payload + 2xstop bits
#define dmxChannelSize 8								// dmx channel is: 8-bit payload = user value for lighting data
#define dmxChannelCount 512

/////////////////////
// dmx user data
/////////////////////

//typedefs
typedef uint8_t DmxChannel_t;							// 8-bit dmx channel values 0..255
typedef uint8_t DmxAllChannels_t[dmxChannelCount];		// array of 512 user data, hence dmx512

// variables
//static DmxChannel_t dmxChan;							// single dmx channel
static DmxAllChannels_t dmxAllChannels;					// array for all channels


/////////////////////////////////////
// hardware ie low level data structures
/////////////////////////////////////

typedef struct {
	const uint8_t startBits;
	uint8_t payload[dmxChannelSize];
	const uint8_t stopBits[2];
} DmxLLFrame_t;


static DmxLLFrame_t dmxLLFrame = {
		// DMX low-level frame: start bit + 8bit_payload + 2x stop bits
		// each bit is REPRESENTED BY ONE BYTE here
	.startBits =  LO,
	.payload = { 0, 0, 0, 0, 0, 0, 0, 0 },	// 8bit channel data
	.stopBits = { HI, HI }
};


typedef struct {
    uint8_t combined[ dmxHeaderOffset + 512 * dmxFrameSize ];
} DmxLLPkt_t;

static DmxLLPkt_t dmxLLPkt = {
	.combined = {
		LO10,	LO10,	LO,	LO,  LO,	// "break >=88uS": 					i choose 23 bytes = 92uS @ 2MHz SPI clock
		HI,	HI,	HI,						// "mark after break >=8uS":		i choose 3 bytes  = 12uS @ 2MHz
		LO,								// "start bit"						1 byte = 4uS @ 2MHz
		LO5,	LO,LO,LO,				// "start frame is ZERO x 8bits":	32uS ALWAYS ZERO in DMX lighting!!!
		HI,		HI						// "stop bits x 2"
		// user payload of 512 channels continue from here, structured like this
		// 1 start bit LO
		// 8 data bits xx
		// 2 stop bits HI HI
		// ...
    }
};


int setChannel(uint16_t dmxAddress_range_1_to_512, uint16_t value);
void setChannels(uint16_t fromDmxAddress, uint16_t toDmxAddress, uint16_t value);
void setAllChannels(uint8_t val);
void clearAllChannels();

DmxChannel_t getChannel(uint16_t dmxChannel);
DmxAllChannels_t* getAllChannels();
DmxLLPkt_t* getLLPkt();



#endif /* INC_DMX512_H_ */
