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

#define dmxHeaderOffset 37		// dmx channel payloads start 37 'bits' from start of DmxPkt_t
#define dmxFrameSize 11			// dmx frame is: start bit + 8bit payload + 2xstop bits
#define dmxChannelSize 8		// dmx channel is: 8-bit payload = user value for lighting data
#define dmxChannelCount 512

/////////////////////
// raw dmx user data
/////////////////////

//typedefs
typedef uint8_t DmxChannel_t;							// 8-bit dmx channel values 0..255
typedef uint8_t DmxAllChannels_t[dmxChannelCount];		// array of 512 user data, hence dmx512

// variables
//static DmxChannel_t dmxChan;						// single dmx channel
static DmxAllChannels_t dmxAllChannels;		// array for all channels


/////////////////////////////////////
// hardware and low level data structures
/////////////////////////////////////

typedef struct {
	const uint8_t start;
	uint8_t payload[dmxChannelSize];
	const uint8_t stop[2];
} DmxLLFrame_t;


static DmxLLFrame_t dmxLLFrame = {
		// DMX low-level frame: start bit + 8bit_payload + 2x stop bits
		// each bit is REPRESENTED BY ONE BYTE here
	.start =  LO,
	.payload = { 0, 0, 0, 0, 0, 0, 0, 0 },	// 8bit channel data
	.stop = { HI, HI }
};


typedef struct {
    uint8_t combined[ dmxHeaderOffset + 512 * dmxFrameSize ];
} DmxLLPkt_t;

static DmxLLPkt_t dmxLLPkt = {
	.combined = {
		LO10,	LO10,	LO,	LO,	LO,		// "break": 			92uS / 4 = 23 bytes @ 2MHz
		HI,		HI,		HI,				// "mark after break": 	12uS / 4 = 3 bytes @ 2MHz
		LO,								// 1 x low = "start bit"
		LO5,	LO,	LO, LO,				// 8 x low = 8bit "start frame": always zero in DMX lighting
		HI,		HI						// 2 x hi = "stop bits"
    }
};


void setChannel(uint16_t dmxAddr, uint8_t value);
DmxChannel_t getChannel(uint16_t dmxAddress);


#endif /* INC_DMX512_H_ */
