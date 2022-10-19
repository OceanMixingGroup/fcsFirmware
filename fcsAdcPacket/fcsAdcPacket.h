/*
This code is customized to use on Teensy4.1 for acquisition of LTC1867 ADC data for FCS Instrument.
Author: Pavan Vutukur
Ocean Mixing Group / Oregon State University
pavan.vutukur@oregonstate.edu
Date: Oct 19, 2022
*/

#ifndef fcsAdcPacket_h
#define fcsAdcPacket_h


#include <ltc1867ADC.h>
#include <arm_math.h>


#define           NUM_ADC_CHANNELS    8                      //NUMBER OF ADC SENSOR CHANNELS TO BE SAMPLED (ex: 8ch LTC1867)
#define           LTC1867_CS          10                     //LTC1867 SPI CS PIN
#define           CW(N)               ((0x80+(N<<4)+0x04)<<8)        // Bitwise OR with channel commands for unipolar mode (16 bit)
#define           FILT_BUFFER_SIZE    2048                    //Length of 400Hz samples channel BUFFER for acq and digital IIR filtering 
#define           DIVE_PACKET         10                      //length of dive packet before we start data reduction algorithms
#define           DR_BUFFER_SIZE      FILT_BUFFER_SIZE/4

/*structure for LTC1867 ADC raw data aquisition ch0-ch7 in uint16_t*/
typedef struct {
        uint16_t ch0[FILT_BUFFER_SIZE];
        uint16_t ch1[FILT_BUFFER_SIZE];
        uint16_t ch2[FILT_BUFFER_SIZE];
        uint16_t ch3[FILT_BUFFER_SIZE];
        uint16_t ch4[FILT_BUFFER_SIZE];
        uint16_t ch5[FILT_BUFFER_SIZE];
        uint16_t ch6[FILT_BUFFER_SIZE];
        uint16_t ch7[FILT_BUFFER_SIZE];
        uint16_t ax[FILT_BUFFER_SIZE];
        uint16_t ay[FILT_BUFFER_SIZE];
        uint16_t az[FILT_BUFFER_SIZE]; 
        uint32_t seconds[FILT_BUFFER_SIZE];
        uint16_t volts[FILT_BUFFER_SIZE];
}adcBuffer;				

/*structure for LTC1867 ADC data ch0-ch7 converted to float32_t (volts)*/
typedef struct {
        float32_t ch0[FILT_BUFFER_SIZE];
        float32_t ch1[FILT_BUFFER_SIZE];
        float32_t ch2[FILT_BUFFER_SIZE];
        float32_t ch3[FILT_BUFFER_SIZE];
        float32_t ch4[FILT_BUFFER_SIZE];
        float32_t ch5[FILT_BUFFER_SIZE];
        float32_t ch6[FILT_BUFFER_SIZE];
        float32_t ch7[FILT_BUFFER_SIZE];
        float32_t ax[FILT_BUFFER_SIZE];
        float32_t ay[FILT_BUFFER_SIZE];
        float32_t az[FILT_BUFFER_SIZE]; 
}adcBufferVolts;

/*structure for LTC1867 ADC data ch0-ch7 downsampled to raw counts from 400Hz to 100Hz*/
typedef struct {
        uint16_t ch0[DR_BUFFER_SIZE];
        uint16_t ch1[DR_BUFFER_SIZE];
        uint16_t ch2[DR_BUFFER_SIZE];
        uint16_t ch3[DR_BUFFER_SIZE];
        uint16_t ch4[DR_BUFFER_SIZE];
        uint16_t ch5[DR_BUFFER_SIZE];
        uint16_t ch6[DR_BUFFER_SIZE];
        uint16_t ch7[DR_BUFFER_SIZE];
        uint16_t ax[DR_BUFFER_SIZE];
        uint16_t ay[DR_BUFFER_SIZE];
        uint16_t az[DR_BUFFER_SIZE]; 
        uint32_t seconds[DR_BUFFER_SIZE];
        uint16_t volts[DR_BUFFER_SIZE];
}adcPacket;

/*structure for LTC1867 ADC data ch0-ch7 downsampled to volts from 400Hz to 100Hz*/
typedef struct {
        float32_t ch0[DR_BUFFER_SIZE];
        float32_t ch1[DR_BUFFER_SIZE];
        float32_t ch2[DR_BUFFER_SIZE];
        float32_t ch3[DR_BUFFER_SIZE];
        float32_t ch4[DR_BUFFER_SIZE];
        float32_t ch5[DR_BUFFER_SIZE];
        float32_t ch6[DR_BUFFER_SIZE];
        float32_t ch7[DR_BUFFER_SIZE];
        float32_t ax[DR_BUFFER_SIZE];
        float32_t ay[DR_BUFFER_SIZE];
        float32_t az[DR_BUFFER_SIZE]; 
        float32_t seconds[DR_BUFFER_SIZE];
        float32_t volts[DR_BUFFER_SIZE];
}adcVoltsPacket;

	
void getLTC1867RawData(adcBuffer 		*pSrcBuffer, 
                       uint16_t 		*chanNum, 
                       uint16_t 		blockSize);

void getLTC1867FilterData(adcBufferVolts 	*pSrcBuffer, 
			  adcBufferVolts 	*pDstBuffer, 
		          uint16_t 	     	blockSize);
						  
void convertRawToVolts(adcBuffer 		*pSrcBuffer, 
		       adcBufferVolts 		*pDstBuffer,
		       uint16_t         	blockSize);

void convertVoltstoRaw(adcBufferVolts 		*pSrcBuffer,
		       adcBuffer 		*pDstBuffer,
		       uint16_t 		blockSize);
					   
void arm_fill_uint16(uint16_t 			value,
		     uint16_t 			*pDst,
		     uint32_t 			blockSize);

void arm_float32_to_uint16(float32_t 		*pSrc,
			   uint16_t 		*pDst,
			   uint32_t 		blockSize);

void arm_uint16_to_float32(uint16_t 		*pSrc,
			   float32_t 		*pDst,
			   uint32_t 		blockSize);
						   

void downSample400HzTo100Hz(adcBufferVolts      *pSrcBuffer, 
                            adcVoltsPacket      *pDstBuffer,
                            uint16_t            blockSize);

void initializeIIR(void);
        
#endif
