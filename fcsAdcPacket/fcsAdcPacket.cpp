/*
This code is customized to use on Teensy4.1 for acquisition of LTC1867 ADC data for FCS Instrument.
Author: Pavan Vutukur
Ocean Mixing Group / Oregon State University
pavan.vutukur@oregonstate.edu
Date: Oct 19, 2022
*/

#include "fcsAdcPacket.h"
#include "digitalLowPassFilter.h"

void getLTC1867RawData(adcBuffer *pSrcBuffer, uint16_t *chanelNum, uint16_t blockSize)
{
/**
 * @brief Acquires 16-bit raw data from LTC1867 ADC 8 channels 
 * @param[in]       *pSrcBuffer pointer to structure uint16_t buffer to store ADC samples
 * @param[in]      	*chanelNum pointer to array uint16_t address of each ADC channel for SPI communication    
 * @param[in]       blockSize length of the buffer
 * @return none.    
 *    
 */
  uint16_t *ch = chanelNum;
  spiTransferWord(LTC1867_CS, *ch++,&(pSrcBuffer->ch0[blockSize]));
  spiTransferWord(LTC1867_CS, *ch++,&(pSrcBuffer->ch1[blockSize]));
  spiTransferWord(LTC1867_CS, *ch++,&(pSrcBuffer->ch2[blockSize]));
  spiTransferWord(LTC1867_CS, *ch++,&(pSrcBuffer->ch3[blockSize]));
  spiTransferWord(LTC1867_CS, *ch++,&(pSrcBuffer->ch4[blockSize]));
  spiTransferWord(LTC1867_CS, *ch++,&(pSrcBuffer->ch5[blockSize]));
  spiTransferWord(LTC1867_CS, *ch++,&(pSrcBuffer->ch6[blockSize]));
  spiTransferWord(LTC1867_CS, *ch++,&(pSrcBuffer->ch7[blockSize]));
}

void getLTC1867FilterData(adcBufferVolts *pSrcBuffer, adcBufferVolts *pDstBuffer, uint16_t blockSize)
{
/**    
 * @brief Performs chebyshevTypeII_7pole_8ChannelLPF   
 * @param[in]       *pSrcBuffer pointer to structure float32_t unfiltered value input  
 * @param[out]      *pDstBuffer pointer to structure float32_t filtered value output    
 * @param[in]       blockSize length of the buffer
 * @return none.    
 *    
 */
	arm_biquad_cascade_df1_f32(&adcFiltCh0,&(pSrcBuffer->ch0[0]),&(pDstBuffer->ch0[0]),blockSize);
	arm_biquad_cascade_df1_f32(&adcFiltCh1,&(pSrcBuffer->ch1[0]),&(pDstBuffer->ch1[0]),blockSize);
	arm_biquad_cascade_df1_f32(&adcFiltCh2,&(pSrcBuffer->ch2[0]),&(pDstBuffer->ch2[0]),blockSize);
	arm_biquad_cascade_df1_f32(&adcFiltCh3,&(pSrcBuffer->ch3[0]),&(pDstBuffer->ch3[0]),blockSize);
	arm_biquad_cascade_df1_f32(&adcFiltCh4,&(pSrcBuffer->ch4[0]),&(pDstBuffer->ch4[0]),blockSize);
	arm_biquad_cascade_df1_f32(&adcFiltCh5,&(pSrcBuffer->ch5[0]),&(pDstBuffer->ch5[0]),blockSize);
	arm_biquad_cascade_df1_f32(&adcFiltCh6,&(pSrcBuffer->ch6[0]),&(pDstBuffer->ch6[0]),blockSize);
	arm_biquad_cascade_df1_f32(&adcFiltCh7,&(pSrcBuffer->ch7[0]),&(pDstBuffer->ch7[0]),blockSize);
}

void convertRawToVolts(adcBuffer *pSrcBuffer, adcBufferVolts *pDstBuffer, uint16_t blockSize)
{
		/**    
 * @brief Converts the array inside the structure from uint16_t to float32_t    
 * @param[in]       *pSrcBuffer pointer to structure uint16_t value input 
 * @param[out]      *pDstBuffer pointer to structure float32_t value output    
 * @param[in]       blockSize length of the buffer
 * @return none.    
 *    
 */
	arm_uint16_to_float32(&(pSrcBuffer->ch0[0]),&(pDstBuffer->ch0[0]),blockSize);
	arm_uint16_to_float32(&(pSrcBuffer->ch1[0]),&(pDstBuffer->ch1[0]),blockSize);
	arm_uint16_to_float32(&(pSrcBuffer->ch2[0]),&(pDstBuffer->ch2[0]),blockSize);
	arm_uint16_to_float32(&(pSrcBuffer->ch3[0]),&(pDstBuffer->ch3[0]),blockSize);
	arm_uint16_to_float32(&(pSrcBuffer->ch4[0]),&(pDstBuffer->ch4[0]),blockSize);
	arm_uint16_to_float32(&(pSrcBuffer->ch5[0]),&(pDstBuffer->ch5[0]),blockSize);
	arm_uint16_to_float32(&(pSrcBuffer->ch6[0]),&(pDstBuffer->ch6[0]),blockSize);
	arm_uint16_to_float32(&(pSrcBuffer->ch7[0]),&(pDstBuffer->ch7[0]),blockSize);
}

void convertVoltstoRaw(adcBufferVolts *pSrcBuffer, adcBuffer *pDstBuffer, uint16_t blockSize)
{
	/**    
 * @brief Converts the array inside the structure from float32_t to uint16_t    
 * @param[in]       *pSrcBuffer pointer to structure float32_t value input 
 * @param[out]      *pDstBuffer pointer to structure uint16_t value output    
 * @param[in]       blockSize length of the buffer
 * @return none.    
 *    
 */
	arm_float32_to_uint16(&(pSrcBuffer->ch0[0]),&(pDstBuffer->ch0[0]),blockSize);
	arm_float32_to_uint16(&(pSrcBuffer->ch1[0]),&(pDstBuffer->ch1[0]),blockSize);
	arm_float32_to_uint16(&(pSrcBuffer->ch2[0]),&(pDstBuffer->ch2[0]),blockSize);
	arm_float32_to_uint16(&(pSrcBuffer->ch3[0]),&(pDstBuffer->ch3[0]),blockSize);
	arm_float32_to_uint16(&(pSrcBuffer->ch4[0]),&(pDstBuffer->ch4[0]),blockSize);
	arm_float32_to_uint16(&(pSrcBuffer->ch5[0]),&(pDstBuffer->ch5[0]),blockSize);
	arm_float32_to_uint16(&(pSrcBuffer->ch6[0]),&(pDstBuffer->ch6[0]),blockSize);
	arm_float32_to_uint16(&(pSrcBuffer->ch7[0]),&(pDstBuffer->ch7[0]),blockSize);
	
	
}

void downSample400HzTo100Hz(adcBufferVolts *pSrcBuffer, adcVoltsPacket *pDstBuffer, uint16_t blockSize)
{
  /**    
 * @brief Downsamples a 400Hz sampled array to 100 Hz sampled array   
 * @param[in]       *pSrcBuffer pointer to structure float32_t 400Hz array value input 
 * @param[out]      *pDstBuffer pointer to structure float32_t 100Hz array value output    
 * @param[in]       blockSize length of the buffer to be downsized to i.e. 100Hz 
 * @return none.    
 *    
 */
  uint16_t blkCnt = 0;
  adcBufferVolts tempBuffer;
  tempBuffer = *pSrcBuffer;
  while(blkCnt < blockSize)
  {
    (pDstBuffer->ch0[blkCnt])      =  (tempBuffer.ch0[4*blkCnt]);
    (pDstBuffer->ch1[blkCnt])      =  (tempBuffer.ch1[4*blkCnt]);
    (pDstBuffer->ch2[blkCnt])      =  (tempBuffer.ch2[4*blkCnt]);
    (pDstBuffer->ch3[blkCnt])      =  (tempBuffer.ch3[4*blkCnt]);
    (pDstBuffer->ch4[blkCnt])      =  (tempBuffer.ch4[4*blkCnt]);
    (pDstBuffer->ch5[blkCnt])      =  (tempBuffer.ch5[4*blkCnt]);
    (pDstBuffer->ch6[blkCnt])      =  (tempBuffer.ch6[4*blkCnt]);
    (pDstBuffer->ch7[blkCnt])      =  (tempBuffer.ch7[4*blkCnt]);
    blkCnt++;
  }
}

void initializeIIR(void)
{
/*
This function calls arm_biquad_cascade_df1_init_f32 for all channels that need Chebyshev Type II low pass filtering. Detailed description below

void arm_biquad_cascade_df1_init_f32	(	arm_biquad_casd_df1_inst_f32 * 	S,
uint8_t 	numStages,
const float32_t * 	pCoeffs,
float32_t * 	pState 
)	
Parameters
[in,out]	S	points to an instance of the floating-point Biquad cascade structure.
[in]	numStages	number of 2nd order stages in the filter.
[in]	pCoeffs	points to the filter coefficients.
[in]	pState	points to the state buffer.
Returns
none
Coefficient and State Ordering
The coefficients are stored in the array pCoeffs in the following order:
    {b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...}
where b1x and a1x are the coefficients for the first stage, b2x and a2x are the coefficients for the second stage, and so on. The pCoeffs array contains a total of 5*numStages values.
The pState is a pointer to state array. Each Biquad stage has 4 state variables x[n-1], x[n-2], y[n-1], and y[n-2]. The state variables are arranged in the pState array as:
    {x[n-1], x[n-2], y[n-1], y[n-2]}
The 4 state variables for stage 1 are first, then the 4 state variables for stage 2, and so on. The state array has a total length of 4*numStages values. The state variables are updated after each block of data is processed; the coefficients are untouched.
For MVE code, an additional buffer of modified coefficients is required.
Its size is numStages and each element of this buffer has type arm_biquad_mod_coef_f32. So, its total size is 32*numStages float32_t elements.
The initialization function which must be used is arm_biquad_cascade_df1_mve_init_f32.
*/
  arm_biquad_cascade_df1_init_f32(&adcFiltCh0, NUM_SECTIONS, pCoeff400HzSampling, pStateCh0);
  arm_biquad_cascade_df1_init_f32(&adcFiltCh1, NUM_SECTIONS, pCoeff400HzSampling, pStateCh1);
  arm_biquad_cascade_df1_init_f32(&adcFiltCh2, NUM_SECTIONS, pCoeff400HzSampling, pStateCh2);
  arm_biquad_cascade_df1_init_f32(&adcFiltCh3, NUM_SECTIONS, pCoeff400HzSampling, pStateCh3);
  arm_biquad_cascade_df1_init_f32(&adcFiltCh4, NUM_SECTIONS, pCoeff400HzSampling, pStateCh4);
  arm_biquad_cascade_df1_init_f32(&adcFiltCh5, NUM_SECTIONS, pCoeff400HzSampling, pStateCh5);
  arm_biquad_cascade_df1_init_f32(&adcFiltCh6, NUM_SECTIONS, pCoeff400HzSampling, pStateCh6);
  arm_biquad_cascade_df1_init_f32(&adcFiltCh7, NUM_SECTIONS, pCoeff400HzSampling, pStateCh7);
}

void arm_fill_uint16(uint16_t value,  uint16_t * pDst,  uint32_t blockSize)
{
/**    
 * @brief Fills a constant value into a uint16_t vector. Adapted from arm_fill_q15 function from CMSIS Github page
 https://arm-software.github.io/CMSIS_5/DSP/html/index.html
 * @param[in]       value input value to be filled   
 * @param[out]      *pDst points to output vector    
 * @param[in]       blockSize length of the output vector   
 * @return none.    
 *    
 */
  uint32_t blkCnt;                               /* loop counter */

  #ifndef ARM_MATH_CM0_FAMILY

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  uint32_t packedValue;                             /* value packed to 32 bits */


  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* Packing two 16 bit values to 32 bit value in order to use SIMD */
  packedValue = __PKHBT(value, value, 16u);

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = value */
    /* Fill the value in the destination buffer */
    *__SIMD32(pDst)++ = packedValue;
    *__SIMD32(pDst)++ = packedValue;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.    
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

  #else

  /* Run the below code for Cortex-M0 */

  /* Loop over blockSize number of values */
  blkCnt = blockSize;

 #endif /* #ifndef ARM_MATH_CM0_FAMILY */

  while(blkCnt > 0u)
  {
    /* C = value */
    /* Fill the value in the destination buffer */
    *pDst++ = value;

    /* Decrement the loop counter */
    blkCnt--;
  }
}

void arm_float32_to_uint16(float32_t * pSrc,  uint16_t * pDst, uint32_t blockSize)
{
  /**    
  adapted from ARM CMSIS DSP Github page https://arm-software.github.io/CMSIS_5/DSP/html/index.html
 * @brief  Converts the elements of the floating point vector to uint16 vector.     
 * @param[in]       *pSrc points to the floating point input vector    
 * @param[out]      *pDst points to the uint16 output vector   
 * @param[in]       blockSize length of the input vector    
 * @return none.    
 *    
 * \par Description:    
 *    
 * The equation used for the conversion process is:    
 *   
 * <pre>    
 *  pDst[n] = (uint16_t) pSrc[n] * 65536 / 4.096;   0 <= n < blockSize.    
 * </pre>    
 *   
 */
 
  float32_t *pIn = pSrc;    /*Src Pointer */
  uint32_t  blkCnt;       /*loop counter*/

  #ifdef ARM_MATH_ROUNDING

    float32_t in;

  #endif /*      #ifdef ARM_MATH_ROUNDING        */

    #ifndef ARM_MATH_CM0_FAMILY
    /* Run the below code for Cortex-M4 and Cortex-M3 */
    /*loop Unrolling */
    blkCnt = blockSize >> 2u;
    
    /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = (uint16_t) A * 65536 / 4.096 */
    /* convert from q15 to float and then store the results in the destination buffer */
    *pDst++ = (uint16_t) (*pIn++ * 65536.0f / 4.096f);
    *pDst++ = (uint16_t) (*pIn++ * 65536.0f / 4.096f);
    *pDst++ = (uint16_t) (*pIn++ * 65536.0f / 4.096f);
    *pDst++ = (uint16_t) (*pIn++ * 65536.0f / 4.096f);

    /* Decrement the loop counter */
    blkCnt--;
  }
  #else

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.    
   ** No loop unrolling is used. */
   blkCnt = blockSize % 0x4u;
   while(blkCnt > 0u)
   {
     *pDst++ = (uint16_t) (*pIn++ * 65536.0f / 4.096f);
     blkCnt--;
   }
  #endif /* #ifndef ARM_MATH_CM0_FAMILY */
}

void arm_uint16_to_float32(uint16_t * pSrc,  float32_t * pDst, uint32_t blockSize)
{
  /**    
  adapted from ARM CMSIS DSP Github page https://arm-software.github.io/CMSIS_5/DSP/html/index.html
 * @brief  Converts the elements of the uint16 vector to floating-point vector.     
 * @param[in]       *pSrc points to the uint16 input vector    
 * @param[out]      *pDst points to the floating-point output vector   
 * @param[in]       blockSize length of the input vector    
 * @return none.    
 *    
 * \par Description:    
 *    
 * The equation used for the conversion process is:    
 *   
 * <pre>    
 *  pDst[n] = (float32_t) pSrc[n] / 65536;   0 <= n < blockSize.    
 * </pre>    
 *   
 */

  uint16_t *pIn = pSrc;                             /* Src pointer */
  uint32_t blkCnt;                               /* loop counter */


#ifndef ARM_MATH_CM0_FAMILY

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = (float32_t) A / 65536 */
    /* convert from q15 to float and then store the results in the destination buffer */
    *pDst++ = ((float32_t) * pIn++ * 4.096 / 65536.0f);
    *pDst++ = ((float32_t) * pIn++ * 4.096 / 65536.0f);
    *pDst++ = ((float32_t) * pIn++ * 4.096 / 65536.0f);
    *pDst++ = ((float32_t) * pIn++ * 4.096 / 65536.0f);

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.    
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

#else

  /* Run the below code for Cortex-M0 */

  /* Loop over blockSize number of values */
  blkCnt = blockSize;

#endif /* #ifndef ARM_MATH_CM0_FAMILY */

  while(blkCnt > 0u)
  {
    /* C = (float32_t) A / 65536 */
    /* convert from q15 to float and then store the results in the destination buffer */
    *pDst++ = ((float32_t) * pIn++ * 4.096 / 65536.0f);

    /* Decrement the loop counter */
    blkCnt--;
  }
}
