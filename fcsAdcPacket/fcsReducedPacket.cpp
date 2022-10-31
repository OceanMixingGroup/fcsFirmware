#include <fcsReducedPacket.h>


void calculateDRPacket(adcVoltsPacket *pSrcBuffer, reducedPacket *pDstBuffer, uint16_t blockSize)
{
	/**    
 * @brief Performs data reduction process from chiDR.h
 
 * @param[in]       *pSrcBuffer pointer to structure float32_t raw voltages  
 * @param[out]      *pDstBuffer pointer to structure float32_t reduced values    
 * @param[in]       blockSize length of the buffer
 * @return none.    
 *    
 */
	arm_mean_f32(&(pSrcBuffer->ch4[0]),blockSize,&(pDstBuffer->t1Mean));
	arm_mean_f32(&(pSrcBuffer->ch3[0]),blockSize,&(pDstBuffer->t2Mean));
	arm_mean_f32(&(pSrcBuffer->ch5[0]),blockSize,&(pDstBuffer->pMean));
	float32_t tempBuffer[blockSize];
	arm_fill_f32(0,&tempBuffer[0],blockSize);
	arm_mult_f32(&(pSrcBuffer->ch1[0]),&(pSrcBuffer->ch1[0]), &tempBuffer[0], blockSize);
	arm_mean_f32(&tempBuffer[0],blockSize,&(pDstBuffer->t1SqMean));
	
	arm_fill_f32(0,&tempBuffer[0],blockSize);
	arm_mult_f32(&(pSrcBuffer->ch6[0]),&(pSrcBuffer->ch6[0]), &tempBuffer[0], blockSize);
	arm_mean_f32(&tempBuffer[0],blockSize,&(pDstBuffer->t2SqMean));
	despikeShearSegment(&(pSrcBuffer->ch2[0]),blockSize);
    despikeShearSegment(&(pSrcBuffer->ch7[0]),blockSize);
}






