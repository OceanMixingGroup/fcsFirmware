#ifndef fcsReducedPacket_h
#define fcsReducedPacket_h

#include <arm_math.h>
#include <fcsAdcPacket.h>
#include <chiDR.h>

typedef struct{
        uint32_t  unixSeconds;
        float32_t t1Mean,
                  t2Mean,
                  t1SqMean,
                  t2SqMean,
                  pMean,
                  Wspd,
                  WspdMin,
                  psiS1Fit1,
                  psiS1Fit2,
                  psiS2Fit1,
                  psiS2Fit2,
                  psiT1pFit1,
                  psiT1pFit2,
                  psiT2pFit1,
                  psiT2pFit2; 
}reducedPacket;

void calculateDRPacket(adcVoltsPacket 	*pSrcBuffer, 
					   reducedPacket 	*pDstBuffer, 
					   uint16_t       	blockSize);

#endif
                         