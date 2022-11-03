/*
This code is customized to use on Teensy4.1 for filtering using CMSIS DSP
chebyshevTypeII_7pole_8ChannelLPF
Author: Pavan Vutukur
Ocean Mixing Group / Oregon State University
pavan.vutukur@oregonstate.edu
Date: Oct 19, 2022
*/

#ifndef digitalLowPassFilter_h
#define digitalLowPassFilter_h

#include <arm_math.h>
#include "arm_const_structs.h"

#define           NUM_SECTIONS 4


/* chebyshevTypeII_7pole_8ChannelLPF
Generated with MatlabSOS2CMSIS LP_Chebyshev_TypeII_7Pole
https://github.com/docPhil99/DSP/blob/master/MatlabSOS2CMSIS.m
Phil Birch, University of Sussex, 2017*/
float32_t pCoeff400HzSampling[]={ 5.70643361E-01, -7.92260263E-01, 5.70643361E-01, 1.49510943E+00, -8.44135893E-01,
 4.11324026E-01, -4.62049994E-01, 4.11324026E-01, 1.18820138E+00, -5.48799440E-01,
 2.15300472E-01, -1.99633765E-02, 2.15300472E-01, 8.46881415E-01, -2.57518983E-01,
 3.32572945E-01, 3.32572945E-01, 0.00000000E+00, 3.34854110E-01, -0.00000000E+00,};

/*float32_t pCoeffs_100Hz[]={ 8.50672418E-01, 1.39103865E+00, 8.50672418E-01, -1.27824158E+00, -8.14141912E-01,
 7.22065720E-01, 1.26904352E+00, 7.22065720E-01, -1.17007837E+00, -5.43096591E-01,
 6.45685884E-01, 1.24104103E+00, 6.45685884E-01, -1.14866931E+00, -3.83743479E-01,
 7.87342409E-01, 7.87342409E-01, 0.00000000E+00, -5.74684819E-01, -0.00000000E+00,};*/
 
/*Example usage:
#include "digitalLowPassFilter.h"
float32_t pState[NUM_SECTIONS*4]={0};
arm_biquad_casd_df1_inst_f32 S;

In the main function init the filter arm_biquad_cascade_df1_init_f32(&S,NUM_SECTIONS,pCoeffs,pState);

To run the filter:
arm_biquad_cascade_df1_f32(&S,pSrc,pDest,BUFFER_SIZE);
See CMSIS doc for varible descriptions
*/

//CMSIS Biquad Cascade IIR Filters Using Direct Form I (Instance)
arm_biquad_casd_df1_inst_f32 adcFiltCh0;  
arm_biquad_casd_df1_inst_f32 adcFiltCh1;  
arm_biquad_casd_df1_inst_f32 adcFiltCh2;  
arm_biquad_casd_df1_inst_f32 adcFiltCh3;  
arm_biquad_casd_df1_inst_f32 adcFiltCh4;  
arm_biquad_casd_df1_inst_f32 adcFiltCh5;  
arm_biquad_casd_df1_inst_f32 adcFiltCh6;
arm_biquad_casd_df1_inst_f32 adcFiltCh7;  
arm_biquad_casd_df1_inst_f32 adcFiltAx;  
arm_biquad_casd_df1_inst_f32 adcFiltAy;  
arm_biquad_casd_df1_inst_f32 adcFiltAz;    
/*The coefficients and state variables for a filter are stored together in an instance data structure. 
A separate instance structure must be defined for each filter. 
Coefficient arrays may be shared among several instances while state variable arrays cannot be shared. 
There are separate instance structure declarations for each of the 3 supported data types.*/

//CMSIS Biquad Cascade IIR Filters Using Direct Form I (pState)
float32_t         pStateCh0[NUM_SECTIONS * 4] = {0};
float32_t         pStateCh1[NUM_SECTIONS * 4] = {0};
float32_t         pStateCh2[NUM_SECTIONS * 4] = {0};
float32_t         pStateCh3[NUM_SECTIONS * 4] = {0};
float32_t         pStateCh4[NUM_SECTIONS * 4] = {0};
float32_t         pStateCh5[NUM_SECTIONS * 4] = {0};
float32_t         pStateCh6[NUM_SECTIONS * 4] = {0};
float32_t         pStateCh7[NUM_SECTIONS * 4] = {0};
float32_t         pStateAx[NUM_SECTIONS * 4] = {0};
float32_t         pStateAy[NUM_SECTIONS * 4] = {0};
float32_t         pStateAz[NUM_SECTIONS * 4] = {0};


/*The pState points to state variables array. Each Biquad stage has 4 state variables x[n-1], x[n-2], y[n-1], and y[n-2]. 
 The state variables are arranged in the pState array as:
    {x[n-1], x[n-2], y[n-1], y[n-2]}
The 4 state variables for stage 1 are first, then the 4 state variables for stage 2, and so on. 
The state array has a total length of 4*numStages values. 
The state variables are updated after each block of data is processed, the coefficients are untouched.*/


//void initializeIIR(void);
/*Init Function
There is also an associated initialization function for each data type. 
The initialization function performs following operations:
    Sets the values of the internal structure fields.
    Zeros out the values in the state buffer. 

To do this manually without calling the init function, assign the follow subfields of the instance structure: 
    numStages, pCoeffs, pState. Also set all of the values in pState to zero.
Use of the initialization function is optional. 
However, if the initialization function is used, then the instance structure cannot be placed into a const data section. */

#endif
