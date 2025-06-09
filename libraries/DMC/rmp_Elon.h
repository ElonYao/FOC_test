
#ifndef __RMP_ELON_H__
#define __RMP_ELON_H__

//V2 ramp add the acceration control

typedef struct { float    targetValue; 	// Input: Target input
				 float    maxDelta;	//define the maximum acceration step
				 float    rampLowLimit;	// Parameter: Minimum limit
				 float    rampHighLimit;	// Parameter: Maximum limit
				 float    setPoint;	// Output: Target output
				 float	  error;			// Variable: Temp variable
				 float    maxAccel_Hzps; // Maximum speed changing slope
		  	   } RMPCNTL_t;	            


/*-----------------------------------------------------------------------------
Default initalizer for the RMPCNTL object.
-----------------------------------------------------------------------------*/                     
#define RMPCNTL_DEFAULTS {  0.0f, 		 \
                            0.0022f,	/*maxAccel_Hzps *ctrlPeriod_sec*/	 \
                           -600.0f,  \
                           600.0f,   \
                            0.0f,       \
                          	0.0f,       \
                          	2.0f\
                   		  }

/*------------------------------------------------------------------------------
 	RAMP Controller Macro Definition
------------------------------------------------------------------------------*/

#define RC_MACRO(m)\
    m.error = m.targetValue - m.setPoint;\
	m.setPoint +=__fsat(m.error, m.maxDelta, -m.maxDelta);\
	m.setPoint=__fsat(m.setPoint, m.rampHighLimit, m.rampLowLimit);	\
	/*m.setPoint=((m.setPoint<m.rampLowLimit)? m.rampLowLimit:((m.setPoint>m.rampHighLimit)? m.rampHighLimit : m.setPoint));*/

#endif //
