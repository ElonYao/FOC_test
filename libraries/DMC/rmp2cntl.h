#ifndef __RMP2_CNTL_H__
#define __RMP2_CNTL_H__

typedef struct { int16_t DesiredInput; 	// Input: Desired ramp input 
                 int16_t Ramp2Max; 		// Parameter: Maximum limit 
				 int16_t Ramp2Min;		// Parameter: Minimum limit 		
				 uint32_t Ramp2DelayCount; // Variable: Incremental delay 	
				 uint32_t Ramp2Delay;		// Parameter: Ramp2 delay expressed in no of sampling period 		
				 int16_t Out;				// Output: Ramp2 output 
		  	  	
				 } RMP2_t;

/*-----------------------------------------------------------------------------
Default initalizer for the RMP2 object.
-----------------------------------------------------------------------------*/                     
#define RMP2_DEFAULTS { 0, \
                        3000, \
                        15, \
                        0, \
                        30, \
                       	3000, \
             		   }

/*------------------------------------------------------------------------------
	 RMP2CNTL Macro Definitions
------------------------------------------------------------------------------*/


#define RC2_MACRO(v)							\
if (v.Out != v.DesiredInput)					\
{												\
	v.Ramp2DelayCount++;						\
	if (v.Ramp2DelayCount >= v.Ramp2Delay)		\
	{											\
		v.Ramp2DelayCount = 0;					\
		if (v.Out < v.DesiredInput)				\
		{  										\
			v.Out++;							\
			if (v.Out > v.Ramp2Max)				\
				v.Out = v.Ramp2Max;				\
		}										\
		else									\
		{ 										\
			v.Out--;							\
			if (v.Out < v.Ramp2Min)				\
				v.Out = v.Ramp2Min;				\
		}										\
	}											\
}

#endif // __RMP2_CNTL_H__
