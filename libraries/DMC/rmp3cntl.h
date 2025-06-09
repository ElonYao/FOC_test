#ifndef __RMP3_CNTL_H__
#define __RMP3_CNTL_H__

typedef struct { 
         uint32_t DesiredInput; 		// Input: Desired ramp input
				 uint32_t Ramp3Delay;			// Parameter: Ramp3 delay expressed in no of sampling period	
		 	 	 uint32_t Ramp3DelayCount;	// Variable: Counter for rmp3 delay 
				 int32_t Out;					// Output: Ramp3 output 
				 int32_t Ramp3Min;	    	// Parameter: Minimum ramp output 
				 uint16_t Ramp3DoneFlag;		// Output: Flag output 
		  	  	 
				 } RMP3_t;	            


/*-----------------------------------------------------------------------------
Default initalizer for the RMP3 object.
-----------------------------------------------------------------------------*/                     
#define RMP3_DEFAULTS { 168, \
                        10, \
                        0, \
                        1024, \
                        80, \
                       	0, \
             		  }


/*------------------------------------------------------------------------------
 RMP3CNTL Macro Definition
------------------------------------------------------------------------------*/

#define RC3_MACRO(v)								\
   if (v.Out == v.DesiredInput)						\
      v.Ramp3DoneFlag = 1;					\
   else												\
    {												\
													\
      v.Ramp3DelayCount++;							\
													\
      if (v.Ramp3DelayCount >= v.Ramp3Delay)		\
      {												\
        v.Out--;									\
													\
        if (v.Out < v.Ramp3Min)						\
           v.Out = v.Ramp3Min;						\
													\
        v.Ramp3DelayCount = 0;						\
      }                          					\
 													\
    }

#endif // __RMP3_CNTL_H__

