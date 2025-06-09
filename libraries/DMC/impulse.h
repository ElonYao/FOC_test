#ifndef __IMPULSE_H__
#define __IMPULSE_H__

typedef struct { uint32_t  Period;		// Input: Period of output impulse in a number of sampling cycles (Q0)			
				 uint32_t  Out;      		// Output: Impulse generator output - Q0 (0x00000000 or 0x00007FFF)
		  	  	 uint32_t  Counter;   	// Variable: Impulse generator counter (Q0)
			   } IMPULSE_t;	            

/*-----------------------------------------------------------------------------
Default initalizer for the IMPULSE object.
-----------------------------------------------------------------------------*/                     
#define IMPULSE_DEFAULTS { 1000,0,0 }

/*------------------------------------------------------------------------------
	IMPULSE Macro Definition
------------------------------------------------------------------------------*/

#define IMPULSE_MACRO(v)										\
  																\
  v.Out = 0;      /* Always clear impulse output at entry*/		\
  v.Counter++;    /* Increment the skip counter*/				\
																\
  if (v.Counter >= v.Period)									\
  {																\
     v.Out = 1;										\
     v.Counter = 0;        /* Reset counter*/					\
  } 

#endif // __IMPULSE_H__
