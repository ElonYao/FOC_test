#ifndef __MOD6_CNT_H__
#define __MOD6_CNT_H__

typedef struct { uint32_t  TrigInput;   	// Input: Modulo 6 counter trigger input 
				 uint16_t  Counter;	    // Output: Modulo 6 counter output
			   } MOD6CNT_t;	            

/*-----------------------------------------------------------------------------
Default initalizer for the MOD6CNT object.
-----------------------------------------------------------------------------*/                     
#define MOD6CNT_DEFAULTS { 0,0 }

/*------------------------------------------------------------------------------
	MOD6_CNT Macro Definition
------------------------------------------------------------------------------*/


#define MOD6CNT_MACRO(v)												\
																		\
 if (v.TrigInput > 0)													\
   {																	\
     if (v.Counter == 5)    /* Reset the counter when it is 5 */		\
       v.Counter = 0;													\
     else																\
       v.Counter++;         /* Otherwise, increment by 1 */				\
   }																	

#endif // __MOD_6CNT_H__ 
