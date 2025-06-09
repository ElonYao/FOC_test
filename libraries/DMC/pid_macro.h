#ifndef __PID_H__
#define __PID_H__

typedef struct {  float  Ref;   			// Input: reference set-point
				  float  Fbk;   			// Input: feedback
				  float  Out;   			// Output: controller output 
				  float  c1;   			// Internal: derivative filter coefficient 1
				  float  c2;   			// Internal: derivative filter coefficient 2
				} PID_TERMINALS;
				// note: c1 & c2 placed here to keep structure size under 8 words

typedef struct {  float  Kr;				// Parameter: reference set-point weighting 
				  float  Kp;				// Parameter: proportional loop gain
				  float  Ki;			    // Parameter: integral gain
				  float  Kd; 		        // Parameter: derivative gain
				  float  Km; 		        // Parameter: derivative weighting
				  float  Umax;			// Parameter: upper saturation limit
				  float  Umin;			// Parameter: lower saturation limit
				} PID_PARAMETERS;

typedef struct {  float  up;				// Data: proportional term
				  float  ui;				// Data: integral term
				  float  ud;				// Data: derivative term
				  float  v1;				// Data: pre-saturated controller output
				  float  i1;				// Data: integrator storage: ui(k-1)
				  float  d1;				// Data: differentiator storage: ud(k-1)
				  float  d2;				// Data: differentiator storage: d2(k-1) 
				  float  w1;				// Data: saturation record: [u(k-1) - v(k-1)]
				} PID_DATA;


typedef struct {  PID_TERMINALS	term;
				  PID_PARAMETERS param;
				  PID_DATA		data;
				} PID_CONTROLLER;

/*-----------------------------------------------------------------------------
Default initalisation values for the PID objects
-----------------------------------------------------------------------------*/                     

#define PID_TERM_DEFAULTS {				\
						   0, 			\
                           0, 			\
                           0, 			\
                           0, 			\
						   0 			\
              			  }

#define PID_PARAM_DEFAULTS {			\
                           1.0f,	\
                           0.060f, 	\
                           0.005f,	\
                           0.0f,	\
                           0.0f,	\
                           0.95f,	\
                           0.0f	\
              			  }

#define PID_DATA_DEFAULTS {			    \
                           0.0f,	\
                           0.0f, 	\
                           0.0f,	\
                           0.0f,	\
                           0.0f, 	\
                           0.0f,	\
                           0.0f,	\
                           1.0f 	\
              			  }


/*------------------------------------------------------------------------------
 	PID Macro Definition
------------------------------------------------------------------------------*/

#define PID_MACRO(v)																				\
																									\
	/* proportional term */ 																		\
	v.data.up = (v.param.Kr*v.term.Ref) - v.term.Fbk;	        									\
																									\
	/* integral term */ 																			\
	v.data.ui =(v.param.Ki*(v.data.w1*(v.term.Ref - v.term.Fbk))) + v.data.i1;						\
	v.data.i1 = v.data.ui;																			\
																									\
	/* derivative term */ 																			\
	v.data.d2 = (v.param.Kd*(v.term.c1*((v.term.Ref*v.param.Km) - v.term.Fbk))) - v.data.d2;	\
	v.data.ud = v.data.d2 + v.data.d1;																\
	v.data.d1 = v.data.ud*v.term.c2;														\
																									\
	/* control output */ 																			\
	v.data.v1 = v.param.Kp*(v.data.up + v.data.ui + v.data.ud);							\
	v.term.Out= ((v.data.v1<v.param.Umin)? v.param.Umin : ((v.data.v1>v.param.Umax)? v.param.Umax : v.data.v1));										\
	v.data.w1 = (v.term.Out == v.data.v1) ? 1.0f : 0.0f;									\
	
#endif //

