#ifndef _SV_TI_H
#define _SV_TI_H

#ifdef __cplusplus
extern "C"
{
#endif

//Code area

typedef struct 	{ float  Ualpha; 			// Input: reference alpha-axis phase voltage 
				  float  Ubeta;			// Input: reference beta-axis phase voltage 
				  float  Ta;				// Output: reference phase-a switching function		
				  float  Tb;				// Output: reference phase-b switching function 
				  float  Tc;				// Output: reference phase-c switching function
				  float  tmp1;			// Variable: temp variable
				  float  tmp2;			// Variable: temp variable
				  float  tmp3;			// Variable: temp variable
				  uint16_t VecSector;		// Space vector sector
				} SVGEN;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																				
#define MATH_SQRT3_OVER_2 0.866025403785f
/*-----------------------------------------------------------------------------
Default initalizer for the SVGEN object.
-----------------------------------------------------------------------------*/                     
#define SVGEN_DEFAULTS { 0,0,0,0,0 }                       

/*------------------------------------------------------------------------------
	Space Vector  Generator (SVGEN) Macro Definition
------------------------------------------------------------------------------*/


#define SVGENDQ_MACRO(m)														\
	m.tmp1= m.Ubeta;															\
	m.tmp2= 0.5f*(m.Ubeta) + (MATH_SQRT3_OVER_2*m.Ualpha);				    	\
    m.tmp3= m.tmp2 - m.tmp1;													\
																				\
	m.VecSector=3;																\
	m.VecSector=(m.tmp2> 0)?( m.VecSector-1):m.VecSector;						\
	m.VecSector=(m.tmp3> 0)?( m.VecSector-1):m.VecSector;						\
	m.VecSector=(m.tmp1< 0)?(7-m.VecSector) :m.VecSector;						\
																				\
	if     (m.VecSector==1 || m.VecSector==4)                                   \
      {     m.Ta= m.tmp2; 														\
      		m.Tb= m.tmp1-m.tmp3; 												\
      		m.Tc=-m.tmp2;														\
      }								    										\
   																				\
    else if(m.VecSector==2 || m.VecSector==5)                                   \
      {     m.Ta= m.tmp3+m.tmp2; 												\
      		m.Tb= m.tmp1; 														\
      		m.Tc=-m.tmp1;														\
      }																	   		\
   																				\
    else                                                                        \
      {     m.Ta= m.tmp3; 														\
      		m.Tb=-m.tmp3; 														\
      		m.Tc=-(m.tmp1+m.tmp2);												\
      }																	   		\
																				\

#ifdef __cplusplus
}
#endif


#endif 

