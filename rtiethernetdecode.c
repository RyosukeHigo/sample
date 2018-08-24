/*************************************************************************
 * FILE:
 *   rtiethernetdecode.c
 *
 * DESCRIPTION:
 *
 * REMARKS:
 *
 * AUTHOR(S):   
 *     A. sughayer 
 *
 * Copyright (c) 1997-2002 dSPACE GmbH, GERMANY
 *
 * $RCSfile: rtiethernetdecode.c $ $Revision: 1.3 $ $Date: 2014/05/06 08:41:27MESZ $
 * $ProjectName: e:/ARC/Products/ImplSW/RTISW/RTIETHERNET/Components/Demos/SFunctions/project.pj $
 *
 **************************************************************************/

#define S_FUNCTION_NAME  rtiethernetdecode
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>

#ifndef MATLAB_MEX_FILE
 #include <brtenv.h>
 #include <dstypes.h>
 #include <rti_msg_access.h>
 #include <rti_common_msg.h>
#endif

/*=======================================================================*/
#if ((defined _DS1005) || (defined _DS1007) || (defined _DS1202))        /* Big Indian System */
  typedef struct    
             {                
               uint8_T byte7; 
               uint8_T byte6; 
               uint8_T byte5;
               uint8_T byte4; 
               uint8_T byte3;
               uint8_T byte2;
               uint8_T byte1;
               uint8_T byte0; 
             } uint64_by_uint8_t;

  /*===============================*/
  typedef struct    
             { 
               uint8_T byte3;
               uint8_T byte2;
               uint8_T byte1;
               uint8_T byte0; 
             } uint32_by_uint8_t;

  /*===============================*/
  typedef struct    
             {              
               uint8_T byte1;
               uint8_T byte0; 
             } uint16_by_uint8_t;

  /*===============================*/
  typedef union 
            {
              uint32_by_uint8_t  uint32_r  ;
              real32_T           float32_r ; 
            } ds4502_float32_t;

  /*===============================*/
  typedef union 
            {
              uint64_by_uint8_t  uint64_r ;
              real_T             float64_r; 
            } ds4502_float64_t;

  /*===============================*/
  typedef union    
             { 
               uint32_by_uint8_t  uint32_r; 
               uint32_T           int32_r ;
             } ds4502_uint32_t;

  /*===============================*/
  typedef union    
             { 
               uint16_by_uint8_t  uint16_r ; 
               uint16_T           int16_r  ;
             } ds4502_uint16_t;

/*===============================================================================*/
#elif ((defined _DS1006) || (defined MATLAB_MEX_FILE))        /* Little Indian Systems */
  typedef struct    
             { 
               uint8_T byte0;
               uint8_T byte1; 
               uint8_T byte2;
               uint8_T byte3; 
               uint8_T byte4;
               uint8_T byte5;
               uint8_T byte6;
               uint8_T byte7; 
             } uint64_by_uint8_t;
  /*===============================*/
  typedef struct    
             { 
               uint8_T byte0;
               uint8_T byte1;
               uint8_T byte2;
               uint8_T byte3; 
             } uint32_by_uint8_t;
  /*===============================*/
  typedef struct    
             {              
               uint8_T byte0;
               uint8_T byte1; 
             } uint16_by_uint8_t;
  /*===============================*/
  typedef union 
            {
              uint32_by_uint8_t  uint32_r ;
              real32_T           float32_r; 
            } ds4502_float32_t;
  /*===============================*/
  typedef union 
            {
              uint64_by_uint8_t  uint64_r ;
              real_T             float64_r; 
            } ds4502_float64_t;
  /*===============================*/
  typedef union    
             { 
               uint32_by_uint8_t  uint32_r; 
               uint32_T           int32_r ;
             } ds4502_uint32_t;
  /*===============================*/
  typedef union    
             { 
               uint16_by_uint8_t  uint16_r; 
               uint32_T           int16_r ;
             } ds4502_uint16_t;

#endif
             
/*=======================================================================*/
#ifdef MATLAB_MEX_FILE
  static char *RCSfile = "$RCSfile: rtiethernetdecode.c $";
  static char *RCSrev  = "$Revision: 1.3 $";
  static char *RCSdate = "$Date: 2014/05/06 08:41:27MESZ $";
#endif

/* Datatypes */
#define DT_BOOLEAN              1
#define DT_INT8                 2   
#define DT_UINT8                3
#define DT_INT16                4
#define DT_UINT16               5
#define DT_INT32                6
#define DT_UINT32               7
#define DT_FLOAT                8
#define DT_DOUBLE               9

#define BIG_ENDIAN              1 
#define LITTLE_ENDIAN           2

#define OUTPUTPORTSWIDTH        (OFFSET_NUM)

#define PROCESSORTYPE_IDX       (0)
#define PROCESSORTYPE           ( (uint32_T ) (mxGetPr(ssGetSFcnParam(S, PROCESSORTYPE_IDX))[0]))
#define OFFSET_IDX              (PROCESSORTYPE_IDX+1)
#define OFFSET(n)               ( (uint32_T ) (mxGetPr(ssGetSFcnParam(S, OFFSET_IDX))[n])       )
#define OFFSET_NUM              ( (uint32_T ) (mxGetN (ssGetSFcnParam(S, OFFSET_IDX)))          )
#define AUTO_OFFSET_IDX         (OFFSET_IDX+1)
#define AUTO_OFFSET             ( (uint32_T ) (mxGetPr(ssGetSFcnParam(S, AUTO_OFFSET_IDX))[0])  )
#define DATA_TYPE_IDX           (AUTO_OFFSET_IDX+1)
#define DATA_TYPE(d)            ( (uint32_T ) (mxGetPr(ssGetSFcnParam(S, DATA_TYPE_IDX))[d])    )
#define DATA_TYPE_NUM           ( (uint32_T ) (mxGetN (ssGetSFcnParam(S, DATA_TYPE_IDX)))       )
#define ONE_DATATYPE_IDX        (DATA_TYPE_IDX+1)
#define ONE_DATATYPE            ( (uint32_T ) (mxGetPr(ssGetSFcnParam(S, ONE_DATATYPE_IDX))[0]) )
#define SAMPLE_TIME_IDX         (ONE_DATATYPE_IDX+1)
#define SAMPLE_TIME             ( (real_T   ) (mxGetPr(ssGetSFcnParam(S, SAMPLE_TIME_IDX))[0])  )
                             
#define NUM_PARAM               (SAMPLE_TIME_IDX + 1)                                        
#define PARAM_NOT_TUNABLE_SIZE  (NUM_PARAM)                                        
                                        
#define PARAM_NOT_TUNABLE       { PROCESSORTYPE_IDX,\
                                  OFFSET_IDX,       \
                                  AUTO_OFFSET_IDX,  \
                                  DATA_TYPE_IDX,    \
                                  ONE_DATATYPE_IDX, \
                                  SAMPLE_TIME_IDX\
                                }      

/* XWork access */
#define IWORK_SIZE              (1)
#define RWORK_SIZE              (0)
#define PWORK_SIZE              (1)

#define NUM_DIRECT_FEEDTHROUGH  (1)
#define NUM_SAMPLE_TIMES        (1)

#define MY_OFFSET               ((uint32_T *)  PWorkPtr[0])


/*=======================================================================*/
#if defined(MATLAB_MEX_FILE)
# define MDL_CHECK_PARAMETERS
static void mdlCheckParameters(SimStruct *S)
{
  uint32_T i;

  if (ONE_DATATYPE)
  {
    if( (DATA_TYPE(0) < 1) || (DATA_TYPE(0) > 9 ))
    {
      ssSetErrorStatus(S,"The first element of datatype-vector-parameter must "
                         "be scalar 1-9 ('One datatype only' enabled)");
      return;
    }
  }
  else
  {
    if( (!AUTO_OFFSET) && (OFFSET_NUM != DATA_TYPE_NUM) )
    {  
      ssSetErrorStatus(S,"Width of offset-vector-parameter must be equal "
                         "to width of datatype-vector-parameter.");
      return;
    } 

    for(i=0; i<DATA_TYPE_NUM; i++)
    { 
      if( (DATA_TYPE(i) < 1) || (DATA_TYPE(i) > 9 ))
      {  
        ssSetErrorStatus(S,"Elements of datatype-vector-parameter must be scalar 1-9");
        return;
      }  
    }    
  } 
}
#endif


/*=======================================================================*/
static void mdlInitializeSizes(SimStruct *S)
{
  uint32_T i;
  const uint32_T ParamNotTunable[] = PARAM_NOT_TUNABLE;
  
  for (i = 0; i < PARAM_NOT_TUNABLE_SIZE; i++)
  {
    ssSetSFcnParamNotTunable(S, ParamNotTunable[i]); /*not tunable*/
  }

  ssSetNumSFcnParams(S, NUM_PARAM);
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
  {
    #ifndef MATLAB_MEX_FILE
      rti_msg_error_set(RTI_SFUNCTION_PARAM_ERROR); 
    #endif /* MATLAB_MEX_FILE */
    
    ssSetErrorStatus(S, "Number of actual S-function parameters differs from its definition");    
    return; 
  }
  else
  {
    #if defined(MATLAB_MEX_FILE)
      mdlCheckParameters(S);
      if (ssGetErrorStatus(S) != NULL) return;      
    #endif
  }

  /*==========================*/
  if (!ssSetNumInputPorts(S, 1))
  {
    return; // input ports, that a block has  
  }   
     
  ssSetInputPortWidth(             S, 0, DYNAMICALLY_SIZED);  
  ssSetInputPortDirectFeedThrough( S, 0, NUM_DIRECT_FEEDTHROUGH);
  ssSetInputPortDataType(S, 0, SS_UINT8);

  /*==========================*/
  if (!ssSetNumOutputPorts(S,1))
	{
		return; // oputput ports, that a block has
	} 
    
  ssSetOutputPortWidth(   S, 0, DYNAMICALLY_SIZED);
  ssSetOutputPortDataType(S, 0,         SS_DOUBLE);
 
  /*==========================*/  
  ssSetNumIWork(                   S, IWORK_SIZE);
  ssSetNumRWork(                   S, RWORK_SIZE);
  ssSetNumPWork(                   S, PWORK_SIZE);
  
  ssSetNumContStates(              S, 0);
  ssSetNumDiscStates(              S, 0);
  ssSetNumSampleTimes(             S, NUM_SAMPLE_TIMES);
  
  ssSetNumModes(                   S, 0);
  ssSetNumNonsampledZCs(           S, 0);
  ssSetOptions(S, 0 | SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME); 
}


/*=======================================================================*/
#if defined(MATLAB_MEX_FILE)
# define MDL_SET_INPUT_PORT_WIDTH
static void mdlSetInputPortWidth(SimStruct *S, int_T port, int_T inputPortWidth)
{
  uint32_T i=0, d=0;
  uint32_T DATA_SIZE[]={1,1,1,2,2,4,4,4,8};  /* in byte */
  uint32_T IN_SIZE=0, OUT_SIZE=0;

  ssSetInputPortWidth(S,port,inputPortWidth);  
  IN_SIZE=ssGetInputPortWidth(S,0);
 

  if (!AUTO_OFFSET)
  { 
    for(d=0, i=0; i < OFFSET_NUM; i++)
    {
      if (ONE_DATATYPE)  d = 0 ; 
      else               d = i ;
    
      if( (OFFSET(i) + DATA_SIZE[DATA_TYPE(d) - 1]) > IN_SIZE )
      {
        ssSetErrorStatus(S,"Width of input port must be greater than or equal to the elements of offset-vector-parameter");
        return;
      } 
    }    
    ssSetOutputPortWidth(S,port,OUTPUTPORTSWIDTH);
   } 

  else
  {    
    if (ONE_DATATYPE)  OUT_SIZE = IN_SIZE / DATA_SIZE[DATA_TYPE(0) - 1];  
    else
    {
      if (DATA_TYPE_NUM > IN_SIZE)
      {
        ssSetErrorStatus(S,"Width of input port must be greater than or equal to the elements of datatype-vector-parameter");
        return;
      }      
      OUT_SIZE = DATA_TYPE_NUM ;  
    }    
    ssSetOutputPortWidth(S,port,OUT_SIZE);
  }   
}  

/*==============================*/
# define MDL_SET_OUTPUT_PORT_WIDTH
static void mdlSetOutputPortWidth(SimStruct *S, int_T port, int_T outputPortWidth)
{
  ssSetOutputPortWidth(S,port,outputPortWidth);
}

#endif


/*=======================================================================*/
static void mdlInitializeSampleTimes(SimStruct *S)
{
  real_T  sampleTime = SAMPLE_TIME;

  /* set sample time from parameter list */
  if (sampleTime == -1.0)          /* inherited */
  {
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
  }
  else if (sampleTime == 0.0)     /* continuous */
  {
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
  }
  else                            /* discrete */
  {
    ssSetSampleTime(S, 0, sampleTime);
    ssSetOffsetTime(S, 0, 0.0);
  }
}


/*=======================================================================*/
#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{  
  int_T               IN_SIZE    = ssGetInputPortWidth (S,0), i=0 ;
  int_T               OUT_SIZE   = ssGetOutputPortWidth(S,0)      ;
  uint8_T    *        outPtr     = (uint8_T  * ) ssGetOutputPortSignal(S,0);
  uint32_T  ** const  PWorkPtr   = (uint32_T **) ssGetPWork(S) ;
  int_T      * const  N_DATATYPE = (int_T    * ) ssGetIWork(S) ;
  int_T               DATA_SIZE[]={1,1,1,2,2,4,4,4,8}, typesize=0; /*in Byte*/

  uint32_T            next_offset=0,datatype=0, d=0;
 

  PWorkPtr[0] = (uint32_T *) malloc( IN_SIZE * sizeof(uint32_T) ); 

  if( PWorkPtr[0] == NULL) 
  {  
    ssSetErrorStatus(S,"Memory allocation error for the auto-generated offset array 'PWorkPtr[0]' " );
    return;
  }    
   
  if(AUTO_OFFSET)
  {     
    if (ONE_DATATYPE)
    {  
      N_DATATYPE[0]   =  OUT_SIZE ;
      typesize        =  DATA_SIZE[DATA_TYPE(0)-1] ;

      for (i=0;  i < N_DATATYPE[0]  ; i++)
      {     
        MY_OFFSET[i] =  i * typesize ;                 
      }
    }
    else
    {
      N_DATATYPE[0] = OUT_SIZE;

      for (d=0, next_offset=0, i=0; i < N_DATATYPE[0] ; i++,  d=1)
      {
        MY_OFFSET[i]   =  d * (next_offset);          
        next_offset    =  MY_OFFSET[i] + DATA_SIZE[DATA_TYPE(i) - 1];          
      }    
    }  
  }
  else 
  {
    N_DATATYPE[0]  =  OUT_SIZE ;

    for (i=0;  i < N_DATATYPE[0]  ; i++)
    { 
      MY_OFFSET[i]   =  OFFSET(i);       
    } 
  } 
  

  /*reset in all memory cells of the outputport*/ 
  for(i=0;i<OUT_SIZE;i++)
  {
    outPtr[i]=0; 
  }
}
#endif


/*=======================================================================*/
static void mdlOutputs(SimStruct *S, int_T tid)
{

  uint8_T    **        inPtr     = (uint8_T **) ssGetInputPortSignalPtrs(S,0);
  real_T     *        outPtr     = ssGetOutputPortRealSignal(S,0);
  uint32_T   ** const  PWorkPtr  = (uint32_T **) ssGetPWork(S) ;
  int_T      * const  N_DATATYPE = (int_T     *) ssGetIWork(S);
  int_T      i=0, d=0;  
  
  ds4502_uint16_t  temp ;
  ds4502_uint32_t  temp0;
  ds4502_float32_t temp1;
  ds4502_float64_t temp2;

  
  /*=====================================================================*/
	                   /* byte ordering = little-endian */ 
  /*=====================================================================*/
  if (PROCESSORTYPE == LITTLE_ENDIAN) 
  {
    for(i=0; i < N_DATATYPE[0]; i++)
    {
      if (ONE_DATATYPE)  d = 0 ; 
      else               d = i ;  

      switch( DATA_TYPE(d) )
      {
        case DT_BOOLEAN: outPtr[i]  = (boolean_T) (*inPtr[MY_OFFSET[i]]);                             
                         break;

        case DT_INT8:    outPtr[i]  = (int8_T)    (*inPtr[MY_OFFSET[i]]);                                     
                         break;
                        
        case DT_UINT8:   outPtr[i]  = (uint8_T)   (*inPtr[MY_OFFSET[i]]);
                         break;
                        
        case DT_INT16:   temp.uint16_r.byte0 = *inPtr[MY_OFFSET[i]  ] ;
                         temp.uint16_r.byte1 = *inPtr[MY_OFFSET[i]+1] ; 
                         outPtr[i]           = (int16_T) temp.int16_r;                                              
                         break;
                        
        case DT_UINT16:  temp.uint16_r.byte0 = *inPtr[MY_OFFSET[i]  ] ;
                         temp.uint16_r.byte1 = *inPtr[MY_OFFSET[i]+1] ;
                         outPtr[i]           = (uint16_T) temp.int16_r;                                    
                         break;
                        
        case DT_INT32:   temp0.uint32_r.byte0 = *inPtr[MY_OFFSET[i]  ] ; 
                         temp0.uint32_r.byte1 = *inPtr[MY_OFFSET[i]+1] ;
                         temp0.uint32_r.byte2 = *inPtr[MY_OFFSET[i]+2] ;
                         temp0.uint32_r.byte3 = *inPtr[MY_OFFSET[i]+3] ;   
                         outPtr[i]            = (int32_T) temp0.int32_r ;    
                         break;
                        
        case DT_UINT32:  
                         temp0.uint32_r.byte0 = (uint8_T) *inPtr[MY_OFFSET[i]  ] ;
                         temp0.uint32_r.byte1 = (uint8_T) *inPtr[MY_OFFSET[i]+1] ;
                         temp0.uint32_r.byte2 = (uint8_T) *inPtr[MY_OFFSET[i]+2] ;
                         temp0.uint32_r.byte3 = (uint8_T) *inPtr[MY_OFFSET[i]+3] ;
                         outPtr[i]            = (uint32_T) temp0.int32_r ; 
                         break;      
                        
                        
         case DT_FLOAT:          
                         temp1.uint32_r.byte0 = *inPtr[MY_OFFSET[i]  ] ;
                         temp1.uint32_r.byte1 = *inPtr[MY_OFFSET[i]+1] ; 
                         temp1.uint32_r.byte2 = *inPtr[MY_OFFSET[i]+2] ; 
                         temp1.uint32_r.byte3 = *inPtr[MY_OFFSET[i]+3] ;                          
                         outPtr[i]            = (real32_T) temp1.float32_r;                        
                         break;
                        
        case DT_DOUBLE:  
                         temp2.uint64_r.byte0 = *inPtr[MY_OFFSET[i]  ] ;
                         temp2.uint64_r.byte1 = *inPtr[MY_OFFSET[i]+1] ; 
                         temp2.uint64_r.byte2 = *inPtr[MY_OFFSET[i]+2] ; 
                         temp2.uint64_r.byte3 = *inPtr[MY_OFFSET[i]+3] ;    
                         temp2.uint64_r.byte4 = *inPtr[MY_OFFSET[i]+4] ;
                         temp2.uint64_r.byte5 = *inPtr[MY_OFFSET[i]+5] ; 
                         temp2.uint64_r.byte6 = *inPtr[MY_OFFSET[i]+6] ; 
                         temp2.uint64_r.byte7 = *inPtr[MY_OFFSET[i]+7] ;                       
                         outPtr[i]            = (real_T) temp2.float64_r;                        
                         break;
                                     
        default:         break;
      }
    }
  }	   
  /*=====================================================================*/
				       /* byte ordering = Big-endian */
  /*=====================================================================*/
  else                   
  {
    for(i=0; i < N_DATATYPE[0]; i++)    
    {
      if (ONE_DATATYPE)  d = 0 ; 
      else               d = i ;  

      switch(DATA_TYPE(d))
      {
        case DT_BOOLEAN: outPtr[i] = (boolean_T) (*inPtr[MY_OFFSET[i]]);                         
                         break;

        case DT_INT8:   outPtr[i]  = (int8_T   ) (*inPtr[MY_OFFSET[i]]);
                        break;

        case DT_UINT8:  outPtr[i]  = (uint8_T  ) (*inPtr[MY_OFFSET[i]]);
                        break;

        case DT_INT16:  temp.uint16_r.byte1 = *inPtr[MY_OFFSET[i]  ] ;
                        temp.uint16_r.byte0 = *inPtr[MY_OFFSET[i]+1] ;      
                        outPtr[i]           = (int16_T) temp.int16_r;
                        break;

        case DT_UINT16: temp.uint16_r.byte1 = *inPtr[MY_OFFSET[i]  ] ;
                        temp.uint16_r.byte0 = *inPtr[MY_OFFSET[i]+1] ;     
                        outPtr[i]           = (uint16_T) temp.int16_r;
                        break;

        case DT_INT32:  temp0.uint32_r.byte3 = *inPtr[MY_OFFSET[i]  ]  ;
                        temp0.uint32_r.byte2 = *inPtr[MY_OFFSET[i]+1]  ;
                        temp0.uint32_r.byte1 = *inPtr[MY_OFFSET[i]+2]  ;
                        temp0.uint32_r.byte0 = *inPtr[MY_OFFSET[i]+3]  ;     
                        outPtr[i]            = (int32_T) temp0.int32_r ;
                        break;

        case DT_UINT32: temp0.uint32_r.byte3 = *inPtr[MY_OFFSET[i]  ]  ;
                        temp0.uint32_r.byte2 = *inPtr[MY_OFFSET[i]+1]  ;
                        temp0.uint32_r.byte1 = *inPtr[MY_OFFSET[i]+2]  ;
                        temp0.uint32_r.byte0 = *inPtr[MY_OFFSET[i]+3]  ;    
                        outPtr[i]            = (uint32_T) temp0.int32_r; 
                        break;      
          

        case DT_FLOAT:  temp1.uint32_r.byte3 = *inPtr[MY_OFFSET[i]  ]   ;
                        temp1.uint32_r.byte2 = *inPtr[MY_OFFSET[i]+1]   ; 
                        temp1.uint32_r.byte1 = *inPtr[MY_OFFSET[i]+2]   ; 
                        temp1.uint32_r.byte0 = *inPtr[MY_OFFSET[i]+3]   ;                         
                        outPtr[i]            = (real32_T) temp1.float32_r;
                        break;

        case DT_DOUBLE: temp2.uint64_r.byte7 = *inPtr[MY_OFFSET[i]  ] ;
                        temp2.uint64_r.byte6 = *inPtr[MY_OFFSET[i]+1] ; 
                        temp2.uint64_r.byte5 = *inPtr[MY_OFFSET[i]+2] ; 
                        temp2.uint64_r.byte4 = *inPtr[MY_OFFSET[i]+3] ;    
                        temp2.uint64_r.byte3 = *inPtr[MY_OFFSET[i]+4] ;
                        temp2.uint64_r.byte2 = *inPtr[MY_OFFSET[i]+5] ; 
                        temp2.uint64_r.byte1 = *inPtr[MY_OFFSET[i]+6] ; 
                        temp2.uint64_r.byte0 = *inPtr[MY_OFFSET[i]+7] ;                     
                        outPtr[i]            = (real_T) temp2.float64_r;
                        break;
        default:        break;
      }
    }
  }
} 


/*=======================================================================*/
static void mdlTerminate(SimStruct *S)
{
    int  i;
    void *p;

    for (i=0; i<ssGetNumPWork(S); i++)  
    {
      if ((p = ssGetPWorkValue(S,i)) != NULL) 
      { 
        free(p);
        ssSetPWorkValue(S,i,(void *)0); 
      } 
    } 
} 


#ifdef MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file? */
#include "simulink.c"   /* MEX-File interface mechanism */
#else
#include "cg_sfun.h"    /* Code generation registration function */
#endif                  /* MATLAB_MEX_FILE */
