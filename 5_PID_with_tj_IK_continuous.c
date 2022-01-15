//The following program drives the Mitsubishi RV-M1 manipulator.
//Simple PI controler is used to control the joint angles.
//Trajectory generation, Inverse Kinematics are applied.
//Pick and Place operation is performed.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <time.h>
#include <sched.h>
#include <sys/io.h>

#include "fbida.h"
#include "fbipenc.h"
#include "fbiad.h"

int LPC		=	1;
int PEX_analog	=	3;
int PEX_encoder	=	2;

double K;
double T ;
double xt ;
double tj_t ;
double prv_tj_t = 0;
double start_time  = 0;
double end_time  = 0 ;
int flag = 0 ;
int gripper_flag = 0 ;
int update_flag = 0;
int home_flag = 0;
int indicate_flag = 0;

unsigned short		wData[4];
unsigned short 		LPC_wData[2];

FILE *outputfile ;

int ret ;
 
unsigned long out_data	=	0xF3FF;		// digital outputs from PEX_encoder for brakes // F3 both on// FB first on// F7 second on


#define pre_set_value 0x10000000
unsigned long count_1, count_2, count_3, count_4, count_5 ;
double counts_1, counts_2, counts_3, counts_4, counts_5  ;
double angle_1 , angle_2 , angle_3 , angle_4 , angle_5 ;
double err_1 , err_2 , err_3 , err_4 , err_5 ; 
double kp_1 , kp_2 , kp_3 , kp_4 , kp_5 ;
double output_1 , output_2 , output_3 , output_4 , output_5 , output_6 ;
double ki_1 , ki_2 , ki_3 , ki_4 , ki_5 ; 
double ingl_1 , ingl_2 , ingl_3 , ingl_4 , ingl_5 ; 
double prv_ingl1 , prv_ingl2 , prv_ingl3 , prv_ingl4 , prv_ingl5 ; 
double des_angle_1 , des_angle_2 , des_angle_3 , des_angle_4 , des_angle_5 , des_angle ;
double prv_des_angle_1 , prv_des_angle_2 , prv_des_angle_3 , prv_des_angle_4 , prv_des_angle_5 , prv_des_angle ;
double tj_des_angle_1 , tj_des_angle_2 , tj_des_angle_3 , tj_des_angle_4 , tj_des_angle_5 , tj_des_angle ;
double c_0 , c_1 , c_2 , c_3 , tau ;
double vdes_1 , vdes_2 , vdes_3 ,vdes_4 ,vdes_5 , vdes_6 ;
double v_1 , v_2 , v_3 , v_4, v_5 ;
double prv_ang_1 , prv_ang_2 , prv_ang_3 , prv_ang_4 , prv_ang_5 ;
double pi , f ;
double xdes , ydes , zdes , w4 , w5 , w6 ;
double b1 , b2 , b , r11 , r12 , r13 , r21 , r22 , r23 , r31 , r32 , r33 , th1 , th2 , th3 , th4 , th5 ,th234;
double pi = 3.142 ;
double a2 = 250 ; double a3 = 160 ; double a4 = 0 ;double d1 = 300 ;double d5 = 72 + 107 ;




void read();
void desired_angles();
void algorithm();
void gains();
int trajectory( double des_angle , double prv_des_angle , double tau);
int  IK(double xdes , double ydes ,double zdes);

int  IK(double xdes , double ydes ,double zdes)           //Inverse Kinematics 5DOF
{

r11 	=	0 ;      r12		=	1       ;           r13		=	0;
r21 	=	1 ;      r22		=	0     	;           r23		=	0;
r31 	=	0 ;      r32		=	0       ;           r33		=	-1;

th1 	=	atan2 (ydes,xdes) 								;

th5 	=	atan2( (sin(th1)*r11 - cos(th1)*r21),( sin(th1) * r12 - cos(th1)*r22) )	;


w4	=	-exp(th5/pi) * r13							;
w5	=	-exp(th5/pi) * r23							;
w6	=	-exp(th5/pi) * r33							;


th234 	=	atan2( (cos(th1)*w4 + sin(th1)*w5 ), w6 )				;


b1      =   cos(th1) * xdes + sin(th1) * ydes - a4 * cos(th234) + d5 * sin(th234)   ;
b2      =   d1 - a4 * sin (th234) - d5 * cos(th234) - zdes                        ;

b   	=	sqrt( b1 * b1 + b2 * b2 )		;				

th3 	=	acos(((b*b)-(a2*a2)-(a3*a3))/(2*a2*a3))							;
th2 	=	atan2(((a2+a3*cos(th3))*b2-(a3*sin(th3)*b1)),((a2+a3*cos(th3))*b1+(a3*sin(th3)*b2)))	;
th4 	=	th234 - th2 - th3									;


th1	=	th1*180/pi  ;
th2	=	th2*180/pi  ;
th3	=	th3*180/pi  ;
th4	=	th4*180/pi  ;
th5	=	th5*180/pi  ;

th1     =   th1 + 60 ;
th2     =   th2 + 90 + 10 ;
th3     =   th3 ;
th4     =   th4 ; 
th5     =   th5 + 90 ;

if ( th1 < 0 && th1 > 300.0 ) { printf("out of range ")	;th1 = 0;th2 = 0 ; th3 = 0 ;th4 = 0 ;th5 = 0 ;}
if ( th2 < 0 && th2 > 126 ) { printf("out of range ")	;th1 = 0;th2 = 0 ; th3 = 0 ;th4 = 0 ;th5 = 0 ; }
if ( th3 < 0 && th3 > 110 ) { printf("out of range ")	;th1 = 0;th2 = 0 ; th3 = 0 ;th4 = 0 ;th5 = 0 ; }
if ( th4 < -180 && th4 > 0 ) { printf("out of range ")	;th1 = 0;th2 = 0 ; th3 = 0 ;th4 = 0 ;th5 = 0 ;}

 
return th1;
return th2;
return th3;
return th4;
return th5;

}

void velocity()
{
  v_1	=	( angle_1 - prv_ang_1 ) / T ;
  v_2	=	( angle_2 - prv_ang_2 ) / T ;
  v_3	=	( angle_3 - prv_ang_3 ) / T ;
  v_4	=	( angle_4 - prv_ang_4 ) / T ;
  v_5	=	( angle_5 - prv_ang_5 ) / T ;
  
  prv_ang_1	=	angle_1 ;
  prv_ang_2	=	angle_2 ;
  prv_ang_3	=	angle_3 ;
  prv_ang_4	=	angle_4 ;
  prv_ang_5	=	angle_5 ;
}

void desired_angles()
{
  des_angle_1	=	th1;
  des_angle_2	=	th2;
  des_angle_3	=	th3;
  des_angle_4	=	th4;
  des_angle_5	=	th5;
}

void gains()
{
kp_1	=	10000.0;ki_1	=	50.0;
kp_2	=	10000.0;ki_2	=	50.0;
kp_3	=	10000.0;ki_3	=	50.0;
kp_4	=	10000.0;ki_4	=	50.0;
kp_5	=	10000.0;ki_5	=	50.0;
}




struct timespec t;
struct sched_param param;

long interval=1000000;
#define NSEC_PER_SEC 1000000000
extern int clock_nanosleep(clockid_t __clock_id, int __flags, __const struct timespec *__req,struct timespec *__rem);

static inline void tsnorm(struct timespec *ts)
{
   while (ts->tv_nsec >= NSEC_PER_SEC) {
      ts->tv_nsec -= NSEC_PER_SEC;
      ts->tv_sec++;
   }
}


int main(int argc , char** argv)
{
  
  //outputfile	=	fopen("outputfile.txt","w+") ;
  
   if  ( argc >= 2 && atoi (argv [1] )>0  )
  { 
    printf( "using realtime, priority : %d\n",atoi (argv[1] ) );
    param.sched_priority 	=	atoi ( argv[1] );
    if ( sched_setscheduler(0, SCHED_FIFO, &param ) == -1 )
    { 
      perror( "sched_setscheduler failed ");
      exit (-1);
    }
  }
  if (argc >= 3 )
  { 
    interval = atoi ( argv[2]);
  }
    clock_gettime (0,&t);
    t.tv_sec++;
  
DABOARDSPEC Spec;
DASMPLREQ   SmplConfig;	

DABOARDSPEC LPC_Spec;
DASMPLREQ   LPC_SmplConfig;


DASMPLCHREQ 		DaSmplChReq[4];	


DASMPLCHREQ		LPC_DaSmplChReq[2];

unsigned long		digital_inputvalue;
  
  	ret	=	DaOpen(PEX_analog);		//printf("ret = %i\n ",ret);
	ret	=	PencOpen(PEX_encoder,0);	//printf("ret = %i\n ",ret);
	ret	=	PencOpen(PEX_analog,0);		//printf("ret = %i\n ",ret);
	ret 	= 	DaOpen(LPC);			//printf("ret = %i\n ",ret);
	ret	=	AdOpen(LPC);			//printf("ret = %i\n ",ret);
	
	
	ret	=		PencSetMode(PEX_encoder, 1, 6, 0, 0, 0);
	ret	=		PencSetMode(PEX_encoder, 2, 6, 0, 0, 0);
	ret	=		PencSetMode(PEX_encoder, 3, 6, 0, 0, 0);
	ret	=		PencSetMode(PEX_encoder, 4, 6, 0, 0, 0);
	ret	=		PencSetMode(PEX_analog, 1, 6, 0, 0, 0);
	ret	=		PencSetCounter(PEX_encoder , 1 , pre_set_value);
	ret	=		PencSetCounter(PEX_encoder , 2 , pre_set_value);
	ret	=		PencSetCounter(PEX_encoder , 3 , pre_set_value);
	ret	=		PencSetCounter(PEX_encoder , 4 , pre_set_value);
	ret	=		PencSetCounter(PEX_analog , 1 , pre_set_value);
	
	DaGetDeviceInfo(PEX_analog, &Spec);		//printf("ret = %i\n ",ret);
        DaGetSamplingConfig(PEX_analog, &SmplConfig);	//printf("ret = %i\n ",ret);
	
	DaGetDeviceInfo(LPC, &LPC_Spec);		//printf("ret = %i\n ",ret);
        DaGetSamplingConfig(LPC, &LPC_SmplConfig);	//printf("ret = %i\n ",ret);

	DaSmplChReq[0].ulChNo 	= 1;
	DaSmplChReq[0].ulRange 	= SmplConfig.SmplChReq[0].ulRange;
	DaSmplChReq[1].ulChNo 	= 2;
	DaSmplChReq[1].ulRange 	= SmplConfig.SmplChReq[0].ulRange;
	DaSmplChReq[2].ulChNo 	= 3;
	DaSmplChReq[2].ulRange 	= SmplConfig.SmplChReq[0].ulRange;	
	DaSmplChReq[3].ulChNo 	= 4;
	DaSmplChReq[3].ulRange 	= SmplConfig.SmplChReq[0].ulRange;	
	
	LPC_DaSmplChReq[0].ulChNo 	= 1;
	LPC_DaSmplChReq[0].ulRange 	= LPC_SmplConfig.SmplChReq[0].ulRange;
	LPC_DaSmplChReq[1].ulChNo 	= 2;
	LPC_DaSmplChReq[1].ulRange 	= LPC_SmplConfig.SmplChReq[0].ulRange;
	
	
	gains();
	tau 	=	5;
	end_time=	5;
	
	if ( flag == 0 )	{ IK(200,-200,100);desired_angles();}
	
	while (1)
	{
	clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);
	
	T 	=	0.001	;
	K++	;	
	xt	=	K * T  ;
	
	read();
	
	if ( xt >= start_time && xt < end_time  )
	{      
	trajectory(des_angle_1 , prv_des_angle_1, tau ) ; 	tj_des_angle_1	=	tj_des_angle ;
	trajectory(des_angle_2 , prv_des_angle_2, tau ) ; 	tj_des_angle_2	=	tj_des_angle ;
	trajectory(des_angle_3 , prv_des_angle_3, tau ) ; 	tj_des_angle_3	=	tj_des_angle ;
	trajectory(des_angle_4 , prv_des_angle_4, tau ) ; 	tj_des_angle_4	=	tj_des_angle ;
	trajectory(des_angle_5 , prv_des_angle_5, tau ) ; 	tj_des_angle_5	=	tj_des_angle ;
	update_flag	=	0 ;
	}
	
	//if ( xt > end_time && xt < end_time + 0.5 )
	else 
	{		
	tj_des_angle_1 	=	des_angle_1 ; prv_des_angle_1 = des_angle_1;
	tj_des_angle_2 	=	des_angle_2 ; prv_des_angle_2 = des_angle_2;
	tj_des_angle_3 	=	des_angle_3 ; prv_des_angle_3 = des_angle_3;
	tj_des_angle_4 	=	des_angle_4 ; prv_des_angle_4 = des_angle_4;
	tj_des_angle_5 	=	des_angle_5 ; prv_des_angle_5 = des_angle_5;
	update_flag 	=	1 ;
	}
	
	if ( xt == end_time  )	{ start_time =  end_time ;
				  end_time = end_time + 5 ; if ( indicate_flag == 0 ){ flag = flag + 1; }
							    else		     { flag = flag - 1 ;} 
				}
	
	if ( update_flag == 1 )
	{ 
	if ( flag == 0 )	{ IK(200,-200,100);desired_angles();prv_tj_t = xt ;indicate_flag = 0 ;}   // pick position
	if ( flag == 1 )	{ IK(200,-200,330);desired_angles();prv_tj_t = xt ;}                      // intermediate position 1
	if ( flag == 2 )	{ IK(200,200,330); desired_angles();prv_tj_t = xt ;}                      // intermediate position 2
	if ( flag == 3 )	{ IK(200,200,100); desired_angles();prv_tj_t = xt ;indicate_flag = 1 ;}   //  place position

  // YouTube link
  // https://youtu.be/8PYh98VjtcA

	//fprintf(outputfile,"\n\n\n\n\n\n\n\n\n\n\n\n");
	}

	


	
	algorithm() ; 
	velocity () ; 
	
	
	
	ret	=	PencOutputDO(PEX_encoder, out_data);	//printf("ret = %i\n ",ret);
	ret	=	DaOutputDA( PEX_analog, 4, DaSmplChReq, wData );		//printf("ret = %i\n ",ret);
	ret	=	AdInputDI(LPC, &digital_inputvalue);				//printf("DI value : %lu",digital_inputvalue);printf("   ");
	
	
	//printf(" %G %G %G %G %G %G %G %G %G %G %G %G %G %G %G %G %G \n",angle_1, angle_2, angle_3, angle_4, angle_5, tj_des_angle_1 , tj_des_angle_2 , tj_des_angle_3 , tj_des_angle_4 , tj_des_angle_5 ,vdes_1 ,vdes_2 ,vdes_3 ,vdes_4 ,vdes_5 , vdes_6, xt);
	//fprintf(outputfile, "%G %G %G %G %G %G %G %G %G %G %G %G %G %G %G %G \n",angle_1, angle_2, angle_3, angle_4, angle_5 , tj_des_angle_1 , tj_des_angle_2 , tj_des_angle_3 , tj_des_angle_4 , tj_des_angle_5 ,vdes_1 ,vdes_2 ,vdes_3 ,vdes_4 ,vdes_5 , xt);
	
	ret 	= 	DaOutputDA( LPC	      , 2, LPC_DaSmplChReq, LPC_wData );//printf("ret = %i\n ",ret);
	
	t.tv_nsec +=interval ;
	tsnorm(&t);
	}
	ret	=	DaClose( PEX_analog );		printf("ret = %i\n ",ret);
	ret	=	DaClose( LPC );			printf("ret = %i\n ",ret);
	ret	=	PencClose(PEX_encoder);		printf("ret = %i\n ",ret);
	ret	=	PencClose(PEX_analog);		printf("ret = %i\n ",ret);
	ret	=	AdClose(LPC);			printf("ret = %i\n ",ret);
	//fclose (outputfile);
	return 0;
	
}

void read()
{

  PencGetCounter(PEX_encoder , 1, &count_1);
  PencGetCounter(PEX_encoder , 2, &count_2);
  PencGetCounter(PEX_encoder , 3, &count_3);
  PencGetCounter(PEX_encoder , 4, &count_4);
  PencGetCounter(PEX_analog , 1, &count_5);
  
  counts_1  = ( double )count_1 - (double) pre_set_value ;
  counts_2  = ( double )count_2 - (double) pre_set_value ;
  counts_3  = ( double )count_3 - (double) pre_set_value ;
  counts_4  = ( double )count_4 - (double) pre_set_value ; 
  counts_5  = ( double )count_5 - (double) pre_set_value ;
  
  angle_1	=	360*counts_1/4000000.0;
  angle_2	=	360*counts_2/6668504.0;
  angle_3	=	360*counts_3/4400000.0;
  angle_4	=	360*counts_4/3600000.0;
  angle_5	=	360*counts_5/1721920.0;

}


int trajectory( double des_angle ,double prv_des_angle , double tau)
{
  tj_t 		=	xt - prv_tj_t ; 
  c_0		=	-2 * (des_angle - prv_des_angle) / ( tau * tau * tau ) ;
  c_1		=	3 * (des_angle  - prv_des_angle) / ( tau * tau ) ;
  c_2		=	0	;
  c_3		=	prv_des_angle	;
  tj_des_angle	=	( c_0 * tj_t * tj_t * tj_t ) + ( c_1 * tj_t * tj_t ) + ( c_2 * tj_t ) + c_3 ;
  return tj_des_angle ;
}


void algorithm()
{
  
  
  err_1		=	tj_des_angle_1 - angle_1 ;
  err_2		=	tj_des_angle_2 - angle_2 ;
  err_3		=	tj_des_angle_3 - angle_3 ;
  err_4		=	tj_des_angle_4 - angle_4 ;
  err_5		=	tj_des_angle_5 - angle_5 ;
  
  
  ingl_1	=	( err_1 * T )+ prv_ingl1 ;
  output_1	=	kp_1  * err_1 + ki_1 * ingl_1 ;
  ingl_2	=	( err_2 * T )+ prv_ingl2 ;
  output_2	=	kp_1  * err_2 + ki_2 * ingl_2 ;
  ingl_3	=	( err_3 * T )+ prv_ingl3 ;
  output_3	=	kp_1  * err_3 + ki_3 * ingl_3 ; 
  ingl_4	=	( err_4 * T )+ prv_ingl4 ;
  output_4	=	kp_1  * err_4 + ki_4 * ingl_4 ; 
  ingl_5	=	( err_5 * T )+ prv_ingl5 ;
  output_5	=	kp_1  * err_5 + ki_5 * ingl_5 ; 
  
  prv_ingl1 	=	ingl_1 ;
  prv_ingl2 	=	ingl_2 ;
  prv_ingl3 	=	ingl_3 ;
  prv_ingl4 	=	ingl_4 ;
  prv_ingl5 	=	ingl_5 ;
  
  output_1	=	abs(output_1) ;
  if 	  ( output_1 > 49152 ){ output_1 = 49152 ;  }
  else if ( output_1 < 32768 ){ output_1 = 32768 + ( output_1 / 2 ) ; } 
  else 			      { output_1 = output_1 ;  }
  
  output_2	=	abs(output_2) ;
  if 	  ( output_2 > 49152 ){ output_2 = 49152 ;  }
  else if ( output_2 < 32768 ){ output_2 = 32768 + ( output_2 / 2 ) ; } 
  else 			      { output_2 = output_2 ;  }
  
  output_3	=	abs(output_3) ;
  if 	  ( output_3 > 49152 ){ output_3 = 49152 ;  }
  else if ( output_3 < 32768 ){ output_3 = 32768 + ( output_3 / 2 ) ; } 
  else 			      { output_3 = output_3 ;  }
  
  output_4	=	abs(output_4) ;
  if 	  ( output_4 > 49152 ){ output_4 = 49152 ;  }
  else if ( output_4 < 32768 ){ output_4 = 32768 + ( output_4 / 2 ) ; } 
  else 			      { output_4 = output_4 ;  } 
  
  output_5	=	abs(output_5) ;
  if 	  ( output_5 > 49152 ){ output_5 = 49152 ;  }
  else if ( output_5 < 32768 ){ output_5 = 32768 + ( output_5 / 2 ) ; } 
  else 			      { output_5 = output_5 ;  }  
  
  vdes_6	=	12;
  if ( xt  >0.55 ){ vdes_6	=	0 ; }
  output_6	=	( vdes_6 * 16384 / 12 ) + 32768 ; 
  
  
  
  
  	wData[0]		= output_1 ; 
	wData[1]		= output_2 ; 
	wData[2]		= output_3 ;
	wData[3]		= output_4 ;
	LPC_wData[0]		= output_5 ;
	LPC_wData[1]		= output_6 ;
	
	vdes_1 		=	( wData[0] - 32768 ) * 24 / 16384  ;
	vdes_2 		=	( wData[1] - 32768 ) * 24 / 16384  ;
	vdes_3 		=	( wData[2] - 32768 ) * 24 / 16384  ;
	vdes_4 		=	( wData[3] - 32768 ) * 24 / 16384  ;
	vdes_5 		=	( LPC_wData[0] - 32768 ) * 24 / 16384  ;
	
	
	
  if ( err_1 >  0) { out_data = out_data & 65532; out_data = out_data | 2; vdes_1 = vdes_1 ;}
  else       	   { out_data = out_data & 65532; out_data = out_data | 1; vdes_1 = -vdes_1 ;}
  
  if ( err_2 >  0) { out_data = out_data & 65523; out_data = out_data | 8; vdes_2 = vdes_2 ;}	
  else       	   { out_data = out_data & 65523; out_data = out_data | 4; vdes_2 = -vdes_2 ;}
  
  if ( err_3 >  0) { out_data = out_data & 65487; out_data = out_data | 32; vdes_3 = vdes_3 ;}	
  else       	   { out_data = out_data & 65487; out_data = out_data | 16; vdes_3 = -vdes_3 ;}
  
  if ( err_4 >  0) { out_data = out_data & 65343; out_data = out_data | 128; vdes_4 = vdes_4 ;}	
  else       	   { out_data = out_data & 65343; out_data = out_data | 64; vdes_4 = -vdes_4 ;}
  
  if ( err_5 >  0) { out_data = out_data & 64767; out_data = out_data | 256; vdes_5 = vdes_5 ;}	
  else       	   { out_data = out_data & 64767; out_data = out_data | 512; vdes_5 = -vdes_5 ;}
  
 
  
  vdes_6	=	0;
  
  if ( vdes_6 > 0 ) { out_data = out_data & 53247; out_data = out_data | 8192;}	
  else 		    { out_data = out_data & 53247; out_data = out_data | 4096;}		
	
}


//gcc -o  5_PID_with_tj_IK_continuous 5_PID_with_tj_IK_continuous.c -lgpg3300 -lgpg6204 -lgpg3100 -lrt -lm -Wall
 
 
 // ./5_PID_with_tj_IK_continuous 1
	