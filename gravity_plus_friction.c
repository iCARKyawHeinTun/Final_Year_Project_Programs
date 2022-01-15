// sensorless force-sensing
// Gravity Compensation ( 2 Joints Planner Model )
// Friction Compensation ( Stribeck Friction Model )
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
int flag 		= 	0 ;
int gripper_flag 	= 	0 ;
int update_flag 	= 	0;
int home_flag 		= 	0;
int indicate_flag 	= 	0;
int grip_flag		=	0;
int ungrip_flag 	=	0;

unsigned short		wData[4];
unsigned short 		LPC_wData[2];

FILE *outputfile ;

int ret ;
 
unsigned long out_data	=	0xFFFF;		// digital outputs from PEX_encoder for brakes // F3 both on// FB first on// F7 second on


#define pre_set_value 0x10000000
unsigned long count_1, count_2, count_3, count_4, count_5 ;
double counts_1, counts_2, counts_3, counts_4, counts_5  ;
double angle_1 , angle_2 , angle_3 , angle_4 , angle_5 ;
double output_1 , output_2 , output_3 , output_4 , output_5 , output_6 ;
double vdes_1 , vdes_2 , vdes_3 ,vdes_4 ,vdes_5  ;
double vel_1 , vel_2 , vel_3 , vel_4, vel_5 ;
double prv_ang_1 , prv_ang_2 , prv_ang_3 , prv_ang_4 , prv_ang_5 ;
double m1, m2, l1, l2, g , pi ;
double thgd2, thgd3;
double thg2, thg3 ; 
double alpha ;
double p1, p2 ;
double tau2 ,tau3 ;
double G_volt_2 ;
double G_volt_3 ;
double fri_volt_1 , fri_volt_2, fri_volt_3 , fri_volt_4 ,fri_volt_5 ;
double vcal_1 ,vcal_2 , vcal_3 ,vcal_4 ,vcal_5 ;

void gravity_counter_voltage() ;
void read();
void desired_angles();
void algorithm();
void friction_counter_volt() ; 
void velocity() ;



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
  
  outputfile	=	fopen("outputfile.txt","w+") ;
  
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
	
	while (1)
	{
	clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);
	
	T 	=	0.001	;
	K++	;	
	xt	=	K * T  ;
	
	read();
	algorithm() ; 
	velocity() ; 
	gravity_counter_voltage() ; 
	friction_counter_volt() ;

	ret	=	PencOutputDO(PEX_encoder, out_data);
	ret	=	DaOutputDA( PEX_analog, 4, DaSmplChReq, wData );
	ret	=	AdInputDI(LPC, &digital_inputvalue);		
	
	
	printf(" %lf %lf %lf %lf %lf    \n",vdes_1,vdes_2, vdes_3 ,vdes_4, vdes_5);
	fprintf(outputfile, "%lf %lf %lf %lf %lf  \n",vdes_1, vdes_2, vdes_3 ,vdes_4 ,vdes_5 );
	
	ret 	= 	DaOutputDA( LPC	      , 2, LPC_DaSmplChReq, LPC_wData );
	
	t.tv_nsec +=interval ;
	tsnorm(&t);
	}
	ret	=	DaClose( PEX_analog );		printf("ret = %i\n ",ret);
	ret	=	DaClose( LPC );			printf("ret = %i\n ",ret); 
	ret	=	PencClose(PEX_encoder);		printf("ret = %i\n ",ret);
	ret	=	PencClose(PEX_analog);		printf("ret = %i\n ",ret);
	ret	=	AdClose(LPC);			printf("ret = %i\n ",ret);
	fclose (outputfile);
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


// This algorithm sense the applied human force 
// Since the friction force is a function of velocity
// when the human operator apply force to the robotic arm
// there is slight change in the encoder readings 
// which can give out a value from the friction compensator and
// feed-forward or apply a voltage in the same direction
// 
// YouTube Link
// https://youtu.be/4U_QQx4yF1k
void algorithm()
{
  vdes_1 = fri_volt_1  ;
  vdes_2 = 1* fri_volt_2 - G_volt_2    ;
  vdes_3 = 1*fri_volt_3 - 1*G_volt_3    ;
  vdes_4 = fri_volt_4  ;
  vdes_5 = fri_volt_5  ;
  
  
  vcal_1	=	abs(vdes_1) ;
  vcal_2	=	abs(vdes_2) ;
  vcal_3	=	abs(vdes_3) ;
  vcal_4	=	abs(vdes_4) ;
  vcal_5	=	abs(vdes_5) ;
  
//    vcal_1	=	0 ; 
//      vcal_2	=	0 ; 
  // vcal_3	=	0 ; 
   //vcal_4	=	0 ; 
   //vcal_5	=	0 ; 
  
  output_1	=	((vcal_1 * 16384)  / 24.0 ) + 32768 ;	  
  output_2	=	((vcal_2 * 16384)  / 24.0 ) + 32768 ;
  output_3	=	((vcal_3 * 16384)  / 24.0 ) + 32768 ;
  output_4	=	((vcal_4 * 16384)  / 24.0 ) + 32768 ;
  output_5	=	((vcal_5 * 16384)  / 24.0 ) + 32768 ;
  

  
  
  output_1	=	abs(output_1) ;
  if 	  ( output_1 > 49152 ){ output_1 = 49152 ;  }
  else if ( output_1 < 32768 ){ output_1 = 32768 ;} 
  else 			      { output_1 = output_1 ;  }
  
  output_2	=	abs(output_2) ;
  if 	  ( output_2 > 49152 ){ output_2 = 49152 ;  }
  else if ( output_2 < 32768 ){ output_2 = 32768  ; } 
  else 			      { output_2 = output_2 ;  }
  
  output_3	=	abs(output_3) ;
  if 	  ( output_3 > 49152 ){ output_3 = 49152 ;  }
  else if ( output_3 < 32768 ){ output_3 = 32768 ; } 
  else 			      { output_3 = output_3 ;  }
  
  output_4	=	abs(output_4) ;
  if 	  ( output_4 > 49152 ){ output_4 = 49152 ;  }
  else if ( output_4 < 32768 ){ output_4 = 32768 ; } 
  else 			      { output_4 = output_4 ;  } 
  
  output_5	=	abs(output_5) ;
  if 	  ( output_5 > 49152 ){ output_5 = 49152 ;  }
  else if ( output_5 < 32768 ){ output_5 = 32768 ; } 
  else 			      { output_5 = output_5 ;  }  
  
  	wData[0]		= output_1 ; 
	wData[1]		= output_2 ; 
	wData[2]		= output_3 ;
	wData[3]		= output_4 ;
	LPC_wData[0]		= output_5 ;	
	
  if ( vdes_1 >  0) { out_data = out_data & 65532; out_data = out_data | 2; } //2
  else       	   { out_data = out_data & 65532; out_data = out_data | 1; } //1
  
  if ( vdes_2 >  0) { out_data = out_data & 65523; out_data = out_data | 8;}	//8
  else       	   { out_data = out_data & 65523; out_data = out_data | 4; } //4
  
  if ( vdes_3 >  0) { out_data = out_data & 65487; out_data = out_data | 32;}	//32
  else       	   { out_data = out_data & 65487; out_data = out_data | 16; }//16
  
  if ( vdes_4 >  0) { out_data = out_data & 65343; out_data = out_data | 128; }	//128
  else       	   { out_data = out_data & 65343; out_data = out_data | 64; } //64
  
  if ( vdes_5 >  0) { out_data = out_data & 64767; out_data = out_data | 256;}	 // 256
  else       	   { out_data = out_data & 64767; out_data = out_data | 512; }  //512 
	
}
void velocity()
{
  vel_1	=	( angle_1 - prv_ang_1 ) / T ;
  vel_2	=	( angle_2 - prv_ang_2 ) / T ;
  vel_3	=	( angle_3 - prv_ang_3 ) / T ;
  vel_4	=	( angle_4 - prv_ang_4 ) / T ;
  vel_5	=	( angle_5 - prv_ang_5 ) / T ;
  
  prv_ang_1	=	angle_1 ;
  prv_ang_2	=	angle_2 ;
  prv_ang_3	=	angle_3 ;
  prv_ang_4	=	angle_4 ;
  prv_ang_5	=	angle_5 ;
}

// Gravity Compensation 
void gravity_counter_voltage()
{

m1  =   7.0 ; 
m2  =  5.0 ; // 5.0
l1  = 	0.25 ; 
l2  =   0.18 ; 
g   =   9.8 ; 
pi  = 	3.142 ;

thgd2 =  100-angle_2 ;
thgd3 =  angle_3 ;

thg2 = thgd2 * pi / 180.0 ;
thg3 = thgd3 * pi / 180.0 ;

alpha = thg2 -thg3 ;

if ( alpha < 0  )	{ alpha = -alpha ; }
else			{ alpha =  alpha ; }


p1 = (m1+m2)*g*l1*cos(thg2) ;
p2 = m2*g*l2*cos(alpha) ; 


tau2 = p1 +  p2 ;
tau3 = p2  ; 

G_volt_2 = tau2 / 10 	;
G_volt_3 = tau3 / 5 	;

}

// Friction compensation Models
void friction_counter_volt()
{
  if ( vel_1	> 0  ){    fri_volt_1 = 0.03528 * vel_1 + 0.4461  ;  }
  if ( vel_1	< 0  ){    fri_volt_1 = 0.03696 * vel_1 - 0.3536   ;  }  
  if ( vel_2	> 0  ){    fri_volt_2 = 0.075 * vel_2  ;  }
  if ( vel_2	< 0  ){    fri_volt_2 = 0.075 * vel_2   ;  }  
  if ( vel_3	> 0  ){    fri_volt_3 = 0.06 * vel_3  +  0.6;  }
  if ( vel_3	< 0  ){    fri_volt_3 = 0.08 * vel_3  - 0.6 ;  }
  if ( vel_4	> 0  ){    fri_volt_4 = 0.06 * vel_4 + 0.7  ;  }
  if ( vel_4	< 0  ){    fri_volt_4 = 0.09 * vel_4 - 0.7   ;  } 
  if ( vel_5	> 0  ){    fri_volt_5 = 0.062 * vel_5 + 2.5  ;  }
  if ( vel_5	< 0  ){    fri_volt_5 = 0.062 * vel_5 - 2.5   ;  }   

}

//gcc -o GF  gravity_plus_friction.c -lgpg3300 -lgpg6204 -lgpg3100 -lrt -lm -Wall

// ./GF 1

// cd /home/researcher/Desktop/thesis/gravity_plus_friction

// cd /home/researcher/Desktop/thesis/home

// ./home_position
	


