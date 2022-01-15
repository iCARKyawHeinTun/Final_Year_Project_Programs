#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string.h>

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

unsigned short		wData[4];
unsigned short 		LPC_wData[2];

unsigned short 		AdData[256];
unsigned short 		AdData_1[256];
unsigned long 		count, Len;
int current_value , prv_value ;
int ii ;

unsigned short addata_array[10] = {};
double des_array[6]	=	{0.0,0.0,0.0,0.0,0.0,0.0};
double prv_des_array[5]	=	{0.0,0.0,0.0,0.0,0.0};
double output_average_1 = 0;
double output_average_2 = 0; 

unsigned long		limit_switches;

FILE *outputfile ;
FILE *trajectory ;

int ret ;
 
unsigned long out_data	=	0xFFFF;		// digital outputs from PEX_encoder for brakes // F3 both on// FB first on// F7 second on


#define pre_set_value 0x10000000
unsigned long count_1, count_2, count_3, count_4, count_5 ;
double counts_1, counts_2, counts_3, counts_4, counts_5  ;
double angle_1 , angle_2 , angle_3 , angle_4 , angle_5 ;
double des_angle_1 , des_angle_2 , des_angle_3 , des_angle_4 , des_angle_5 ;
double err_1 , err_2 , err_3 , err_4 , err_5 ;
double kp_1, kp_2, kp_3, kp_4, kp_5 ;
double ki_1 , ki_2 , ki_3 , ki_4 , ki_5 ; 
double ingl_1 , ingl_2 , ingl_3 , ingl_4 , ingl_5 ; 
double prv_ingl1 , prv_ingl2 , prv_ingl3 , prv_ingl4 , prv_ingl5 ; 
double output_1 , output_2 , output_3 , output_4 , output_5 , output_6 ;
double vdes_1 , vdes_2 , vdes_3 ,vdes_4 ,vdes_5  , vdes_6 ;
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

double prv_des_angle_1 , prv_des_angle_2 , prv_des_angle_3 , prv_des_angle_4 , prv_des_angle_5 , prv_des_angle , tj_t, prv_tj_t ;
double tj_des_angle_1 , tj_des_angle_2 , tj_des_angle_3 , tj_des_angle_4 , tj_des_angle_5 , tj_des_angle ;
double c_0 , c_1 , c_2 , c_3 , tau ;

int flag = 0 ;
int ii = 0 ;
int flag_for_traj_file = 0 ;
int flag_for_closing_file = 0 ;
int flag_for_finish = 0 ;
int tj_finish_flag	=	0	;
double aaa; 
int flag_for_tau	=	0 	;
int gripper_flag	=	0	;

char c[100];
char *p;
double value;


void gravity_counter_voltage() ;
void read();
void teaching_algorithm();
void replay_algorithm();
void friction_counter_volt() ; 
void velocity() ;
void volt_to_bit();
void direction();
void gains() ;
void desire_angles();
void home_position();
double  absolute(double input_value);
double  trajectory_generate( double des_angle ,double prv_des_angle , double tau);
void grip() ;
void ungrip() ;

struct timespec t;
struct sched_param param;

long interval=10000000;
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

ADSMPLREQ Conf;
  
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
	
	while (1)
	{
	clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);
	
	T 	=	0.01	;
	K++	;	
	xt	=	K * T  ;
	
	AdGetSamplingConfig(LPC, &Conf);
	Conf.ulChCount = 4;
	Conf.ulSingleDiff = AD_INPUT_SINGLE;
	Conf.ulSmplNum = 1;
	Conf.fSmplFreq = 1000.0;
	Conf.SmplChReq[0].ulChNo = 1;
	Conf.SmplChReq[0].ulRange = Conf.SmplChReq[1].ulRange;
	Conf.SmplChReq[1].ulChNo = 2;
	Conf.SmplChReq[1].ulRange = Conf.SmplChReq[1].ulRange;
	Conf.SmplChReq[2].ulChNo = 3;
	Conf.SmplChReq[2].ulRange = Conf.SmplChReq[1].ulRange;
	Conf.SmplChReq[3].ulChNo = 4;
	Conf.SmplChReq[3].ulRange = Conf.SmplChReq[1].ulRange;	
	
	AdSetSamplingConfig(LPC, &Conf);
	AdStartSampling(LPC, FLAG_SYNC);
	Len = 1;
   	AdGetSamplingData(LPC, &AdData_1[0], &Len);
//      	printf("output average1 = %u \n", AdData_1[0] );
	
//    	AdGetSamplingData(LPC, &AdData[1], &Len);	
//      	printf("output average2 = %u \n", AdData_1[1] );
	
//     	AdGetSamplingData(LPC, &AdData[0], &Len);	
//     	printf("output average3 = %u \n", AdData_1[2] );
	
//   	AdGetSamplingData(LPC, &AdData_1[3], &Len);
//    	printf("output average4 = %u \n", AdData_1[3] );
	
	read();
	
	
	
	if ( AdData_1[0] > 40960 ) // teaching_mode
	{
	  if ( flag_for_traj_file == 0  ){ trajectory	=	fopen("trajectory.txt","w+") ; flag_for_traj_file = 1;flag_for_closing_file = 0;}
	  printf("teaching mode       ");
	  gravity_counter_voltage();
	  velocity();
	  friction_counter_volt();
	  if ( AdData_1[3] > 48000 ){ grip(); }
	  else			    { ungrip(); }
	  teaching_algorithm() ;
	}
	if  ( AdData_1[1] > 40960 )//playing_back
	{
	  printf(" replay mode \n");
	  if ( flag_for_traj_file == 0 )
	  {   trajectory	=	fopen("trajectory.txt","r");flag_for_traj_file = 1;flag_for_closing_file = 0 ;}
	  desire_angles();
	  replay_algorithm();
	  if ( gripper_flag == 1 ){ grip(); }
	  else			    { ungrip(); }
	}
	if ( AdData_1[0] < 40960 && AdData_1[1] < 40960 )
	{
	  
	  if (flag_for_traj_file == 1 && flag_for_closing_file == 0 ){	fclose(trajectory);flag_for_closing_file = 1;flag_for_traj_file = 0 ;	}
	  
	  flag_for_traj_file = 0;
	  printf("home position\n");
	  // to go back to home position
	vdes_1			=	-1;//0.9;
	vdes_2			=	-5;//1.3;
	vdes_3			=	-3;//2.5;
	vdes_4			=	3;//3;
	vdes_5			=	10;//3;	  
	  
	AdInputDI(LPC,&limit_switches); 
	limit_switches 	=	limit_switches & 1 ;	
	if ( limit_switches == 0 ) { vdes_1	= 0;}
// 	printf("diginal value = %lu \n",limit_switches);
 	
	
	AdInputDI(LPC,&limit_switches); 
	limit_switches 	=	limit_switches & 2 ;	
	if ( limit_switches == 0 ) { vdes_2	= 0;}
//  	printf("diginal value = %lu \n",limit_switches);
	
	AdInputDI(LPC,&limit_switches); 
	limit_switches 	=	limit_switches & 4 ;	
	if ( limit_switches == 0 ) { vdes_3	= 0;}
//  	printf("diginal value = %lu \n",limit_switches);	
	
	AdInputDI(LPC,&limit_switches); 
	limit_switches 	=	limit_switches & 32 ;	
	if ( limit_switches == 0 ) { vdes_4	= 0;}
//  	printf("diginal value = %lu \n",limit_switches);
	
	AdInputDI(LPC,&limit_switches); 
	limit_switches 	=	limit_switches & 64 ;	
	if ( limit_switches == 0 ) { vdes_5	= 0;}	
//  	printf("diginal value = %lu \n",limit_switches);
	
	
	
	}
	
	
	
	
	volt_to_bit();
	direction();
	
	output_average_1	=	0 ;
	output_average_2	=	0 ;

	ret	=	PencOutputDO(PEX_encoder, out_data);
	ret	=	DaOutputDA( PEX_analog, 4, DaSmplChReq, wData );	
	
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
	fclose (trajectory);
	return 0;
}

// The joint angles are written to the text file named 'trajectory'.
void teaching_algorithm()
{
  vdes_1 = fri_volt_1  ;
  vdes_2 = fri_volt_2 - G_volt_2    ;
  vdes_3 = 1*fri_volt_3 - 1*G_volt_3    ;
  vdes_4 = fri_volt_4  ;
  vdes_5 = fri_volt_5  ;	
  
  if (flag == 0){ /* fprintf(trajectory,"%s\n"," "); */flag = 1; }
  
  printf( "%lf %lf %lf %lf %lf\n",angle_1,angle_2,angle_3,angle_4,angle_5);
	
	current_value	=	AdData_1[2] ;
//  	fprintf(trajectory,"%u \n " , AdData[count*2+2] ) ;
	if ( current_value < 48500){ current_value	=	30000; }	else			   { current_value	=	50000; }
	if ( current_value != prv_value && current_value == 50000){   fprintf(trajectory, "\n%lf %lf %lf %lf %lf %i\n",angle_1,angle_2,angle_3,angle_4,angle_5,gripper_flag); } 
	prv_value	=	current_value ;


}

// To replay the same movement
// the memorized angles from the file 'trajectory' are re-read and
// this algorithm is used to track the same trajectory.
void replay_algorithm()
{  
  
  err_1		=	tj_des_angle_1 - angle_1 ;
  err_2		=	tj_des_angle_2 - angle_2 ;
  err_3		=	tj_des_angle_3 - angle_3 ;
  err_4		=	tj_des_angle_4 - angle_4 ;
  err_5		=	tj_des_angle_5 - angle_5 ;
  
  ingl_1	=	( err_1 * T )+ prv_ingl1 ;
  vdes_1	=	kp_1  * err_1 + ki_1 * ingl_1 ;
  ingl_2	=	( err_2 * T )+ prv_ingl2 ;
  vdes_2	=	kp_1  * err_2 + ki_2 * ingl_2 ;
  ingl_3	=	( err_3 * T )+ prv_ingl3 ;
  vdes_3	=	kp_1  * err_3 + ki_3 * ingl_3 ; 
  ingl_4	=	( err_4 * T )+ prv_ingl4 ;
  vdes_4	=	kp_1  * err_4 + ki_4 * ingl_4 ; 
  ingl_5	=	( err_5 * T )+ prv_ingl5 ;
  vdes_5	=	kp_1  * err_5 + ki_5 * ingl_5 ; 
  
  prv_ingl1 	=	ingl_1 ;
  prv_ingl2 	=	ingl_2 ;
  prv_ingl3 	=	ingl_3 ;
  prv_ingl4 	=	ingl_4 ;
  prv_ingl5 	=	ingl_5 ;
  
//   printf("apply voltages : %lf %lf %lf %lf %lf\n" , vdes_1, vdes_2, vdes_3, vdes_4, vdes_5 ) ;
   fprintf(outputfile, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",tj_des_angle_1 , tj_des_angle_2 , tj_des_angle_3 , tj_des_angle_4 , tj_des_angle_5,angle_1 , angle_2 , angle_3 , angle_4 , angle_5,xt);
}

void desire_angles()
{
  
if ( tj_finish_flag	==	0 )  
{  
  char c[100];
  if ( fgets(c,70,trajectory ) == NULL  )
      {
	printf("NULL\n"); 
	flag_for_finish	=	1 ; 	
	
	des_angle_1	=	prv_des_angle_1 ;
	des_angle_2	=	prv_des_angle_2 ;
	des_angle_3	=	prv_des_angle_3 ;
	des_angle_4	=	prv_des_angle_4 ;
	des_angle_5	=	prv_des_angle_5 ;
      }
  if  ( fgets(c,70,trajectory ) != NULL )
  {
    
    printf (" reading text file " ) ;
//       printf("read data = %s  \n", c) ;
      p	=	strtok(c," ");
         while ( p!= NULL )
   {
//      printf("%s\n",p);
     value	=	atof(p) ; 
     ii++;
     des_array[ii]	=	value ;
//      printf("integer value	=	%lf\n",des_array[ii]);
     p		=	strtok (NULL," ");
   }
    ii = 0;
    des_angle_1	=	des_array[1] ;
    des_angle_2	=	des_array[2] ;
    des_angle_3	=	des_array[3] ;
    des_angle_4	=	des_array[4] ;
    des_angle_5	=	des_array[5] ;
    gripper_flag =      des_array[6] ;
    
    prv_tj_t	=	xt ;aaa = xt  ;
    tj_finish_flag	=	1;
  }   
  
  if ( flag_for_tau == 0  ){ tau = 5.0 ;flag_for_tau = 1 ; }
  else 			   { tau = 2.0 ; }
  
  }
  
  
  
  
  if ( xt < aaa+tau )
  {
  tj_des_angle_1	=	trajectory_generate(des_angle_1 ,prv_des_angle_1,tau) ;
  tj_des_angle_2	=	trajectory_generate(des_angle_2 ,prv_des_angle_2,tau) ;
  tj_des_angle_3	=	trajectory_generate(des_angle_3 ,prv_des_angle_3,tau) ;
  tj_des_angle_4	=	trajectory_generate(des_angle_4 ,prv_des_angle_4,tau) ;
  tj_des_angle_5	=	trajectory_generate(des_angle_5 ,prv_des_angle_5,tau) ;    
  }
   else
   {
     tj_des_angle_1	=	des_angle_1 ; prv_des_angle_1	=	des_angle_1 ;
     tj_des_angle_2	=	des_angle_2 ; prv_des_angle_2	=	des_angle_2 ;
     tj_des_angle_3	=	des_angle_3 ; prv_des_angle_3	=	des_angle_3 ;
     tj_des_angle_4	=	des_angle_4 ; prv_des_angle_4	=	des_angle_4 ;
     tj_des_angle_5	=	des_angle_5 ; prv_des_angle_5	=	des_angle_5 ;
     tj_finish_flag	=	0 ;
   }
   
  printf ( " desire angles are %lf %lf %lf %lf %lf \n",tj_des_angle_1, tj_des_angle_2, tj_des_angle_3 ,tj_des_angle_4 , tj_des_angle_5);   
  
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

void gains()
{
    kp_1	=	2.0;	ki_1	=	0.003;
    kp_2	=	10.0;	ki_2	=	0.003;
    kp_3	=	10.0;	ki_3	=	0.003;
    kp_4	=	10.0;	ki_4	=	0.003;
    kp_5	=	10.0;	ki_5	=	0.003;
}

double trajectory_generate( double des_angle ,double prv_des_angle , double tau)
{
  tj_t 		=	xt - prv_tj_t ; 
  c_0		=	-2.0 * (des_angle - prv_des_angle) / ( tau * tau * tau ) ;
  c_1		=	3.0 * (des_angle  - prv_des_angle) / ( tau * tau ) ;
  c_2		=	0.0	;
  c_3		=	prv_des_angle	;
  tj_des_angle	=	( c_0 * tj_t * tj_t * tj_t ) + ( c_1 * tj_t * tj_t ) + ( c_2 * tj_t ) + c_3 ;
  
  return tj_des_angle ;
}

// Read the joint encoders to measure angles
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

// gravity compensation algorithm for Joint 2 and Joint 3
// Euler-Lagrance's dynamics equations
// 2 Joints planner robot model
// masses are derived experimentally
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


// Friction compensation algorithm
// The require friction model are drived experimentally.
// Matlab is used to analyse the results from the experiment and find the required parameters.
void friction_counter_volt()
{
  if ( vel_1	> 0  ){    fri_volt_1 = 0.03528 * vel_1 + 0.11461  ;  }//0.4461
  if ( vel_1	< 0  ){    fri_volt_1 = 0.03696 * vel_1 - 0.1536   ;  }  //0.3536 
  if ( vel_2	> 0  ){    fri_volt_2 = 0.075 * vel_2  + 0.2 ;  }
  if ( vel_2	< 0  ){    fri_volt_2 = 0.075 * vel_2  - 0.2  ;  }  
  if ( vel_3	> 0  ){    fri_volt_3 = 0.06 * vel_3  +  0.6;  }
  if ( vel_3	< 0  ){    fri_volt_3 = 0.08 * vel_3  - 0.6 ;  }
  if ( vel_4	> 0  ){    fri_volt_4 = 0.06 * vel_4 + 0.7  ;  }
  if ( vel_4	< 0  ){    fri_volt_4 = 0.09 * vel_4 - 0.7   ;  } 
  if ( vel_5	> 0  ){    fri_volt_5 = 0.062 * vel_5 + 2.5 ;  }
  if ( vel_5	< 0  ){    fri_volt_5 = 0.062 * vel_5 - 2.5   ;  }   
}

void volt_to_bit()
{
  vcal_1	=	absolute(vdes_1) ;
  vcal_2	=	absolute(vdes_2) ;
  vcal_3	=	absolute(vdes_3) ;
  vcal_4	=	absolute(vdes_4) ;
  vcal_5	=	absolute(vdes_5) ;
    
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
}

void direction()
{
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


double absolute(double  input_value)
{
  if ( input_value >= 0.0 )	{ input_value = input_value ;}
  else 				{ input_value = -input_value; }
  return input_value;
}

void grip()
{
  vdes_6		=	24 ;
  output_6		=	( vdes_6 * 16384 / 24 ) + 32768 ; 
  LPC_wData[1]		= 	output_6 ;
  out_data = out_data & 53247; out_data = out_data | 8192;
  gripper_flag		=	1  ; 
}

void ungrip()
{
  vdes_6		=	24 ;
  output_6		=	( vdes_6 * 16384 / 24 ) + 32768 ; 
  LPC_wData[1]		= 	output_6 ;
  out_data = out_data & 53247; out_data = out_data | 4096;  
  gripper_flag		=	0  ; 
}

//gcc -o lead  lead_through_programming.c -lgpg3300 -lgpg6204 -lgpg3100 -lrt -lm -Wall

// ./lead 1

// cd /home/researcher/Desktop/thesis/lead_through_programming

// cd /home/researcher/Desktop/thesis/home

// ./home_position



