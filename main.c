/*
 * Lab 7: Velocity Control of a DC Motor
 * Author: Nor Izzanny
 * Date: March 14 2025
 

/* includes */
#include <stdio.h>
#include "MyRio.h"
#include "T1.h"
#include "ctable2.h"
#include "TimerIRQ.h"
#include "IRQConfigure.h"
#include <pthread.h>
#include <math.h>
#include "AIO.h"
#include "matlabfiles.h"

struct biquad {// biquad struct of coefficients
	double b0; double b1; double b2;
	double a0; double a1; double a2;
	double x0; double x1; double x2;
	double y1; double y2;
};

/* prototypes */

int32_t Irq_RegisterTimerIrq(MyRio_IrqTimer* irqChannel,
                             NiFpga_IrqContext* irqContext,
                             uint32_t timeout);

void Irq_Wait(NiFpga_IrqContext irqContext,
		NiFpga_Irq irqNumber,
		uint32_t *irqAssert,
		NiFpga_Bool *continueWaiting);


void *Timer_ISR(void *resource);

int pthread_create(pthread_t *thread,
		const pthread_attr_t *attr,
		void *(*start_routine) (void *),
		void *arg);

NiFpga_Status EncoderC_initialize(NiFpga_Session myrio_session,
		MyRio_Encoder *channel);
uint32_t Encoder_Counter(MyRio_Encoder *channel);

double cascade(double xin,         // input
               struct biquad *fa,  // biquad array
               int    ns,          // no. segments
               double ymin,        // min output
               double ymax);       // max output

void Aio_InitCO0(MyRio_Aio *AOC0);
void Aio_Write(MyRio_Aio *channel, double value);

MATFILE *openmatfile(char *fname, int *err);

int matfile_addmatrix(
  MATFILE *mf,
  char *name,
  double *data,
  int m,
  int n,
  int transpose
);

int	ctable2(char *TableTitle, table *my_table, int nval);

/* definitions */
#define IMAX 250 //buflen
#define SATURATE(x,lo,hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))



typedef struct {
	NiFpga_IrqContext irqContext;
	table *a_table;
	NiFpga_Bool irqThreadRdy;
} ThreadResource;

char *Table_Title = "Parameters";
table my_table[]={
		{"V_R: rpm   ",1,0.0},
		{"V_J: rpm   ",0,0.0},
    	{"VDAout: mV ",0,0.0},
    	{"Kp: (V-s/r)",1,0.0},
    	{"Ki: (V/r)  ",1,0.0},
    	{"BTI: ms    ",1,5.0}
    };

int32_t irq_status;
MyRio_IrqTimer irqTimer0;
ThreadResource irqThread0;
pthread_t thread;
NiFpga_Session myrio_session;
MyRio_Encoder encC0;

double vel(void);
static double Omega_JBUF[IMAX];
static double *Omega_JPTR = Omega_JBUF;
static double VDAoutBUF[IMAX];
static double *VDAoutPTR = VDAoutBUF;
double prev_Omega_R = 0.0;

int main(int argc, char **argv)
{
	NiFpga_Status status;

    status = MyRio_Open();		    			// open FPGA session
    if (MyRio_IsNotSuccess(status)) return status;

    //my code here

    irqTimer0.timerWrite = IRQTIMERWRITE;//IRQ channel registers
    irqTimer0.timerSet = IRQTIMERSETTIME;
    uint32_t timeoutValue = 500;

    irq_status = Irq_RegisterTimerIrq(&irqTimer0,
    		&irqThread0.irqContext,
       		timeoutValue);

    irqThread0.a_table = my_table;
    irqThread0.irqThreadRdy = NiFpga_True;//indicator to allow new thread

    irq_status = pthread_create(&thread,// create new thread to catch IRQ
    		NULL,
       		Timer_ISR,
       		&irqThread0);

    ctable2(Table_Title, my_table, 6); //call table editor

    irqThread0.irqThreadRdy = NiFpga_False; // signal thread to terminate
    irq_status = pthread_join(thread, NULL);

    irq_status = Irq_UnregisterTimerIrq(&irqTimer0,
    		irqThread0.irqContext);


	status = MyRio_Close();						// close FPGA session

	return status;
}

double value;
void* Timer_ISR(void *resource){

	ThreadResource *threadResource = (ThreadResource*) resource;// cast input resource

	double *Omega_R = &((threadResource->a_table + 0)->value);// declare names for table entries
	double *Omega_J = &((threadResource->a_table + 1)->value);
	double *VDAout = &((threadResource->a_table + 2)->value);
	double *Kp = &((threadResource->a_table + 3)->value);
	double *Ki = &((threadResource->a_table + 4)->value);
	double *BTI = &((threadResource->a_table + 5)->value);

	MyRio_Aio AIC0; //declare input outputs
	MyRio_Aio AOC0;
	MyRio_Aio AOC1;
	Aio_InitCI0(&AIC0);
	Aio_InitCO0(&AOC0);
	Aio_InitCO1(&AOC1);

	Aio_Write(&AOC0, 0.0); //initialize output

	 /*initialize encoder interface*/
	    EncoderC_initialize(myrio_session,&encC0);// set up encoder interface

	    struct biquad myFilter[]={{0., 0., 0., 1., -1., 0., 0., 0., 0., 0., 0.}}; // init biquad with a0=1, a1=-1

	while (threadResource->irqThreadRdy == NiFpga_True){// while loop processing each interrupt
			uint32_t irqAssert = 0;
			Irq_Wait(threadResource->irqContext,// pause loop waiting for interrupt
					TIMERIRQNO,
					&irqAssert,
					(NiFpga_Bool*) &(threadResource->irqThreadRdy));

			if (irqAssert & ( 1 << TIMERIRQNO )){// check if Timer IRQ asserted
				uint32_t timeoutValue = *BTI*1000;

				NiFpga_WriteU32(myrio_session,
					IRQTIMERWRITE,
					timeoutValue);
				NiFpga_WriteBool(myrio_session,
					IRQTIMERSETTIME,
					NiFpga_True);

				*Omega_J = vel()/2048./(*BTI/1000.)*60.; // rpm

				myFilter->b0 =  *Kp + *Ki/2.0 * *BTI/1000.;// Update current coefficients in biquad
				myFilter->b1 = -*Kp + *Ki/2.0 * *BTI/1000.;

				double error;

				error = (*Omega_R - *Omega_J)*2*M_PI/60.; // rad/s

				*VDAout = cascade(error, myFilter, 1, -10, 10);// compute measured velocity

				Aio_Write(&AOC0,*VDAout);// write measured velocity



				if (*Omega_R != prev_Omega_R){// define prev omegaR for matlab
					prev_Omega_R=*Omega_R;
				}

				if(Omega_JPTR < Omega_JBUF + IMAX){
								*Omega_JPTR++ = *Omega_J;//store in buffer
				}

				if(VDAoutPTR < VDAoutBUF + IMAX){//store in buffer
					*VDAoutPTR++ = *VDAout;
				}

				Irq_Acknowledge(irqAssert);//acknowledge interrupt
			}
		}

	int err;
	double ref_vels[2]={*Omega_R,prev_Omega_R};//save data to matlab file
	MATFILE *mf;
	mf = openmatfile("Lab7Izzanny.mat", &err);
	if(!mf) printf("Can't open mat file %d\n", err);
	matfile_addstring(mf, "myName", "Izzanny");
	matfile_addmatrix(mf, "Omega_J", Omega_JBUF, IMAX, 1, 0);
	matfile_addmatrix(mf, "VDAout", VDAoutBUF, IMAX, 1, 0);
	matfile_addmatrix(mf, "ref_vels", ref_vels, 2, 1, 0);
	matfile_addmatrix(mf, "Kp", Kp, 1, 1, 0);
	matfile_addmatrix(mf, "Ki", Ki, 1, 1, 0);

	double T = *BTI / 1000.0; // convert units
	matfile_addmatrix(mf, "T", &T, 1, 1, 0);
	matfile_close(mf);

	pthread_exit(NULL);										//terminate thread
		return NULL;
}

static int cn;
static int cn_1;

double vel(void){//compute velocity
	static int firstcall=1;
    cn=Encoder_Counter(&encC0);
    if (firstcall){
    	cn_1=cn;
    	firstcall=0;
    }
    double speed=cn-cn_1;
    cn_1=cn;

    return speed;
}

double cascade(double xin,
               struct biquad *fa,
               int    ns,
               double ymin,
               double ymax){
	double y0;
	struct biquad *f = fa;

	int i;

	for (i = 0; i < ns; i++){// implement difference equation for output approximation
		if (i==0){
			f->x0 = xin;
		}else{
			f->x0 = y0;
		}

		y0 =(f->b0 * f->x0 + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2) / f->a0;

		f->x2 = f->x1;// reassign next values
		f->x1 = f->x0;
		f->y2 = f->y1;
		if (i==(ns-1)){
			y0 = SATURATE(y0, ymin, ymax); //correct output values
		}
		f->y1 = y0;

		f++;

	}
	return y0;
}

