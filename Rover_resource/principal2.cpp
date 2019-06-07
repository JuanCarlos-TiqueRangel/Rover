#define G_SI 9.80665
#define PI   3.141592653589793

// IMU
#include "Common/MPU9250.h"
#include "Common/Util.h"
#include <unistd.h>
#include <string>
#include <memory>
// Time
#include <sys/time.h>
// PPM
#include <cstdio>
#include <Navio2/RCInput_Navio2.h>
#include <Navio+/RCInput_Navio.h>
#include <Common/Util.h>
#define READ_FAILED -1
// PWM
#include "Navio2/PWM.h"
#include "Navio+/RCOutput_Navio.h"
#include "Navio2/RCOutput_Navio2.h"
#define SERVO_MIN 1250 /*mS*/
#define SERVO_MAX 1750 /*mS*/
#define Dir 0
#define Vel 1
// LED
#include <Navio2/Led_Navio2.h>
#include <Navio+/Led_Navio.h>
// I2C
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <fcntl.h>
// RS232
#include "Common/rs232.h"
#include "Common/rs232.c"
// AHRS
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include "AHRS.hpp"
#include "AHRS.cpp"



#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

// Filtro de Kalman
#include <Common/matrix.h>
#include <Common/matrix.cpp>

#include <iostream>
#include <fstream>

using namespace Navio;

char hola[2];
int inicial=0;

float Ts = 50.0;//100.0;	// mseg
struct timeval t_ini;
struct timeval t_fin;

double secs;

float salidas[2];

float ajusteyaw=0.0,UL,UR,Velocidad,Giro;
int giro=0, correccion=0;
float Vr,Vl;
float r=0.2;

int contGPS=0;

int  ref=0;
bool contaux=1;

float XN,YN,angle;

int contPd=1;

//=============================================================================
// Kalman filter
float X_e,Y_e,T_e;

//=============================================================================
// RS232
int	n4,n5;

int	cport_nr=38;		// /dev/tty_Roboteq	-> Roboteq
int	cport_nr3=17;		// /dev/ttyUSB1	-> GPS
int	cport_nr4=22;		// /dev/ttyAMA0	-> RTK
int	cport_nr5=16;		// /dev/ttyUSB0	-> Telemetria
int	cport_nr6=18;		// /dev/ttyUSB2	-> XSens

unsigned char buf[4096];
unsigned char buf1[4095];
unsigned char buf3[4096];
unsigned char buf4[4096];
unsigned char buf5[4096];
unsigned char buf6[4096];

int		bdrate=115200;		// 115200 baud
int		bdrate3=115200;		// 115200 baud  GPS
int		bdrate4=57600;		// 115200 baud  RTK
int		bdrate5=57600;		// 57600  baud  Telemetria 
int		bdrate6=115200;		// 115200 baud  X

char mode[]={'8','N','1',0};

int	telemetria=0;
int	telemetryAll=0;
unsigned char enviar[1]; //para saltar linea telemetria
char	str[3][50];
char	strE[50];
char	strN[50];
char	strAuxAll[50];
char	strI[5];
int		fin=0;

//=============================================================================
// Trajectory Generator and Control
float pX[10]={0.0};
float pY[10]={0.0};
int	  p=0;

float UV=10;
float UW=10;
float Xd,Xd1;					//posicion deseada en X
float Yd,Yd1;					//posicion deseada en Y
float Xk=0.0;
float Yk=0.0;

float errorTheta;
float Thetad, alpha, ThetaR;
float Vd;					//velocidad lineal deseada
float Vdmax;				//velocidad lineal deseada maxima
float Uit[2]={0.0,0.0};	
float Udt[2]={0.0,0.0};
float Wd[2]={0.0,0.0};
float Wmax=2.0;
float Vmax=2.0;
float errort[2]={0.0,0.0};
float Upt=0.0;
float DWd[2]={0.0,0.0};



float X_1,Y_1;
float pos_df;
int   contP=0; 

//=============================================================================
// I2C
const int HMC5883L_I2C_ADDR = 0x1E;
unsigned char buf2[16];
int fd2;
short x,y,z;

//=============================================================================
double timeval_diff(struct timeval *a, struct timeval *b){
  return    (double)(a->tv_sec + (double)a->tv_usec/1000000) - (double)(b->tv_sec + (double)b->tv_usec/1000000);
}
double time_sec(struct timeval *a){
  return    (double)(a->tv_sec + (double)a->tv_usec/1000000);
}

//=============================================================================
// PWM
auto pwm = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
int PWM_config(){
	if (getuid()) {
        fprintf(stderr, "Not root.\n");
    }
	if( !(pwm->initialize(Dir)) || !(pwm->initialize(Vel)) ) {
        return 1;
    }
	pwm->set_frequency(Dir, 50);
	pwm->set_frequency(Vel, 50);
	if ( !(pwm->enable(Dir)) || !(pwm->enable(Vel)) ) {
	    return 1;
	}
}
//=============================================================================
// I2C
void selectDevice(int fd2, int addr, char * name){
    if (ioctl(fd2, I2C_SLAVE, addr) < 0){
        fprintf(stderr, "%s not present\n", name);
        //exit(1);
    }
}

void writeToDevice(int fd2, int reg, int val){
    buf2[0]=reg;
    buf2[1]=val;

    if (write(fd2, buf2, 2) != 2){
        fprintf(stderr, "Can't write to ADXL345\n");
        //exit(1);
    }
}

int DatosI2C(){
	writeToDevice(fd2, 0x02, 0x01);		// Configura modo continuo de medicion
	a:
	usleep(7000);
	int aux=6;
	buf2[0] = 0x03;

	if ((write(fd2, buf2, 1)) != 1){ // Send the register to white from
		fprintf(stderr, "Error writing to I2C slave\n");
		goto a;
	}

	if (read(fd2, buf2, aux) != aux)
		fprintf(stderr, "Unable to read from HMC5883L\n");
	else{
		x = (buf2[0] << 8) | buf2[1];
        y = (buf2[4] << 8) | buf2[5];
		z = (buf2[2] << 8) | buf2[3];
	}
	
	if ( (y<-600) || (y>600) )	goto a;
	if ( (x<-600) || (x>600) )	goto a;
}

//=============================================================================
// RS232
int readTelemetry(){
	int apuntador=999,apuntador2=999;
	int conttele=0;
	fin=0;
	unsigned char bufaux[4095];//[650];
	
	// Read TELEMETRIA
	usleep(1000000);
	bufaux[0]=0;
	n5 = RS232_PollComport(cport_nr5, bufaux, 4095);
	// printf ("buf= |%s|\n", bufaux);
	// printf ("n= %d\n", n5);
	if(n5 > 25){
		bufaux[n5] = 0;   /* always put a "null" at the end of a string! */
		for(int i=0; i < n5; i++){
			if(bufaux[i] < 32)	bufaux[i] = '.';	/* replace unreadable control-codes by dots 48*/
		}
		//	Encontrar cabezera
		for (int i=0;i<n5;i++){
			if (bufaux[i]=='$' && apuntador == 999){
				apuntador=i;
				// printf ("apuntador= %d\n", apuntador);
			}
			if (apuntador!=999){
				if (bufaux[i]!='#' && bufaux[i+1]!='#'){
					buf5[conttele]=bufaux[i];
					conttele++;
				}
				else{
					buf5[conttele]=0;
					i=4100;
					fin=1;
				}
			}
		}
		// printf ("|%s|\n", bufaux);
		// for(int i=10; i < n5; i++){
			// if(buf5[i]=='#'&& buf5[i+1]=='#' )		fin=1;
			// if(fin)			buf5[i]= 0;
		// }
		// printf ("Fin= %d\n", fin);
		// printf ("|%s|\n", buf5);
		// printf ("|%s|\n", buf5);
		buf5[conttele]=' ';
		// printf ("|%s|\n", buf5);
		return fin;
	}
}

/*int SerialGPS(float E,float N){
	// Envio Telemetria
	// RS232_SendBuf(cport_nr5,buf3,n3);
  	if(telemetria > 10){
		strcpy(str[0], "#");
		RS232_cputs(cport_nr5, str[0]);
		
		snprintf(strE, sizeof strE, "%f", E);
		RS232_cputs(cport_nr5, strE);
		
		strcpy(str[1], "%");
		RS232_cputs(cport_nr5, str[1]);
		
		snprintf(strN, sizeof strN, "%f", N);
		RS232_cputs(cport_nr5, strN);
		
		strcpy(str[2], "&\n");
		RS232_cputs(cport_nr5, str[2]);
		
		enviar[0]=13;	// con este simbolo bajamos un reglon para una buena vizualizacion.
		RS232_SendBuf(cport_nr5,enviar,1);
		telemetria=0;
	}
	telemetria = telemetria + 1; 
    // printf("enviar");
	// enviar[0]=13;	// con este simbolo bajamos un reglon para una buena vizualizacion.
	// RS232_SendBuf(cport_nr5,enviar,1);
}*/

/* int SerialGPSyRTK(float E,float N,float E_RTK,float N_RTK){
	// Envio Telemetria
  	if(telemetria > 10){
		strcpy(str[0], "#");
		RS232_cputs(cport_nr5, str[0]);
		
		snprintf(strE, sizeof strE, "%f", E);
		RS232_cputs(cport_nr5, strE);
		
		strcpy(str[1], "%");
		RS232_cputs(cport_nr5, str[1]);
		
		snprintf(strN, sizeof strN, "%f", N);
		RS232_cputs(cport_nr5, strN);
		
		strcpy(str[0], "$");
		RS232_cputs(cport_nr5, str[0]);
		
		snprintf(strE, sizeof strE, "%f", E_RTK);
		RS232_cputs(cport_nr5, strE);
		
		strcpy(str[1], "?");
		RS232_cputs(cport_nr5, str[1]);
		
		snprintf(strN, sizeof strN, "%f", N_RTK);
		RS232_cputs(cport_nr5, strN);
		
		strcpy(str[2], "&\n");
		RS232_cputs(cport_nr5, str[2]);
		
		enviar[0]=13;	// con este simbolo bajamos un reglon para una buena vizualizacion.
		RS232_SendBuf(cport_nr5,enviar,1);
		telemetria=0;
	}
	telemetria = telemetria + 1; 
	// enviar[0]=13;	// con este simbolo bajamos un reglon para una buena vizualizacion.
	// RS232_SendBuf(cport_nr5,enviar,1);
} */

int SerialAll(float buf_datosc[70]){
//---------- envio Telemetria ---
  	// if(telemetryAll > 10){
		// printf("puntos = %d\n",p);
		strcpy(str[0], "#");
		RS232_cputs(cport_nr5, str[0]);
		strcpy(str[0], "%");
		RS232_cputs(cport_nr5, str[0]);
		for (int i = 0; i < 70; ++i){
			// printf("dato%d = %.4f\n",i,buf_datosc[i]);
			
			snprintf(strI, sizeof strI, "%d!", i);
			RS232_cputs(cport_nr5, strI);
		
			snprintf(strAuxAll, sizeof strAuxAll, "%f", buf_datosc[i]);
			RS232_cputs(cport_nr5, strAuxAll);
			
			strcpy(str[1], "%");
			RS232_cputs(cport_nr5, str[1]);
			usleep(50);
		}
		
		strcpy(str[2], "&\n");
		RS232_cputs(cport_nr5, str[2]);
		
		enviar[0]=13;	// con este simbolo bajamos un reglon para una buena vizualizacion.
		RS232_SendBuf(cport_nr5,enviar,1);
		// telemetryAll=0;
	// }
}

//=============================================================================
// GPS2UTM
// double Xm,Ym,XmRTK,YmRTK;
# include "GPS2UTM.cpp"

//=============================================================================
// RoboteQ
# include "ROBOTEQ.cpp"

//=============================================================================
// Xsens
// # include "example_xsens.cpp"

//=============================================================================
// RTK
# include "RTK.cpp"

//=============================================================================
// GPS
# include "GPS.cpp"

//=============================================================================
// Kalman
// #include "Kalman.cpp"
// #include "KalmanV1.cpp"
#include "KalmanLP.cpp"
#include "KalmanLP_o.cpp"

//=============================================================================
// AHRS
// Orientation data
float roll, pitch, yaw;


//=============================================================================
// Save Data
bool auxA1=0;
float buf_datosc[70];
#include "SavingData.cpp"

//=============================================================================
// Compass Calibration

float Xsf=0.97,Ysf=1.03,Xoff=37.74,Yoff=112.13;
float Xo=477906.72,Yo=491725.97;
void Calibration(){
	short Xmax=0,Xmin=0,Ymax=0,Ymin=0;
	int prom=0;
	printf("Beginning compass ang GPS calibration...\n");
	printf("Xsf= %.2f	Ysf= %.2f	Xoff= %.2f	Yoff= %.2f\n",Xsf,Ysf,Xoff,Yoff);
	printf("Xo=%.2f\tYo=%.2f\n",Xo,Yo);
	Rs232GPS(cport_nr3, buf3, 4096);// Recibe datos desde el GPS
	DesencriptarGPS();
	gps2utm(Latitud,Longitud,'n',1);
	if (prom==0){
		Xo_[0]=Xm;
		Xo_[1]=Ym;
	}
	
	pwm->set_duty_cycle(Dir, 10*1000);
	pwm->set_duty_cycle(Vel, 10*1000);
	gettimeofday(&t_ini,NULL);
	while(secs<=44){
		DatosI2C();
		
		Rs232GPS(cport_nr3, buf3, 4096);// Recibe datos desde el GPS
		DesencriptarGPS();
		gps2utm(Latitud,Longitud,'n',1);
		Kalman_o(Xm,Ym);
		Xo=Xo_[0];Yo=Xo_[1];
		prom++;
		// printf("prom = %d\n",prom);
		
		gettimeofday(&t_fin,NULL);
		secs = timeval_diff(&t_fin, &t_ini);
	}
	
	int Tcal=8;
	secs=0.0;
	gettimeofday(&t_ini,NULL);
	while((secs)<Tcal){
		pwm->set_duty_cycle(Dir, 18*1000);
		pwm->set_duty_cycle(Vel, 10*1000);
		
		DatosI2C();	
		if(x<Xmin)	Xmin=x;
		if(x>Xmax)	Xmax=x;
		if(y<Ymin)	Ymin=y;
		if(y>Ymax)	Ymax=y;
		
		Rs232GPS(cport_nr3, buf3, 4096);// Recibe datos desde el GPS
		DesencriptarGPS();
		gps2utm(Latitud,Longitud,'n',1);
		Kalman_o(Xm,Ym);
		Xo=Xo_[0];Yo=Xo_[1];
		prom++;
		// printf("prom = %d\n",prom);
		
		gettimeofday(&t_fin,NULL);
		secs = timeval_diff(&t_fin, &t_ini);
	}
	pwm->set_duty_cycle(Dir, 10*1000);
	pwm->set_duty_cycle(Vel, 10*1000);
	// printf("%d	%d	%d	%d\n",Xmax,Xmin,Ymax,Ymin);
	
	long aux100,aux101;
	aux100=Ymax-Ymin;
	aux101=Xmax-Xmin;
	
	Xsf=(float)aux100/aux101;
	Ysf=(float)aux101/aux100;
	
	Xoff=(float)aux101/2.0;
	Xoff-=Xmax;
	Xoff*=Xsf;
	
	Yoff=(float)aux100/2.0;
	Yoff-=Ymax;
	Yoff*=Ysf;
	
	secs=0.0;
	Xmax=0;Xmin=20000;Ymax=0;Ymin=20000;
	gettimeofday(&t_ini,NULL);
	while((secs)<Tcal){
		pwm->set_duty_cycle(Dir, 18*1000);
		pwm->set_duty_cycle(Vel, 10*1000);
		
		DatosI2C();
		
		XN=(float)(Xsf*x)+Xoff;
		YN=(float)(Ysf*y)+Yoff;
		
		if(XN>Xmax)	Xmax=XN;
		if(YN>Ymax)	Ymax=YN;
		printf("%d\t%d\t%.2f\t%.2f\n",x,y,XN,YN);
		
		Rs232GPS(cport_nr3, buf3, 4096);// Recibe datos desde el GPS
		DesencriptarGPS();
		gps2utm(Latitud,Longitud,'n',1);
		Kalman_o(Xm,Ym);
		Xo=Xo_[0];Yo=Xo_[1];
		prom++;
		// printf("prom = %d\n",prom);
		
		gettimeofday(&t_fin,NULL);
		secs = timeval_diff(&t_fin, &t_ini);
	}
	pwm->set_duty_cycle(Dir, 10*1000);
	pwm->set_duty_cycle(Vel, 10*1000);
	
	if ( (Xsf>Ysf+0.5) || (Ysf>Xsf+0.5) ){
		printf("Calibration error\n You should do it again\n");
		goto NO;
	}
	
	printf(">>	Mag	ok\n");
	auxA1=1;
	NO:
	printf("Xsf= %.2f	Ysf= %.2f	Xoff= %.2f	Yoff= %.2f\n",Xsf,Ysf,Xoff,Yoff);
	printf("cont = %d\n",prom);
	printf("Xo=%.2f\tYo=%.2f\n",Xo,Yo);
	
	correccion=correccion+1;
	
}

//=============================================================================
// Trajectory Generator
int Trayectoria(){
    char Xaux[80];
    char Yaux[80];
    int marcadorX,marcadorY,i, bEnPalabra, npal = 0;
    printf ("|%s|\n", buf5);
    bEnPalabra =0;
    for ( i = 1; buf5[i]; ++i)
       if (buf5[i] == 'X'){
		  printf ("%s\n", "MarcadorX");
          marcadorX=1;
          marcadorY=0;
		  bEnPalabra = 0;}
       else if (buf5[i] == 'Y'){
		  printf ("%s\n", "MarcadorY");
          marcadorY=1;
		  marcadorX=0;
          bEnPalabra = 0;}
       else if (marcadorX == 1 && marcadorY == 0 && !isspace (buf5[i]) )
          Xaux[bEnPalabra++] = buf5[i];
       else if (marcadorY == 1 && !isspace (buf5[i]) )
          Yaux[bEnPalabra++] = buf5[i];
       else if (bEnPalabra){
          Xaux[bEnPalabra] = '\0';
          Yaux[bEnPalabra] = '\0';
          bEnPalabra,marcadorX,marcadorY = 0;
          printf ("#%d: %s\t   %s \n", npal++ + 1, Xaux, Yaux);
          pX[p]=atof(Xaux);
          pY[p]=atof(Yaux);
          printf ("%f\t %f\n",pX[p],pY[p]);
          p++;
      }
}

float rt=1.2; //radio de tolerancia
int coordenadasDeseadas(float x1, float y1, float x2, float y2){
	printf("X= %.4f	Y= %.4f	Xd= %.4f	Yd= %.4f\n",x1,y1,pX[contP]-Xo,pY[contP]-Xo);
    
	x1 += Xo;	y1 += Yo;
	x2 += Xo;	y2 += Yo;
	
	float U1=x1-pX[contP];
	float U2=y1-pY[contP];
	float mag=sqrt(U1*U1+U2*U2);
	
	U1=x2-pX[contP];
	U2=y2-pY[contP];
	float magR=sqrt(U1*U1+U2*U2);
	
	if( contP < p){ // pasa por todos los puntos
        // if( (x1+Xo) > pX[contP]-rt && x1+Xo < pX[contP]+rt && y1+Yo > pY[contP]-rt && y1+Yo < pY[contP]+rt ){
		if(mag > 0.05)
			contPd++;
		if (magR < rt) {
			contPd=1;
            if(contP==p-1)
                return 1;
            contP++;
			printf("Buscar punto |%d|",contP);
        }
    }
    Xd=pX[contP]-Xo;
    Yd=pY[contP]-Yo;
	
	if (contP==0){
		Xd1=X_1;
		Yd1=Y_1;
	}else{
		Xd1=pX[contP-1]-Xo;
		Yd1=pY[contP-1]-Yo;
	}
	
    return 0;
}
float V=1;
void VDeseadoErrorTheta(float x1, float y1, float yaw1, float Xd, float Yd){
	
	float U1=Xd-x1;
	float U2=Yd-y1;
	pos_df=sqrt(U1*U1+U2*U2);
		
	if (pos_df>rt){
		U1=Xd-Xd1;
		U2=Yd-Yd1;
		
		alpha=atan2(U2,U1);
		float Pd=sqrt(U1*U1+U2*U2);
		
		Xk=V*contPd*0.05*cos(alpha)+Xd1;
		Yk=V*contPd*0.05*sin(alpha)+Yd1;
		
		float Xdr=Xk-x1;
		float Ydr=Yk-y1;
		
		Thetad=atan2(Ydr,Xdr);
		float Posd=sqrt(Xdr*Xdr+Ydr*Ydr);
		
		errorTheta=Thetad-yaw1;
		if ( errorTheta > PI )			errorTheta= errorTheta-(2*PI);
		else if ( errorTheta < -PI )	errorTheta= errorTheta+(2*PI);
		
		printf("Xk= %.4f	Yk= %.4f	Pd= %.2f	Posd= %.2f\n",Xk,Yk,Pd,Posd);
		
		Vd=Posd*1;
		if (Vd>Vmax)	Vd=Vmax;
	}
}

//=============================================================================
// PID Wd
int PIDWd(float errorTheta){
	float Kpt=4.847;//4.847;
	float Kit=0.928*0.05; //0.928*0.05;
	float Kdt=1.8/0.05; //1.8/0.05;
	float Kb=0.01;
	errort[0]=errorTheta;
	
	// P-gain ---------------------------------------------------------------------
	Upt = Kpt * errort[0];
	// I-gain ---------------------------------------------------------------------
	Uit[0] = Kit * errort[0] + Uit[1]+Kb*DWd[0];	
	// D-gain ---------------------------------------------------------------------
	Udt[0] = Kdt * (errort[0] - errort[1]);

	Wd[0] = Upt + Uit[0] + Udt[0];
	
	if (Wd[0]<-Wmax)	Wd[1]=-Wmax;
	else if  (Wd[0]>Wmax)		Wd[1]=Wmax;  
	else Wd[1]=Wd[0];
		
	DWd[0]=Wd[1]-Wd[0];
	Uit[1]=Uit[0];
	errort[1] = errort[0];
	Wd[0]=Wd[1];
}

int PIDBNWd(float errorTheta){
	float Kpt=7.1786;
	float Kit=0.2768*0.05; 
	float Kb=0.01;
	float D=5.6296; 
	float N=37.1452;
	errort[0]=errorTheta;
	
	// P-gain ---------------------------------------------------------------------
	Upt = Kpt * errort[0];
	// I-gain ---------------------------------------------------------------------
	Uit[0] = Kit * errort[0] + Uit[1]+Kb*DWd[0];	
	// D-gain ---------------------------------------------------------------------
	Udt[0] = D*N * (errort[0] - errort[1])+ Udt[1]*(1-N*0.05);
	
	Wd[0] = Upt + Uit[0] + Udt[0];
	
	if (Wd[0]<-Wmax)	Wd[1]=-Wmax;
	else if  (Wd[0]>Wmax)		Wd[1]=Wmax;  
	else Wd[1]=Wd[0];
	
	DWd[0]=Wd[1]-Wd[0];
	Uit[1]=Uit[0];
	Udt[1]=Udt[0];
	errort[1] = errort[0];
	Wd[0]=Wd[1];
}

int PIDTTWd(float errorTheta){
	float Kpt=6.5541;
	float Kit=0.2853*0.05;
	float Kb=0.01;
	float D=6.9724; 
	float N=37.4937;
	errort[0]=errorTheta;
	
	// P-gain ---------------------------------------------------------------------
	Upt = Kpt * errort[0];
	// I-gain ---------------------------------------------------------------------
	Uit[0] = Kit/2 * (errort[0]+errort[1]) + Uit[1]+Kb*(DWd[0]+DWd[1]);	
	// D-gain ---------------------------------------------------------------------
	Udt[0] = (2*D*N * (errort[0]-errort[1]) - Udt[1]*(N*0.05-2))/(N*0.05+2);
	
	Wd[0] = Upt + Uit[0] + Udt[0];
	
	if (Wd[0]<-Wmax)	Wd[1]=-Wmax;
	else if  (Wd[0]>Wmax)		Wd[1]=Wmax;  
	else Wd[1]=Wd[0];
	
	DWd[0]=Wd[1]-Wd[0];
	Uit[1]=Uit[0];
	Udt[1]=Udt[0];
	errort[1] = errort[0];
	Wd[0]=Wd[1];
	DWd[1]=DWd[0];
}

void ReinicioVal(){
	pX[0]=0.0;
	pY[0]=0.0;
	p=0;

	UV=10;
	UW=10;

	errorTheta=0;
	Thetad=0;
	alpha=0;
	ThetaR=0;
	Vd=0;					//velocidad lineal deseada
	Vdmax=0;				//velocidad lineal deseada maxima
	Uit[0]=0.0;	
	Udt[0]=0.0;
	Wd[0]=0.0;
	errort[0]=0.0;
	Upt=0.0;
	DWd[0]=0.0;

	contP=0;
	contGPS=0;
	// Restar Error
	errort[0]=0.0;
	// Restar Control Action
	Wd[0] =0.0;
	

}

//=============================================================================

// int main(int argc, char *argv[]){
int main(int argc, char** argv){
	
	if (check_apm()) {
        return 1;
    }
//-------------------------------------------------------------------------
	// IMU
    auto sensor_name = "mpu";;

    auto sensor = get_inertial_sensor(sensor_name);

    if (!sensor) {
        printf("Wrong sensor name. Select: mpu or lsm\n");
        return EXIT_FAILURE;
    }

    if (!sensor->probe()) {
        printf("Sensor not enabled\n");
        return EXIT_FAILURE;
    }
    sensor->initialize();

    float ax, ay, az;
    float gx, gy, gz;
	
	printf("IMU		OK\n");
	
//-------------------------------------------------------------------------
	// AHRS
	// float ax, ay, az;
    // float gx, gy, gz;

    auto sensor_name2 = "mpu";

    auto imu = get_inertial_sensor(sensor_name2);

    if (!imu) {
        printf("Wrong sensor name. Select: mpu or lsm\n");
        return EXIT_FAILURE;
    }

    if (!imu->probe()) {
        printf("Sensor not enable\n");
        return EXIT_FAILURE;
    }

    // Network setup

    // Socket sock;

    // if (argc == 5)
        // sock = Socket(argv[3], argv[4]);
    // else if ( (get_navio_version() == NAVIO) && (argc == 3) )
        // sock = Socket(argv[1], argv[2]);
    // else
        // sock = Socket();

    auto ahrs = std::unique_ptr <AHRS>{new AHRS(move(imu)) };

    // Setup gyroscope offset

    ahrs->setGyroOffset();
    
//-------------------------------------------------------------------------
	// PPM
	auto rcin = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
	rcin->initialize();
	printf("PPM		OK\n");
	
	int channels[4];
//-------------------------------------------------------------------------
	// PWM
	PWM_config();
	printf("PWM		OK\n");
//-------------------------------------------------------------------------
	// LED
	auto led = std::unique_ptr <Led>{ new Led_Navio2() };
    if (!led->initialize())
        return EXIT_FAILURE;
	// PreloadColors
	// Black
    // Red
    // Green
    // Blue
    // Cyan
    // Magenta
    // Yellow
    // White
//-------------------------------------------------------------------------
	// I2C
	if ((fd2 = open("/dev/i2c-1", O_RDWR)) < 0){
        //Open port for reading and writing
        fprintf(stderr, "Failed to open I2C bus\n");
        return 1;
    }

    /* initialise HMC5883L */
    selectDevice(fd2, HMC5883L_I2C_ADDR, "HMC5883L");

    writeToDevice(fd2, 0x00, 0x68);		// Output rata=3 Hz with 8 samples average
	writeToDevice(fd2, 0x01, 0x00);		// Configura resolucion 0.73 mGauss/LSb
    writeToDevice(fd2, 0x02, 0x03);		// Configura modo continuo de medicion
	printf("Mag - HMC5883L	OK\n");
//-------------------------------------------------------------------------
	// RS232
	
	if(RS232_OpenComport(cport_nr, bdrate, mode)){
		printf("Cannot open ROBOTEQ\n");
		return(0);
	}
	char prueba[7];
	strcpy(prueba,"%EELD\r");
	RS232_cputs(cport_nr,prueba);
	printf("RoboteQ		OK\n");
	
	if(RS232_OpenComport(cport_nr3, bdrate3, mode)){
		printf("Cannot open GPS\n");
		return(0);
	}
	
	// if(RS232_OpenComport(cport_nr4,bdrate4,mode))	{
		// printf("Cannot open RTK\n");
		// return(0);
	// }
	// printf("RTK		OK\n");
	
	if(RS232_OpenComport(cport_nr5,bdrate5,mode))	{
		printf("Cannot open Telemetry\n");
		return(0);
	}
	printf("Telemetry	OK\n");
	
	// if(RS232_OpenComport(cport_nr6,bdrate6,mode))	{
		// printf("Cannot open Xsens Module\n");
		// return(0);
	// }
	// printf("Xsens	OK\n");

//-------------------------------------------------------------------------
	//  Saving Data
	CreateFile();

	printf("Crear txt	OK\n");
	printf("While -0");	
	int cont=0;
	inicial=0;
//-------------------------------------------------------------------------
	int min=1000000, max=0;
	char conf;
	
	ini_Matrix(Pok, 0, 100, 2);
	ini_Matrix(Io, 0, 1, 2);
	
	int cont200=0;
	printf("While 0");
    while(1) {
		gettimeofday(&t_ini, NULL);
		printf("While 1");
		// printf("t0 = %.2f\n",time_sec(&t_ini));
		
		
		// IMU
		sensor->update();
        sensor->read_accelerometer(&ax, &ay, &az);
        sensor->read_gyroscope(&gx, &gy, &gz);
		// printf("Acel: %.3f %.3f %.3f\n", ax, ay, az);
		// Accelerometer
		buf_datosc[0]=ax;	buf_datosc[1]=ay;	buf_datosc[2]=az;
		// Gyroscope
		buf_datosc[3]=gx;	buf_datosc[4]=gy;	buf_datosc[5]=gz;
		
		// printf("Acc: %+7.3f %+7.3f %+7.3f\t", ax, ay, az);
        // printf("Gyr: %+8.3f %+8.3f %+8.3f\t", gx, gy, gz);
		
		
		// Mag
		DatosI2C();	
		XN=(float)(Xsf*x)+Xoff;
		YN=(float)(Ysf*y)+Yoff;
		// angle = atan2(XN,-YN);
		angle = atan2(-YN,-XN);
		
		if ((angle<-3.0543) || (angle>3.0543))
			angle=PI;
		
		// printf("Hola\n");
		buf_datosc[6]=XN;	buf_datosc[7]=YN;	buf_datosc[8]=z;
		buf_datosc[11]=angle;
		// printf("Mag: %+7.3f %+7.3f %+7.3f\n", XN, YN, z);
		// printf("\n");printf("Yaw: %+7.3f\n", angle);
		
		//	AHRS
		ahrs->updateIMU(Ts/1000);
		ahrs->getEuler(&roll, &pitch, &yaw);
		
		buf_datosc[9]=roll;	buf_datosc[10]=pitch;
		// printf("Roll: %+7.3f\tPitch: %+7.3f", roll, pitch);
		// printf("\n");
		
		//	AHRS
		Rs232RoboteQ(cport_nr, buf, 4096);	// Recibe datos desde el ROBOTEQ
		DesencriptarRoboteq();
		buf_datosc[12]=batery;
		buf_datosc[13]=TempDerecho;	buf_datosc[14]=TempIzquierdo;
		buf_datosc[15]=IDerecha;	buf_datosc[16]=IIzquierda;
		buf_datosc[17]=RpmDerecha;	buf_datosc[18]=RpmIzquierda;
		// printf("\n");printf("Port: %d\n",cport_nr);
		// printf("%s\n",buf);
		// printf("Bat: %.2f\nTr: %.2f\tTl: %.2f\nIr: %.2f\tIl: %.2f\nRPMr: %.2f\tRPMl: %.2f\nCont: %.2f\n",batery,TempDerecho,TempIzquierdo,IDerecha,IIzquierda,RpmDerecha,RpmIzquierda,contador);
		
		if (contGPS==0){
			do{
				// system("clear");
				Rs232GPS(cport_nr3, buf3, 4096);// Recibe datos desde el GPS
				DesencriptarGPS();
				gps2utm(Latitud,Longitud,'n',1);
				// printf("Xm: %.2f\tYm: %.2f\n",Xm,Ym);
				if(Ym>100.0 && inicial==0){
					// printf("hola\n");
					X_1=Xm-Xo;		Y_1=Ym-Yo;
					// Xo_[0]=Xm;		Xo_[1]=Ym;
					printf("X1: %.2f\tY1: %.2f\n",X_1,Y_1);
					cont200++;
				}
				// printf("\n");printf("Port: %d\n",cport_nr3);
				// printf("|%s|\n",buf3);
				// printf("Lat: %f\tLon: %f\nSat: %d\n",Latitud,Longitud,Satellites);
				
			}while ( (Y_1==0) || (cont200!=5) );
			
			// Rs232RTK(cport_nr4, buf4, 4096);
			// DesencriptarRTK();
			// gps2utm(LatitudRTK,LongitudRTK,'n',2);
			
			// printf("\n");printf("Port: %d\n",cport_nr4);
			// printf("%s\n",buf4);
			// printf("Lat: %f\tLon: %f\nSat: %d\n",LatitudRTK,LongitudRTK,SatellitesRTK);
			// printf("Xm: %.2f\tYm: %.2f\n",XmRTK,YmRTK);
			
			Xm-=Xo;
			Ym-=Yo;	
			// printf("Xm=%.2f\tYm=%.2f\n",Xm,Ym);
		}
		
		// Rs232RTK(cport_nr4, buf4, 4096);
		// DesencriptarRTK();
		// gps2utm(LatitudRTK,LongitudRTK,'n',2);
			
			// printf("Port: %d\n",cport_nr4);
			// printf("%s\n\n",buf4);
		
		// printf("hola %f\n",Xm);
		//	GPS
		buf_datosc[20]=Latitud;		buf_datosc[21]=Longitud;
		//	Conversion to UTM
		buf_datosc[22]=Xm;			buf_datosc[23]=Ym;
		buf_datosc[24]=Satellites;
		
		// //	RTK
		// buf_datosc[28]=LatitudRTK;		buf_datosc[29]=LongitudRTK;
		// //	Conversion to UTM
		// buf_datosc[30]=XmRTK;			buf_datosc[31]=YmRTK;
		// buf_datosc[32]=SatellitesRTK;
		
		for(int i=0;i<4;i++){
			channels[i] = rcin->read(i);
			if (channels[i] == READ_FAILED)
				return EXIT_FAILURE;
		}
		printf("\n");printf("CH0=%d\t CH1=%d\t CH2=%d\t CH3=%d\n", channels[0],channels[1],channels[2],channels[3]);
		// printf("x=%d, y=%d, z=%d\n", x, y, z);
		
		if(inicial ==0){
			printf("X_1=%.2f\tY_1=%.2f\n",X_1,Y_1);
			X_[0]=X_1;
			X_[1]=Y_1;
			X_[2]=angle;
		}
		// printMatrix(X_, 3, 1);
		// printf("X=%.4f	Y=%.4f	Th=%.3f	Wr=%.4f	Wl=%.4f\n",Xm,Ym,angle,RpmDerecha,RpmIzquierda);
		Kalman(RpmDerecha,RpmIzquierda,angle,Xm,Ym,contGPS);
		X_e=X_[0];Y_e=X_[1];T_e=X_[2];
		// printMatrix(X_, 3, 1);
		
		buf_datosc[30]=X_e;			buf_datosc[31]=Y_e;
		buf_datosc[32]=T_e;	
		printf("Xe=%.4f\tYe=%.4f\tThe=%.3f	Dx=%.4f	Dy=%.4f\n",X_e,Y_e,T_e,std_lat,std_long);
		if (X_e>2000)
			return EXIT_FAILURE;
		// printf("\n");
		
		if(inicial ==0){
			printf("Which Controler will you use?\n\t(1)\tBackward Euler\n\t(2)\tBackward Euler with Filter\n\t(3)\tTrapezoidal with Trapezoidal Filter");
			printf("\nNumber:\t");
			scanf("%d",&ref);
			inicial=1;	
		}

		// -------------------------------------------------------- main --------------------------------------------------------
		// NEUTRO----------------------
		if (channels[2]<940 && channels[2]>500){
			// LED red
			led->setColor(Colors::Red);
			
			pwm->set_duty_cycle(Dir, 10*1000);
			pwm->set_duty_cycle(Vel, 10*1000);
			
			// ReinicioVal();
		}
		// MANUAL-----------------------------
		else if (channels[2]<1540 && channels[2]>1510){
			// LED yellow
			led->setColor(Colors::Yellow);
			
			// Direccion -------------
			// channels[0]=[987 2089];
			// salidas[0]=[3 17];
			salidas[0]=-0.0144*channels[0]+32.2142;
			if(salidas[0]<2.0) salidas[0]=2.0;
			if(salidas[0]>18.0) salidas[0]=18.0;
			
			// Velocidad -------------
			// channels[1]=[1124 1930];
			// salidas[1]=[3 17];
			salidas[1]=-0.01985*channels[1]+40.3127;
			if(salidas[1]<2.0) salidas[1]=2.0;
			if(salidas[1]>18.0) salidas[1]=18.0;
			
			//  SALIDAS ----------------
			pwm->set_duty_cycle(Dir, salidas[1]*1000);
			pwm->set_duty_cycle(Vel, salidas[0]*1000);
			
			// SerialAll(buf_datosc);
			//SerialGPS(Xm,Ym);
			
			//printf("%d\t%d\n	%.2f\t%.2f\t",x,y,XN,YN);
			// printf("\n");
			//printf("%f\t	%f\t",Latitud,Longitud);
			//printf("%.2f\n",angle*180/PI);
			// printf("E=%f\t	N=%f\t  #S=%i\n",Xm,Ym,Satellites);
			//printf("%f\t %f\t %f\t %f\n",Xm, Ym, XmRTK, YmRTK);
			printf("%.4f  %.4f  %.4f  %.4f  %.4f %.4f %.4f\n",salidas[1], salidas[0],RpmDerecha,RpmIzquierda,IDerecha,IIzquierda,angle);
			// printf("%.4f	%.4f\t",RpmDerecha,RpmIzquierda);
			// printf("%.2f	%.2f	%.2f\n",XN,YN,angle);
			// printf("\nDerecha: %2.4f Izquierda: %2.4f Derecha: %2.4f Izquierda: %2.4f\n",salidas[1],salidas[0],salidas[3],salidas[2]);
		   
		}
		//  MODO AUTOMATICO  ------------------------
		
		else if (channels[2]<2100 && channels[2]> 2080){
			// LED green
			do{	led->setColor(Colors::Green);	}while(0);
			
			// UW=10;
			// if(cont<80){
				// UV=5;
			// }
			// else if(cont==80){
				// UV=2;
			// }
			// else if(cont==140){
				// UV=6;
			// }
			// else if(cont==200){
				// UV=7.5;
			// }
			// else if(cont==260){
				// UV=4;
			// }
			// else if (cont==320){
				// UV=7;
			// }
			// else if (cont==380){
				// UV=10;
			// }
			// cont++;
			// printf("%d\t",cont);
			// pwm->set_duty_cycle(Dir, UW*1000);
			// pwm->set_duty_cycle(Vel, UV*1000);
			
			// printf("%.4f  %.4f  %.4f  %.4f  %.4f %.4f %.4f\n",UW, UV,RpmDerecha,RpmIzquierda,IDerecha,IIzquierda,angle);
			no:
			if(p == 0){
				printf ("Esperando puntos...\n");
				readTelemetry();
				if(fin==1){
					Trayectoria();
					// printf("\n #1 para confimar\t");
					// printf("\nConfirmar:\t");
					// scanf("%d",&conf);
					usleep(1000000);
					printf("N° points=%d\n",p);
					for (int i=0;i<100;i++){
						strcpy(str[0], "#");
						RS232_cputs(cport_nr5, str[0]);
			
						snprintf(strN, sizeof strN, "%d", p);
						RS232_cputs(cport_nr5, strN);
						
						strcpy(str[1], "!");
						RS232_cputs(cport_nr5, str[1]);
						
						strcpy(str[0], "1");
						RS232_cputs(cport_nr5, str[0]);
						
						enviar[0]=13;	// con este simbolo bajamos un reglon para una buena vizualizacion.
						RS232_SendBuf(cport_nr5,enviar,1);
						usleep(50000);
					}
					// usleep(2500000);
					printf ("confirmando...\t");
					n5=0;
					int aux200=0;
					usleep(2000000);
					buf1[0]=0;
					while(aux200==0){
						n5 = RS232_PollComport(cport_nr5, buf1, 200);
						
						if(n5 > 5){
							buf1[n5] = 0;   /* always put a "null" at the end of a string! */
							for(int i=0; i < n5; i++){
								if(buf1[i] < 32)	buf1[i] = '.';	/* replace unreadable control-codes by dots 48*/
							}
							//	Encontrar cabezera
							for(int i=0; i < n5-3; i++){
								// printf ("|%c|\n", buf1[i]);
								if((buf1[i] == '$') && (buf1[i+2] == '!') && (buf1[i+3] == 'A')){
									hola[0]=buf1[i+1];
									hola[1]=0;
									i=n5;
									aux200=1;
								}
							}
						}
					}
					conf=atoi(hola);
					if ((conf!=1)){
						p=0;
						printf ("Fallo!\n");
						goto no;
					}
					printf ("Confirmado!\n");	
					Xk=X_1;
					Yk=Y_1;
				}
			}
			else{
				// SerialGPS(Xm,Ym);
				// SerialGPSyRTK(Xm,Ym,X_e,Y_e);
				// int terminar=coordenadasDeseadas(Xm,Ym);		// Trabajar con GPS
				// int terminar=coordenadasDeseadas(X_e,Y_e);	// Trabajar con Estimacion
				int terminar=coordenadasDeseadas(Xk,Yk,X_e,Y_e);		// Trabajar con generador de trayectoria
				if (terminar == 1){//detener
					pwm->set_duty_cycle(Dir, 10*1000);
					pwm->set_duty_cycle(Vel, 10*1000);
				}
				else{
					// VDeseadoErrorTheta(Xm,Ym,angle,Xd,Yd);		// Trabajar con GPS
					VDeseadoErrorTheta(X_e,Y_e,T_e,Xd,Yd);		// Trabajar con Estimacion
					buf_datosc[35]=Xk;		buf_datosc[36]=Yk;
					switch(ref){
						case 1:
							PIDWd(errorTheta);
							break;
						case 2:
							PIDBNWd(errorTheta);
							break;
						case 3:
							PIDTTWd(errorTheta);
							break;
						default:
							printf("\nPID method incorrect -- FATAL ERROR\n");
							pwm->set_duty_cycle(Dir, 10*1000);
							pwm->set_duty_cycle(Vel, 10*1000);
							return EXIT_FAILURE;
					}
			   	
					buf_datosc[25]=Thetad;	buf_datosc[28]=alpha;	buf_datosc[29]=ThetaR;
					buf_datosc[33]=Xd;	buf_datosc[34]=Yd;
			   
					UV=(-4*Vd)+10;
					UW=(4*Wd[0])+10;
			    
					pwm->set_duty_cycle(Dir, UW*1000);
					pwm->set_duty_cycle(Vel, UV*1000);
				}
         
				contaux=0;
				// printf("%.4f	%.4f	%.4f	%.4f	%.4f	%.4f	%.4f	%.4f	%.4f	%.4f	%.4f	%.4f\n",UW, UV,RpmDerecha,RpmIzquierda,angle,errorTheta,Wd[0],Xd,Yd,Xm,Ym,Vd);
				// printf("%.1f\t",ref);
				// printf("%.1f	%.3f	%.3f\t",angle,errory[0],Wd[0]);
				// printf("%.2f	%.2f\t",WtR,WtL);
				// printf("%.3f	%.3f	%.1f	%.1f	%.3f	%.3f\n",errorR[0],errorL[0],RpmDerecha,RpmIzquierda,UtR[0],UtL[0]);
				// printf("X=%f	Y=%f\t",Xm,Ym);
				// printf("Angulo=%.2f\n",angle);
				// printf("Xd=%f	Yd=%f\t",Xd,Yd);
				// printf("Vd=%f   Error=%.2f\n",Vd,errorTheta);
				// printf("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",Thetad,angle,errorTheta,Xd,Yd,Xm,Ym,Wd[0],Vd);
				printf("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",alpha,ThetaR,Thetad,T_e,errorTheta,Xd1,Yd1,Xd,Yd);
				
			}
		}
		//else {printf("\n");printf("sale CHs=%d\n", channels[2]);}
		//	Linear velocity of wheels
		Vr=RpmDerecha*r;
		Vl=RpmIzquierda*r;
		buf_datosc[26]=Vr;			buf_datosc[27]=Vl;
		
		//Modos de grabacion y calibracion
		//	GRABAR DATOS  ------------------------
		if (channels[3]<930){
			correccion=1; auxA1=1;
			if ((correccion > 0) && (auxA1))	Guardar(buf_datosc);
			// else if (!auxA1)	printf("Do not save Data, copmass is not calibrated\n");
		}
		//	CALIBRATION  ------------------------
		if (channels[3]> 1000){
			// LED cyan
			do{	led->setColor(Colors::Cyan);	}while(0);
			
			Calibration();
		}
		
		
		
		// Ts configuration   -------------------------
		gettimeofday(&t_fin, NULL); // final del contador, para una iteracion.
		secs = timeval_diff(&t_fin, &t_ini);
		//printf("\n");printf("Dt = %.4f 	s\n",secs);
		while((secs*1000.0)<Ts){
			gettimeofday(&t_fin,NULL);
			secs = timeval_diff(&t_fin, &t_ini);
		}
		//	Step time
		buf_datosc[19]=secs;
		
		// if (p != 0){
			// printf("P=%d\n",p);
			// SerialAll(buf_datosc);
		// }
		// printf("tf = %.2f\n",time_sec(&t_fin));
		// printf("\n");printf("Ts = %.4f 	s\n",secs);
		// printf("\n");
		// system("clear");
	   contGPS--;
    }
}
