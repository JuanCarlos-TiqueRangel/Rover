// GPS2UTM
#include <math.h>
// #define PI 3.141592653589793


double Xm,Ym,XmRTK,YmRTK;

int gps2utm(float Latitud,float Longitud,char lat_,int N){
	float x,y;
	// Sobre la geometría del delipsoide WGS84
	double a = 6378137.0;
	double b = 6356752.31414;
	
	double e1 = 6.6943800229e-3;
	double e2 = 6.73949677548e-3;
	
	// radio polar de curvatura = c; 
	double c=6399593.626;
	double long_gd_rad, lat_gd_rad;
	
	long_gd_rad = Longitud * (PI/180.0);
	lat_gd_rad = Latitud * (PI/180.0);
	
	// Huso en el que esta Colombia
	int huso = 18;
	
	// Obtencion del meridiano central del uso = lambda0
	double lambda0, delta_lambda;
	
	lambda0 = (huso * 6.0 - 183.0)*(PI/180.0);
	
	// Determinacion de la distancia angular que existe entre la longitud del punto (long_gd_rad) y
	// el meridiano central del huso (lamda0)
	delta_lambda = long_gd_rad - lambda0;
	
	// Ecuaciones de Coticchia-Surace para el Problema Directo (Paso de Geograficas a UTM)
	// Calculo de Parametros
	double A, xi, eta, nu, zeta, A1, A2, J2, J4, J6, alpha2, beta, gamma, B_phi;
	// printf("hola %f\n",lambda0);
	A = cos(lat_gd_rad) * sin(delta_lambda); 
	xi = 0.5 * log((1+A)/(1-A)); 
	eta = atan(tan(lat_gd_rad)/cos(delta_lambda)) - lat_gd_rad; 
	nu = (c*0.9996)/sqrt((1 + e2*cos(lat_gd_rad)*cos(lat_gd_rad))); 
	zeta = (e2/2)*(xi*xi)*(cos(lat_gd_rad)*cos(lat_gd_rad)); 
	A1 = sin(2.0*lat_gd_rad); 
	A2 = A1 * (cos(lat_gd_rad)*cos(lat_gd_rad)); 
	J2 = lat_gd_rad + A1/2.0; 
	J4 = (3*J2 + A2)/4; 
	J6 = (5*J4 + A2 * (cos(lat_gd_rad)*cos(lat_gd_rad)))/3; 
	alpha2 = (3.0/4.0)*(e2); 
	beta = (5.0/3.0)*(alpha2*alpha2); 
	gamma = (35.0/27.0)*(pow(alpha2,3)); 
	B_phi = 0.9996 * c * (lat_gd_rad - alpha2 * J2 + beta * J4 - gamma * J6); 
	
	x = xi*nu*(1+zeta/3.0)+500000.0; 
	y = eta*nu*(1+zeta)+B_phi;
	
	// printf("X: %.2f\tY: %.2f\n",x,y);
			
	x += 0.22*cos(angle) + 0.10*sin(angle);
	y += 0.22*sin(angle) - 0.10*cos(angle);
	
	switch (N){
		case 1:
			Xm=x;Ym=y;
			break;
		case 2:
			XmRTK=x;YmRTK=y;
			break;
		default:
			break;
	}
	// printf("Xc: %.2f\tYc: %.2f\n",x,y);
	// x +=0.10*sin(angle);
	// y -=0.10*cos(angle);
	// printf("%f	%f	%f\n",lat_gd_rad,Latitud,Longitud);
}
