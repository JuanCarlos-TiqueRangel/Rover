float wr1=0.0,wl1=0.0;
float X1=0.0,Y1=0.0,theta1=0.0;
float L=0.6;

extern "C" {
extern int dgeev_(char*,char*,int*,double*,int*,double*, double*, double*, int*, double*, int*, double*, int*, int*);
// multiplicar matrices 
extern int dgemm_(char*,char*,int*,int*,int*,double*,double*,int*, double*, int*, double*, double*, int*);
// multiplicar matriceVector 
extern int dgemv_(char*,int*,int*,double*,double*,int*, double*, int*, double*, double*, int*);
// // multiplicar matrices 
// extern int dgemm_(char*,char*,int*,int*,int*,double*,double*,int*, double*, int*, double*, double*, int*);
// inicializar matrices
extern int dlaset_(char*,int*,int*,double*,double*, double*, int*);
// invertir matrices
	// LU decomoposition of a general matrix
    void dgetrf_(int* M, int *N, double* A, int* lda, int* IPIV, int* INFO);

    // generate inverse of a matrix given its LU decomposition
    void dgetri_(int* N, double* A, int* lda, int* IPIV, double* WORK, int* lwork, int* INFO);
};


double I[3*3]={
	1,	0,	0,
	0,	1,	0,
	0,	0,	1
};

// Inicializacion de P1
double P1k[3*3]={
	100,	0,		0,
	0,		100,	0,
	0,		0,		100
};
double P2k[3*3]={
	100,	0,		0,
	0,		100,	0,
	0,		0,		100
};

// Matriz de covariancia de ruido de Proceso
double covX1=1,covY1=1,covtheta1=0.5;
double Q1k[3*3]={
	covX1,	0,		0,
	0,		covY1,	0,
	0,		0,		covtheta1
};
double covX2=0.5,covY2=0.5,covtheta2=0.001;
double Q2k[3*3]{
	covX2,	0,		0,
	0,		covY2,	0,
	0,		0,		covtheta2
};

// Matriz de covariance del ruido de medicion
double R1[1]={0.01};
double R1k[1];

double covX_gps=2,covY_gps=2,covtheta_mag=0.01;
double R[3*3]{
	covX_gps,	0,			0,
	0,			covY_gps,	0,
	0,			0,			covtheta_mag
};
double R2[3*3];

double Ak[3*3];
double K2[3*3];
double Bu[3];
double X_[3];
double Z2[3];
double C1k[3]{
	0,	0,	1
};
double K1[3];
double Y2[3];

void inv(double* A, int N){
    int *IPIV = new int[N+1];
    int LWORK = N*N;
    double *WORK = new double[LWORK];
    int INFO;

    dgetrf_(&N,&N,A,&N,IPIV,&INFO);
    dgetri_(&N,A,&N,IPIV,WORK,&LWORK,&INFO);

    delete IPIV;
    delete WORK;
}

void Multi_MM(double* A, double* B, double* C, char TRANSA, char TRANSB, int m, int n, int k){
	double alpha=1;
	double beta=0;
	
	// calculate multiplication using the DGEMM subroutine
	dgemm_(&TRANSA,&TRANSB,&m,&n,&k, &alpha,A,&m,B,&k, &beta,C,&m);
}

void Multi_MV(double* A, double* x, double* y, char TRANSA, char TRANSB, int row, int col, int k){
	double alpha=1;
	double beta=0;
	int uno=1;
	
	// calculate multiplication using the DGEMM subroutine
	dgemv_(&TRANSA,&row,&col, &alpha,A,&row,x,&uno, &beta,y,&uno);
}

void ini_Matrix(double* A, double alpha, double beta, int m){
	
	char Nchar='U';
	
	dlaset_(&Nchar,&m,&m,&alpha,&beta,A,&m);
}

void printMatrix(double* A, int row, int col){
	for (int i=0;i<row;i++){
	  std::cout << "( ";
		for (int j=0;j<col;j++){
			std::cout << A[j*row+i] << "\t";
		}
	std::cout << " )" << endl;
	}
	std::cout << endl;
}

int Kalman(float wr, float wl, float theta, float x, float y,int cont){
	
	int n=3;
	
	float Ts=0.05;
	float Ds=(Ts*0.2/2)*(wr1+wl1);
	float Dtheta=(Ts*0.2/(2*0.38))*(wr1-wl1);
	// printf("Ds=%.4f	DT=%.4f\n",Ds,Dtheta);

	// Modelo
	Bu[0]=Ds*cos(X_[2]+Dtheta/2);
	Bu[1]=Ds*sin(X_[2]+Dtheta/2);
	Bu[2]=Dtheta;
	
	// printMatrix(Bu, n, 1);
	// printMatrix(X_, n, 1);
	
	for (int i=0;i<n;i++){
		X_[i] += Bu[i];	//Salida de modelo
	};
	
	// printMatrix(X_, n, 1);
	
	if ( (X_[2]-theta) < -3*PI/2 )		X_[2] += 2*PI;
	else if ( (X_[2]-theta) > 3*PI/2 )	X_[2] -= 2*PI;
	
	// printMatrix(X_, n, 1);
	
	// if (X_[2]>PI)			X_[2] -= 2*PI;
	// else if (X_[2]<-PI)		X_[2] += 2*PI;
	
	// printMatrix(X_, n, 1);

	float a=-Ds*sin(X_[2]+Dtheta/2);
	float b=Ds*cos(X_[2]+Dtheta/2);
	
	Ak[0]=1;	Ak[n+0]=0;	Ak[2*n+0]=a;
	Ak[1]=0;	Ak[n+1]=1;	Ak[2*n+1]=b;
	Ak[2]=0;	Ak[n+2]=0;	Ak[2*n+2]=1;

	// printMatrix(Ak, n, n);
	
	//kalman_rover
	float Z1=theta;
	float Yk1=X_[2];
	// printf("Z1=%.4f	Y1=%.4f\n",Z1,Yk1);
	
	double M1[n*n];
	// printMatrix(P1k, n, n);
	// // P1k=((Ak*P1k)*Ak.transpose())+Q1k;
	Multi_MM(Ak, P1k, M1, 'N', 'N', n, n, n);
		Multi_MM(M1, Ak, P1k, 'N', 'T', n, n, n);
		for (int i=0;i<n*n;i++){
			P1k[i] += Q1k[i];
		};
	
	// printMatrix(P1k, n, n);
	
	double M2[n];
	// // R1k=R1+((C1k*P1k)*C1k.transpose());
	Multi_MM(C1k, P1k, M2, 'N', 'N', 1, n, n);
		Multi_MM(M2, C1k, R1k, 'N', 'N', 1, 1, n);
		R1k[0] += R1[0];
	
	// printMatrix(R1k, 1, 1);
	
	double M3[n];
	// // K1=(P1k*C1k.transpose())*R1k.inverse();
	Multi_MM(P1k, C1k, M3, 'N', 'N', n, 1, n);
		inv(R1k, 1);
		Multi_MM(M3, R1k, K1, 'N', 'N', n, 1, 1);
	// printMatrix(P1k, n, n);
	// printMatrix(C1k, n, 1);
	// printMatrix(M3, n, 1);
	// printMatrix(R1k, 1, 1);
	// printMatrix(K1, n, 1);
	
	// // X_=X_+(K1*(Z1-Yk1));
	for (int i=0;i<n;i++){
		X_[i] += K1[i]*(Z1-Yk1);
	};
	
	// printMatrix(X_, n, 1);
	
	double M4[n*n];
	double M4_[n*n];
	ini_Matrix(I, 0, 1, 3);
	// // P1k=(I-(K1*C1k))*P1k;
	Multi_MM(K1, C1k, M4, 'N', 'N', n, n, 1);
		for (int i=0;i<n*n;i++){
			M4[i] = I[i] - M4[i];
		};
		Multi_MM(M4, P1k, M4_, 'N', 'N', n, n, n);
		for (int i=0;i<n*n;i++){
			P1k[i] = M4_[i];
		};
	
	// printMatrix(P1k, n, n);
	
	// printf("s=%d\n",Satellites);
	if (cont==0){
		if (Satellites>=6){
			// printf("cont=20\n");
			Z2[0]=x;
			Z2[1]=y;
			Z2[2]=theta;
			for (int i=0;i<n;i++){
				Y2[i] = X_[i];
			};
			// printMatrix(Z2, n, 1);
			// printMatrix(Y2, n, 1);
			
			double M5[n*n];
			// printMatrix(P2k, n, n);
			// // P2k=((Ak*P2k)*Ak.transpose())+Q2k;
			Multi_MM(Ak, P2k, M5, 'N', 'N', n, n, n);
				Multi_MM(M5, Ak, P2k, 'N', 'T', n, n, n);
				for (int i=0;i<n*n;i++){
					P2k[i] += Q2k[i];
				};
			
			// printMatrix(P2k, n, n);
			
			 for (int i=0;i<n*n;i++){
				R2[i] = R[i]+P2k[i];
			};
			
			// printMatrix(R2, n, n);
			
			// // K2=P2k*R2.inverse();
			inv(R2, n);
				Multi_MM(P2k, R2, K2, 'N', 'N', n, n, n);
			
			// printMatrix(R2, n, n);
			// printMatrix(K2, n, n);
			
			double M6[n];
			double M7[n];
			// // X_=X_+K2*(Z2-Y2);
			for (int i=0;i<n;i++){
				M6[i]=Z2[i]-Y2[i];
			};
				Multi_MM(K2, M6, M7, 'N', 'N', n, 1, n);
				for (int i=0;i<n;i++){
					X_[i] += M7[i];
				};
			// printMatrix(M6, n, 1);
			// printMatrix(M7, n, 1);
			// printMatrix(X_, n, 1);
			
			double M8[n*n];
			double M8_[n*n];
			ini_Matrix(I, 0, 1, 3);
			// // P2k=(I-K2)*P2k;
			for (int i=0;i<n*n;i++){
				M8[i]=I[i]-K2[i];
			};
				Multi_MM(M8, P2k, M8_, 'N', 'N', n, n, n);
				for (int i=0;i<n*n;i++){
					P2k[i]= M8_[i];
				};
	
			// printMatrix(P2k, n, n);
		}
		
		contGPS=20;
	}

	wr1=wr;wl1=wl;
}