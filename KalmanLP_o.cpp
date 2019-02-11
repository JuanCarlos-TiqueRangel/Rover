int n=2;
double Io[2*2];

// Inicializacion de P1
double Pok[2*2];

// Matriz de covariancia de ruido de Proceso
double Qok[2*2]={
	covX2,	0,
	0,		covY2,
};

// Matriz de covariance del ruido de medicion
double Ro[2*2]={
	covX_gps,	0,
	0,			covY_gps
};
double Rok[2*2];

double Ko[2*2];
double Xo_[2];
double Zo[2];
double Y_o[2];

int Kalman_o(float x, float y){
	
	int n=2;
	
	if (Satellites>=6){
		// printf("cont=20");
		Zo[0]=x;
		Zo[1]=y;
		for (int i=0;i<n;i++){
			Y_o[i] = Xo_[i];
		};
		
		// // Pok=Pok+Qok;
		for (int i=0;i<n*n;i++){
			Pok[i] += Qok[i];
		};
		// // Rok=Ro+Pok;
		for (int i=0;i<n*n;i++){
			Rok[i] = Ro[i] + Pok[i];
		};
		// // Ko=Pok*Rok.inverse();
		inv(Rok, n);
			Multi_MM(Pok, Rok, Ko, 'N', 'N', n, n, n);
		
		double M1[n];	
		double M2[n];		
		// // Xo_=Xo_+Ko*(Zo-Y_o);
		for (int i=0;i<n;i++){
			M1[i]=Zo[i]-Y_o[i];
		};
			Multi_MM(Ko, M1, M2, 'N', 'N', n, 1, n);
			for (int i=0;i<n;i++){
				Xo_[i] += M2[i];
			};
		
		// printMatrix(Xo_, n, 1);
		
		double M3[n*n];
		ini_Matrix(Io, 0, 1, 2);
		// // Pok=(Io-Ko)*Pok;
		for (int i=0;i<n*n;i++){
			M3[i]=Io[i]-Ko[i];
		};
			Multi_MM(M3, Pok, Pok, 'N', 'N', n, n, n);
	
		// printMatrix(Pok, n, n);
	}
	
}