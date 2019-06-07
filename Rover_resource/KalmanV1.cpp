Matrix Io(2,2);

// Inicializacion de P1
Matrix Pok(2,2);

// Matriz de covariancia de ruido de Proceso
Matrix Qok(2,2);

// Matriz de covariance del ruido de medicion
Matrix Ro(2,2);
Matrix Rok(2,2);

Matrix Xo_(2,1);
Matrix Zo(2,1);
Matrix Y_o(2,1);
Matrix Ko(2,2);

Matrix Kalman_o(float x, float y){
	
	Qok(0,0)=covX2;
	Qok(1,1)=covY2;
	
	Ro(0,0)=covX_gps;
	Ro(1,1)=covY_gps;
	
	Io=Matrix::createIdentity(2);
	
	if (Satellites>=6){
		// printf("cont=20");
		Zo(0,0)=x;
		Zo(1,0)=y;	
		Y_o=Xo_;
		// printf("Pok\n");
		// std::cout<<Pok;
		// printf("Rok\n");
		// std::cout<<Rok;
		
		Pok=Pok+Qok;
		Rok=Ro+Pok;
		Ko=Pok*Rok.inverse();
		Xo_=Xo_+Ko*(Zo-Y_o);
		
		// printf("R\n");
		// std::cout<<R;
		// printf("Pok\n");
		// std	::cout<<Pok;
		// printf("Rok\n");
		// std::cout<<Rok;
		// printf("Ko\n");
		// std::cout<<Ko;
		// printf("Zo\n");
		// std::cout<<Zo;
		// printf("Y_o\n");
		// std::cout<<Y_o;
		
		Pok=(Io-Ko)*Pok;
		}
	
	return Xo_;
}
