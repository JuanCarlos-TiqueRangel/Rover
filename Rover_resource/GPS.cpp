// Read GPS

void Rs232GPS(int cport_nr3, unsigned char buf3[4096], int lon){
	bool	auxST=1;
	int		contST=0;
	while(auxST){
		auto n3 = RS232_PollComport(cport_nr3, buf3, lon);
		
		if(n3 > 0){
			buf3[n3] = 0;   /* always put a "null" at the end of a string! */
			for(int i=0; i < n3; i++){
				if(buf3[i] < 32)	buf3[i] = '.';	/* replace unreadable control-codes by dots 48*/
				if(buf3[0]=='$')
					auxST=0;
			}
		}
		contST++;
		if(contST==3)	auxST=0;
	}
	// printf("|%s|\n",buf3);
}


//=============================================================================
// GPS
const int lon=50;
char aux1gps[lon],aux2gps[lon],aux3gps[lon],aux4gps[lon],auxSgps[lon];
float Latitud=0, Longitud=0, LongitudAnt=0, LatitudAnt=0;
int Satellites=0;
float PDOP=0, HDOP=0, VDOP=0;
float std_lat=0,std_long=0;
float Spd=0,Cog=0,mv=0;

void DesencriptarGPS(){		
	int apuntador=999;
	int apuntador2=0;
	int apuntador3=0;
	int apuntador4=0;
	int cambio = 0;
	auxSgps[0]='0';
	
	//-----------------------------------------------------------------------------------------//
	// GGA
	for(int i=0;i<lon;i++){
		aux1gps[i] = ' ';
		aux2gps[i] = ' ';
		aux3gps[i] = ' ';
		aux4gps[i] = ' ';
	}

	for(int i=0;i<200;i++){
		if(buf3[i]=='G' && buf3[i+1]=='A' && buf3[i+2]==',' && buf3[i+12]==',' && buf3[i+24]=='N' && buf3[i+25]==',' && buf3[i+37]==',' && buf3[i+38]=='W' && buf3[i+39]==','){
			cambio = 1;
			apuntador = i+13;
			apuntador2=i;
			i = 200;
		}
	}
	// Extracts Latitud
	for(int i=0;i<11;i++){
		if(buf3[apuntador]==',' && buf3[apuntador+1]=='N'){
			apuntador = apuntador + 2;
			i=200;
		}
		else{
			aux1gps[i]=buf3[apuntador];
		}
		apuntador++;
	}
	// printf("|%s|\n",aux1gps);
	// Extracts Longitud 
	for(int i=0;i<11;i++){
		if(buf3[apuntador]==',' && buf3[apuntador+1]=='W')	i=200;
		else			aux2gps[i]=buf3[apuntador];
		apuntador++;
	}
    //printf("|%s|",aux2gps);
    apuntador=apuntador+5;
    
    // Extracts #satellites
    for(int i=0;i<2;i++){
        if(buf3[apuntador]==',' && buf3[apuntador+1]=='0' && buf3[apuntador+2]=='.')	i=200;
		else			auxSgps[i]=buf3[apuntador];
		apuntador++;
	}
	// printf("|%s|\n",auxSgps);
	Satellites= atoi(auxSgps);
	// printf("%i\n",Satellites);
	if(cambio==1){
		// Latitud
		apuntador=0;
		for(int i=0;i<2;i++){
			aux3gps[i]=aux1gps[apuntador];	//grados
			apuntador++;
		}
		for(int i=0;i<=sizeof(aux1gps);i++){
			aux4gps[i]=aux1gps[apuntador];	//minutos
			apuntador++;
		}
		apuntador++;

		Latitud=atof(aux4gps);						//minutos
		Latitud=(Latitud/60)+(atof(aux3gps));		//grados
		//printf("Lat:	%.4f\n",Latitud);

		// Longitud
		apuntador=0;
		for(int i=0;i<3;i++){
			aux3gps[i]=aux2gps[apuntador];	//grados
			apuntador++;
		}
		for(int i=0;i<sizeof(aux1gps);i++){
			aux4gps[i]=aux2gps[apuntador];	//minutos
			apuntador++;
		}
		apuntador++;

		Longitud=atof(aux4gps);					//minutos
		Longitud=(Longitud/60)+(atof(aux3gps));	//grados
		Longitud*=-1.0;
		//printf("Long:	%.4f\n",Longitud);
		

		LongitudAnt=Longitud;
		LatitudAnt=Latitud;
	}
	
	//-----------------------------------------------------------------------------------------//
	// GSA
	apuntador=999;
	cambio = 0;
	auxSgps[0]='0';
	for(int i=0;i<lon;i++){
		aux1gps[i] = ' ';
		aux2gps[i] = ' ';
		aux3gps[i] = ' ';
		aux4gps[i] = ' ';
	}

	for(int i=0;i<200;i++){
		if(buf3[i+apuntador2]=='S' && buf3[i+1+apuntador2]=='A' && buf3[i+2+apuntador2]==',' && buf3[i+6+apuntador2]==','){
			cambio = 1;
			apuntador = i+7+apuntador2;
			apuntador3=i+apuntador2;
			i = 200;
		}
	}
	int coma=0;
	for(int i=0;i<200;i++){
		if(buf3[i+apuntador]==','){
			coma++;
			if (coma==12){
				apuntador+=(i+1);
				i=200;
			}
		}
	}
	// printf("--%d--\n",apuntador);
	// printf("||%c||\n",buf3[apuntador]);
	
	// Extracts PDOP
	for(int i=0;i<5;i++){
		if(buf3[apuntador]==','){
			i=200;
		}
		else{
			aux1gps[i]=buf3[apuntador];
		}
		apuntador++;
	}
	// printf("|%s|\n",aux1gps);
	PDOP= atof(aux1gps);
	// printf("PDOP=	%f\n",PDOP);
	
	// Extracts HDOP
	for(int i=0;i<5;i++){
		if(buf3[apuntador]==','){
			i=200;
		}
		else{
			aux2gps[i]=buf3[apuntador];
		}
		apuntador++;
	}
	// printf("|%s|\n",aux2gps);
	HDOP= atof(aux2gps);
	// printf("HDOP=	%f\n",HDOP);
	
	// Extracts VDOP
	for(int i=0;i<5;i++){
		if(buf3[apuntador]=='*'){
			i=200;
		}
		else{
			aux3gps[i]=buf3[apuntador];
		}
		apuntador++;
	}
	// printf("|%s|\n",aux3gps);
	VDOP= atof(aux3gps);
	// printf("VDOP=	%f\n",VDOP);
	
	//-----------------------------------------------------------------------------------------//
	// GST
	apuntador=999;
	cambio = 0;
	auxSgps[0]='0';
	for(int i=0;i<lon;i++){
		aux1gps[i] = ' ';
		aux2gps[i] = ' ';
		aux3gps[i] = ' ';
		aux4gps[i] = ' ';
	}

	for(int i=0;i<200;i++){
		if(buf3[i+apuntador3]=='S' && buf3[i+1+apuntador3]=='T' && buf3[i+2+apuntador3]==',' && buf3[i+12+apuntador3]==','){
			cambio = 1;
			apuntador = i+13+apuntador3;
			apuntador4=i+apuntador3;
			i = 200;
		}
	}
	// printf("--%d--\n",apuntador);
	// printf("--%d--\n",apuntador3);
	// printf("||%c||\n",buf3[apuntador]);
	coma=0;
	for(int i=0;i<200;i++){
		if(buf3[i+apuntador]==','){
			coma++;
			if (coma==4){
				apuntador+=(i+1);
				i=200;
			}
		}
	}
	// printf("--%d--\n",apuntador);
	// printf("||%c||\n",buf3[apuntador]);
	
	// Extracts std_lat
	for(int i=0;i<5;i++){
		if(buf3[apuntador]==','){
			i=200;
		}
		else{
			aux1gps[i]=buf3[apuntador];
		}
		apuntador++;
	}
	// printf("|%s|\n",aux1gps);
	std_lat= atof(aux1gps);
	// printf("std_lat=	%f\t",std_lat);
	
	// Extracts std_long
	for(int i=0;i<5;i++){
		if(buf3[apuntador]==','){
			i=200;
		}
		else{
			aux2gps[i]=buf3[apuntador];
		}
		apuntador++;
	}
	// printf("|%s|\n",aux2gps);
	std_long= atof(aux2gps);
	// printf("std_long=	%f\n",std_long);
	
	//-----------------------------------------------------------------------------------------//
	// RMC
	apuntador=999;
	cambio = 0;
	auxSgps[0]='0';
	for(int i=0;i<lon;i++){
		aux1gps[i] = ' ';
		aux2gps[i] = ' ';
		aux3gps[i] = ' ';
		aux4gps[i] = ' ';
	}

	for(int i=0;i<200;i++){
		if(buf3[i+apuntador4]=='M' && buf3[i+1+apuntador4]=='C' && buf3[i+2+apuntador4]==',' && buf3[i+12+apuntador4]==','){
			cambio = 1;
			apuntador = i+13+apuntador4;
			i = 200;
		}
	}
	// printf("--%d--\n",apuntador);
	// printf("||%c||\n",buf3[apuntador]);
	coma=0;
	for(int i=0;i<200;i++){
		if(buf3[i+apuntador]==','){
			coma++;
			if (coma==5){
				apuntador+=(i+1);
				i=200;
			}
		}
	}
	// printf("--%d--\n",apuntador);
	// printf("||%c||\n",buf3[apuntador]);
	
	// Extracts Spd
	for(int i=0;i<5;i++){
		if(buf3[apuntador]==','){
			i=200;
		}
		else{
			aux1gps[i]=buf3[apuntador];
		}
		apuntador++;
	}
	// printf("|%s|\n",aux1gps);
	Spd= atof(aux1gps);
	// printf("Spd=	%f\t",Spd);
	
	// Extracts Cog
	for(int i=0;i<5;i++){
		if(buf3[apuntador]==','){
			i=200;
		}
		else{
			aux2gps[i]=buf3[apuntador];
		}
		apuntador++;
	}
	// printf("|%s|\n",aux2gps);
	Cog= atof(aux2gps);
	// printf("Cog=	%f\t",Cog);
	
	// Extracts mv
	for(int i=0;i<5;i++){
		if(buf3[apuntador]==','){
			i=200;
		}
		else{
			aux3gps[i]=buf3[apuntador];
		}
		apuntador++;
	}
	// printf("|%s|\n",aux3gps);
	mv= atof(aux3gps);
	// printf("mv=	%f\n",mv);
}
