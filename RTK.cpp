// Read RTK

void Rs232RTK(int cport_nr4, unsigned char buf4[4096], int lon){
	
	auto n = RS232_PollComport(cport_nr4, buf4, lon);
		
		if(n > 0){
			buf4[n] = 0;   /* always put a "null" at the end of a string! */
			for(int i=0; i < n; i++){
				if(buf4[i] < 32)		buf4[i] = '.';	/* replace unreadable control-codes by dots */
			}
		}
		else printf("Error reading RTK\n");
}
float LatitudRTK, LongitudRTK,SatellitesRTK, LatitudRTKant, LongitudRTKant;
int DesencriptarRTK(){
	int apuntador=999;
	int cambio = 0;
	int lon=100;

	for(int i=0;i<lon;i++){
		aux1[i] = ' ';
		aux2[i] = ' ';
		aux3[i] = ' ';
		aux4[i] = ' ';
		serial[i]=' ';
	}

	for(int i=0;i<200;i++){
		if(buf4[i]=='2' && buf4[i+1]=='0' && buf4[i+4]=='/'){
			cambio = 1;
			apuntador = i+27;
			i = 200;
		}
	}
	
	// Extracts Latitud
	for(int i=0;i<20;i++){
		if( buf4[apuntador]==' ' ){
			apuntador++;
			i=200;
		}
		else{
			aux1[i]=buf4[apuntador];
		}
		apuntador++;
	}
	LatitudRTK=atof(aux1);
	
	// Extracts Longitud 
	for(int i=0;i<20;i++){
		if(buf4[apuntador]==' '){
			apuntador = apuntador + 17;
			i=200;
		}
		else			aux2[i]=buf4[apuntador];
		apuntador++;
	}
	if (atof(aux2)!=-75.099998)
		LongitudRTK=atof(aux2);
	
	// Extracts #satellites
	for(int i=0;i<20;i++){
		if(buf4[apuntador]==' ')	i=200;
		else			aux3[i]=buf4[apuntador];
		apuntador++;
	}
	SatellitesRTK= atoi(aux3);
	
	LatitudRTKant=LatitudRTK;
	LongitudRTKant=LongitudRTK;
}
