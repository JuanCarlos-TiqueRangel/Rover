// Read Roboteq

void Rs232RoboteQ(int cport_nr, unsigned char buf[4096], int lon){
	int contaux=0;
	buf[0]=0;
	roboteq:
	auto n = RS232_PollComport(cport_nr, buf, lon);
	if ( (n == 0) && (contaux < 2) ){
		goto roboteq;
		contaux++;
	}
	else if(n > 0){
		buf[n] = 0;   /* always put a "null" at the end of a string! */
		for(int i=0; i < n; i++){
			if(buf[i] < 32)		buf[i] = '.';	/* replace unreadable control-codes by dots */
		}
	}
	else printf("Error reading RoboteQ\n");
	// printf("|%s|\n",buf);
}

//=============================================================================
// RoboteQ

float batery,TempDerecho,TempIzquierdo,IDerecha,IIzquierda,RpmDerecha,RpmIzquierda;
float RpmAntDerecha,RpmAntIzquierda,IAntDerecha,IAntIzquierda,bateriaAnt;
float deltatiempoAnt,deltatiempo,contador,contadorant = 0;
char serial[50],aux1[50],aux2[50],aux3[50],aux4[50],aux5[50],aux6[50],aux7[50],aux8[100];

void DesencriptarRoboteq(){
	const float TsRoboteQ=0.048;
	char Cabezera = 'A';
	int apuntador=999,apuntador2=999;
	int salir=999;
	int cont = -1;
	int pos = 0;
	char* palabra = "\t134";
	int lon=50;
	
	batery = 0.0;
	TempDerecho = 0.0;
	TempIzquierdo = 0.0;
	IDerecha = 0.0;
	IIzquierda = 0.0;
	RpmDerecha = 0.0;
	RpmIzquierda = 0.0;
	contador = 0.0;
	
	for(int i=0;i<lon;i++){
		aux1[i] = ' ';
		aux2[i] = ' ';
		aux3[i] = ' ';
		aux4[i] = ' ';
		aux5[i] = ' ';
		aux6[i] = ' ';
		aux7[i] = ' ';
		aux8[i] = ' ';
		serial[i]=' ';
	}
//	Encontrar cabezera
	for (int i=0;i<lon;i++){
		if (buf[i]==Cabezera && apuntador == 999 && buf[i+1]!='.')	apuntador =i;
		if (buf[i]==Cabezera && apuntador == i && apuntador2 == 999){
			apuntador2 =i;
			i=lon;
		}
	}
	
	for (int i=0;i<lon;i++){
		int j =i+apuntador;
		if (buf[j]==',' || buf[j]=='A'){
			cont++;
			pos=0;
			serial[i]='\t';
			if (buf[j]=='A' && salir == 1)		i=lon;
			if (buf[j]=='A' && salir == 999){
				serial[i]=' ';
				salir = 1;
			}
		}else	serial[i] =buf[j];
		
		switch(cont){
			case 0:
				aux1[i] = serial[i];  //Valor de la batery
				break;
			case 1:
				aux2[pos] = serial[i]; // Valor de la temperatura motor derecho
				pos++;
				break;
			case 2:
				aux3[pos] = serial[i];  //Valor de la temperatura motor izquierdo
				pos++;
				break;
			case 3:
				aux4[pos] = serial[i];  //Valor de la corriente motor derecho
				pos++;
				break;
			case 4:
				aux5[pos] = serial[i];  //Valor de la corriente motor izquierdo
				pos++;
				break;
			case 5:
				aux6[pos] = serial[i];  //Valor de la Rpm motor derecho
				pos++;
				break;
			case 6:
				aux7[pos] = serial[i];  //Valor de la Rpm motor izquierdo
				pos++;
				break;
			case 7:
				aux8[pos] = serial[i];  //Valor de la contador
				pos++;
				break;
			default:
				break;
		}
		
	}
	// Ajuste de los datos del roboteq a datos reales
	
	// Temperature	
	TempDerecho = atoi(aux2);
	TempIzquierdo = atoi(aux3);
	
	// Current
	IDerecha = atoi(aux4)/10;
	IIzquierda = atoi(aux5)/10;
	
	// Sample time
	contador = atoi(aux8);
	deltatiempo = contador - contadorant;
	if (deltatiempo==0) deltatiempo=deltatiempoAnt;

	// Angular velocity
	RpmDerecha = (atoi(aux6)-RpmAntDerecha)*(2*3.141592)/(360*TsRoboteQ*deltatiempo*3.888888);
	RpmIzquierda = (atoi(aux7)-RpmAntIzquierda)*(2*3.141592)/(360*TsRoboteQ*deltatiempo*3.888888);
	if(RpmDerecha > 20 || RpmIzquierda > 20){
		printf("Error reading Enders\n");
		RpmAntDerecha = RpmDerecha;
		RpmAntIzquierda = RpmIzquierda;
	}
	
	// Battery
	batery = atoi(aux1)*0.0024+0.1;
	if(batery> 20.0 || batery < 3.0){
		printf("Battery out of range\n");
		printf("\tBattery level:	%.2f\n",batery);
		batery = bateriaAnt;
		IIzquierda =IAntIzquierda;
		IDerecha =IAntDerecha;
		RpmAntDerecha = RpmDerecha;
		RpmAntIzquierda = RpmIzquierda;
	}
	
	// Econders
	if(abs(RpmDerecha)> 20.0 || abs(RpmIzquierda) > 20.0){
		RpmDerecha		= RpmAntDerecha;
		RpmIzquierda	= RpmAntIzquierda;
	}
	// Save old data
	RpmAntDerecha = atoi(aux6);
	RpmAntIzquierda = atoi(aux7);
	IAntDerecha = atoi(aux4)/10;
	IAntIzquierda = atoi(aux5)/10;
	bateriaAnt = atoi(aux1)*0.0024+0.1;
	contadorant = contador;
	deltatiempoAnt=deltatiempo;
}
