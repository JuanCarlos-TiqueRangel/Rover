//=============================================================================
// Saving Data
#include <stdio.h>
FILE *pFile;

int CreateFile(){
	pFile=fopen("Datos.txt","w");
	if((pFile=fopen("Datos.txt","w")) == NULL){
		printf("No se pudo crear el archivo");
		return 1;
	}
	fputs("ax		",pFile);		//0
	fputs("ay		",pFile);		//1
	fputs("az		",pFile);		//2
	fputs("gx		",pFile);		//3
	fputs("gy		",pFile);		//4
	fputs("gz		",pFile);		//5
	fputs("mx		",pFile);		//6
	fputs("my		",pFile);		//7
	fputs("mz		",pFile);		//8
	fputs("roll		",pFile);		//9
	fputs("pitch	",pFile);		//10
	fputs("yaw_Gps	",pFile);		//11
	fputs("Battery	",pFile);		//12
	fputs("TempR	",pFile);		//13
	fputs("TempL	",pFile);		//14
	fputs("IR		",pFile);		//15
	fputs("IL		",pFile);		//16
	fputs("WR		",pFile);		//17
	fputs("WL		",pFile);		//18
	fputs("Ts		",pFile);		//19
	fputs("Latitud	",pFile);		//20
	fputs("Longitud	",pFile);		//21
	fputs("x		",pFile);		//22
	fputs("y		",pFile);		//23
	fputs("satelites",pFile);		//24
	fputs("ThetaD	",pFile);		//25
	fputs("V_R		",pFile);		//26
	fputs("V_L		",pFile);		//27
	fputs("alpha	",pFile);		//28
	fputs("ThetaR	",pFile);		//29
	fputs("Xe		",pFile);		//30
	fputs("Ye		",pFile);		//31
	fputs("Te		",pFile);		//32
	fputs("Xd		",pFile);		//33
	fputs("Yd		",pFile);		//34
	fputs("Xk		",pFile);		//35
	fputs("Yk		",pFile);		//36
	fputs("LatRTK	",pFile);		//37
	fputs("LongRTK	",pFile);		//38
	fputs("xRTK		",pFile);		//39
	fputs("yRTK		",pFile);		//40
	fputs("satRTK	",pFile);		//41
	
	fputs("\n",pFile);
	
	fclose(pFile);
}

int Guardar(float buf_datosc[]){
	int k=50;
	char bufc[14];
	auxA1=1;
	pFile=fopen("Datos.txt","a");
	for (int i = 0; i <= k; i++){
		// save new data in archive
		snprintf(bufc, sizeof bufc, "%f", buf_datosc[i]);
		fputs(bufc,pFile);
		if (i<k)	fputs("\t",pFile);
		else		fputs("\n",pFile);
	}fputs("\n",pFile);
	fclose(pFile);
}