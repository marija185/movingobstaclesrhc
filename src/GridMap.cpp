/***************************************************************************
                          GridMap.cpp  -  description
                             -------------------
    begin                : Tue Jun 17 2003
    copyright            : (C) 2003 by 
    email                : 
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#include "GridMap.h"    //
#if RECTANGULAR
#include <cspacevoronoi.h>
extern CSpaceVoronoi *cspace;
#endif

extern WorkHorse *WH;
//constructor GridMap prima kao argument cell_size
//tu se allocira mapa s obzirom na dimenzije karte
//isto tako se ucita staticka mapa prostora
GridMap::GridMap(double cell_size)
{
          counter = 0;
		  counter_moving = -3; ////-1 i -2 i -3 su rezervirani za mjesta sudara s pokretnim preprekama, mora bit razlicit od countera, to su negativni brojevi (ne smije bit preklapanja)
		  numnewmovings = 0; numnewstatic = 0;numnewmovingcells = 0;numoldmovings = 0;pocetak=0;
		  numnewcg =0;numoldcg =0;
		  num_col_points_old=0;num_col_points=0;  numindeksimapepunjenje=0;  numindeksimapepraznjenje=0;numindeksizanepunjenje=0;
          Map_Cell_Size = cell_size;
	  linije = (double **)malloc(900*sizeof(double *)) ;
	  for (int i=0; i<900; i++)
		  linije[i] = (double *)malloc(4*sizeof(double)) ;
#if (READWLD==1)
	  if(fill_Map_Init())  //unutra se alocira mapa prema wld datoteci
          {
             printf("Konstruiran je Grid Map objekt iz wld datoteke\n");
          }else{
             printf(" Nije konstruiran je Grid Map objekt iz wld datoteke\n");
	   }
#endif
	//alociranje memorije
	   indeksimapepunjenje = (I_point *)malloc(PLAYER_LASER_MAX_SAMPLES*sizeof(I_point)) ;
	   indeksimapepraznjenje = (I_point *)malloc(PLAYER_LASER_MAX_SAMPLES*sizeof(I_point)) ;
	   indeksizanepunjenje = (I_point *)malloc(PLAYER_LASER_MAX_SAMPLES*sizeof(I_point)) ;
	   moving= (R_point *)malloc(PLAYER_LASER_MAX_SAMPLES*sizeof(R_point)) ;
	   statickiHitovi= (R_point *)malloc(PLAYER_LASER_MAX_SAMPLES*sizeof(R_point)) ;
	   moving_cell= (R_point *)malloc(PLAYER_LASER_MAX_SAMPLES*sizeof(R_point)) ;
	   stare_moving= (R_point *)malloc(PLAYER_LASER_MAX_SAMPLES*sizeof(R_point)) ;
	   stare_moving_pom= (R_point *)malloc(PLAYER_LASER_MAX_SAMPLES*sizeof(R_point)) ;
	   moving_index= (int *)malloc(101*sizeof(int)) ;
	   moving_cell_index= (int *)malloc(101*sizeof(int)) ;
	   col_moving = (I_point *)malloc(101*sizeof(I_point)) ;
	   col_moving_old = (I_point *)malloc(101*sizeof(I_point)) ;
	   stare_cg= (R_point *)malloc(101*sizeof(R_point)) ;
	   cg= (R_point *)malloc(101*sizeof(R_point)) ;
}

void GridMap::reset()
{
 	//resetiramo sva ucitanja u GridMapu osim onih koji su vezani za staticke celije
  		for (int i=0; i < Map_Dim_X; i++)
		{
			for (int j=0; j < Map_Dim_Y; j++){
			
				if(Map[i][j].static_cell){
					Map[i][j].occupancy=GRID_MAP_OCCUPANCY_TRESHOLD+5; //zbog exploration_node
//----------------------------------------------------------------
#if RECTANGULAR
	cspace->updateClearance(i,j,0);
#endif
//----------------------------------------------------------------	  
				}else{
				Map[i][j].occupancy=0;
//----------------------------------------------------------------
#if RECTANGULAR
	cspace->updateClearance(i,j,INT_MAX);
#endif
//----------------------------------------------------------------	  
				}
				Map[i][j].time_stamp=0;
			}
		}
//----------------------------------------------------------------
#if RECTANGULAR
#if DSTAR3DORI
//  cspace->initializeDStar();
#else
//#endif
	cspace->update(false);
#endif
#endif
//----------------------------------------------------------------	  
		printf("GM: reset()\n");
		//globalni counter preko kojeg se moze racunati time_stamp
		counter=0;counter_moving=-3;//-1 i -2 i -3 su rezervirani za mjesta sudara s pokretnim preprekama, prvi dekrement ce ga postaviti u -4
		numnewmovings=0; numnewstatic=0;numnewmovingcells=0;numoldmovings = 0;pocetak=0;numnewcg=0;numoldcg=0;num_col_points_old=0;num_col_points=0;  numindeksimapepraznjenje=0; numindeksimapepunjenje=0;numindeksizanepunjenje=0;brojnovih=0;
		nonempty=1;
}


//da li je tocka unutar dimenzija mape
int GridMap::check_point(R_point current_point)
{
  double x,y;
  x=current_point.x-Map_Home.x;
  y=current_point.y-Map_Home.y;
#if RECTANGULAR
  double threal=fmod(current_point.th,2*M_PI);
  if (threal<0) threal+=2*M_PI;
  if (threal>2*M_PI) threal-=2*M_PI;
	cell_point_temp.th=cspace->worldToMapTheta(threal);
	if ((cell_point_temp.th<0)||(cell_point_temp.th>OrientationIntervals::getMaxOrientation())){
    printf("nevaljali kut %d\n",cell_point_temp.th);
	}
#endif
  if((x>=0.0)&&(x<=Map_Dim_X_A)&&(y>=0.0)&&(y<=Map_Dim_Y_A)){
      cell_point_temp.x=(int)floor(x/Map_Cell_Size);
      cell_point_temp.y=(int)floor(y/Map_Cell_Size);
    return 1;
  }else{
    return 0;
  }
}



//inicijalizacija grid mape iz wld datoteke
int GridMap::fill_Map_Init()
{
  FILE *F, *D;
  D = fopen("data","w+");
  //double width = 0.0;
  //double height = 0.0;
  int brojlinija = 0;
  char rdLine[81]="";
	char *line;
	char *word;
	int i, j;
	double temp[4]; // za x1 y1 x2 y2
	//inicijalizacija
	cstransf.stack[0][0] = 0.0;     // pocetna vrijednost na stacku
  cstransf.stack[0][1] = 0.0;
  cstransf.stack[0][2] = 0.0;
	cstransf.top = 1;
  for (i=0; i < 3; i++)
    for (j=0; j < 3; j++)
      {
        if (i==j) cstransf.T[i][j] = 1;
        else cstransf.T[i][j] = 0;
      }
	double orientation, x_next, y_next, x0new, y0new, thnew;
  
  if ((F = fopen(WH->wld,"r")) != NULL)
	{
    fprintf(D,"otvorena wld datoteka za citanje\n");
    while (fgets(rdLine,50,F) != NULL)
		{
			line=&rdLine[0];
			if (line[0] == ';' || line[0] == '#')
			{
				fprintf(D,"komentar\n");
				continue;
			}
			word = strtok (line," ");


			if (strcmp(word,"width")==0)
			{
			  word = strtok (NULL, " \n");
			  Map_Dim_X_A = atoi(word);
        fprintf(D,"Map_Dim_X_A=%f\n",Map_Dim_X_A);
			}
      else if (strcmp(word,"height")==0)
			{
			  word = strtok (NULL, " \n");
			  Map_Dim_Y_A = atoi(word);
        fprintf(D,"Map_Dim_Y_A=%f\n",Map_Dim_Y_A);
			}
			else if (strcmp(word,"origin")==0)
			{
					word = strtok (NULL, " \n");
					Map_Home.x = atoi(word);
					word = strtok (NULL, " \n");
					Map_Home.y = atoi(word);
					fprintf(D,"Map_Home=(%f,%f)\n",Map_Home.x,Map_Home.y);
			}
			else if (strcmp(word,"push")==0)
			{
					printf("push a %s\n",word);
					word = strtok (NULL, " \n");
					x0new = atoi(word);
					word = strtok (NULL, " \n");
					y0new = atoi(word);
					word = strtok (NULL, " \n");
					thnew = atoi(word) * SuR;
					push(x0new, y0new, thnew);
			}
			else if (strcmp(word,"pop")==0)
			{
				  pop();
			}
			else if (strcmp(word,"position")==0)
			{
			}
			else
      {
					//linije = (features *)realloc(linije,(brojlinija + 1)*sizeof(features));
			  i=0;
			  while (word != NULL)
			  {
					temp[i] = atoi(word);
					i++;
					word = strtok (NULL, " \n");
			  }
					//mnozenje s matricom transformacije koord.sus.
			  linije[brojlinija][0] = cstransf.T[0][0] * temp[0] + cstransf.T[0][1] * temp[1] + cstransf.T[0][2];
			  linije[brojlinija][1] = cstransf.T[1][0] * temp[0] + cstransf.T[1][1] * temp[1] + cstransf.T[1][2];
			  linije[brojlinija][2] = cstransf.T[0][0] * temp[2] + cstransf.T[0][1] * temp[3] + cstransf.T[0][2];
			  linije[brojlinija][3] = cstransf.T[1][0] * temp[2] + cstransf.T[1][1] * temp[3] + cstransf.T[1][2];
			  brojlinija++;
			
			}
		}
		
    //zavrsetak citanja linija iz mape
    printf("====GridMap::fill_Map_Init()> broj linija je=%d=====\n", brojlinija);
    fclose(F);
        
	if ((Map_Dim_X_A != 0.0) && (Map_Dim_Y_A != 0.0))
		{
			Map_Dim_X = int (floor (Map_Dim_X_A / Map_Cell_Size))+1;
			Map_Dim_Y = int (floor (Map_Dim_Y_A / Map_Cell_Size))+1;
      alloc_Map();
			for (i=0; i < brojlinija; i++)
			{
        orientation = atan2((linije[i][3] - linije[i][1]),(linije[i][2] - linije[i][0]));
				mapper_point_temp.x = linije[i][0];
				mapper_point_temp.y = linije[i][1];
				if (check_point(mapper_point_temp))
				{
					Map[cell_point_temp.x][cell_point_temp.y].occupancy = GRID_MAP_OCCUPANCY_TRESHOLD + 1;
					Map[cell_point_temp.x][cell_point_temp.y].x=mapper_point_temp.x-Map_Home.x-Map_Dim_X_A/2.;          //prema player-stage karti
					Map[cell_point_temp.x][cell_point_temp.y].y=mapper_point_temp.y-Map_Home.y-Map_Dim_Y_A/2.;
				} else fprintf(D,"javi!");

				mapper_point_temp.x = linije[i][2];
				mapper_point_temp.y = linije[i][3];

				if (check_point(mapper_point_temp))
				{
					Map[cell_point_temp.x][cell_point_temp.y].occupancy = GRID_MAP_OCCUPANCY_TRESHOLD + 1;
					Map[cell_point_temp.x][cell_point_temp.y].x=mapper_point_temp.x-Map_Home.x-Map_Dim_X_A/2.;       //prema player-stage karti
					Map[cell_point_temp.x][cell_point_temp.y].y=mapper_point_temp.y-Map_Home.y-Map_Dim_Y_A/2.;
				} else fprintf(D,"javi!%f %f\n",mapper_point_temp.x,mapper_point_temp.y);


				x_next = linije[i][0] + (Map_Cell_Size/2) * cos(orientation);
				y_next = linije[i][1] + (Map_Cell_Size/2) * sin(orientation);


				while (sqrt((x_next - linije[i][2]) * (x_next - linije[i][2]) + (y_next - linije[i][3]) * (y_next - linije[i][3])) > (Map_Cell_Size/2))
				{
					mapper_point_temp.x = x_next;
					mapper_point_temp.y = y_next;
					if (check_point(mapper_point_temp))
					{
						Map[cell_point_temp.x][cell_point_temp.y].occupancy = GRID_MAP_OCCUPANCY_TRESHOLD + 1;
						Map[cell_point_temp.x][cell_point_temp.y].x=mapper_point_temp.x-Map_Home.x-Map_Dim_X_A/2.;       //prema player-stage karti
						Map[cell_point_temp.x][cell_point_temp.y].y=mapper_point_temp.y-Map_Home.y-Map_Dim_Y_A/2.;
				

						x_next += (Map_Cell_Size/2) * cos(orientation);
						y_next += (Map_Cell_Size/2) * sin(orientation);

					}
          else
					{
						fprintf(D,"nije dobar%f %f\n",mapper_point_temp.x,mapper_point_temp.y);
						break;
					}
				}   //gotov while
			
      }  //gotov for
		} else {
			printf("dimenzije u wld karti ne valjaju\n");
			return 0;}   //od ifa

	} else {
		printf("ne mogu otvoriti datoteku wld\n");
		return 0;}       //nije uspio otvoriti datoteku wld

	if ((F = fopen("gridmapa.dat","wt")) != NULL)
	{
		for (i=0; i < Map_Dim_X; i++)
		{
			for (j=0; j < Map_Dim_Y; j++)
				fprintf(F,"%d ",Map[i][j].occupancy);
			fprintf(F,"\n");
		}
		fclose(F);
	}
  //labeliranje ucitanih podataka u mapu kao static
  	for (i=0; i < Map_Dim_X; i++)
	{
		for (j=0; j < Map_Dim_Y; j++){
			
			if(Map[i][j].occupancy>GRID_MAP_OCCUPANCY_TRESHOLD){
				Map[i][j].static_cell=true;
			}
		}
	}
	//sad drugi home za player-stage
	Map_Home.x=Map_Dim_X_A/(-2.);
	Map_Home.y=Map_Dim_Y_A/(-2.);
	fprintf(D,"home za player-stage: Map_Home.x=%f\n",Map_Home.x);
	fprintf(D,"home za player-stage: Map_Home.y=%f\n",Map_Home.y);
	fclose(D);
	if (((D = fopen("logger//origin.dat","wt")) != NULL))
	{
		fprintf(D,"%f %f\n",Map_Home.x,Map_Home.y);
		fclose(D);
	}
  	if (((D = fopen("logger//cell_size.dat","wt")) != NULL))
  	{
	  fprintf(D,"%f\n",Map_Cell_Size);
	  fclose(D);
  	}
	return 1;
}


//dodavanje novog koordinatnog sustava kao nacin punjenja mape
void GridMap::push (double x0, double y0, double th)
{
	double c, s, temp0, temp1, temp2;
	cstransf.stack[cstransf.top][0] = x0;
	cstransf.stack[cstransf.top][1] = y0;
	cstransf.stack[cstransf.top][2] = th;
	cstransf.top++;
	c = cos(th);
	s = sin(th);
	//mnozenje matrica
	temp0 = cstransf.T[0][0] * c - cstransf.T[0][1] * s;
	temp1 = cstransf.T[0][0] * s + cstransf.T[0][1] * c;
	temp2 = cstransf.T[0][0] * x0 + cstransf.T[0][1] * y0 + cstransf.T[0][2];
	//prvi red ...
	cstransf.T[0][0] = temp0;
	cstransf.T[0][1] = temp1;
	cstransf.T[0][2] = temp2;

	temp0 = cstransf.T[1][0] * c - cstransf.T[1][1] * s;
	temp1 = cstransf.T[1][0] * s + cstransf.T[1][1] * c;
	temp2 = cstransf.T[1][0] * x0 + cstransf.T[1][1] * y0 + cstransf.T[1][2];
	//drugi red ...
	cstransf.T[1][0] = temp0;
	cstransf.T[1][1] = temp1;
	cstransf.T[1][2] = temp2;

	temp0 = cstransf.T[2][0] * c - cstransf.T[2][1] * s;
	temp1 = cstransf.T[2][0] * s + cstransf.T[2][1] * c;
	temp2 = cstransf.T[2][0] * x0 + cstransf.T[2][1] * y0 + cstransf.T[0][2];
	//treci red ...
	cstransf.T[2][0] = temp0;
	cstransf.T[2][1] = temp1;
	cstransf.T[2][2] = temp2;
}

void GridMap::pop()
{
	if (cstransf.top > 1)
	{
		double s, c, temp0, temp1, temp2;
		int i,j;
		cstransf.top--;
    // opet inicijalizacija
    for (i=0; i < 3; i++)
      for (j=0; j < 3; j++)
        {
          if (i==j) cstransf.T[i][j] = 1;
          else cstransf.T[i][j] = 0;
        }

    for (i=0; i < cstransf.top; i++)
		{
			c = cos(cstransf.stack[i][2]);
			s = sin(cstransf.stack[i][2]);
			//mnozenje matrica
			temp0 = cstransf.T[0][0] * c - cstransf.T[0][1] * s;
			temp1 = cstransf.T[0][0] * s + cstransf.T[0][1] * c;
			temp2 = cstransf.T[0][0] * cstransf.stack[i][0] + cstransf.T[0][1] * cstransf.stack[i][1] + cstransf.T[0][2];
			//prvi red ...
			cstransf.T[0][0] = temp0;
			cstransf.T[0][1] = temp1;
			cstransf.T[0][2] = temp2;

			temp0 = cstransf.T[1][0] * c - cstransf.T[1][1] * s;
			temp1 = cstransf.T[1][0] * s + cstransf.T[1][1] * c;
			temp2 = cstransf.T[1][0] * cstransf.stack[i][0] + cstransf.T[1][1] * cstransf.stack[i][1] + cstransf.T[1][2];
			//drugi red ...
			cstransf.T[1][0] = temp0;
			cstransf.T[1][1] = temp1;
			cstransf.T[1][2] = temp2;

			temp0 = cstransf.T[2][0] * c - cstransf.T[2][1] * s;
			temp1 = cstransf.T[2][0] * s + cstransf.T[2][1] * c;
			temp2 = cstransf.T[2][0] * cstransf.stack[i][0] + cstransf.T[2][1] * cstransf.stack[i][1] + cstransf.T[0][2];
			//treci red ...
			cstransf.T[2][0] = temp0;
			cstransf.T[2][1] = temp1;
			cstransf.T[2][2] = temp2;
		}
	}
}

//sa fill_Map direktno upucavamo laser mjerenja u grid mapu (brojimo hitove  u odredjenoj celiji)
int GridMap::fill_Map()
{
	//numindeksimape=0;//brojim hitove koje treba azurirati u planeru u mapi   u 0 se postavlja u planeru nakon citanja, DW pise isto
	counter++; //brojac ciklusa, oznaka time_stampa svakog hita da se zna u kojem ciklusu su se zapisali
	counter_moving--;//posebni brojac za pokretne prepreke, ista uloga, da se zna u kojem ciklusu se zapisala pokretna prepreka, counter_moving!=counter
	int reading_count=0; //broj laserskih ocitanja koja su unutar mape
	//-------------------------------------------ovaj dio je vezan za trazenje pokretne prepreke --------------------------------------------------------------------------------
	double delta_r,delta_th,teziste_x,teziste_y; //varijable za racunanje tezista pokretnih prepreka
  int pamti=0;//varijabla koja pamti broj nestatickih novih laserskih hitova
  numnewstatic=0;//broj statickih laserskih hitova u ciklusu
  int staticka,slijed_statickih,slijed_nestatickih; //pomocne varijable za racunanje tezista pokretnih prepreka
  I_point prva_tocka,van_tocka;
  R_point prva_tocka_real,van_tocka_real;
  bool jedna_tocka;
  int around_static=2; //moving obstacle must be distanced for this number of cells from the static cell
  teziste_x=0.;teziste_y=0.;//inicijalizacija suma x i y koordinata
  double angleborder=M_PI/2; //definira snitu torte vidljivosti
  double anglefilt=LASER_MIN_MAX_ANGLE*M_PI/180.; //filtered laser readings due to obstacles on the robot shape
  double angleres=0.5*M_PI/180.; //svakih pola stupnja zraka default
//   int tezisteneudstar=-1; //pamti indeks tezista koje se ne smije upisati u dstar mapu jer je u DW zabiljezen sudar s translatiranom pokretnom preprekom
//   I_point pomocnoOdTezista[WH->LB.laser_pointer_new];
//   int brojpomOdTez=0, initTez=0;//gornji red i ova dva sluze za cuvanje polja od jednog tezista za slucaj da se to teziste ne treba upisati u dstar mapu
  WH->LB.broj_tezista=0; //inicijalizacija broja tezista po ciklusu
  int broj=0;
  int ii;
  //position of the first and the second laser in the global coordinate system
  double laserx, lasery, laser2x, laser2y;
//  laserx=WH->Lokalna_u_globalnu_x(WH->RB.x, WH->RB.th, WH->laserx, WH->lasery);
//  lasery=WH->Lokalna_u_globalnu_y(WH->RB.y, WH->RB.th, WH->laserx, WH->lasery);
//  laser2x=WH->Lokalna_u_globalnu_x(WH->RB.x, WH->RB.th, WH->laser2x, WH->laser2y);
//  laser2y=WH->Lokalna_u_globalnu_y(WH->RB.y, WH->RB.th, WH->laser2x, WH->laser2y);
  laserx=WH->laserx;
  lasery=WH->lasery;
  laser2x=WH->laser2x;
  laser2y=WH->laser2y;
  if (WH->LB.laser_pointer_new) {
  	angleborder=fabs(WH->LB.sviscanovi[0].th);//medo prava kutna granica od svih ocitanja lasera
  	angleres=fabs(WH->LB.sviscanovi[WH->LB.laser_pointer_new-1].th-WH->LB.sviscanovi[WH->LB.laser_pointer_new-2].th);
  	printf("angleborder=%f, anglefilt=%f, laser in g.k.s (%f,%f,%f), WH->RB =(%f,%f,%f)\n",angleborder,anglefilt,laserx,lasery,WH->laserth,WH->RB.x,WH->RB.y,WH->RB.th);
  for (int i=0;i<numoldmovings;i++) {//cuvati polje starih pokretnih i brisati ih ako su unutar konusa vidljivosti od 3 m ispred novih ocitanja
	  mapper_point_temp.x=stare_moving[i].x;
	  mapper_point_temp.y=stare_moving[i].y;
	  double angle_old_moving=atan2((stare_moving[i].y-lasery),(stare_moving[i].x-laserx));
	  double distance_old_moving=(stare_moving[i].y-lasery)*(stare_moving[i].y-lasery)+(stare_moving[i].x-laserx)*(stare_moving[i].x-laserx);
//	  double local_th1=angle_old_moving-WH->RB.th-WH->laserth;
	  double local_th1=angle_old_moving-WH->laserth;
	  int zadrzi=0;
	  if (local_th1<-M_PI) local_th1+=2*M_PI;
	  if (local_th1>M_PI) local_th1-=2*M_PI;
	  //ovo s -kutnagranica radi samo ako je laser centriran a to je uvijek s laserima
	  if (check_point(mapper_point_temp)&&(local_th1<anglefilt)&&(local_th1>-1*anglefilt)&&(distance_old_moving<(LASER_RANGE_MAX-2.*Map_Cell_Size)*(LASER_RANGE_MAX-2.*Map_Cell_Size))) {//da ne bi po rubovima pisao brisao
// 		  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp!=-1)) {//u ovom slucaju bi mogao prebrisati -1, pa zato treba taj uvjet ispitati
		  //gledamo da li su stare prepreke blizu onih indeksizanepunjenje pa da ih izbrisemo ali samo iz D* mape. ovdje ce ostati
		  if ((num_col_points) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-1) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -2)){
			  for (int q=0;q<numindeksizanepunjenje;q++){//tu popraviti, neke se proguraju slucajno, tj, mozda se ne poklapaju sve sa tim zapamcenim poljima
				  
//					  if ((indeksizanepunjenje[q].x==cell_point_temp.x)&&(indeksizanepunjenje[q].y==cell_point_temp.y)){
				  if ((abs(indeksizanepunjenje[q].x-cell_point_temp.x)<=4)&&(abs(indeksizanepunjenje[q].y-cell_point_temp.y)<=4)){//stavljam slabiji uvjet, 4 susjedna polja mogu proc
					  indeksimapepraznjenje[numindeksimapepraznjenje]=cell_point_temp;
					  numindeksimapepraznjenje++;
					  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = -2;
					  break;
				  }
			  }
		  }
		  //jos treba uvjeta da ne prebrise prepreku koja je iza zida 
		  ii=int((local_th1+angleborder)/angleres);//medo popravio di je u polju svih laserskih ocitanja kojih ima 181 npr
// 			  ii=int((lokalna_th1+PI/2)*35/PI);//di je u polju svih laserskih ocitanja
// 			  printf("kut prepreke=%f, udaljenost=%f, odredjeni indeks=%d, kut od indeksa=%f, r od indeksa=%f\n",lokalna_th1,trimetra,ii,WH->LB.sviscanovi[ii].th,WH->LB.sviscanovi[ii].r);
/*			  if (ii-2>=0)
				  printf("r od ii-1 = %f, ii-2 = %f\n",WH->LB.sviscanovi[ii-1].r,WH->LB.sviscanovi[ii-2].r);
			  if (ii+2<=360)
				  printf("r od ii+1 = %f, ii+2 = %f\n",WH->LB.sviscanovi[ii+1].r,WH->LB.sviscanovi[ii+2].r);*/
			  if (distance_old_moving<(WH->LB.sviscanovi[ii].r-Map_Cell_Size)*(WH->LB.sviscanovi[ii].r-Map_Cell_Size)){//radim pomake za jedno polje nesigurnosti
// 				  printf("kut prepreke=%f, udaljenost=%f, odredjeni indeks=%d, r od indeksa=%f\n",lokalna_th1,trimetra,ii,WH->LB.sviscanovi[ii]);
// 				  printf("ovu brisem iz GM\n");
				  //oni s -2 su vec u polju za brisanje, a oni s -1 ne smiju ic u polje za brisanje i one s counter_moving su isto vec u polju za brisanje
				  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -2) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -1) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp != counter_moving)){
// 					  printf("ovu cu brisati iz DS\n");
					  indeksimapepraznjenje[numindeksimapepraznjenje].x=cell_point_temp.x;   //azuriram indekse za planera
					  indeksimapepraznjenje[numindeksimapepraznjenje].y=cell_point_temp.y;
					  numindeksimapepraznjenje++;
					  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = counter_moving; //oznaka ciklusa u kojem se brise kao stara pokretna (uociti razliku od -2)
					  //ne trebam ovo raditi jer se pazi da se duplo ne upise pomocu time_stampa
/*					  int duplic=0;
					  for (int z=0;z<numindeksimapepraznjenje;z++){
						  if ((indeksimapepraznjenje[z].x==cell_point_temp.x)&&(indeksimapepraznjenje[z].y==cell_point_temp.y)){
							  duplic=1;
							  break;
						  }
					  }
					  if (!duplic){
						  indeksimapepraznjenje[numindeksimapepraznjenje].x=cell_point_temp.x;   //azuriram indekse za planera
						  indeksimapepraznjenje[numindeksimapepraznjenje].y=cell_point_temp.y;
						  numindeksimapepraznjenje++;
						  printf("ovu cu brisati iz DS\n");
					  }*/
				  }
/*			  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = counter_moving; //oznaka ciklusa u kojem se brise
			  if (num_col_points) {
				  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = -2;
			  }*/
		  	Map[cell_point_temp.x][cell_point_temp.y].occupancy=0;//odmah isprazni staru pokretnu prepreku (ako nije ponovo upisana, naravno)-tu izmijeniti!
			  }else{
				  zadrzi=1;
			  }
/*		  }else{
			  zadrzi=1;
		  }*/
	  }else{
		  zadrzi=1;
	  }
	  if (zadrzi){
/*		  stare_moving_pom[broj].x=stare_moving[i].x;
		  stare_moving_pom[broj].y=stare_moving[i].y;*/
		  stare_moving[broj].x=stare_moving[i].x;//i>=broj pa prepisujemo one koje smo vec prosli
		  stare_moving[broj].y=stare_moving[i].y;
		  broj++;
	  }
  }
  numoldmovings=broj;//stare celije
  //nepotrebno
/*  for (int i=0;i<numoldmovings;i++){
	  stare_moving[i].x=stare_moving_pom[i].x;
	  stare_moving[i].y=stare_moving_pom[i].y;
  }*/
  for(int i=0;i<numnewmovingcells;i++) { //stare pokretne prepreke (hitovi iz proslog ciklusa), ako ih naravno ima
	  mapper_point_temp.x=moving_cell[i].x;    //gledaju se samo celije, a ne sva laserska mjerenja
	  mapper_point_temp.y=moving_cell[i].y;
	  double angle_moving=atan2((mapper_point_temp.y-lasery),(mapper_point_temp.x-laserx));
//	  double local_th=angle_moving-WH->RB.th-WH->laserth;
	  double local_th=angle_moving-WH->laserth;
	  double distance_moving=(moving_cell[i].y-lasery)*(moving_cell[i].y-lasery)+(moving_cell[i].x-laserx)* (moving_cell[i].x-laserx);
	  int prepisati=0;
	  if (local_th<-M_PI) local_th+=2*M_PI;
	  if (local_th>M_PI) local_th-=2*M_PI;
/*	  if ((num_col_points)&&(num_col_points_old==0))
		  printf("tu bi bilo mozda za brisanje, lokalna_th=%f-%f=%f\n",kutic,WH->RB.th,lokalna_th);*/
      //ako je hit unutar mape i ispred robota (u rangeu), da ne bi brisao ono sto je iza njega dok se okrece
	  if(check_point(mapper_point_temp)&&(local_th<anglefilt)&&(local_th>-1*anglefilt)&&(distance_moving<(LASER_RANGE_MAX-2.*Map_Cell_Size)*(LASER_RANGE_MAX-2.*Map_Cell_Size)))
	  {//ispitati slucaj da ne postoje stari col pointovi - samo onda postavljam time_stamp na counter_moving
/*		  if ((num_col_points)&&(num_col_points_old==0))
			  printf("time_stamp=%d, lokalna_th=%f-%f=%f\n",Map[cell_point_temp.x][cell_point_temp.y].time_stamp,kutic,WH->RB.th,lokalna_th);*/
// 		  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp!=-1)){//u ovom slucaju bi mogao prebrisati -1, pa zato treba taj uvjet ispitati. tu je bio vragec (num_col_points_old==0)&&
		  //gledamo da li su stare prepreke blizu onih indeksizanepunjenje pa da ih izbrisemo ali samo iz D* mape. ovdje ce ostati
		  if ((num_col_points) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-1) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -2)){
			  for (int q=0;q<numindeksizanepunjenje;q++){//tu popraviti, neke se proguraju slucajno, tj, mozda se ne poklapaju sve sa tim zapamcenim poljima
				  
//					  if ((indeksizanepunjenje[q].x==cell_point_temp.x)&&(indeksizanepunjenje[q].y==cell_point_temp.y)){
				  if ((abs(indeksizanepunjenje[q].x-cell_point_temp.x)<=4)&&(abs(indeksizanepunjenje[q].y-cell_point_temp.y)<=4)){//stavljam slabiji uvjet, 4 susjedna polja mogu proc
					  indeksimapepraznjenje[numindeksimapepraznjenje]=cell_point_temp;
					  numindeksimapepraznjenje++;
					  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = -2;
					  break;
				  }
			  }
		  }
			  //jos treba uvjeta da ne prebrise iza zida prepreku
		   ii=int((local_th+angleborder)/angleres);//di je u polju svih laserskih ocitanja kojih ima 181
// 		  ii=int((lokalna_th+PI/2)*35/PI);//di je u polju svih laserskih ocitanja
// 			  printf("numnewmovingcells: kut prepreke=%f, udaljenost=%f, odredjeni indeks=%d, kut od indeksa=%f, r od indeksa=%f\n",lokalna_th,trimetra,ii,WH->LB.sviscanovi[ii].th,WH->LB.sviscanovi[ii].r);
/*			  if (ii-2>=0)
				  printf("r od ii-1 = %f, ii-2 = %f\n",WH->LB.sviscanovi[ii-1].r,WH->LB.sviscanovi[ii-2].r);
			  if (ii+2<=360)
				  printf("r od ii+1 = %f, ii+2 = %f\n",WH->LB.sviscanovi[ii+1].r,WH->LB.sviscanovi[ii+2].r);*/
			  if (distance_moving<(WH->LB.sviscanovi[ii].r-Map_Cell_Size)*(WH->LB.sviscanovi[ii].r-Map_Cell_Size)){
// 				  printf("numnewmovingcells: kut prepreke=%f, udaljenost=%f, odredjeni indeks=%d, r od indeksa=%f\n",lokalna_th,trimetra,ii,WH->LB.sviscanovi[ii]);
// 				  printf("ovu brisem iz GM\n");
				  //oni s -2 su vec u polju za brisanje, a oni s -1 ne smiju ic u polje za brisanje i one s counter_moving su isto vec u polju za brisanje
				  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -2) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -1) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp != counter_moving)){
// 					  printf("ovu cu brisati iz DS\n");
					  indeksimapepraznjenje[numindeksimapepraznjenje].x=cell_point_temp.x;   //azuriram indekse za planera
					  indeksimapepraznjenje[numindeksimapepraznjenje].y=cell_point_temp.y;
					  numindeksimapepraznjenje++;
					  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = counter_moving; //oznaka ciklusa u kojem se brise
					  //ne trebam ovo raditi jer se pazi da se duplo ne upise pomocu time_stampa
/*			  int duplic=0;
			  for (int z=0;z<numindeksimapepraznjenje;z++){
				  if ((indeksimapepraznjenje[z].x==cell_point_temp.x)&&(indeksimapepraznjenje[z].y==cell_point_temp.y)){
					  duplic=1;
					  break;
				  }
			  }
			  if (!duplic){
				  indeksimapepraznjenje[numindeksimapepraznjenje].x=cell_point_temp.x;   //azuriram indekse za planera
				  indeksimapepraznjenje[numindeksimapepraznjenje].y=cell_point_temp.y;
				  numindeksimapepraznjenje++;
			  }*/
			}
/*			if (num_col_points) {
				  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = -2;//brise pomocu -2, a occ moze biti >0
			  }*/
			  Map[cell_point_temp.x][cell_point_temp.y].occupancy=0;//odmah isprazni staru pokretnu prepreku (a poslije ce se ponovo upisati)
			  }else{
				  prepisati=1;//oznaka za prepisati u stare_moving
			  }
/*		  } else{
			  prepisati=1;//oznaka za prepisati u stare_moving
		  }*/
	  } //check_point
	  else{
		  prepisati=1;
	  }
	  if (prepisati){
		  //tu moramo gledati duplice jer se u svakom ciklusu puno istih zabiljezava kao moving_cell
		  int duplic=0;//gledamo do broj (to su one od proslih ciklusa a ne ovog, odredjeno iznad u for petlji)
		  int x,y;
		  for (int z=0;z<broj;z++){
			  x=(int)floor((stare_moving[z].x-Map_Home.x)/Map_Cell_Size);
			  y=(int)floor((stare_moving[z].y-Map_Home.y)/Map_Cell_Size);
			  if ((x==cell_point_temp.x)&&(y==cell_point_temp.y)){
				  duplic=1;
				  break;
			  }
		  }
		  if (!duplic){
			  if ((numoldmovings>0)&&(numoldmovings%(PLAYER_LASER_MAX_SAMPLES)==0)){
			  if(((stare_moving = (R_point *)realloc( stare_moving, (numoldmovings+PLAYER_LASER_MAX_SAMPLES)*sizeof(R_point)))!=NULL)){
			  
			printf("GM::moving REALLOC succeeded\n");  
			  }else{
			printf("GM::moving Error allocating memory REALLOC!!!\n");
			return 0;
		}

// 				  printf("pocetak=%d\n",pocetak);
//				  pocetak=pocetak % numoldmovings;  //kruzno polje
//				  stare_moving[pocetak].x=mapper_point_temp.x;
//				  stare_moving[pocetak].y=mapper_point_temp.y;
//				  pocetak++;
				  
			  }
			  //else
			  {
				  pocetak=0;
				  stare_moving[numoldmovings].x=mapper_point_temp.x;
				  stare_moving[numoldmovings].y=mapper_point_temp.y;
				  numoldmovings++;
			  }
		  }
 	  }
}	
//numoldmovings=numnewmovings;
	numnewmovings=0; //resetiranje novih pokretnih prepreka
	numnewmovingcells=0;
	for(int i=0;i<numnewcg;i++) {//stara tezista (iz proslog ciklusa)
		stare_cg[i].x=cg[i].x;
		stare_cg[i].y=cg[i].y;
		numoldcg=numnewcg;
	}
	numnewcg=0;  //resetiranje novih tezista
  }//zatvoren uvjet da li ima uopce ocitanja lasera u ovom ciklusu
	slijed_statickih=0; //varijabla koja oznacuje slijed hitova koji oznacuju staticku prepreku u mapi (ako je slijed_statickih==0 znaci da se pojavila prva staticka u nizu, za prvu staticku treba izracunati teziste od pozbrajanih hitova nestatickih prepreka prije te staticke prepreke)
	slijed_nestatickih=0; //varijabla koja oznacuje slijed hitova koji oznacuju nestaticku prepreku (ako je slijed_nestatickih==0 znaci da se pojavila prva nestaticka u nizu, za prvu nestaticku se ne racuna delta_r i delta_th, a za drugu na dalje se racunaju i ako su preveliki onda se izracuna teziste od pozbrajanih hitova do tog debeljka)
	//hitovi su u redoslijedu po rastucim kutevoma od kuta -pi/2 do pi/2 u koordinatnom sustavu robota
	//--------------------------------------------sad dolazi mali dio opceg dijela za ubacivanje prepreke u mapu----------------------------------------------------------
// 	for(int i=0;i<WH->LB.laser_pointer_new;i++){
	for(int i=0;i<WH->LB.laser_manji_new;i++){
     //raw data sa lasera u globalnim koordinatama prostora
//       mapper_point_temp.x=(WH->LB.LB_pose[i]).getX();
//       mapper_point_temp.y=(WH->LB.LB_pose[i]).getY();
      mapper_point_temp.x=(WH->LB.LB_pose_manji[i].x);
      mapper_point_temp.y=(WH->LB.LB_pose_manji[i].y);
//       distance=((WH->RB.x-mapper_point_temp.x)*(WH->RB.x-mapper_point_temp.x)+(WH->RB.y-mapper_point_temp.y)*(WH->RB.y-mapper_point_temp.y));
    //  if ((1||(mapper_point_temp.x<18000.)||(mapper_point_temp.x>18300.)||(mapper_point_temp.y<8600.)||(mapper_point_temp.y>8900.))&&((mapper_point_temp.x<12900.)||(mapper_point_temp.x>13200.)||(mapper_point_temp.y<14700.)||(mapper_point_temp.y>15200.)))
      //ako je hit unutar mape
      if(check_point(mapper_point_temp))//&&(distance<LASER_RANGE_MAX*LASER_RANGE_MAX))
      {
//treba detektirati usrednjenu vrijednost iz lasera koja je na sredini uskog prolaza, nju ne upisujem u D* mapu, vec se upisala u gmap
	      jedna_tocka=false;//radit ce se o samo jednoj tocki pa nju ne upisujem (pamti==1)
//-------------------------------------------sad opet dio vezan za trazenje pokretne prepreke --------------------------------------------------------------------------------
		  //racunamo teziste samo od neunesenih prepreka u GM, znaci od nestatickih prepreka i ne smije biti ni u okolini staticke prepreke (dvije celije) (jedna)
		  staticka=0;//varijabla koja oznacava da li ima staticke prepreke u okolini novog hita lasera
		  for(int k=-around_static;k<=around_static;k++){
			  for(int l=-around_static;l<=around_static;l++){
				  if (((cell_point_temp.x+k)>=0)&&((cell_point_temp.x+k)<Map_Dim_X)&&((cell_point_temp.y+l)>=0)&&((cell_point_temp.y+l)<Map_Dim_Y)){
				  if (Map[cell_point_temp.x+k][cell_point_temp.y+l].static_cell) {
						staticka=1;
						break;//cim nadje brejka
					}
				  }
			  }
				if (staticka==1)      //izlazi iz obje for petlje i ne zeli racunati teziste
					break;
			}
			if ((staticka==1)&&(slijed_statickih==0)) {     //to je prva staticka kao prvi hit ili prva staticka nakon slijeda nestatickih
			  if (pamti>0){  //ako je bio slijed nestatickih treba izracunati teziste (za jednu na dalje nestaticke racuna teziste.......)
				  if (pamti>1){
					  WH->LB.teziste[WH->LB.broj_tezista].x=teziste_x/(pamti);  //sprema teziste u WH da se poslije mogu sva izlogirati
				  WH->LB.teziste[WH->LB.broj_tezista].y=teziste_y/(pamti);
				  WH->LB.broj_tezista++; //ova varijabla pamti koliko ima tezista po ciklusu
				  cg[numnewcg].x=teziste_x/(pamti);   //za GM ga zanima samo teziste iz trenutnog i proslog ciklusa
				  cg[numnewcg].y=teziste_y/(pamti);
				  moving_index[numnewcg]=numnewmovings;//ovo polje sadrzi indekse za polje moving tako da se zna odrediti koji skup hitova zapisan u polje moving je oznacavao koje teziste. numnewmovings je koliko ih ima sveukupno pokretnih hitova, a i-to teziste se racuna za hitove od indeksa moving_index[i-1] do indeksa strogo manjeg od moving_index[i]. Prvom izracunatom tezistu (zapravo nultom) odgovaraju clanovi polja moving od 0-tog indeksa do indeksa strogo manjeg od moving_index[0], a zadnjem izracunatom tezistu odgovaraju clanovi polja moving od indeksa moving_index[numnewcg-1] do indeksa strogo manjeg od moving_index[numnewcg]. Broj clanova polja moving_index je jednak broju clanova polja tezista
				  moving_cell_index[numnewcg]=numnewmovingcells;  //ko gore samo za indekse
				  numnewcg++;
				  }
				  if (pamti==1){
					  jedna_tocka=true;
						van_tocka=prva_tocka;
						van_tocka_real=prva_tocka_real;
//					  numnewmovings--;
				  }
			  }
			  teziste_x=0.;teziste_y=0.; slijed_nestatickih=0;//resetiranje slijeda nestatickih
			  pamti=0; slijed_statickih=1;  //oznacava da je prva staticka zabiljezena i nece vise ici u ovaj uvjet dok ne dodje nestaticka pa opet prva staticka
		  }
		  if ((staticka==0)&&(slijed_nestatickih==1)) { //druga na dalje nestaticka u nizu
			  delta_th=fabs(WH->LB.LB_scan[i].th-WH->LB.LB_scan[i-1].th);
			  delta_r=fabs(WH->LB.LB_scan[i].r-WH->LB.LB_scan[i-1].r);
			  if (delta_th>M_PI){   //ovo je za svaki slucaj ako kutevi divljaju, al trebali bi svi kutevi biti u intervalu [-M_PI/2, M_PI/2]
				  delta_th-=2*M_PI;
				  delta_th=fabs(delta_th);
			  } //ako je delta_th takav da izmedju dva susjedna hita fali jedna zraka (rupa, odnosno postoji laser zraka ali veca od onog postavljenog makismalnog dometa)
// 			  printf("druga tocka (%f,%f): delta_th=%f, delta_r=%f\n",mapper_point_temp.x,mapper_point_temp.y,delta_th,delta_r);
			  if ((delta_th>1.5*angleres)||(delta_r>CELL_DIM)){  //medo ili ako su rangeovi od dva susjedna hita razliciti za vise od 3 velicine celije ili kutovi razliciti za 1.5 debljine zrake (RES je od player-stagea koji ima duplo gusce zrake od arije)
// 				  delta_r=MAX(fabs((WH->LB.LB_pose[i]).getX()-(WH->LB.LB_pose[i-1]).getX()),fabs((WH->LB.LB_pose[i]).getY()-(WH->LB.LB_pose[i-1]).getY()));
// 			  if ((delta_r>3*CELL_DIM)){  //ili ako su rangeovi od dva susjedna hita razliciti za vise od 3 velicine celije
				  if (pamti>0){
					  if (pamti>1){
				  	WH->LB.teziste[WH->LB.broj_tezista].x=teziste_x/(pamti);  //onda zaokruzi dosadasnji skup tocaka u jedno teziste
				  	WH->LB.teziste[WH->LB.broj_tezista].y=teziste_y/(pamti);
				  	WH->LB.broj_tezista++;
				  	cg[numnewcg].x=teziste_x/(pamti);
					cg[numnewcg].y=teziste_y/(pamti);
					moving_index[numnewcg]=numnewmovings;
					moving_cell_index[numnewcg]=numnewmovingcells;
				  	numnewcg++;
					  }
					if (pamti==1){
						van_tocka=prva_tocka;
						van_tocka_real=prva_tocka_real;
						jedna_tocka=true;
// 						printf("prva tocka ide van\n");
//						numnewmovings--;
					}
				  }
				  teziste_x=0.;teziste_y=0.;
				  pamti=0;
				  prva_tocka=cell_point_temp;//zapamti prvu tocku ako je jedina da ju se obrise
// 				  printf("prva nestaticka tocka (%d,%d)\n",prva_tocka.x,prva_tocka.y);
			  }
			  teziste_x+=mapper_point_temp.x; //to je WH->LB.LB_pose[i].x;
			  teziste_y+=mapper_point_temp.y;//to je WH->LB.LB_pose[i].y;
			  pamti++; //kolko ih ima u slijedu nestatickih
			  moving[numnewmovings].x=mapper_point_temp.x;//ova prepreka se ubraja u pokretnu
			  moving[numnewmovings].y=mapper_point_temp.y;
			  numnewmovings++;
		  }
		  if ((staticka==0)&&(slijed_nestatickih==0)){            //prva nestaticka kao prvi hit ili prva nestaticka nakon slijeda statickih
			  slijed_statickih=0;//resetiranje slijeda statickih
			  teziste_x+=mapper_point_temp.x; //to je WH->LB.LB_pose[i].x;  //uzme ju
			  teziste_y+=mapper_point_temp.y;//to je WH->LB.LB_pose[i].y;
			  pamti++; //kolko ih ima u slijedu nestatickih
			  slijed_nestatickih=1;  //oznacava da je prva nestaticka zabiljezena i nece vise ici u ovaj uvjet dok ne dodje staticka pa opet prva nestaticka
			  moving[numnewmovings].x=mapper_point_temp.x;//ova prepreka se ubraja u pokretnu
			  moving[numnewmovings].y=mapper_point_temp.y;
			  numnewmovings++;  //tu kad ju stavim onda su guste tocke (sve iz lasera, a tu dolje je jedna tocka po celiji)
			  prva_tocka=cell_point_temp;//zapamti prvu tocku ako je jedina da ju se obrise
// 			  printf("prva nestaticka tocka (%d,%d)\n",prva_tocka.x,prva_tocka.y);
		  }
//--------------------------------------------sad opet opci dio za ubacivanje prepreke u mapu----------------------------------------------------------
		  //tocke lasera su puno gusce od grida tako da jednu celiju pogodimo vise puta - gledamo samo prvi pogodak celije u jednom ciklusu
// 		  if (((Map[cell_point_temp.x][cell_point_temp.y].time_stamp != counter)&&(Map[cell_point_temp.x][cell_point_temp.y].time_stamp != counter_moving)&&(Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-1)&&(Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-2)&&(Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-3))||((Map[cell_point_temp.x][cell_point_temp.y].occupancy==0)&&((Map[cell_point_temp.x][cell_point_temp.y].time_stamp == counter_moving)||(Map[cell_point_temp.x][cell_point_temp.y].time_stamp ==-1)||(Map[cell_point_temp.x][cell_point_temp.y].time_stamp ==-2)||(Map[cell_point_temp.x][cell_point_temp.y].time_stamp ==-3))))
		  if ((((Map[cell_point_temp.x][cell_point_temp.y].time_stamp != counter)&&(Map[cell_point_temp.x][cell_point_temp.y].time_stamp != counter_moving))||((Map[cell_point_temp.x][cell_point_temp.y].occupancy==0)&&(Map[cell_point_temp.x][cell_point_temp.y].time_stamp == counter_moving))) && (Map[cell_point_temp.x][cell_point_temp.y].static_cell==false))
			   //occ==0 ce znaciti da smo tu gore obrisali stari moving koji je na istoj celiji kao novi
		  {
/*			  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-1)&&(Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-2))
				  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = -3;//stavljam defaultni za slucaj kad se ne upisuju pokretne u dstar mapu*/
			  
/*			  if  (num_col_points==0){//da ne prebrise -1 i -2, odnosno samo -1 ako -2 nema
			  	Map[cell_point_temp.x][cell_point_temp.y].time_stamp = counter_moving;//oznaka da je ne obrisem kad zelim brisat stari moving
			  }*/
//znaci  time_stamp ce imati vrijednost counter_moving za punjenje samo ako je num col points==0, a za praznjenje ako su i num col points==0 i num col points old==0
//time_stamp ce biti -2 za praznjenje ako se dogodi num col points!=0 i num col points old==0 s tim da ce biti naznaka za praznjenje iako ce ovdje dobiti occ>0 ako se radi o istoj celiji, a punjenje se nece dogoditi jer ce mu ostati -2
/*			  if ((num_col_points!=0)&&(num_col_points_old==0)&&(Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-1)){
				  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = -2;//tretiramo kao brisanu pokretnu iz dstar mape
			  }*/
			  Map[cell_point_temp.x][cell_point_temp.y].occupancy++;
			  Map[cell_point_temp.x][cell_point_temp.y].x=mapper_point_temp.x;
			  Map[cell_point_temp.x][cell_point_temp.y].y=mapper_point_temp.y;
			  //provjeravam da li su blizu onih za ne punjenje (to sam napravila za stare prepreke, a moram i za nove jer novo ocitanje moze vezano za istu pokretnu prepreku)
			  if ((num_col_points) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-1) ){// &&(Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -2)
				  int blizak=0;
				  for (int q=0;q<numindeksizanepunjenje;q++){//tu popraviti, neke se proguraju slucajno, tj, mozda se ne poklapaju sve sa tim zapamcenim poljima
				  
//					  if ((indeksizanepunjenje[q].x==cell_point_temp.x)&&(indeksizanepunjenje[q].y==cell_point_temp.y)){
					  if ((abs(indeksizanepunjenje[q].x-cell_point_temp.x)<=4)&&(abs(indeksizanepunjenje[q].y-cell_point_temp.y)<=4)){//stavljam slabiji uvjet, 4 susjedna polja mogu proc
						  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -2)){
							  indeksimapepraznjenje[numindeksimapepraznjenje]=cell_point_temp;
							  numindeksimapepraznjenje++;
							  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = -2;
						  }
						  blizak=1;
						  break;
					  }
				  }
				  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp == -2)&&(blizak==0)){
					  //brisem ga iz polja za brisanje
					  for (int z=0;z<numindeksimapepraznjenje;z++){
						  if ((indeksimapepraznjenje[z].x==cell_point_temp.x)&&(indeksimapepraznjenje[z].y==cell_point_temp.y)){
							  indeksimapepraznjenje[z]=indeksimapepraznjenje[--numindeksimapepraznjenje];//brise ga iz polja
							  break;
						  }
					  }
					  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = -3;//da bude neutralan, da se moze puniti
				  }
			  }
			  //ovo sad vise ne radim jer sam u DW oznacila sve translatirane s -2 i nalaze se u polju za brisanje
/*			  if ((num_col_points) && (tezisteneudstar!=numnewcg) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-1) && (pamti)){//ako se i jedna celija od tezista iz proslog ciklusa zabiljezenog u DW kao mjesto sudara kad se translatira poklapa s celijom iz ovog,onda to ne u dstar   (Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-2) &&
				  for (int q=0;q<numindeksizanepunjenje;q++){//tu popraviti, neke se proguraju slucajno, tj, mozda se ne poklapaju sve sa tim zapamcenim poljima
				  
//					  if ((indeksizanepunjenje[q].x==cell_point_temp.x)&&(indeksizanepunjenje[q].y==cell_point_temp.y)){
					  if ((abs(indeksizanepunjenje[q].x-cell_point_temp.x)<=1)&&(abs(indeksizanepunjenje[q].y-cell_point_temp.y)<=1)){//stavljam slabiji uvjet, polje susjedno moze proc
							  tezisteneudstar=numnewcg;
					  	printf("teziste %d ne ide u dstar zbog polja (%d,%d)\n",numnewcg,cell_point_temp.x,cell_point_temp.y);
					  //zapamceno polje tog tezista prebaciti iz indeksipapepunjenje u indeksipapepraznjenje
						if (initTez==numnewcg){//da ne bi bilo da se promijenilo teziste bas sad
							for (int p=0;p<brojpomOdTez;p++){
					  			for (int z=0;z<numindeksimapepunjenje;z++){
									if ((indeksimapepunjenje[z].x==pomocnoOdTezista[p].x) && (indeksimapepunjenje[z].y==pomocnoOdTezista[p].y)){//ne smije bas bit jedan od col_mov-a
							  			indeksimapepunjenje[z]=indeksimapepunjenje[--numindeksimapepunjenje];//brise ga iz polja
								//u praznjenjima nema onih kojih ima u punjenjima
										indeksimapepraznjenje[numindeksimapepraznjenje].x=pomocnoOdTezista[p].x;   //azuriram indekse za planera
										indeksimapepraznjenje[numindeksimapepraznjenje].y=pomocnoOdTezista[p].y;
										numindeksimapepraznjenje++;
										printf("GM> brise od tezista %d : (%d,%d)\n",numnewcg,pomocnoOdTezista[p].x,pomocnoOdTezista[p].y);
										break;
						  			}
					  			}
// 					  			int duplic=0;
// 					  			for (int z=0;z<numindeksimapepraznjenje;z++){
// 						  			if ((indeksimapepraznjenje[z].x==pomocnoOdTezista[p].x)&&(indeksimapepraznjenje[z].y==pomocnoOdTezista[p].y)){
// 							  			duplic=1;
// 							  			break;
// 						  			}
// 					  			}
// 					  			if (!duplic){
// 						  			indeksimapepraznjenje[numindeksimapepraznjenje].x=pomocnoOdTezista[p].x;   //azuriram indekse za planera
// 						  			indeksimapepraznjenje[numindeksimapepraznjenje].y=pomocnoOdTezista[p].y;
// 						  			numindeksimapepraznjenje++;
// 									printf("GM> brise od tezista %d : (%d,%d)\n",numnewcg,pomocnoOdTezista[p].x,pomocnoOdTezista[p].y);
// 					  			}
					  		}
						}
					  	break;
				  	}
			  	}
			  	if (tezisteneudstar!=numnewcg){//ako nije nasao polje u indeksima za nepunjenje onda spremaj polja u pomocnoOdTezista
				  	if (initTez!=numnewcg){
					  	brojpomOdTez=0;//ako se promijenilo teziste onda puni pomocno polje ispocetka
					  	initTez=numnewcg;//i namjesti varijablu koja prati koje je teziste u tijeku
				  	}
				  	pomocnoOdTezista[brojpomOdTez].x=cell_point_temp.x;
				  	pomocnoOdTezista[brojpomOdTez].y=cell_point_temp.y;
				  	brojpomOdTez++;
					printf("GM> pomocno polje od tezista %d : (%d,%d)\n",numnewcg,cell_point_temp.x,cell_point_temp.y);
			  	}
			  }*/
/*			  if ((numnewcg==1)&&(tezisteneudstar!=numnewcg)&&(brojpomOdTez>3)){
				  printf("debug");
			  }*/
			  //po ovome se -2 brise iz praznjenja, a ne zelim to, samo u slucaju num_col_points==0
// 			  if ((tezisteneudstar!=numnewcg)&&((Map[cell_point_temp.x][cell_point_temp.y].time_stamp ==-2) || (Map[cell_point_temp.x][cell_point_temp.y].time_stamp ==counter_moving))){
			  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp ==counter_moving)||((Map[cell_point_temp.x][cell_point_temp.y].time_stamp ==-2)&&(num_col_points==0))){//ako je ==counter_moving znaci da mu je occ==0 pa ga moramo maknuti iz praznjenja
				  for (int z=0;z<numindeksimapepraznjenje;z++){
					  if ((indeksimapepraznjenje[z].x==cell_point_temp.x)&&(indeksimapepraznjenje[z].y==cell_point_temp.y)){
						  indeksimapepraznjenje[z]=indeksimapepraznjenje[--numindeksimapepraznjenje];//brise ga iz polja
						  break;
					  }
				  }
			  }
/*			  if ((num_col_points==0)||(pamti==0)||(tezisteneudstar!=numnewcg)) //nece ga stavljati u polje za punjenje ako postoji col point i pokretna je i radi se o tezistu koje ne smije ic u dstar i nije -1 ||(Map[cell_point_temp.x][cell_point_temp.y].time_stamp ==-1)
			  {
				  int duplic=0;
			  for (int z=0;z<numindeksimapepraznjenje;z++){
				  if ((indeksimapepraznjenje[z].x==cell_point_temp.x)&&(indeksimapepraznjenje[z].y==cell_point_temp.y)){
					  indeksimapepraznjenje[z]=indeksimapepraznjenje[--numindeksimapepraznjenje];//brise ga iz polja
					  break;
				  }
			  }
			  for (int z=0;z<numindeksimapepunjenje;z++){
				  if ((indeksimapepunjenje[z].x==cell_point_temp.x)&&(indeksimapepunjenje[z].y==cell_point_temp.y)){
					  duplic=1;
					  break;
				  }
			  }
			  if (!duplic){
				  indeksimapepunjenje[numindeksimapepunjenje].x=cell_point_temp.x;   //azuriram indekse za planera
				  indeksimapepunjenje[numindeksimapepunjenje].y=cell_point_temp.y;
				  numindeksimapepunjenje++;
			  }
			  }*/
			  if (pamti==0){    //znaci da se ne uracunava u teziste ova celija inace bi ova varijabla bila > 0 da se radi o nestatickoj prepreci
// 				  printf("pamti==0 (%d,%d), time_stamp=%d\n",cell_point_temp.x,cell_point_temp.y,Map[cell_point_temp.x][cell_point_temp.y].time_stamp);
				  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = counter;
				  if (0){//ovo dodajem, sad se pune i staticki hitovi (narkoza)
				  indeksimapepunjenje[numindeksimapepunjenje].x=cell_point_temp.x;   //azuriram indekse za planera
			          indeksimapepunjenje[numindeksimapepunjenje].y=cell_point_temp.y;
				  numindeksimapepunjenje++;
				  }

			  }else {
				  if (pamti==1)
					  prva_tocka_real=mapper_point_temp;
				  moving_cell[numnewmovingcells].x=mapper_point_temp.x;//ova prepreka se ubraja u pokretnu
				  moving_cell[numnewmovingcells].y=mapper_point_temp.y;
				  numnewmovingcells++;
				  //da, tu se svi pune osim onih koji su vec napunjeni, a trebalo bi pripaziti na -2 da se ne puni
// 				  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-1) && (tezisteneudstar!=numnewcg)){
				  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-1) && ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-2)||((Map[cell_point_temp.x][cell_point_temp.y].time_stamp ==-2)&&(num_col_points==0)))) {
					  indeksimapepunjenje[numindeksimapepunjenje].x=cell_point_temp.x;   //azuriram indekse za planera
					  indeksimapepunjenje[numindeksimapepunjenje].y=cell_point_temp.y;
					  numindeksimapepunjenje++;
				  }
/*				  if (tezisteneudstar==numnewcg){
					  printf("ovaj nejde (%d,%d), time_stamp=%d\n",cell_point_temp.x,cell_point_temp.y,Map[cell_point_temp.x][cell_point_temp.y].time_stamp);
				  }*/
				  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = counter_moving; //sad vise ne bu bilo -1 i -2 ali nije ni potrebno jer je sve popunjeno/popraznjeno
			  }
		  }
		  //oznaka trenutno ocitane prepreke laserom - moze vec bit zabiljezena u gridmapi a i ne mora, nema provjere za to
            reading_count++;
	    if (pamti==0){//zapisivanje statickih laserskih hitova da ne trosim vrijeme u DW.cpp
		    statickiHitovi[numnewstatic]=mapper_point_temp;
/*		    if ((mapper_point_temp.x!=WH->LB.LB_pose_manji[i].x)||(mapper_point_temp.y!=WH->LB.LB_pose_manji[i].y)){
			    printf("prepisivanje ne radi: (%f,%f)->(%f,%f)",(WH->LB.LB_pose_manji[i].x),(WH->LB.LB_pose_manji[i].y),mapper_point_temp.x,mapper_point_temp.y);
		    }*/
		    numnewstatic++;
	    }
	    if (jedna_tocka){
//		    for (int z=0;z<numnewmovingcells;z++){
//			    if ((moving_cell[z].x==van_tocka_real.x)&&(moving_cell[z].y==van_tocka_real.y)){
//				    moving_cell[z]=moving_cell[--numnewmovingcells];//brise ga iz polja
//// 				    printf("brise iz polja pokretnih polja (%f,%f)\n",van_tocka_real.x,van_tocka_real.y);
//				    break;
//			    }
//		    }
		//tu ga brise iz polja za punjenje u D* mapu
		    for (int z=0;z<numindeksimapepunjenje;z++){
			    if ((indeksimapepunjenje[z].x==van_tocka.x)&&(indeksimapepunjenje[z].y==van_tocka.y)){
				    indeksimapepunjenje[z]=indeksimapepunjenje[--numindeksimapepunjenje];//brise ga iz polja
// 				    printf("brise jednu tocku (%d,%d)\n",van_tocka.x,van_tocka.y);
				    break;
			    }
		    }
	    }
      }else{   //if  tu ga zavrsavam
//           printf("GridMap> Laser reading out of map range\n");
      }
  }//od for. sad su popunjena sva nova ocitanja ili stara
  if ((staticka==0)&&(pamti>0)){     //sam nek je nestaticka, a sto je bilo prije nije bitno. moze se racunat teziste za 1 hit i vise
	  if (pamti>1){
	  WH->LB.teziste[WH->LB.broj_tezista].x=teziste_x/(pamti);     //za zadnji odsjecak
	WH->LB.teziste[WH->LB.broj_tezista].y=teziste_y/(pamti);
	WH->LB.broj_tezista++;
	cg[numnewcg].x=teziste_x/(pamti);
	cg[numnewcg].y=teziste_y/(pamti);
	moving_index[numnewcg]=numnewmovings;
	moving_cell_index[numnewcg]=numnewmovingcells;
	numnewcg++;
	  }
	if (pamti==1){
		jedna_tocka=true;
//		for (int z=0;z<numnewmovingcells;z++){
//			if ((moving_cell[z].x==prva_tocka_real.x)&&(moving_cell[z].y==prva_tocka_real.y)){
//				moving_cell[z]=moving_cell[--numnewmovingcells];//brise ga iz polja
//// 				printf("zadnja:brise iz polja pokretnih polja (%f,%f)\n",prva_tocka_real.x,prva_tocka_real.y);
//				break;
//			}
//		}
//		numnewmovings--;
	//tu ga brise iz polja za punjenje u D* mapu
		for (int z=0;z<numindeksimapepunjenje;z++){
			if ((indeksimapepunjenje[z].x==prva_tocka.x)&&(indeksimapepunjenje[z].y==prva_tocka.y)){
				indeksimapepunjenje[z]=indeksimapepunjenje[--numindeksimapepunjenje];//brise ga iz polja
// 				printf("zadnja:brise jednu tocku (%d,%d)\n",prva_tocka.x,prva_tocka.y);
				break;
			}
		}
	}
  }
  //if (numnewcg>0) printf("\nGM: broj tezista=%d\n",numnewcg);
  if(reading_count==0){
//     printf("GridMap> Nema laserovih readinga unutar mape! %d\n",WH->LB.laser_pointer_new);
    }
/*    if ((numnewmovingcells)&&(numoldmovings==0)){//za debugiranje
	    printf("GM:tu ga cekam\n");
    }*/
    
    //for laser 2
  broj=0;
  if (WH->LB2.laser_pointer_new) {
  	angleborder=fabs(WH->LB2.sviscanovi[0].th);//medo prava kutna granica od svih ocitanja lasera
  	angleres=fabs(WH->LB2.sviscanovi[WH->LB2.laser_pointer_new-1].th-WH->LB2.sviscanovi[WH->LB2.laser_pointer_new-2].th);
  	for (int i=0;i<numoldmovings;i++) {//cuvati polje starih pokretnih i brisati ih ako su unutar konusa vidljivosti od 3 m ispred novih ocitanja
	  mapper_point_temp.x=stare_moving[i].x;
	  mapper_point_temp.y=stare_moving[i].y;
	  double angle_old_moving=atan2((stare_moving[i].y-laser2y),(stare_moving[i].x-laser2x));
	  double distance_old_moving=(stare_moving[i].y-laser2y)*(stare_moving[i].y-laser2y)+(stare_moving[i].x-laser2x)*(stare_moving[i].x-laser2x);
//	  double local_th1=angle_old_moving-WH->RB.th-WH->laser2th;
	  double local_th1=angle_old_moving-WH->laser2th;
	  int zadrzi=0;
	  if (local_th1<-M_PI) local_th1+=2*M_PI;
	  if (local_th1>M_PI) local_th1-=2*M_PI;
	  //ovo s -kutnagranica radi samo ako je laser centriran
	  if (check_point(mapper_point_temp)&&(local_th1<angleborder)&&(local_th1>-1*angleborder)&&(distance_old_moving<(LASER_RANGE_MAX-2.*Map_Cell_Size)*(LASER_RANGE_MAX-2.*Map_Cell_Size))) {//da ne bi po rubovima pisao brisao
		  //gledamo da li su stare prepreke blizu onih indeksizanepunjenje pa da ih izbrisemo ali samo iz D* mape. ovdje ce ostati
		  if ((num_col_points) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-1) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -2)){
			  for (int q=0;q<numindeksizanepunjenje;q++){//tu popraviti, neke se proguraju slucajno, tj, mozda se ne poklapaju sve sa tim zapamcenim poljima
				  if ((abs(indeksizanepunjenje[q].x-cell_point_temp.x)<=4)&&(abs(indeksizanepunjenje[q].y-cell_point_temp.y)<=4)){//stavljam slabiji uvjet, 4 susjedna polja mogu proc
					  indeksimapepraznjenje[numindeksimapepraznjenje]=cell_point_temp;
					  numindeksimapepraznjenje++;
					  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = -2;
					  break;
				  }
			  }
		  }
		  //jos treba uvjeta da ne prebrise prepreku koja je iza zida 
		  ii=int((local_th1+angleborder)/angleres);//medo popravio di je u polju svih laserskih ocitanja kojih ima 181
			  if (distance_old_moving<(WH->LB2.sviscanovi[ii].r-Map_Cell_Size)*(WH->LB2.sviscanovi[ii].r-Map_Cell_Size)){//radim pomake za jedno polje nesigurnosti
				  //oni s -2 su vec u polju za brisanje, a oni s -1 ne smiju ic u polje za brisanje i one s counter_moving su isto vec u polju za brisanje
				  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -2) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -1) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp != counter_moving)){
// 					  printf("ovu cu brisati iz DS\n");
					  indeksimapepraznjenje[numindeksimapepraznjenje].x=cell_point_temp.x;   //azuriram indekse za planera
					  indeksimapepraznjenje[numindeksimapepraznjenje].y=cell_point_temp.y;
					  numindeksimapepraznjenje++;
					  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = counter_moving; //oznaka ciklusa u kojem se brise kao stara pokretna (uociti razliku od -2)
				  }
		  	Map[cell_point_temp.x][cell_point_temp.y].occupancy=0;//odmah isprazni staru pokretnu prepreku (ako nije ponovo upisana, naravno)-tu izmijeniti!
			  }else{
				  zadrzi=1;
			  }
	  }else{
		  zadrzi=1;
	  }
	  if (zadrzi){
		  stare_moving[broj].x=stare_moving[i].x;//i>=broj pa prepisujemo one koje smo vec prosli
		  stare_moving[broj].y=stare_moving[i].y;
		  broj++;
	  }
  }
  numoldmovings=broj;//stare celije

if (WH->LB.laser_pointer_new==0) //ako nema ocitanja od prvog lasera
{
  for(int i=0;i<numnewmovingcells;i++) { //stare pokretne prepreke (hitovi iz proslog ciklusa), ako ih naravno ima
	  mapper_point_temp.x=moving_cell[i].x;    //gledaju se samo celije, a ne sva laserska mjerenja
	  mapper_point_temp.y=moving_cell[i].y;
	  double angle_moving=atan2((mapper_point_temp.y-laser2y),(mapper_point_temp.x-laser2x));
//	  double local_th=angle_moving-WH->RB.th-WH->laser2th;
	  double local_th=angle_moving-WH->laser2th;
	  double distance_moving=(moving_cell[i].y-laser2y)*(moving_cell[i].y-laser2y)+(moving_cell[i].x-laser2x)* (moving_cell[i].x-laser2x);
	  int prepisati=0;
	  if (local_th<-M_PI) local_th+=2*M_PI;
	  if (local_th>M_PI) local_th-=2*M_PI;
	  if(check_point(mapper_point_temp)&&(local_th<angleborder)&&(local_th>-1*angleborder)&&(distance_moving<(LASER_RANGE_MAX-2.*Map_Cell_Size)*(LASER_RANGE_MAX-2.*Map_Cell_Size)))
	  {
		  if ((num_col_points) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-1) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -2)){
			  for (int q=0;q<numindeksizanepunjenje;q++){//tu popraviti, neke se proguraju slucajno, tj, mozda se ne poklapaju sve sa tim zapamcenim poljima
				  if ((abs(indeksizanepunjenje[q].x-cell_point_temp.x)<=4)&&(abs(indeksizanepunjenje[q].y-cell_point_temp.y)<=4)){//stavljam slabiji uvjet, 4 susjedna polja mogu proc
					  indeksimapepraznjenje[numindeksimapepraznjenje]=cell_point_temp;
					  numindeksimapepraznjenje++;
					  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = -2;
					  break;
				  }
			  }
		  }
			  //jos treba uvjeta da ne prebrise iza zida prepreku
		   ii=int((local_th+angleborder)/angleres);//di je u polju svih laserskih ocitanja kojih ima 181
			  if (distance_moving<(WH->LB2.sviscanovi[ii].r-Map_Cell_Size)*(WH->LB2.sviscanovi[ii].r-Map_Cell_Size)){
				  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -2) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -1) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp != counter_moving)){
// 					  printf("ovu cu brisati iz DS\n");
					  indeksimapepraznjenje[numindeksimapepraznjenje].x=cell_point_temp.x;   //azuriram indekse za planera
					  indeksimapepraznjenje[numindeksimapepraznjenje].y=cell_point_temp.y;
					  numindeksimapepraznjenje++;
					  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = counter_moving; //oznaka ciklusa u kojem se brise
			}
			  Map[cell_point_temp.x][cell_point_temp.y].occupancy=0;//odmah isprazni staru pokretnu prepreku (a poslije ce se ponovo upisati)
			  }else{
				  prepisati=1;//oznaka za prepisati u stare_moving
			  }
	  } //check_point
	  else{
		  prepisati=1;
	  }
	  if (prepisati){
		  //tu moramo gledati duplice jer se u svakom ciklusu puno istih zabiljezava kao moving_cell
		  int duplic=0;//gledamo do broj (to su one od proslih ciklusa a ne ovog, odredjeno iznad u for petlji)
		  int x,y;
		  for (int z=0;z<broj;z++){
			  x=(int)floor((stare_moving[z].x-Map_Home.x)/Map_Cell_Size);
			  y=(int)floor((stare_moving[z].y-Map_Home.y)/Map_Cell_Size);
			  if ((x==cell_point_temp.x)&&(y==cell_point_temp.y)){
				  duplic=1;
				  break;
			  }
		  }
		  if (!duplic){
			  if ((numoldmovings>0)&&(numoldmovings%(PLAYER_LASER_MAX_SAMPLES)==0)){
			  if(((stare_moving = (R_point *)realloc( stare_moving, (numoldmovings+PLAYER_LASER_MAX_SAMPLES)*sizeof(R_point)))!=NULL)){
			  
			printf("GM::moving REALLOC succeeded\n");  
			  }else{
			printf("GM::moving Error allocating memory REALLOC!!!\n");
			return 0;
		}
				  
			  }
			  //else
			  {
				  pocetak=0;
				  stare_moving[numoldmovings].x=mapper_point_temp.x;
				  stare_moving[numoldmovings].y=mapper_point_temp.y;
				  numoldmovings++;
			  }
		  }
 	  }
}	
//numoldmovings=numnewmovings;
	numnewmovings=0; //resetiranje novih pokretnih prepreka
	numnewmovingcells=0;
	for(int i=0;i<numnewcg;i++) {//stara tezista (iz proslog ciklusa)
		stare_cg[i].x=cg[i].x;
		stare_cg[i].y=cg[i].y;
		numoldcg=numnewcg;
	}
	numnewcg=0;  //resetiranje novih tezista




}//ako nema ocitanja prvog lasera


}//zatvoren uvjet da li ima uopce ocitanja lasera u ovom ciklusu

//nastavak za mali laser2
	slijed_statickih=0; 
	slijed_nestatickih=0;
	pamti=0;
	teziste_x=0.;teziste_y=0.;//inicijalizacija suma x i y koordinata
	for(int i=0;i<WH->LB2.laser_manji_new;i++){
      mapper_point_temp.x=(WH->LB2.LB_pose_manji[i].x);
      mapper_point_temp.y=(WH->LB2.LB_pose_manji[i].y);
      if(check_point(mapper_point_temp))//&&(distance<LASER_RANGE_MAX*LASER_RANGE_MAX))
      {
	      jedna_tocka=false;//radit ce se o samo jednoj tocki pa nju ne upisujem (pamti==1)
		  staticka=0;//varijabla koja oznacava da li ima staticke prepreke u okolini novog hita lasera
		  for(int k=-around_static;k<=around_static;k++){
			  for(int l=-around_static;l<=around_static;l++){
				  if (((cell_point_temp.x+k)>=0)&&((cell_point_temp.x+k)<Map_Dim_X)&&((cell_point_temp.y+l)>=0)&&((cell_point_temp.y+l)<Map_Dim_Y)){
				  if (Map[cell_point_temp.x+k][cell_point_temp.y+l].static_cell) {
						staticka=1;
						break;//cim nadje brejka
					}
				  }
			  }
				if (staticka==1)      //izlazi iz obje for petlje i ne zeli racunati teziste
					break;
			}
			if ((staticka==1)&&(slijed_statickih==0)) {     //to je prva staticka kao prvi hit ili prva staticka nakon slijeda nestatickih
			  if (pamti>0){  //ako je bio slijed nestatickih treba izracunati teziste (za jednu na dalje nestaticke racuna teziste.......)
				  if (pamti>1){
					  WH->LB.teziste[WH->LB.broj_tezista].x=teziste_x/(pamti);  //sprema teziste u WH da se poslije mogu sva izlogirati
				  WH->LB.teziste[WH->LB.broj_tezista].y=teziste_y/(pamti);
				  WH->LB.broj_tezista++; //ova varijabla pamti koliko ima tezista po ciklusu
				  cg[numnewcg].x=teziste_x/(pamti);   //za GM ga zanima samo teziste iz trenutnog i proslog ciklusa
				  cg[numnewcg].y=teziste_y/(pamti);
				  moving_index[numnewcg]=numnewmovings;
				  moving_cell_index[numnewcg]=numnewmovingcells;  //ko gore samo za indekse
				  numnewcg++;
				  }
				  if (pamti==1){
					  jedna_tocka=true;
						van_tocka=prva_tocka;
						van_tocka_real=prva_tocka_real;
//					  numnewmovings--;
				  }
			  }
			  teziste_x=0.;teziste_y=0.; slijed_nestatickih=0;//resetiranje slijeda nestatickih
			  pamti=0; slijed_statickih=1;  //oznacava da je prva staticka zabiljezena i nece vise ici u ovaj uvjet dok ne dodje nestaticka pa opet prva staticka
		  }
		  if ((staticka==0)&&(slijed_nestatickih==1)) { //druga na dalje nestaticka u nizu
			  delta_th=fabs(WH->LB2.LB_scan[i].th-WH->LB2.LB_scan[i-1].th);
			  delta_r=fabs(WH->LB2.LB_scan[i].r-WH->LB2.LB_scan[i-1].r);
			  if (delta_th>M_PI){   //ovo je za svaki slucaj ako kutevi divljaju, al trebali bi svi kutevi biti u intervalu [-M_PI/2, M_PI/2] ajaoooo
				  delta_th-=2*M_PI;
				  delta_th=fabs(delta_th);
			  }//nemam provjeru za drugu stranu
			  if ((delta_th>1.5*angleres)||(delta_r>CELL_DIM)){ 
				  if (pamti>0){
					  if (pamti>=1){
				  	WH->LB.teziste[WH->LB.broj_tezista].x=teziste_x/(pamti);  //onda zaokruzi dosadasnji skup tocaka u jedno teziste
				  	WH->LB.teziste[WH->LB.broj_tezista].y=teziste_y/(pamti);
				  	WH->LB.broj_tezista++;
				  	cg[numnewcg].x=teziste_x/(pamti);
					cg[numnewcg].y=teziste_y/(pamti);
					moving_index[numnewcg]=numnewmovings;
					moving_cell_index[numnewcg]=numnewmovingcells;
				  	numnewcg++;
					  }
					if (pamti==1){
						van_tocka=prva_tocka;
						van_tocka_real=prva_tocka_real;
						jedna_tocka=true;
// 						printf("prva tocka ide van\n");
//						numnewmovings--;
					}
				  }
				  teziste_x=0.;teziste_y=0.;
				  pamti=0;
				  prva_tocka=cell_point_temp;//zapamti prvu tocku ako je jedina da ju se obrise
// 				  printf("prva nestaticka tocka (%d,%d)\n",prva_tocka.x,prva_tocka.y);
			  }
			  teziste_x+=mapper_point_temp.x; //to je WH->LB.LB_pose[i].x;
			  teziste_y+=mapper_point_temp.y;//to je WH->LB.LB_pose[i].y;
			  pamti++; //kolko ih ima u slijedu nestatickih
			  moving[numnewmovings].x=mapper_point_temp.x;//ova prepreka se ubraja u pokretnu
			  moving[numnewmovings].y=mapper_point_temp.y;
			  numnewmovings++;
		  }
		  if ((staticka==0)&&(slijed_nestatickih==0)){            //prva nestaticka kao prvi hit ili prva nestaticka nakon slijeda statickih
			  slijed_statickih=0;//resetiranje slijeda statickih
			  teziste_x+=mapper_point_temp.x; //to je WH->LB.LB_pose[i].x;  //uzme ju
			  teziste_y+=mapper_point_temp.y;//to je WH->LB.LB_pose[i].y;
			  pamti++; //kolko ih ima u slijedu nestatickih
			  slijed_nestatickih=1;  //oznacava da je prva nestaticka zabiljezena i nece vise ici u ovaj uvjet dok ne dodje staticka pa opet prva nestaticka
			  moving[numnewmovings].x=mapper_point_temp.x;//ova prepreka se ubraja u pokretnu
			  moving[numnewmovings].y=mapper_point_temp.y;
			  numnewmovings++;  //tu kad ju stavim onda su guste tocke (sve iz lasera, a tu dolje je jedna tocka po celiji)
			  prva_tocka=cell_point_temp;//zapamti prvu tocku ako je jedina da ju se obrise
// 			  printf("prva nestaticka tocka (%d,%d)\n",prva_tocka.x,prva_tocka.y);
		  }
		  if ((((Map[cell_point_temp.x][cell_point_temp.y].time_stamp != counter)&&(Map[cell_point_temp.x][cell_point_temp.y].time_stamp != counter_moving))||((Map[cell_point_temp.x][cell_point_temp.y].occupancy==0)&&(Map[cell_point_temp.x][cell_point_temp.y].time_stamp == counter_moving))) && (Map[cell_point_temp.x][cell_point_temp.y].static_cell==false))
		  {
			  Map[cell_point_temp.x][cell_point_temp.y].occupancy++;
			  Map[cell_point_temp.x][cell_point_temp.y].x=mapper_point_temp.x;
			  Map[cell_point_temp.x][cell_point_temp.y].y=mapper_point_temp.y;
			  if ((num_col_points) && (Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-1) ){// &&(Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -2)
				  int blizak=0;
				  for (int q=0;q<numindeksizanepunjenje;q++){//tu popraviti, neke se proguraju slucajno, tj, mozda se ne poklapaju sve sa tim zapamcenim poljima
					  if ((abs(indeksizanepunjenje[q].x-cell_point_temp.x)<=4)&&(abs(indeksizanepunjenje[q].y-cell_point_temp.y)<=4)){//stavljam slabiji uvjet, 4 susjedna polja mogu proc
						  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp != -2)){
							  indeksimapepraznjenje[numindeksimapepraznjenje]=cell_point_temp;
							  numindeksimapepraznjenje++;
							  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = -2;
						  }
						  blizak=1;
						  break;
					  }
				  }
				  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp == -2)&&(blizak==0)){
					  //brisem ga iz polja za brisanje
					  for (int z=0;z<numindeksimapepraznjenje;z++){
						  if ((indeksimapepraznjenje[z].x==cell_point_temp.x)&&(indeksimapepraznjenje[z].y==cell_point_temp.y)){
							  indeksimapepraznjenje[z]=indeksimapepraznjenje[--numindeksimapepraznjenje];//brise ga iz polja
							  break;
						  }
					  }
					  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = -3;//da bude neutralan, da se moze puniti
				  }
			  }

			  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp ==counter_moving)||((Map[cell_point_temp.x][cell_point_temp.y].time_stamp ==-2)&&(num_col_points==0))){//ako je ==counter_moving znaci da mu je occ==0 pa ga moramo maknuti iz praznjenja
				  for (int z=0;z<numindeksimapepraznjenje;z++){
					  if ((indeksimapepraznjenje[z].x==cell_point_temp.x)&&(indeksimapepraznjenje[z].y==cell_point_temp.y)){
						  indeksimapepraznjenje[z]=indeksimapepraznjenje[--numindeksimapepraznjenje];//brise ga iz polja
						  break;
					  }
				  }
			  }

			  if (pamti==0){    
				  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = counter;
				  if (0){//ovo dodajem, sad se pune i staticki hitovi (narkoza)
				  indeksimapepunjenje[numindeksimapepunjenje].x=cell_point_temp.x; 
			          indeksimapepunjenje[numindeksimapepunjenje].y=cell_point_temp.y;
				  numindeksimapepunjenje++;
				  }

			  }else {
				  if (pamti==1)
					  prva_tocka_real=mapper_point_temp;
				  moving_cell[numnewmovingcells].x=mapper_point_temp.x;//ova prepreka se ubraja u pokretnu
				  moving_cell[numnewmovingcells].y=mapper_point_temp.y;
				  numnewmovingcells++;

				  if ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-1) && ((Map[cell_point_temp.x][cell_point_temp.y].time_stamp !=-2)||((Map[cell_point_temp.x][cell_point_temp.y].time_stamp ==-2)&&(num_col_points==0)))) {
					  indeksimapepunjenje[numindeksimapepunjenje].x=cell_point_temp.x;   //azuriram indekse za planera
					  indeksimapepunjenje[numindeksimapepunjenje].y=cell_point_temp.y;
					  numindeksimapepunjenje++;
				  }

				  Map[cell_point_temp.x][cell_point_temp.y].time_stamp = counter_moving; //sad vise ne bu bilo -1 i -2 ali nije ni potrebno jer je sve popunjeno/popraznjeno
			  }
		  }
		  //oznaka trenutno ocitane prepreke laserom - moze vec bit zabiljezena u gridmapi a i ne mora, nema provjere za to
            reading_count++;
	    if (pamti==0){//zapisivanje statickih laserskih hitova da ne trosim vrijeme u DW.cpp
		    statickiHitovi[numnewstatic]=mapper_point_temp;

		    numnewstatic++;
	    }
	    if (jedna_tocka){
//		    for (int z=0;z<numnewmovingcells;z++){
//			    if ((moving_cell[z].x==van_tocka_real.x)&&(moving_cell[z].y==van_tocka_real.y)){
//				    moving_cell[z]=moving_cell[--numnewmovingcells];//brise ga iz polja
//// 				    printf("brise iz polja pokretnih polja (%f,%f)\n",van_tocka_real.x,van_tocka_real.y);
//				    break;
//			    }
//		    }
		//tu ga brise iz polja za punjenje u D* mapu
		    for (int z=0;z<numindeksimapepunjenje;z++){
			    if ((indeksimapepunjenje[z].x==van_tocka.x)&&(indeksimapepunjenje[z].y==van_tocka.y)){
				    indeksimapepunjenje[z]=indeksimapepunjenje[--numindeksimapepunjenje];//brise ga iz polja
// 				    printf("brise jednu tocku (%d,%d)\n",van_tocka.x,van_tocka.y);
				    break;
			    }
		    }
	    }
      }else{   //if  tu ga zavrsavam
//           printf("GridMap> Laser reading out of map range\n");
      }
  }//od for. sad su popunjena sva nova ocitanja ili stara
  if ((staticka==0)&&(pamti>0)){     //sam nek je nestaticka, a sto je bilo prije nije bitno. moze se racunat teziste za 1 hit i vise
	  if (pamti>1){
	  WH->LB.teziste[WH->LB.broj_tezista].x=teziste_x/(pamti);     //za zadnji odsjecak
	WH->LB.teziste[WH->LB.broj_tezista].y=teziste_y/(pamti);
	WH->LB.broj_tezista++;
	cg[numnewcg].x=teziste_x/(pamti);
	cg[numnewcg].y=teziste_y/(pamti);
	moving_index[numnewcg]=numnewmovings;
	moving_cell_index[numnewcg]=numnewmovingcells;
	numnewcg++;
	  }
	if (pamti==1){
		jedna_tocka=true;
//		for (int z=0;z<numnewmovingcells;z++){
//			if ((moving_cell[z].x==prva_tocka_real.x)&&(moving_cell[z].y==prva_tocka_real.y)){
//				moving_cell[z]=moving_cell[--numnewmovingcells];//brise ga iz polja
//// 				printf("zadnja:brise iz polja pokretnih polja (%f,%f)\n",prva_tocka_real.x,prva_tocka_real.y);
//				break;
//			}
//		}
//		numnewmovings--;
	//tu ga brise iz polja za punjenje u D* mapu
		for (int z=0;z<numindeksimapepunjenje;z++){
			if ((indeksimapepunjenje[z].x==prva_tocka.x)&&(indeksimapepunjenje[z].y==prva_tocka.y)){
				indeksimapepunjenje[z]=indeksimapepunjenje[--numindeksimapepunjenje];//brise ga iz polja
// 				printf("zadnja:brise jednu tocku (%d,%d)\n",prva_tocka.x,prva_tocka.y);
				break;
			}
		}
	}
  }
    
    
	printf("GM: for emptying obstacles:");
	for (int z=0;z<numindeksimapepraznjenje;z++) {
		printf("(%d,%d)",indeksimapepraznjenje[z].x,indeksimapepraznjenje[z].y);
	}
	printf("\nGM: for filling obstacles:");
	for (int z=0;z<numindeksimapepunjenje;z++){
		printf("(%d,%d)",indeksimapepunjenje[z].x,indeksimapepunjenje[z].y);
	}
	printf("\nGM: not for filling:");
	for (int z=0;z<numindeksizanepunjenje;z++){
		printf("(%d,%d)",indeksizanepunjenje[z].x,indeksizanepunjenje[z].y);
	}
	printf("\n");
   numindeksizanepunjenje=0;//resetiramo za ponovni upis u DW
 return 1;
}


void GridMap::alloc_Map()
{
  int i, j ;
   //mogli bi koristiti i novije new i delete operatore
   //umjesto malloca
  //(*map) = (Mapper_cell **)malloc(row*sizeof(Mapper_cell *)) ;
  Map = (GridMap_cell **)malloc(Map_Dim_X*sizeof(GridMap_cell *)) ;
  for (i=0; i<Map_Dim_X; i++)
  Map[i] = (GridMap_cell *)malloc(Map_Dim_Y*sizeof(GridMap_cell)) ;


  for (i=0; i<Map_Dim_X; i++){
    for (j=0; j<Map_Dim_Y; j++){

      Map[i][j].occupancy=0;
	  Map[i][j].time_stamp=0;
      Map[i][j].static_cell=false;
      }
      }

}

void GridMap::alloc_Mapfrompng(double width, double height, double resolution, double originx, double originy)
{
  int i, j ;
  int metric=1000;
  FILE *D;
  Map_Dim_X_A=(width*resolution*metric);
  Map_Dim_Y_A=(height*resolution*metric);
//  Map_Dim_X=(int)ceil(width*resolution/Map_Cell_Size*metric);
//  Map_Dim_Y=(int)ceil(height*resolution/Map_Cell_Size*metric);
  Map_Dim_X = int (floor (Map_Dim_X_A / Map_Cell_Size))+1;
  Map_Dim_Y = int (floor (Map_Dim_Y_A / Map_Cell_Size))+1;
  Map_Home.x=originx*metric;
  Map_Home.y=originy*metric;
  if (((D = fopen("logger//origin.dat","wt")) != NULL))
	{
		fprintf(D,"%f %f\n",Map_Home.x,Map_Home.y);
		fclose(D);
	}
  if (((D = fopen("logger//traj","wt")) != NULL))
	{
		fprintf(D,"0 0 0 0 0 0 0\n");
		fclose(D);
	}
  if (((D = fopen("logger//newgoals","wt")) != NULL))
	{
		fprintf(D,"0 0 0 0\n");
		fclose(D);
	}
  if (((D = fopen("logger//cell_size.dat","wt")) != NULL))
  {
	  fprintf(D,"%f\n",Map_Cell_Size);
	  fclose(D);
  }
   //mogli bi koristiti i novije new i delete operatore
   //umjesto malloca
  Map = (GridMap_cell **)malloc(Map_Dim_X*sizeof(GridMap_cell *)) ;
  for (i=0; i<Map_Dim_X; i++)
  Map[i] = (GridMap_cell *)malloc(Map_Dim_Y*sizeof(GridMap_cell)) ;


  for (i=0; i<Map_Dim_X; i++){
    for (j=0; j<Map_Dim_Y; j++){

      Map[i][j].occupancy=0;
	  Map[i][j].time_stamp=0;
      Map[i][j].static_cell=false;
      }
      }

}


void GridMap::free_Map()
{

  for (int i=0; i<Map_Dim_X; i++){
    free(Map[i]);
  }
    free(Map);
    free(indeksimapepunjenje);
    free(indeksimapepraznjenje);
    free(indeksizanepunjenje);
    free(linije);
    free(moving);
    free(moving_cell);
    free(moving_cell_index);
    free(moving_index);
    free(cg);
    free(stare_cg);
    free(stare_moving);
    free(stare_moving_pom);
    free(statickiHitovi);
    free(col_moving);
    free(col_moving_old);
}



