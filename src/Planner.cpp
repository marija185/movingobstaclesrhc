/***************************************************************************
                          Planner.cpp  -  description
                             -------------------
    begin                : Wed Jun 18 2003
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
#include "WorkHorse.h"      //Planner.h Params.h je unutra
#include "GridMap.h"    //
#include "DStar.h"    //Params.h
#include "moj.h"
#include "DynamicWindow.h"
#if RECTANGULAR
#include <cspacevoronoi.h>
#endif

extern DStar       *DS;
extern DynamicWindow       *DW;
extern GridMap       *GM;
extern WorkHorse       *WH;
extern moj *M;
#if RECTANGULAR
extern CSpaceVoronoi *cspace;
#endif

 //konstruktor klase
Planner::Planner(){
  //path stuff
	path = NULL;//int path od DS
	path_r=NULL; //pretvoren u real
	path_length = 0;
  printf("Planner::Planner()> The Planner object is created\n");
  if ((pamtiindekse= (I_point *)malloc(GM->Map_Dim_X*GM->Map_Dim_Y*sizeof(I_point)))==NULL)//13*13 za robot + cost masku 
  {
	  printf("patak je budala\n");
  }

}

void Planner::reset()
{
	struct timeval timeStart;
	struct timeval timeNow;
	int mySecStart, myMSecStart,mySecNow, myMSecNow, vremenska_razlika;

   //LOGGER VREMENA  --POCETAK RESETA ------------------------------
	if (gettimeofday(&timeStart, NULL) == 0)
	{
		mySecStart = timeStart.tv_sec;
		myMSecStart = timeStart.tv_usec / 1000;
	}

  path_length = 0;
  path_length_max=0;
  start_r.x=0.0;  start_r.y=0.0;//start_r se upisuje svaki ciklus
  counter_debug=0;
  //ciklus_col_points=-2;
  	//zadaje se globalni cilj samo jednom, neeee, i u sekvenci treba
  global_goal_r.x= WH->global_goal_workhorse.x;
  global_goal_r.y= WH->global_goal_workhorse.y;
  
  int star_size_x=DS->GetMapSizeX();
  int star_size_y=DS->GetMapSizeY();

//OVDJE SMO RUCNO ZADALI ROBOT_MASKU!!!!!!!!
//   int robot_mask=(int)floor(RR/CELL_DIM); //predpostavljamo kvadratnu masku
//   int robot_mask=(int)ceil(RR/CELL_DIM);
  int robot_mask=(int)ROBOT_MASK;
//int robot_mask=0;
//cost maska ako je definirana...
  int cost_mask=(int) COST_MASK;
  printf("Size of the robot mask=%d and cost mask=%d\n", robot_mask, cost_mask);
   //UCITAVANJE DSTAR MAPE POMOCU INICIJALNE GRID MAPE
//PAZNJA - u Planneru se resetira samo mapa za pretrazivanje, tj. pronalazenje slobodnog puta
// a ne i grid mapa prostora (to se radi u GM u reset())
	//star mapa se puni za pretrazivanje prema zauzetosti polja u GridMapi
           for (int i=0; i<star_size_x; i++){
                 for (int j=0; j<star_size_y; j++){
                  //punimo DStar mapu!!!
                  if(GM->Map[i][j].occupancy>=GRID_MAP_OCCUPANCY_TRESHOLD){
			  Puni(i, j, star_size_x, star_size_y, robot_mask, cost_mask);
		  }//GM
                }  //for
            }  //for
#if RECTANGULAR
	DS->Numindicesocc=0;
	// from a height map set DS costmap cells to occupied for which all orientations collide
           for (int i=0; i<star_size_x; i++){
                 for (int j=0; j<star_size_y; j++){
			if ((DS->map[i][j].prepreka_bool==false) && (DS->map[i][j].cspace_occupied==false) && (cspace->collidesInAllOrientations(i,j)))
//			if (cspace->collidesInAllOrientations(i,j))
			{
				DS->map[i][j].cspace_occupied=true;
				DS->indicesocc[DS->Numindicesocc].x=i;
				DS->indicesocc[DS->Numindicesocc].y=j;
				DS->Numindicesocc++;
			}
		}
	}
	printf("Planner::reset> Numindicesocc=%d\n",DS->Numindicesocc);
#endif	
	DW->locmincnt=0;//reset

 if (gettimeofday(&timeNow, NULL) == 0)
 {
	 mySecNow = timeNow.tv_sec;
	 myMSecNow = timeNow.tv_usec / 1000;
 }
 vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
 printf("Planner::reset> - time diff msec: %d\n", vremenska_razlika);
}

int Planner::Sekvenca_izvodjenja()
{
	struct timeval timeStart;
	struct timeval timeNow;
	int mySecStart, myMSecStart,mySecNow, myMSecNow, vremenska_razlika;
	counter_debug++;
// 	   //MJERI VRIJEME
 	if (gettimeofday(&timeStart, NULL) == 0)
 	{
 		mySecStart = timeStart.tv_sec;
 		myMSecStart = timeStart.tv_usec / 1000;
 	}

    //UCITAVANJE GRID MAPE ZBOG NOVIH LASERSKIH OCITANJA
    if(!(GM->fill_Map())){
      printf("Planner> The GM map couldn't be filled!");
    return 2;
    }
	//treba detektirati kratki ostanak bez  GM->num_col_points_old i brojati cikluse koliko ga nema
/*	if ((GM->num_col_points)){
		ciklus_col_points=counter_debug;
	}else{
		if ((counter_debug-ciklus_col_points)<=2) {
			DS->drziStariPut=true;
		}else{
			DS->drziStariPut=false;
		}
	}*/
	   //MJERI VRIJEME
	if (gettimeofday(&timeNow, NULL) == 0)
	{
		mySecNow = timeNow.tv_sec;
		myMSecNow = timeNow.tv_usec / 1000;
	}
	vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
	printf("PL:MainSequence> After GM->fill_Map for no. %d obstacles and no. %d empty cells last for %d ms\n", GM->numindeksimapepunjenje,GM->numindeksimapepraznjenje,vremenska_razlika);
/*	if (GM->numindeksimape>15){
		printf("PL cekam te\n");
	}*/
  int star_size_x=DS->GetMapSizeX();
  int star_size_y=DS->GetMapSizeY();

//OVDJE SMO RUCNO ZADALI ROBOT_MASKU!!!!!!!!
//  int robot_mask=(int)floor(RR/CELL_DIM); //predpostavljamo kvadratnu masku
  int robot_mask=(int)ROBOT_MASK;
  int robot_masky=(int)ROBOT_MASKY;
//   int robot_mask=(int)ceil(RR/CELL_DIM);
//cost maska ako je definirana...
  int cost_mask=(int) COST_MASK;
  //UCITAVANJE DSTAR MAPE POMOCU GRID MAPE
  //umjesto trcanja po cijelom polju idem samo po indeksima zabiljezenim u GM 
  int i,j;
  int brojpam=0;
//   I_point pamtiindekse[(2*(robot_mask+cost_mask)+1)*(2*(robot_mask+cost_mask)+1)*GM->numindeksimapepraznjenje];//13*13 za robot + cost masku 
  DS->promjena=0;//inicijalizacija
//   DS->NumElemPromjena=0;
  DS->NumElemPunjenja=0;DS->NumElemPraznjenja=0;  DS->NumElemCostmaska=0;
  
  for (int ind=0;ind<GM->numindeksimapepunjenje;ind++){
	  i=GM->indeksimapepunjenje[ind].x;
	  j=GM->indeksimapepunjenje[ind].y;
// 	  GM->novi[ind].x=i;//to mi treba za crtanje
// 	  GM->novi[ind].y=j;
    //punimo DStar mapu svim novim ocitanjima o zauzetosti
//ako nema sudara s pokretnom preprekom onda ju upisujemo u dstar kartu
	  Puni(i, j, star_size_x, star_size_y, robot_mask, cost_mask);
//	printf("cspace updateClearance (i,j)=(%d,%d) ",i,j);
//----------------------------------------------------------------
#if RECTANGULAR
	cspace->updateClearance(i,j,0);
#endif
//----------------------------------------------------------------	  
  }//indeksi za punjenje
  for (int ind=0;ind<GM->numindeksimapepraznjenje;ind++){
	  i=GM->indeksimapepraznjenje[ind].x;
	  j=GM->indeksimapepraznjenje[ind].y;
    //praznimo DStar mapu svim starim ocitanjima o zauzetosti kojih vise nema
              //maskiranje u C-obstacle...
//RADIM NA DRUGI NACIN ------------preko ._mask IntPointa. Ne trebam provjeravati kut jer je to vec u GM provjereno, samo u maski gledam koje celije imaju ._mask.x==i ._mask.y==j
	  Prazni(i, j, pamtiindekse, &brojpam, star_size_x, star_size_y, robot_mask, cost_mask, false);//ovdje se puni polje pamtiindekse i mijenja se brojpam
//----------------------------------------------------------------
#if RECTANGULAR
	cspace->updateClearance(i,j,INT_MAX);
#endif
//----------------------------------------------------------------	  
  }//for praznjenje
//----------------------------------------------------------------
#if DSTAR3DORI || TEST3DSEARCH //for test full 3d
if (DS->prviput){
  cspace->initializeDStar();
}
#endif
#if RECTANGULAR && (DSTAR3DORI==0)
	cspace->update(false);
#if 1
  	printf("PL: before checking the cspace array of size %d changes for the costmaks %d, changes to occupied %d, and changes to empty %d\n", DS->Numindicesocc, DS->NumElemCostmaska,DS->NumElemPunjenja,DS->NumElemPraznjenja);
	bool flag_coll;
	int broj=0;
	I_point temp;
#if 0	
	for (int ind=0;ind<DS->Numindicesocc;ind++){
		temp.x=DS->indicesocc[ind].x;
		temp.y=DS->indicesocc[ind].y;
		flag_coll=cspace->collidesInAllOrientations(temp.x,temp.y);
//		printf("PL: cell (%d,%d) collides yes (1) no (0): %d\n",temp.x,temp.y,flag_coll);
		if (flag_coll) {//ako je ispraznio mapu updateamo
			//samo ako je Dstar prepreka false onda punimo u polje
			if (DS->map[temp.x][temp.y].prepreka_bool==false){
			DS->indicesocc[broj].x=DS->indicesocc[ind].x;//ind>=broj pa prepisujemo one koje smo vec prosli
	  		DS->indicesocc[broj].y=DS->indicesocc[ind].y;
	  		broj++;
	  		
			if ((DS->map[temp.x][temp.y].cspace_occupied==false)){
				DS->map[temp.x][temp.y].cspace_occupied=true;
				//dodaj u polje (provjera je gore pa je ovdje viska)
				if (1 && (DS->prviput==0) && (DS->map[temp.x][temp.y].promjena!=2)) // && (DS->map[temp.x][temp.y].prepreka_bool==false))
				{
					DS->listapunjenja[DS->NumElemPunjenja]=temp;
					DS->NumElemPunjenja++;
					DS->promjena=1;
					DS->map[temp.x][temp.y].promjena=2; //ali cemo mu promijeniti oznaku s praznjenja na punjenje
				}
			}
			}
		}else{//ako je ispraznio tako da tamo vise nema prepreke brisemo iz polja
			printf("PL: obstacle colliding for all orientations deleted!\n");
			if ((DS->map[temp.x][temp.y].cspace_occupied==true)){
				DS->map[temp.x][temp.y].cspace_occupied=false;
				//dodaj u polje jer ce sad IsValid javit da je slobodno
				if (1 && (DS->prviput==0) && (DS->map[temp.x][temp.y].promjena!=1) && (DS->map[temp.x][temp.y].prepreka_bool==false))
				{
					DS->listapraznjenja[DS->NumElemPraznjenja]=temp;
					DS->NumElemPraznjenja++;
					DS->promjena=1;
					DS->map[temp.x][temp.y].promjena=1;
				}
			}
		}
	}
	printf("PL: after checking cspace array the new size is %d, changes to occupied %d, changes to empty %d\n",broj,DS->NumElemPunjenja,DS->NumElemPraznjenja);
	DS->Numindicesocc=broj;
#endif
	for (int ind=0;ind<GM->numindeksimapepunjenje;ind++){
	//checking neighbors inside the robot_masky
		i=GM->indeksimapepunjenje[ind].x;
	  	j=GM->indeksimapepunjenje[ind].y;

		for(int k=-robot_masky-1;k<=robot_masky+1;k++){
			for(int l=-robot_masky-1;l<=robot_masky+1;l++){
				temp.x=i+k; temp.y=j+l;
				if((temp.x>=0)&& (temp.x<star_size_x)&&(temp.y>=0)&& (temp.y<star_size_y)){
				if ((DS->map[temp.x][temp.y].cspace_occupied==false) && (DS->map[temp.x][temp.y].prepreka_bool==false)) {
				if (cspace->collidesInAllOrientations(temp.x,temp.y)) {
				DS->map[temp.x][temp.y].cspace_occupied=true;
				//dodaj u polje (viska pa zakomentiravam)
				if (1 && (DS->prviput==0) && (DS->map[temp.x][temp.y].promjena!=2)) // && (DS->map[temp.x][temp.y].prepreka_bool==false))
				{
					DS->listapunjenja[DS->NumElemPunjenja]=temp;
					DS->NumElemPunjenja++;
					DS->promjena=1;
					DS->map[temp.x][temp.y].promjena=2; //ali cemo mu promijeniti oznaku s praznjenja na punjenje
				}
				//nece biti duplic jer je gore provjeren
//				DS->indicesocc[DS->Numindicesocc].x=temp.x;
//				DS->indicesocc[DS->Numindicesocc].y=temp.y;
//				DS->Numindicesocc++;
				}
				}
				}
			}
		}
	}
	for (int ind=0;ind<GM->numindeksimapepraznjenje;ind++){
	//checking neighbors inside the robot_masky
		i=GM->indeksimapepraznjenje[ind].x;
	  	j=GM->indeksimapepraznjenje[ind].y;

		for(int k=-robot_masky-1;k<=robot_masky+1;k++){
			for(int l=-robot_masky-1;l<=robot_masky+1;l++){
				temp.x=i+k; temp.y=j+l;
				if((temp.x>=0)&& (temp.x<star_size_x)&&(temp.y>=0)&& (temp.y<star_size_y)){
				if ((DS->map[temp.x][temp.y].cspace_occupied==true) && (DS->map[temp.x][temp.y].prepreka_bool==false)) {
				if (!cspace->collidesInAllOrientations(temp.x,temp.y)) {
				DS->map[temp.x][temp.y].cspace_occupied=false;
				//dodaj u polje (viska pa zakomentiravam)
				if (1 && (DS->prviput==0) && (DS->map[temp.x][temp.y].promjena!=1)) // && (DS->map[temp.x][temp.y].prepreka_bool==false))
				{
					DS->listapraznjenja[DS->NumElemPraznjenja]=temp;
					DS->NumElemPraznjenja++;
					DS->promjena=1;
					DS->map[temp.x][temp.y].promjena=1;
				}
				}
				}
				}
			}
		}
	}
	printf("PL: after new obstacles for inserting the new number of all orientations colliding cells %d, changes to occupied %d, changes to empty %d\n",DS->Numindicesocc,DS->NumElemPunjenja,DS->NumElemPraznjenja);

#endif
	
#endif
//----------------------------------------------------------------	  
  PrazniCostMasku(pamtiindekse, brojpam, star_size_x, star_size_y, cost_mask);//a ovdje se pomocu tog polja azurira cost maska
/*  if ((DS->NumElemPunjenja>PLAYER_LASER_MAX_SAMPLES)||(DS->NumElemPraznjenja>PLAYER_LASER_MAX_SAMPLES)||(DS->NumElemCostmaska>PLAYER_LASER_MAX_SAMPLES)){
	  printf("punj=%d, praznj=%d, costm=%d\n",DS->NumElemPunjenja,DS->NumElemPraznjenja,DS->NumElemCostmaska);
  }*/
    //MJERI VRIJEME
  if (gettimeofday(&timeNow, NULL) == 0)
  {
	  mySecNow = timeNow.tv_sec;
	  myMSecNow = timeNow.tv_usec / 1000;
  }
  vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
  printf("PL:MainSequence> After filling the obstacles into the DStar map %d ms, no. changes for the costmaks %d, no. changes to occupied %d, and no. changes to empty %d\n", vremenska_razlika,DS->NumElemCostmaska,DS->NumElemPunjenja,DS->NumElemPraznjenja);
/*  if (GM->numindeksimapepunjenje<GM->numindeksimapepraznjenje){
	  printf("tu cekam");
  }*/
	  GM->numindeksimapepraznjenje=0; GM->brojnovih=GM->numindeksimapepunjenje;GM->numindeksimapepunjenje=0;//resetiranje, dalje ga moze puniti DW a on ide nakon PL
  
  //tek tu dolazi ovo zbog gore uvjeta
  for (int i=0;i<GM->num_col_points_old;i++){     //tocke sudara od pokretne prepreke izracunate u DW-u u proslom ciklusu
	I_point temp;
	temp=GM->col_moving[i];
	GM->Map[temp.x][temp.y].time_stamp=-3;//oznaka zabiljezenih starih da ih ne provjeravamo stalno gore
  }
	GM->num_col_points_old=0;//resetiranje starih pokretnih prepreka
  //prepisivanje novih u stare
  for (int i=0;i<GM->num_col_points;i++){     //tocke sudara od pokretne prepreke izracunate u DW-u u proslom ciklusu
	  I_point temp;
			  temp=GM->col_moving[i];
			  if (GM->Map[temp.x][temp.y].time_stamp!=-2){//provjera da li smo vec upisali u polje za praznjenje
				  GM->indeksimapepraznjenje[GM->numindeksimapepraznjenje]=temp;   //azuriram indekse za planera
				  GM->numindeksimapepraznjenje++;
			  
				  GM->Map[temp.x][temp.y].time_stamp=-2;//oznaka starih i da ih ponovo mozemo upisivat u DW (nije -1) ili brisati u sljedecem ciklusu
				  GM->col_moving_old[GM->num_col_points_old]=temp;
				  GM->num_col_points_old++;
			}
/*				  int duplic=0;
				  for (int z=0;z<GM->numindeksimapepraznjenje;z++){
					  if ((GM->indeksimapepraznjenje[z].x==temp.x)&&(GM->indeksimapepraznjenje[z].y==temp.y)){
						  duplic=1;
						  break;
					  }
				  }
				  if (!duplic){
					  GM->indeksimapepraznjenje[GM->numindeksimapepraznjenje]=temp;   //azuriram indekse za planera
				  GM->numindeksimapepraznjenje++;
				  }*/
  }
  GM->num_col_points=0;//upisali smo sve, resetiramo na 0 brojac novih koje ce biti upisane u DW
/*  if (GM->numnewcg){
	  DS->novoTeziste=true;
  }else{
	  DS->novoTeziste=false;
  }*/
  //startna tocka jednaka je poziciji robota
 
  start_r.x=WH->RB.x;
   start_r.y=WH->RB.y;
   R_point start_ri;
   if(!RealToReal(start_r, start_ri, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> Start is out of the map borders!\n");
      return 2;
   }
   DS->start_ri=start_ri;
   if (DS->prviput==2){ //medo
   global_goal_r.x=WH->RB.x;
   global_goal_r.y=WH->RB.y;
   printf("current pos. (%f,%f)\n",WH->RB.x,WH->RB.y);
   }else{
   global_goal_r=WH->global_goal_workhorse;
   }
     if(!RealToInt(start_r, start_i, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> Start is out of the map borders!\n");
      return 2;
   }
  if(!RealToInt(global_goal_r, global_goal_i, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> Goal is out of the map borders!\n");
      return 2;
   }
//u svakom ciklusu se postavljaju init funkcije od Star algoritma
   int flag_debug;
   //prvo gledamo za Start, a onda za StartRacunac da li je zakrcen i oslobadjamo ga
#if DSTAR3D
	double tg=WH->global_goal_workhorse.th;
if (DS->prviput==2) tg=WH->RB.th;
	while (tg>=2*M_PI) tg-=2*M_PI;
	while (tg<0) tg+=2*M_PI;
	global_goal_i.th=cspace->worldToMapTheta(tg);
	double ts=WH->RB.th;
	while (ts>=2*M_PI) ts-=2*M_PI;
	while (ts<0) ts+=2*M_PI;
	start_r.th=ts;
	start_i.th=cspace->worldToMapTheta(ts);
    flag_debug=DS->InitOri(start_i, global_goal_i);
#else
   flag_debug=DS->Init(start_i.x, start_i.y, global_goal_i.x, global_goal_i.y);
   DS->goal_orientation=global_goal_r.th;
#endif
    if(flag_debug==0){
      printf("Planner> DStar has wrong map borders!\n");
      return 2;
    }else if(flag_debug==2){
        int brojac=0;
#if DSTAR3D
	if(DS->IsValidOri(start_i)==2)
#else
	if(DS->IsValid(start_i.x, start_i.y)==2)
#endif
	{
          printf("Planner> Start is occupied. Cleaning the start!\n");
                        //maskiranje u C-obstacle...
	  brojpam=0;
// 	  I_point pamtiindekse[(2*(robot_mask+cost_mask)+1)*(2*(robot_mask+cost_mask)+1)*(2*robot_mask+1)];
// 	  if (DS->prviput){
		  I_point temp_i;
		  if (0){//dolje se to opet radi pa je ovdje za svaki slucaj ako pracenje odrifta
	  for (int d=0;d<8;d++){
		  temp_i.x=start_i.x+xofs[d];
		  temp_i.y=start_i.y+yofs[d];
		  if (DS->IsValid(temp_i.x, temp_i.y)==1){
			  start_i.x=temp_i.x;
			  start_i.y=temp_i.y;
			  break;
		  }
	  }
	  }
// 	  }
//	  printf("odabrani susjedni start: (%d,%d)\n",start_i.x,start_i.y);
	  //DS->Init(start_i.x, start_i.y, global_goal_i.x, global_goal_i.y);//da se inicijalno postavi DS->Start
if(1){//0 kad ne zelim brisanje starta (mine se ne smiju obrisati)
#if RECTANGULAR
//provjera zauzetosti samo za pravokutni oblik, za slozeniji treba raditi po footprintu
double deltar,deltax,deltay,alfa;
	  for (i=start_i.x-robot_masky-robot_mask;i<=start_i.x+robot_masky+robot_mask;i++){
		  for (j=start_i.y-robot_masky-robot_mask;j<=start_i.y+robot_masky+robot_mask;j++){
		    deltar=sqrt((i-start_i.x)*(i-start_i.x)+(j-start_i.y)*(j-start_i.y));
		    alfa=atan2((j-start_i.y),(i-start_i.x));
		    deltax=deltar*sin(start_r.th-alfa);
		    deltay=deltar*cos(start_r.th-alfa);
//		    printf("PL: brisanje (i,j)=(%d,%d), deltar=%f, alfa=%f (deltax,deltay)=(%f,%f)\n",i,j,deltar,alfa,deltax,deltay);
		    if ((fabs(deltax)<=robot_mask+2)&&(fabs(deltay)<=robot_masky+2)){
			    Prazni(i, j, pamtiindekse, &brojpam, star_size_x, star_size_y, robot_mask, cost_mask, true);//ovdje se puni polje pamtiindekse i mijenja se brojpam
	        cspace->updateClearance(i,j,INT_MAX);
        }
		  }//for
	  }//for 
	  printf("finished deleting obstacles inside the robot shape - result is (clear=1, occupied=2) %d",DS->IsValidOri(start_i));
#else
	  for (i=start_i.x-robot_mask;i<=start_i.x+robot_mask;i++){
		  for (j=start_i.y-robot_mask;j<=start_i.y+robot_mask;j++){ 
			  Prazni(i, j, pamtiindekse, &brojpam, star_size_x, star_size_y, robot_mask, cost_mask, true);//ovdje se puni polje pamtiindekse i mijenja se brojpam
		  }//for
	  }//for
#endif
#if DSTAR3DORI
if (DS->prviput){//duplic, ne znam zasto je tu to, znam, zbog praznjenja, samo ako je start zauzet
  cspace->initializeDStar();
}
#endif
	  }
#if RECTANGULAR && (DSTAR3DORI==0)
	cspace->update(false);
#endif
	  if (0)
	  PrazniCostMasku(pamtiindekse, brojpam, star_size_x, star_size_y, cost_mask);//a ovdje se pomocu tog polja azurira cost maska
          if((++brojac)>1){
            printf("PL:MainSequence> Start could not get free\n");
            return 2;
          }
        } //ciscenje startne pozicije  (if)
        }//od flagdebug==2
        {//nova da uvijek provjerava cilj
#if DSTAR3D
  int oricost=0;
	if (((DS->IsValidOri(global_goal_i))==2) || (DS->howmanyOriCollides(global_goal_i)>0)) 
#else
	if (((DS->IsValid(global_goal_i.x, global_goal_i.y))==2)) 
#endif
	{
		printf("PL:MainSequence> Goal is occupied or can not rotate, searching for the free neighbor\n");
		I_point temp_i;
		int nasao=0;
		bool find_first=true;
		I_point candidate;
		if ((abs(global_goal_i.x-start_i.x)<8) && (abs(global_goal_i.y-start_i.y)<8)){
		  nasao=1;
		  global_goal_i=start_i;
		  printf("start and goal are close - choosing goal to be start\n");
		}
    if (nasao==0){
      for (int i=path_length-1;((i>=path_length-1-10)&&(i>=0));i--){
        temp_i=path[i];
        temp_i.th=global_goal_i.th;
#if DSTAR3D
	      if (((DS->IsValidOri(temp_i))==1) && (DS->howmanyOriCollides(temp_i)==0) && (oricost<OBSTACLE)) 
#else
			  if ((DS->IsValid(temp_i.x, temp_i.y)==1) && (DS->map[temp_i.x][temp_i.y].tag!=NEW) && (DS->map[temp_i.x][temp_i.y].traversal_cost==EMPTYC))
#endif
			  {
				  global_goal_i=temp_i;
				  nasao=1;
				  printf("found new on the old path %d cells distanced from the goal\n",path_length-1-i);
				  break;
			  }
  			if (find_first){
#if DSTAR3D
	        if (((DS->IsValidOri(temp_i))==1) && (DS->howmanyOriCollides(temp_i)<5) && (oricost<OBSTACLE)) 
#else
			    if ((DS->IsValid(temp_i.x, temp_i.y)==1) && (DS->map[temp_i.x][temp_i.y].tag!=NEW))
#endif			
			    {
				    find_first=false;
				    candidate=temp_i;
				    printf("found candidate neighbor 1 cell distanced\n");
			    }
			  }
      }
    }
    if (nasao==0){	
		for (int d=0;d<8;d++){
			temp_i.x=global_goal_i.x+xofs[d];
			temp_i.y=global_goal_i.y+yofs[d];
			temp_i.th=global_goal_i.th;
#if DSTAR3D
//  if (DS->prviput==0) oricost=DS->PathCostOri(temp_i);
	if (((DS->IsValidOri(temp_i))==1) && (DS->howmanyOriCollides(temp_i)==0) && (oricost<OBSTACLE)) 
#else
			if ((DS->IsValid(temp_i.x, temp_i.y)==1) && (DS->map[temp_i.x][temp_i.y].tag!=NEW) && (DS->map[temp_i.x][temp_i.y].traversal_cost==EMPTYC))
#endif
			{
				global_goal_i=temp_i;
				nasao=1;
				printf("found neighbor 1 cell distanced\n");
				break;
			}
			if (find_first){
#if DSTAR3D
	if (((DS->IsValidOri(temp_i))==1) && (DS->howmanyOriCollides(temp_i)<5) && (oricost<OBSTACLE)) 
#else
			if ((DS->IsValid(temp_i.x, temp_i.y)==1) && (DS->map[temp_i.x][temp_i.y].tag!=NEW))
#endif			
			{
				find_first=false;
				candidate=temp_i;
				printf("found candidate neighbor 1 cell distanced\n");
			}
			}
		}
		}
		if (nasao==0)
		{
			for (int d=0;d<16;d++)
			{
				temp_i.x=global_goal_i.x+x2ofs[d];
				temp_i.y=global_goal_i.y+y2ofs[d];
#if DSTAR3D
	if (((DS->IsValidOri(temp_i))==1) && (DS->howmanyOriCollides(temp_i)==0)&& (oricost<OBSTACLE)) 
#else
			if ((DS->IsValid(temp_i.x, temp_i.y)==1) && (DS->map[temp_i.x][temp_i.y].tag!=NEW) && (DS->map[temp_i.x][temp_i.y].traversal_cost==EMPTYC))
#endif
			{
				  global_goal_i=temp_i;
					nasao=1;
					printf("found neighbor 2 cells distanced\n");
					break;
				}
				if (find_first){
#if DSTAR3D
	if (((DS->IsValidOri(temp_i))==1) && (DS->howmanyOriCollides(temp_i)<5)&& (oricost<OBSTACLE)) 
#else
				if ((DS->IsValid(temp_i.x, temp_i.y)==1) && (DS->map[temp_i.x][temp_i.y].tag!=NEW))
#endif
				{
				find_first=false;
				candidate=temp_i;
				printf("found candidate neighbor 2 cell distanced\n");
				}
				}
			}
		}
		if (nasao==0)
		{
			for (int d=0;d<24;d++)
			{
				temp_i.x=global_goal_i.x+x3ofs[d];
				temp_i.y=global_goal_i.y+y3ofs[d];
#if DSTAR3D
//  if (DS->prviput==0) oricost=DS->PathCostOri(temp_i);
	if (((DS->IsValidOri(temp_i))==1) && (DS->howmanyOriCollides(temp_i)==0)&& (oricost<OBSTACLE)) 
#else
			if ((DS->IsValid(temp_i.x, temp_i.y)==1) && (DS->map[temp_i.x][temp_i.y].tag!=NEW) && (DS->map[temp_i.x][temp_i.y].traversal_cost==EMPTYC))
#endif
			{
  				global_goal_i=temp_i;
					nasao=1;
					printf("found neighbor 3 cells distanced\n");
					break;
				}
				if (find_first){
#if DSTAR3D
	if (((DS->IsValidOri(temp_i))==1) && (DS->howmanyOriCollides(temp_i)<5)&& (oricost<OBSTACLE)) 
#else
				if ((DS->IsValid(temp_i.x, temp_i.y)==1) && (DS->map[temp_i.x][temp_i.y].tag!=NEW))
#endif
				{
				find_first=false;
				candidate=temp_i;
				printf("found candidate neighbor 3 cell distanced\n");
				}
				}
			}
		}
		
		if (nasao==0)
		{
			for (int d=0;d<32;d++)
			{
				temp_i.x=global_goal_i.x+x4ofs[d];
				temp_i.y=global_goal_i.y+y4ofs[d];
#if DSTAR3D
//  if (DS->prviput==0) oricost=DS->PathCostOri(temp_i);
	if (((DS->IsValidOri(temp_i))==1) && (DS->howmanyOriCollides(temp_i)==0)&& (oricost<OBSTACLE)) 
#else
			if ((DS->IsValid(temp_i.x, temp_i.y)==1) && (DS->map[temp_i.x][temp_i.y].tag!=NEW) && (DS->map[temp_i.x][temp_i.y].traversal_cost==EMPTYC))
#endif
			{
  				global_goal_i=temp_i;
					nasao=1;
					printf("found neighbor 4 cells distanced\n");
					break;
				}
				if (find_first){
#if DSTAR3D
	if (((DS->IsValidOri(temp_i))==1) && (DS->howmanyOriCollides(temp_i)<5)&& (oricost<OBSTACLE)) 
#else
				if ((DS->IsValid(temp_i.x, temp_i.y)==1) && (DS->map[temp_i.x][temp_i.y].tag!=NEW))
#endif
				{
				find_first=false;
				candidate=temp_i;
				printf("found candidate neighbor 4 cell distanced\n");
				}
				}

			}
		}
		if (nasao==0)
		{
			for (int d=0;d<40;d++)
			{
				temp_i.x=global_goal_i.x+x5ofs[d];
				temp_i.y=global_goal_i.y+y5ofs[d];
#if DSTAR3D
//  if (DS->prviput==0) oricost=DS->PathCostOri(temp_i);
	if (((DS->IsValidOri(temp_i))==1) && (DS->howmanyOriCollides(temp_i)==0)&& (oricost<OBSTACLE)) 
#else
			if ((DS->IsValid(temp_i.x, temp_i.y)==1) && (DS->map[temp_i.x][temp_i.y].tag!=NEW) && (DS->map[temp_i.x][temp_i.y].traversal_cost==EMPTYC))
#endif
			{
  				global_goal_i=temp_i;
					nasao=1;
					printf("found neighbor 5 cells distanced\n");
					break;
				}
				if (find_first){
#if DSTAR3D
	if (((DS->IsValidOri(temp_i))==1) && (DS->howmanyOriCollides(temp_i)<5)&& (oricost<OBSTACLE)) 
#else
				if ((DS->IsValid(temp_i.x, temp_i.y)==1) && (DS->map[temp_i.x][temp_i.y].tag!=NEW))
#endif
				{
				find_first=false;
				candidate=temp_i;
				printf("found candidate neighbor 5 cell distanced\n");
				}
				}

			}
		}
		if ((nasao==0)&&(find_first==false)){
			nasao=1;
			global_goal_i=candidate;
				printf("take the candidate neighbor (%d,%d,%d)\n",candidate.x,candidate.y,candidate.th);
		}
		if (nasao==1)//da se dobiju realne koordinate
		{
			WH->global_goal_workhorse.x=global_goal_i.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
			WH->global_goal_workhorse.y=global_goal_i.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
		    	double goal_tolerance;
		    	goal_tolerance=((M->subgoal.x-M->goal.x)*(M->subgoal.x-M->goal.x)+(M->subgoal.y-M->goal.y)*(M->subgoal.y-M->goal.y));
			if (goal_tolerance>GOAL_POSITION_TOLERANCE*GOAL_POSITION_TOLERANCE) 
	 		{
	 			printf("PL> local subgoal occupied!\n");
	 		}else{
				M->goal=WH->global_goal_workhorse;
        M->subgoal=M->goal;//potrebno ako ga ne saljem u NO_PATH
			}
#if RECTANGULAR
//-------------determing orientation of the goal ---------------------
	double t=WH->global_goal_workhorse.th;
	int orientation;
	while (t>=2*M_PI) t-=2*M_PI;
	while (t<0) t+=2*M_PI;
	orientation=cspace->worldToMapTheta(t);
	IntPose checkme((global_goal_i.x),(global_goal_i.y),orientation);
	if (cspace->checkCollision(checkme)){
		printf("PL: goal orientation collides %f deg\n",t*RuS);
		OrientationIntervals ints;
    		ints= cspace->getAdmissibleOrientationStructure(global_goal_i.x,global_goal_i.y);
    		std::cerr<<ints.print()<<std::endl;
       		
	std::cout<<"at "<<global_goal_i.x<<","<<global_goal_i.y<<" orientation "<<orientation<<" is contained: "<<(ints.containsOrientation(orientation)?"yes":"no")<<std::endl;
		int tempOrientation=orientation;
	    	int minD=ints.getMaxOrientation()+1;
	    	for (int j=ints.getMinOrientation(); j<=ints.getMaxOrientation(); j++){
	    		if (ints.containsOrientation(j)){
	    			if ((abs(j-orientation)<minD)||(abs(j+ints.getMaxOrientation()+1-orientation)<minD)||(abs(j-ints.getMaxOrientation()-1-orientation)<minD)){
					minD=std::min(abs(j-orientation),std::min((abs(j+ints.getMaxOrientation()+1-orientation)),(abs(j-ints.getMaxOrientation()-1-orientation))));
					tempOrientation=j;
	    			}
	    		}
	    	}
	    	t=cspace->mapToWorldTheta(tempOrientation);
	    	WH->global_goal_workhorse.th=t;
	    	printf("new orientation is %d (%f deg), max is %d\n", tempOrientation, t*RuS, ints.getMaxOrientation());
//		t=0;
//		while (t<2*M_PI){
//			Pose checkme((global_goal_i.x),(global_goal_i.y),t);
//			if (!cspace->checkCollision(checkme)){
//				WH->global_goal_workhorse.th=t;
//				printf("PL: new orientation is %f deg\n",t*RuS);
//				break;
//			}
//			t+=10*SuR;
//		}
	}
//------------------------------------
#endif
			FILE *logfile;
			if ( (logfile = fopen("logger//newgoals","a")) == NULL ){
	 			printf("Error! komb file couldn't be opened.");
	 		}else{
	 			fprintf(logfile,"1 %f %f %f\n",WH->global_goal_workhorse.x,WH->global_goal_workhorse.y, WH->global_goal_workhorse.th);
	 			fclose(logfile);
	 		}
		}
		if (nasao) return 0; //zasto je tu return iako je mozda nasao slobodnu poziciju? zato sto izadje s nopath i racuna ponovo
//		if (nasao==0) return 2; //radi segm fault ako ne uracuna promjene i trazi cvorove od orija
	}
    }  //od else
#if RECTANGULAR
//-------------determing orientation of the goal ---------------------
//real coordinates within a cell
    	double goal_orientation=WH->global_goal_workhorse.th;
    	double start_orientation=WH->RB.th;
if (DS->prviput==2) goal_orientation=start_orientation;
	while (goal_orientation>=2*M_PI) goal_orientation-=2*M_PI;
	while (goal_orientation<0) goal_orientation+=2*M_PI;
	global_goal_i.th=cspace->worldToMapTheta(goal_orientation);
	while (start_orientation>=2*M_PI) start_orientation-=2*M_PI;
	while (start_orientation<0) start_orientation+=2*M_PI;
	start_i.th = cspace->worldToMapTheta(start_orientation);
	IntPose checkme((global_goal_i.x),(global_goal_i.y),global_goal_i.th);
	std::cout<<"robot collides at goal? "<<global_goal_i.x<<","<<global_goal_i.y<<","<<global_goal_i.th<<","<<goal_orientation<<": "<<(cspace->checkCollision(checkme)?"yes":"no")<<std::endl;
	
	if (cspace->checkCollision(checkme)){
		printf("PL: goal orientation collides %f deg\n",goal_orientation*RuS);
		OrientationIntervals ints;
    		ints= cspace->getAdmissibleOrientationStructure(global_goal_i.x,global_goal_i.y);
    		std::cerr<<ints.print()<<std::endl;
       		
	std::cout<<"at "<<global_goal_i.x<<","<<global_goal_i.y<<" orientation "<<global_goal_i.th<<" is contained: "<<(ints.containsOrientation(global_goal_i.th)?"yes":"no")<<std::endl;
		int tempOrientation=global_goal_i.th;
	    	int minD=ints.getMaxOrientation()+1;
	    	for (int j=ints.getMinOrientation(); j<=ints.getMaxOrientation(); j++){
	    		if (ints.containsOrientation(j)){
	    			if ((abs(j-global_goal_i.th)<minD)||(abs(j+ints.getMaxOrientation()+1-global_goal_i.th)<minD)||(abs(j-ints.getMaxOrientation()-1-global_goal_i.th)<minD)){
					minD=std::min(abs(j-global_goal_i.th),std::min((abs(j+ints.getMaxOrientation()+1-global_goal_i.th)),(abs(j-ints.getMaxOrientation()-1-global_goal_i.th))));
					tempOrientation=j;
	    			}
	    		}
	    	}
	    	global_goal_i.th=tempOrientation;
	    	goal_orientation=cspace->mapToWorldTheta(tempOrientation);
	    	WH->global_goal_workhorse.th=goal_orientation;
	    	printf("new orientation is %d (%f deg), max is %d\n", tempOrientation, goal_orientation*RuS, ints.getMaxOrientation());
		
//		t=0;
//		while (t<2*M_PI){
//			Pose checkme((global_goal_i.x),(global_goal_i.y),t);
//			if (!cspace->checkCollision(checkme)){
//				WH->global_goal_workhorse.th=t;
//				printf("PL: new orientation is %f deg\n",t*RuS);
//				break;
//			}
//			t+=10*SuR;
//		}

	}
	IntPose currentgoal((global_goal_i.x),(global_goal_i.y),(global_goal_i.th));
	std::cout<<"robot collides at goal? "<<currentgoal.x<<","<<currentgoal.y<<","<<currentgoal.theta<<": "<<(cspace->checkCollision(currentgoal)?"yes":"no")<<std::endl;
//	start_i.x=21; start_i.y=40; start_i.th=13;    
//	start_i.x=8; start_i.y=12; start_i.th=18;  
//	start_i.x=52; start_i.y=12; start_i.th=30; 
//	start_i.x=72; start_i.y=35; start_i.th=42; 
//	start_i.x=74; start_i.y=54; start_i.th=37; 
//	start_i.x=158; start_i.y=184; start_i.th=100;  
	IntPose currentstart(start_i.x,start_i.y,start_i.th); 
	std::cout<<"robot collides at start? "<<start_i.x<<","<<start_i.y<<","<<start_i.th<<": "<<(cspace->checkCollision(currentstart)?"yes":"no")<<std::endl;
 

#if 0
//compute a path x,y,th 
	IntPose start, goal;
	start.x = start_i.x;
	start.y = start_i.y;
	start.theta = start_i.th;
	goal.x=global_goal_i.x;
	goal.y=global_goal_i.y;
	goal.theta=global_goal_i.th;
	cspace->cleanup();

	std::list<IntPose>* cspath = cspace->computeShortestPath(start, goal);//(goal,start);//(start, goal);
	if(cspath!=NULL){
		OrientationIntervals ints;
    		ints= cspace->getAdmissibleOrientationStructure(start.x,start.y);
    		std::cerr<<ints.print()<<std::endl;
	  std::cerr<<cspath->size()<<" cells"<<std::endl;
	  std::cout<<"start "<<start.x<<","<<start.y<<","<<start.theta<<std::endl;
	  std::cout<<"goal "<<goal.x<<","<<goal.y<<","<<goal.theta<<std::endl;
	  std::cout<<"robot collides at start? "<<start.x<<","<<start.y<<","<<start.theta<<": "<<(cspace->checkCollision(start)?"yes":"no")<<std::endl;
	std::cout<<"robot collides at goal? "<<goal.x<<","<<goal.y<<","<<goal.theta<<": "<<(cspace->checkCollision(goal)?"yes":"no")<<std::endl;
	std::cout<<"cspace_occupied at start? "<<DS->map[ start.x ][ start.y ].cspace_occupied<<std::endl;

	  FILE *logfile, *logfile2;
	  if ( (logfile = fopen("logger//cspath","wt")) == NULL ){
	 	  printf("Error! cspath file couldn't be opened.");
	  }else{
		  for(std::list<IntPose>::iterator it=cspath->begin(); it!=cspath->end(); ++it){
	    		std::cout<<it->x<<","<<it->y<<","<<it->theta<<std::endl;
	  		fprintf(logfile,"%d %d %f\n",it->x,it->y, cspace->mapToWorldTheta(it->theta));
	  	  }
	 	  fclose(logfile);
	  }
	  
	 }
#endif
//------------------------------------
    if (DW->locmincnt>N_KL){
    	
    	double goal_tolerance;
    	double deltath;
    	double temp_orientation;
    	goal_tolerance=((M->subgoal.x-M->goal.x)*(M->subgoal.x-M->goal.x)+(M->subgoal.y-M->goal.y)*(M->subgoal.y-M->goal.y));
	if (goal_tolerance>GOAL_POSITION_TOLERANCE*GOAL_POSITION_TOLERANCE) 
	 {
	 	WH->global_goal_workhorse=M->goal;
	 	printf("PL local minima: last subgoal not reached! returning to the global goal (%f,%f) in mm\n",WH->global_goal_workhorse.x,WH->global_goal_workhorse.y);
	 	return 0;
	 }
    	I_point temp;
	I_point next, temp2, delta;
	bool findfirst=true;
	DW->locmincnt=0;
	next=DS->map[start_i.x][start_i.y]._next;
	temp=start_i;
	IntPose start, goal;
	start.x = start_i.x;
	start.y = start_i.y;
	start.theta = start_i.th;
	OrientationIntervals ints;
	ints= cspace->getAdmissibleOrientationStructure(start.x,start.y);
	std::cerr<<"start has admissible orientations: "<<ints.print()<<std::endl;

#if (DSTAR3D==0)
	if ((next.x==-1) || (next.y==-1) || (DS->PathLength<2*ROBOT_MASKY))
	{
	//at goal
		goal.x = global_goal_i.x;
		goal.y = global_goal_i.y;
		goal.theta = global_goal_i.th;
	}else{
		goal.x = next.x;
		goal.y = next.y;
//first check if goal orientation is available
		goal.theta = global_goal_i.th;
        	ints= cspace->getAdmissibleOrientationStructure(goal.x,goal.y);
		std::cout<<"at "<<goal.x<<","<<goal.y<<" goal orientation "<<goal.theta<<" is contained: "<<(ints.containsOrientation(goal.theta)?"yes":"no")<<std::endl;
    		std::cerr<<"new goal has admissible orientations: "<<ints.print()<<std::endl;
		if ((ints.containsOrientation(goal.theta)==false))//  (DS->PathLength>ROBOT_MASKY))
		{

		temp2=DS->map[goal.x][goal.y]._next;
		temp_orientation=atan2(temp2.y-goal.y,temp2.x-goal.x);
		while (temp_orientation>=2*M_PI) temp_orientation-=2*M_PI;
		while (temp_orientation<0) temp_orientation+=2*M_PI;
		deltath=std::min(fabs(goal_orientation-temp_orientation),std::min(fabs(goal_orientation-temp_orientation+2*M_PI),fabs(goal_orientation-temp_orientation-2*M_PI)));
		if ((deltath>=0.5*M_PI)&& (1)){
			temp_orientation-=M_PI;
			while (temp_orientation<0) temp_orientation+=2*M_PI;
		}
		//that is closer to goal orientation, maybe closer to start orientation if this one is distanced for pi
		deltath=std::min(fabs(start_orientation-temp_orientation),std::min(fabs(start_orientation-temp_orientation+2*M_PI),fabs(start_orientation-temp_orientation-2*M_PI)));
		if ((deltath>=0.5*M_PI)&& (1)){
			temp_orientation-=M_PI;
			while (temp_orientation<0) temp_orientation+=2*M_PI;
		}
		goal.theta=cspace->worldToMapTheta(temp_orientation);
		std::cout<<"at new "<<goal.x<<","<<goal.y<<" new orientation "<<goal.theta<<" is contained: "<<(ints.containsOrientation(goal.theta)?"yes":"no")<<std::endl;
		
		IntPose checkme((goal.x),(goal.y),goal.theta);
		if (cspace->checkCollision(checkme)){

			printf("PL: subgoal orientation collides %f deg\n",temp_orientation*RuS);
			int tempOrientation=goal.theta;
		    	int minD=ints.getMaxOrientation()+1;
		    	for (int j=ints.getMinOrientation(); j<=ints.getMaxOrientation(); j++){
		    		if (ints.containsOrientation(j)){
		    			if ((abs(j-goal.theta)<minD)||(abs(j+ints.getMaxOrientation()+1-goal.theta)<minD)||(abs(j-ints.getMaxOrientation()-1-goal.theta)<minD)){
						minD=std::min(abs(j-goal.theta),std::min((abs(j+ints.getMaxOrientation()+1-goal.theta)),(abs(j-ints.getMaxOrientation()-1-goal.theta))));
						tempOrientation=j;
		    			}
		    		}
		    	}
		    	temp_orientation=cspace->mapToWorldTheta(tempOrientation);
		    	printf("new orientation is %d (%f deg), max is %d\n", tempOrientation, temp_orientation*RuS, ints.getMaxOrientation());
			goal.theta=tempOrientation;
		}
		}

	}

	std::cout<<"robot collides at start? "<<start.x<<","<<start.y<<","<<start.theta<<": "<<(cspace->checkCollision(start)?"yes":"no")<<std::endl;
	std::cout<<"robot collides at goal? "<<goal.x<<","<<goal.y<<","<<goal.theta<<": "<<(cspace->checkCollision(goal)?"yes":"no")<<std::endl;

//compute a path x,y,th 
	cspace->cleanup();

	std::list<IntPose>* cspath = cspace->computeShortestPath(start, goal);//(goal,start);//(start, goal);
	if(cspath!=NULL){
	  std::cerr<<cspath->size()<<" cells"<<std::endl;
	  std::cout<<"start "<<start.x<<","<<start.y<<","<<start.theta<<std::endl;
	  std::cout<<"goal "<<goal.x<<","<<goal.y<<","<<goal.theta<<std::endl;
	  std::cout<<"robot collides at start? "<<start.x<<","<<start.y<<","<<start.theta<<": "<<(cspace->checkCollision(start)?"yes":"no")<<std::endl;
	std::cout<<"robot collides at goal? "<<goal.x<<","<<goal.y<<","<<goal.theta<<": "<<(cspace->checkCollision(goal)?"yes":"no")<<std::endl;

	  FILE *logfile, *logfile2;
	  if ( (logfile = fopen("logger//cspath","wt")) == NULL ){
	 	  printf("Error! cspath file couldn't be opened.");
	  }else{
		  for(std::list<IntPose>::iterator it=cspath->begin(); it!=cspath->end(); ++it){
	    		std::cout<<it->x<<","<<it->y<<","<<it->theta<<", k="<<DS->map[it->x][it->y].k_cost_int<<std::endl;
	  		fprintf(logfile,"%d %d %f\n",it->x,it->y, cspace->mapToWorldTheta(it->theta));
	  	  }
	 	  fclose(logfile);
	  }
	  IntPose tempgoal;
	  tempgoal.x=start.x;
	  tempgoal.y=start.y;
	  tempgoal.theta=start.theta;
	  int numthetachn=0;
	  bool goalfind=false;
	  int costk=DS->map[tempgoal.x][tempgoal.y].k_cost_int;
	  printf("costk=%d\n",costk);
	  bool obstaclenear=false;
	  for (int d=0;d<8;d++){
	  	temp2.x=tempgoal.x+xofs[d];
	  	temp2.y=tempgoal.y+yofs[d];
	  	if ((DS->IsValid(temp2.x, temp2.y)==2)){
	  		obstaclenear=true;
			break;
		}
	  }
	  for(std::list<IntPose>::iterator it=cspath->begin(); it!=cspath->end(); ++it){
	  	if ((tempgoal.x==it->x)&&(tempgoal.y==it->y)){
	  		numthetachn++;
	  		tempgoal.theta=it->theta;
	  	}else{
	  		if (((costk<=DS->map[it->x][it->y].k_cost_int)&&(DS->map[it->x][it->y].k_cost_int<OBSTACLE))||(obstaclenear)){
	  			obstaclenear=false;
//	  			for (int d=0;d<8;d++){
//					temp2.x=it->x+xofs[d];
//					temp2.y=it->y+yofs[d];
//					if ((DS->IsValid(temp2.x, temp2.y)==2)){
//						obstaclenear=true;
//						break;
//					}
//				}
	  			if (obstaclenear==false){
	  				tempgoal.x=it->x;
	  				tempgoal.y=it->y;
	  				tempgoal.theta=it->theta;
	  				costk=DS->map[tempgoal.x][tempgoal.y].k_cost_int;
	  				printf("costk=%d\n",costk);
	  			}
	  		}
	  		if ((costk>DS->map[it->x][it->y].k_cost_int) && (goalfind==false)){
	  			if (numthetachn>1){
	  				goalfind=true;
	  			}else{
  					tempgoal.x=it->x;
  					tempgoal.y=it->y;
  					tempgoal.theta=it->theta;
  					printf("tempgoal= %d,%d,%d, numthetachn=%d\n", tempgoal.x, tempgoal.y, tempgoal.theta, numthetachn);
  				}
	  		}
	  		numthetachn=1;
//	  		if (numthetachn>1){
//	  			goalfind=true;
//	  		}
//	  		if (goalfind==false){
//	  		tempgoal.x=it->x;
//	  		tempgoal.y=it->y;
//	  		tempgoal.theta=it->theta;
//	  		numthetachn=1;
//	  		}
	  	}
    		if ((it->theta==goal.theta)&&(it->x==goal.x)&&(it->y==goal.y)){
//    			if (goalfind==false){
//    				tempgoal.x=it->x;
//    				tempgoal.y=it->y;
//    				tempgoal.theta=it->theta;
//    			}
			WH->global_goal_workhorse.x=tempgoal.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
			WH->global_goal_workhorse.y=tempgoal.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
			WH->global_goal_workhorse.th=cspace->mapToWorldTheta(tempgoal.theta);
			if (((goal.x!=tempgoal.x)||(goal.y!=tempgoal.y))){
    				M->subgoal2.x=goal.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
    				M->subgoal2.y=goal.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
    				M->subgoal2.th=cspace->mapToWorldTheta(goal.theta);
    				goal_tolerance=((M->subgoal2.x-M->goal.x)*(M->subgoal2.x-M->goal.x)+(M->subgoal2.y-M->goal.y)*(M->subgoal2.y-M->goal.y));
	if (goal_tolerance>GOAL_POSITION_TOLERANCE*GOAL_POSITION_TOLERANCE) 
	 {
    				M->subgoal2flag=true;
	 }

    			}
			printf("PL local minima: chosen goal from cspace path (%d,%d,%d), while goal is (%d,%d,%d). chosen goal is (%f,%f) in mm\n",tempgoal.x,tempgoal.y,tempgoal.theta, goal.x,goal.y,goal.theta,WH->global_goal_workhorse.x,WH->global_goal_workhorse.y);
			
			if ( (logfile2 = fopen("logger//newgoals","a")) == NULL ){
	 			printf("Error! komb file couldn't be opened.");
	 		}else{
	 			fprintf(logfile2,"2 %f %f %f\n",WH->global_goal_workhorse.x,WH->global_goal_workhorse.y, WH->global_goal_workhorse.th);
	 			if (M->subgoal2flag){
	 	 			fprintf(logfile2,"2 %f %f %f\n",M->subgoal2.x,M->subgoal2.y, M->subgoal2.th);			
	 			}
	 			fclose(logfile2);
	 		}
	
			return 0;

    		}
	    	
	}

	  
      }
#endif
//----------------------------	
	if ((next.x!=-1)&&(next.y!=-1)){//only at the goal is -1, but now the neighbor of goal will be chosen
		delta.x=next.x-temp.x;
		delta.y=next.y-temp.y;
		WH->global_goal_workhorse.th=atan2(delta.y,delta.x);
		
	for (int k=0; k<=(robot_masky-robot_mask)+3;k++){
		temp2.x=temp.x-delta.x;
		temp2.y=temp.y-delta.y;//unazad
		next=DS->map[temp2.x][temp2.y]._next;//novi next
		if ((DS->IsValid(temp2.x, temp2.y)!=1) || ((next.x==-1)||(next.y==-1))){
			int foundit=0;
			for (int d=0;d<8;d++){
				temp2.x=temp.x+xofs[d];
				temp2.y=temp.y+yofs[d];
				if ((DS->map[temp2.x][temp2.y]._next.x==temp.x)&& (DS->map[temp2.x][temp2.y]._next.y==temp.y)&& (DS->IsValid(temp2.x, temp2.y)==1)){
					foundit=1;
					next=temp;//novi next
					break;
				}
				if ((DS->IsValid(temp2.x, temp2.y)==1) && (DS->map[temp2.x][temp2.y].tag!=NEW) && (DS->map[temp2.x][temp2.y].k_cost_int>=DS->map[start_i.x][start_i.y].k_cost_int)){
					foundit=2;
					next=temp2;//remember right cell
				}
			}
			if (foundit==2) temp2=next; //rewrite the new right cell
			if ((foundit==0)&&(findfirst)){
				
			WH->global_goal_workhorse.x=temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
			WH->global_goal_workhorse.y=temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
			printf("PL local minima: chosen last available goal (%d,%d) in gridmap indices or (%f,%f) in mm\n",temp.x,temp.y,WH->global_goal_workhorse.x,WH->global_goal_workhorse.y);
			
			FILE *logfile;
			if ( (logfile = fopen("logger//newgoals","a")) == NULL ){
	 			printf("Error! komb file couldn't be opened.");
	 		}else{
	 			fprintf(logfile,"2 %f %f %f\n",WH->global_goal_workhorse.x,WH->global_goal_workhorse.y, WH->global_goal_workhorse.th);
	 			fclose(logfile);
	 		}
	
			return 0;
			}
		}
		temp=temp2;
		if (DS->map[temp.x][temp.y].traversal_cost!=EMPTYC){
			for (int d=0;d<8;d++){
				temp2.x=temp.x+xofs[d];
				temp2.y=temp.y+yofs[d];
				if ((DS->map[temp2.x][temp2.y].traversal_cost==EMPTYC) && (DS->IsValid(temp2.x, temp2.y)==1)){
					temp=temp2;
					break;
				}
			}
		}
//		delta.x=next.x-temp.x;
//		delta.y=next.y-temp.y;
		if ((DS->IsValid(temp.x, temp.y)==1) && (k>=(robot_masky-robot_mask))){
			if ((findfirst) || (DS->map[temp.x][temp.y].traversal_cost==EMPTYC)) {
			findfirst=false;
			WH->global_goal_workhorse.x=temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
			WH->global_goal_workhorse.y=temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
			printf("PL local minima: chosen goal candidate distanced for %d cells (%d,%d) in gridmap indices or (%f,%f) in mm\n",k,temp.x,temp.y,WH->global_goal_workhorse.x,WH->global_goal_workhorse.y);
			}	
		}

		if ((DS->IsValid(temp.x, temp.y)==1) && (k>(robot_masky-robot_mask)+2)&&(DS->map[temp.x][temp.y].traversal_cost==EMPTYC)){
			WH->global_goal_workhorse.x=temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
			WH->global_goal_workhorse.y=temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
			printf("PL local minima: chosen goal distanced for %d cells (%d,%d) in gridmap indices or (%f,%f) in mm\n",k,temp.x,temp.y,WH->global_goal_workhorse.x,WH->global_goal_workhorse.y);	
			FILE *logfile;
			if ( (logfile = fopen("logger//newgoals","a")) == NULL ){
	 			printf("Error! komb file couldn't be opened.");
	 		}else{
	 			fprintf(logfile,"2 %f %f %f\n",WH->global_goal_workhorse.x,WH->global_goal_workhorse.y, WH->global_goal_workhorse.th);
	 			fclose(logfile);
	 		}
			return 0;		
		}
	}
	}
	if (findfirst==false) {
			FILE *logfile;
			if ( (logfile = fopen("logger//newgoals","a")) == NULL ){
	 			printf("Error! komb file couldn't be opened.");
	 		}else{
	 			fprintf(logfile,"2 %f %f %f\n",WH->global_goal_workhorse.x,WH->global_goal_workhorse.y, WH->global_goal_workhorse.th);
	 			fclose(logfile);
	 		}
		return 0;
	}
    }
#endif //od rectangular
    DS->kut_robota=WH->RB.th;
#if DSTAR
	if (0){//stavi 1 kad hoces staro
    if (!DS->prviput){
	//tuUu
	    I_point p,minp;
	    int i,d=0,dmin=2000;
	    for(i=0;i<DS->PathLength-1;i++){
		    p=DS->path[i];
		    d=DS->pathCostEstimateDiagonalStraight(DS->Start,p);
		    if (dmin>=d){
			    dmin=d;
			    minp=p;
		    }
		    //flag_debug=DS->IsValid(p.x, p.y);
		    flag_debug=1; //ionako ce se ocistiti
// 		    printf("p=(%d,%d), d=%d\n",p.x,p.y,d);
		    if ((flag_debug==1)&&((i==DS->PathLength-2)||((d>dmin)&&(i>10)))) { //bilo je 30 umjesto 10, ne znam zasto
// 			    printf("i=%d je malo vece udaljenosti, StartRacunac=(%d,%d) postaje minp=(%d,%d)\n",i,DS->StartRacunac.x,DS->StartRacunac.y,minp.x,minp.y);
			    DS->StartRacunac=minp;
			    break;
		    }
	    }
// 	DS->StartRacunac=start_i;
// 	GM->nonempty=DS->map[DS->StartRacunac.x][DS->StartRacunac.y].traversal_cost;
// 	d=(int)(WH->RB.v*WH->RB.v/DV_MAX/CELL_DIM)+2;//minimalni zaustavni put
	d=(int)abs(WH->RB.v*LASER_RANGE_MAX/V_MAX/2./CELL_DIM);//pola maksimalnog puta
	if (DS->PathLength>d){
		GM->nonempty=std::max(DS->map[DS->path[d].x][DS->path[d].y].traversal_cost,DS->map[start_i.x][start_i.y].traversal_cost);//uzimam d polja dalju tocku,ovisno o trenutnoj brzini
	}
	//provjera pogreske lokalizacije (dogodilo se u simulatoru da se Start prebaci 10 polja udesno u odnosu na StartRacunca, a postoji put i javi se beskonacna petlja
	if ((abs(DS->Start.x-DS->StartRacunac.x)>robot_mask)||(abs(DS->Start.y-DS->StartRacunac.y)>robot_mask)){
		DS->StartRacunac=DS->Start;
	}
    }else{
	    DS->StartRacunac=DS->Start;
	    if (DS->prviput==2){
	    	global_goal_i=DS->Start;//medo
	    }
    }
    }else{
    DS->StartRacunac=DS->Start;//postavljam uvijek trenutnu poziciju!!!
    }
#else
    if (!DS->prviputWitkowski){
	//tuUu
	I_point p,minp;
	int i,d=0,dmin=2000;
	for(i=0;i<DS->PathLength_backward-1;i++){
		p=DS->path_backward[i];
		d=DS->pathCostEstimateDiagonalStraight(DS->Start,p);
		if (dmin>=d){
			dmin=d;
			minp=p;
		}
// 		printf("p=(%d,%d), d=%d\n",p.x,p.y,d);
		if ((i==DS->PathLength_backward-2)||((d>dmin)&&(i>30))) {//&&(i>5)
// 			printf("i=%d je malo vece udaljenosti\n",i);
			DS->StartRacunac=minp;
			break;
		}
	}
    }else{
	    DS->StartRacunac=DS->Start;
    }
#endif
#if (DSTAR3DORI==0)
/*    if (GM->numnewcg==2){
	    printf("dva tezista, debug ce bit\n");
// 	    DS->pipodjednom=true;
    }*/
    flag_debug=DS->IsValid(DS->StartRacunac.x, DS->StartRacunac.y);
    if (flag_debug==2){
    	flag_debug=DS->IsValid(start_i.x, start_i.y);
    	if (flag_debug==1){
    		DS->StartRacunac=start_i;
    	}
    }
    flag_debug=DS->IsValid(DS->StartRacunac.x, DS->StartRacunac.y);
//     printf("PL> DS->Init flag_debug=%d\n",flag_debug);
//     printf("flag_debug=%d nakon # znakova\n", flag_debug);
    if (flag_debug==2){//ako onaj jadan nije bio dobar onda treba uzeti ovoga
	    //susjednog trazi prvo, nemoj brisati
    		  I_point temp_i;
	  if(1){//tu stavi 0 kada ne zelis biranje susjednog
	  for (int d=0;d<8;d++){
		  temp_i.x=DS->StartRacunac.x+xofs[d];
		  temp_i.y=DS->StartRacunac.y+yofs[d];
		  if (DS->IsValid(temp_i.x, temp_i.y)==1) {
			  DS->StartRacunac.x=temp_i.x;
			  DS->StartRacunac.y=temp_i.y;
			  break;
		  }
	  }
	  }
	  printf("PL choosing the new start. instead of (%d,%d) goes (%d,%d), flag=%d\n", start_i.x, start_i.y, DS->StartRacunac.x, DS->StartRacunac.y, flag_debug);

    
    }
    flag_debug=DS->Init(DS->StartRacunac.x, DS->StartRacunac.y, global_goal_i.x, global_goal_i.y);
    flag_debug=DS->IsValid(DS->StartRacunac.x, DS->StartRacunac.y);
   if(flag_debug==2){
	   printf("Planner> StartRacunac occupied. Cleaning the map..\n");
                        //maskiranje u C-obstacle...
// 	   DS->pipodjednom=true;


	   brojpam=0;
// 	   I_point pamtiindekse[(2*(robot_mask+cost_mask)+1)*(2*(robot_mask+cost_mask)+1)*(2*robot_mask+1)];
	  for (i=DS->StartRacunac.x-robot_mask;i<=DS->StartRacunac.x+robot_mask;i++){
			   for (j=DS->StartRacunac.y-robot_mask;j<=DS->StartRacunac.y+robot_mask;j++){ 
			   if (DS->IsValid(i,j)>0)
			   Prazni(i, j, pamtiindekse, &brojpam, star_size_x, star_size_y, robot_mask, cost_mask, true);//ovdje se puni polje pamtiindekse i mijenja se brojpam
		   }//for
	   }//for
	   PrazniCostMasku(pamtiindekse, brojpam, star_size_x, star_size_y, cost_mask);//a ovdje se pomocu tog polja azurira cost maska
   }
   
#endif
//          	   //MJERI VRIJEME
 	if (gettimeofday(&timeNow, NULL) == 0)
 	{
 		mySecNow = timeNow.tv_sec;
 		myMSecNow = timeNow.tv_usec / 1000;
 	}
 	vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
 	printf("PL:MainSequence> before calling the DStar 3D search algorithm %d ms\n", vremenska_razlika);

    path=NULL;
    path_length=-1;
    FILE *logfile;
    search_time=0;
    search_explored=0;
//---------------------------------------------------------
#if DSTAR3D
      DS->racunaoupromjeni=0;//for logging the path in WH
    flag_debug=DS->InitOri(start_i, global_goal_i);
	  std::cout<<"cspace_occupied at start? "<<DS->map[ start_i.x ][ start_i.y ].cspace_occupied<<std::endl;
    if (flag_debug!=1){
         std::cerr<<"PL> no path!"<<std::endl;  
         cspace->initializeDStar(); //zbog segm faulta
            return 2;      
    }
    else 
    {
#if DSTAR3DORI
      std::vector<DStarSearchNode*>* pathori;
	      int traversalcostpath;
	      DStarSearchNode* currentNode;
        int maxOri, x, y, o;
          int minUp,minLo;
        std::vector<OrientationInterval> intervals, intersections;
        OrientationInterval interval, intervalnext;
//        OrientationIntervals ints;
//		ints= cspace->getAdmissibleOrientationStructure(currentstart.x,currentstart.y);
		maxOri=OrientationIntervals::getMaxOrientation();
      if (DS->prviput){
//        if ( (logfile = fopen("logger//traversalcostpath","wt")) == NULL ){
//	 	      printf("Error! file couldn't be opened.");
//	 	      return 0;
//	      }
        pathori = cspace->initialPlanningDStar(currentstart, currentgoal);
      DS->racunaoupromjeni=2;//for logging the path in WH
      }else{
        pathori = cspace->replanningDStar(currentstart, currentgoal);
      DS->racunaoupromjeni=1;//for logging the path in WH
      }
      search_time=cspace->time_search;
      search_explored=cspace->explored_nodes; 
 	if (gettimeofday(&timeNow, NULL) == 0)
 	{
 		mySecNow = timeNow.tv_sec;
 		myMSecNow = timeNow.tv_usec / 1000;
 	}
 	vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart)-vremenska_razlika;
 	printf("PL:MainSequence> DStarOri search algorithm %d ms\n", vremenska_razlika);
  if (pathori==NULL){
        cspace->initializeDStar();
        pathori = cspace->initialPlanningDStar(currentstart, currentgoal);
        if (pathori!=NULL){ 
          printf("there exist path and it is not found in replanning!!!!\n");
        }else{
#if 0
          DStarSearchNode* startNode = cspace->getDStarSearchNode(currentstart.x,currentstart.y,currentstart.theta);
//          DStarSearchNode* goalNode = getDStarSearchNode(currentgoal.x,currentgoal.y,currentgoal.theta);
      		std::cerr<<"\ncosts: "<<(startNode)->h<<" PathCostOri: "<<DS->PathCostOri(start_i)<<std::endl;
          std::vector<DStarSearchNode*>* successors = (startNode)->getSuccessors();
          std::vector<OrientationInterval>* successor_intersections = (startNode)->getSuccessorIntersections();
          OrientationInterval intersection;
          for(std::vector<DStarSearchNode*>::iterator it=successors->begin(), end=successors->end(); it!=end; ++it){
            std::cerr<<"succ ("<<((OIDStarSearchNode*) (*it))->x<<","<<((OIDStarSearchNode*) (*it))->y<<","<<((OIDStarSearchNode*) (*it))->o<<")"<<std::endl;
      		intervals = cspace->getAdmissibleOrientations(((OIDStarSearchNode*) (*it))->x,((OIDStarSearchNode*) (*it))->y);
      		std::cerr<<OrientationIntervals::printOrientationIntervals(intervals)<<std::endl;
      		std::cerr<<"cost h "<<(*it)->h<<" desired ori "<<(*it)->getDesiredOrientation()<<std::endl; 
          }
#endif        
        }
  }
      if(pathori!=NULL){  
        if ((path= (I_point *)malloc(pathori->size()*sizeof(I_point)))==NULL) return 2;
        if ((ori= (ori_point *)malloc(pathori->size()*sizeof(ori_point)))==NULL) return 2;
#if (IDEAL_MODEL==0 || 1)
        if (pathori->size()<=2){
//          DW->naCilju=true;
        }
//        else{
//          DW->naCilju=false;
//        }
        else if (DW->naCilju){
          (pathori->at(0))->setDesiredOrientation(currentgoal.theta);
          printf("goal orientation set!\n");
        }
#endif
      	for(unsigned int i=0; i<pathori->size(); i++){
      		x = ((OIDStarSearchNode*) pathori->at(i))->x; 
      		y = ((OIDStarSearchNode*) pathori->at(i))->y;
      		o = ((OIDStarSearchNode*) pathori->at(i))->o;
      		path[i].x=x;
      		path[i].y=y;
      		path[i].th=(pathori->at(i))->getDesiredOrientation();
      		if (i==0) {
      		  path[i].th=currentstart.theta;
      		}
//      		intervals = cspace->getAdmissibleOrientations(x,y);
      		interval = (pathori->at(i))->getOrientationInterval(); //intervals[o];
          
          ori[i].intlo=interval.lower;
          ori[i].intup=interval.upper;
          ori[i].h=(pathori->at(i))->h;
          
          if (i>0){
            if (abs(path[i-1].x-path[i].x)+abs(path[i-1].y-path[i].y)>2){
              printf("what ?");
            }
          }
      		IntPose point((x),(y),path[i].th);
      		if ((cspace->checkCollision(point))){
      		  std::cerr<<"occupied cell on the path!!!"<<std::endl;
        		std::cerr<<x<<","<<y<<","<<(pathori->at(i))->getDesiredOrientation()<<", ["<<interval.lower<<","<<interval.upper<<"]=["<<(pathori->at(i))->getOrientationInterval().lower<<","<<(pathori->at(i))->getOrientationInterval().upper<<std::endl;
      		}

#if 0
if (i==0){
      		std::cerr<<"\ncosts: "<<(pathori->at(i))->h<<" PathCostOri: "<<DS->PathCostOri(path[i])<<" pathori.size() "<<pathori->size()<<std::endl;
      		std::cerr<<x<<","<<y<<","<<(pathori->at(i))->getDesiredOrientation()<<", ["<<interval.lower<<","<<interval.upper<<"]"<<std::endl;
          int findsucc=0;
          std::vector<DStarSearchNode*>* successors = (pathori->at(i))->getSuccessors();
          std::vector<OrientationInterval>* successor_intersections = (pathori->at(i))->getSuccessorIntersections();
          OrientationInterval intersection;
          for(std::vector<DStarSearchNode*>::iterator it=successors->begin(), end=successors->end(); it!=end; ++it){
            std::cerr<<"succ ("<<((OIDStarSearchNode*) (*it))->x<<","<<((OIDStarSearchNode*) (*it))->y<<","<<((OIDStarSearchNode*) (*it))->o<<")"<<std::endl;
      		intervals = cspace->getAdmissibleOrientations(((OIDStarSearchNode*) (*it))->x,((OIDStarSearchNode*) (*it))->y);
      		std::cerr<<OrientationIntervals::printOrientationIntervals(intervals)<<std::endl;
      		std::cerr<<"cost h "<<(*it)->h<<" desired ori "<<(*it)->getDesiredOrientation()<<std::endl;
            if (((pathori->at(i))->getNext()) == (*it)){
              intersection=successor_intersections->at(it-successors->begin());
              std::cerr<<"intersection ["<<intersection.lower<<","<<intersection.upper<<"]"<<std::endl;
              findsucc=1;
              break;
            }
          }
          if (findsucc==0 && i<pathori->size()-1){
            printf("panic!!!!\n");
          }
}
#endif
//      currentNode = (pathori->at(i))->getNext();
//      if (currentNode==NULL || i==0){
//        path[i].th= currentgoal.theta;
//        if (i==0) path[i].th=currentstart.theta;

//      }else{
//      
//        o = ((OIDStarSearchNode*) currentNode)->o;
//        x = ((OIDStarSearchNode*) currentNode)->x;
//        y = ((OIDStarSearchNode*) currentNode)->y;
//        intervals = cspace->getAdmissibleOrientations(x,y);
//        intervalnext = intervals[o];
//        ints.computeIntersection(intervalnext, interval, intersections);
//        
//        int tempth, minD, desth=path[i-1].th;
//        if (1 || currentNode->getNext()==NULL){
//          desth=currentgoal.theta;
//        }
//        for(unsigned int ii=0; ii<intersections.size(); ii++){
//          if (intersections[ii].lower<=intersections[ii].upper){
//            path[i].th=(int) floor((intersections[ii].upper-intersections[ii].lower)/2.)+intersections[ii].lower;
//          }else{
//            path[i].th= (int) floor((intersections[ii].upper+maxOri+1-intersections[ii].lower)/2.)+ intersections[ii].lower;
//            if (path[i].th>maxOri) path[i].th-=(maxOri+1);
//          }
//          if ((1 || currentNode->getNext()==NULL)&&((intersections[ii].lower<=intersections[ii].upper && intersections[ii].lower<=currentgoal.theta && currentgoal.theta<=intersections[ii].upper)||(intersections[ii].lower>intersections[ii].upper && (intersections[ii].lower<=currentgoal.theta || currentgoal.theta<=intersections[ii].upper)))){
//            path[i].th=currentgoal.theta;
//          }
//          if (ii==0){
//            minD=std::min(abs(path[i].th-desth),std::min((abs(path[i].th+maxOri+1-desth)),(abs(path[i].th-maxOri-1-desth))));
//            tempth=path[i].th;
//          }
//          if (ii==1 && minD<std::min(abs(path[i].th-desth),std::min((abs(path[i].th+maxOri+1-desth)),(abs(path[i].th-maxOri-1-desth)))))
//            path[i].th=tempth;
//            
//      }
//        std::cerr<<"Intersections with next interval: "<<ints.printOrientationIntervals(intersections)<<std::endl;
//        std::cerr<<"middle orientation= "<<path[i].th<<std::endl;
//      
//      }
      
//      		path[i].th=interval.lower;
          if (DS->prviput || 1){
#if USE3DCOST
    IntPose pointX((x),(y),interval.lower);
    traversalcostpath = std::max(1., (MAXDISTANCE - floor(cspace->getDistanceInCells(pointX))));
#else
    traversalcostpath = DS->map[x][y].traversal_cost;
  
//ori cost  
if (pathori->size()==1){
      if (OrientationIntervals::getIntervalSize(interval) < maxOri + 1){
        OrientationIntervals::computeOrientationDistanceInInterval(interval,interval.lower,currentstart.theta,minLo);
        OrientationIntervals::computeOrientationDistanceInInterval(interval,interval.upper,currentstart.theta,minUp);
        traversalcostpath = maxOri+1 - 2*std::min(minLo,minUp);
      }else{
        traversalcostpath=1;
      }

}
if (i<pathori->size()-1){
  DStarSearchNode* xNode = (pathori->at(i));
  DStarSearchNode* yNode = (pathori->at(i+1));
  
  std::vector<OrientationInterval> intersects;
  OrientationInterval intersection;
  OrientationIntervals::computeIntersection(xNode->getOrientationInterval(), yNode->getOrientationInterval(), intersects);
  if (intersects.size()==0){
    printf("no intersection on path point i=%d\n",i);
  }else{
  if (OrientationIntervals::containsOrientation(intersects[0],xNode->getDesiredOrientation())){
    intersection=intersects[0];
  }else{
    intersection=intersects[1]; 
  }
//  if (i>0){
//  OrientationIntervals::computeOrientationDistanceInInterval(xNode->getOrientationInterval(),xNode->getDesiredOrientation(),(pathori->at(i-1))->getDesiredOrientation(),minD);//both angles are contained always in the next node interval but the opposite does not hold
//  }else{
//  OrientationIntervals::computeOrientationDistanceInInterval(xNode->getOrientationInterval(),xNode->getDesiredOrientation(),currentstart.theta,minD); 
//  }
      if (OrientationIntervals::getIntervalSize(intersection) < maxOri + 1){
        OrientationIntervals::computeOrientationDistanceInInterval(intersection,intersection.lower,xNode->getDesiredOrientation(),minLo);
        OrientationIntervals::computeOrientationDistanceInInterval(intersection,intersection.upper,xNode->getDesiredOrientation(),minUp);
        traversalcostpath = maxOri+1 - 2*std::min(minLo,minUp);
      }else{
        traversalcostpath=1;
      }
      }
//      std::cerr<<"intersection with the next node ["<<intersection.lower<<","<<intersection.upper<<"]"<<std::endl;
//			std::cerr<<"print intersections "<<OrientationIntervals::printOrientationIntervals(intersects)<<std::endl;
//      std::cerr<<"traversal cost of the arc is "<<traversalcostpath<<std::endl;

}
//      if (i>0) {
//        DS->arc_costOri(path[i],path[i-1]);
//        std::cerr<<"traversal cost of the arc calc by arc_costOri for i-1 "<<DS->c<<std::endl;
//        DS->arc_costOri(path[i-1],path[i]);
//        std::cerr<<"c opposite for i-1 "<<DS->c<<std::endl;
//      }
#endif
      ori[i].travcost=traversalcostpath;
    if (i>0){
//      fprintf(logfile,"%d %d %d %d\n",traversalcostpath, (pathori->at(i))->h, interval.lower, interval.upper);
    }else{
  OrientationIntervals::computeOrientationDistanceInInterval((pathori->at(i))->getOrientationInterval(),(pathori->at(i))->getDesiredOrientation(),currentstart.theta,minLo); 
      DS->travcost3d=traversalcostpath;
  
//      printf("minLo=%d h=%d, h next=%d, correction cost=%d\n", minLo, (pathori->at(i))->h, (pathori->at(i+1))->h, (COSTROTATION*(minLo)));
      ori[i].h=(pathori->at(i))->h+(COSTROTATION*(minLo));
//      fprintf(logfile,"%d %d %d %d\n",traversalcostpath, (pathori->at(i))->h+(COSTROTATION*(minLo)), interval.lower, interval.upper);    
    }
          }  
      		assert(std::isinf(((OIDStarSearchNode*) pathori->at(i))->costs)==false);
      	}
//      	if (DS->prviput) fclose(logfile);
      	path_length=pathori->size();
      }else{
        std::cerr<<"PL> no path!"<<std::endl; 
      DS->racunaoupromjeni=0;//for logging the path in WH
            return 2; 
      }

#else
    	if (DS->SearchPathOri()){
    		printf("jupi\n");
    	}
      if (DS->prviput)
    	      DS->racunaoupromjeni=2;//for logging the path in WH

	std::cout<<"cspace_occupied at start? "<<DS->map[ start_i.x ][ start_i.y ].cspace_occupied<<std::endl;
    	if((path=(DS->getPathOri()))==NULL){
            printf("Planner> DStar path is NULL!\n");
            return 2;
        }
        path_length=DS->getPathLength();
        if ((ori= (ori_point *)malloc(path_length*sizeof(ori_point)))==NULL) return 2;
      for (int i=0; i<path_length; i++){
        if (i<path_length-1)
          DS->arc_costOri(path[i+1],path[i]);
        ori[i].travcost=DS->c;
        ori[i].h=DS->map[path[i].x][path[i].y].h_cost_intOri[path[i].th];

//        printf("%d. (%d,%d,%d) full3D cost=%d\n",i,path[i].x,path[i].y,path[i].th, DS->map[path[i].x][path[i].y].h_cost_intOri[path[i].th]);
      }

#endif
    }
#endif    
//--------------------------------------------------------	
 	if (gettimeofday(&timeNow, NULL) == 0)
 	{
 		mySecNow = timeNow.tv_sec;
 		myMSecNow = timeNow.tv_usec / 1000;
 	}
 	vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
 	printf("PL:MainSequence> before calling the DStar algorithm %d ms\n", vremenska_razlika);
//    printf("Planner::trenutna pozicija (%d,%d)\n",start_i.x,start_i.y);
#if DSTAR_REVERSE
    if (DS->prviput){
	    if (DS->SearchPathReverse()){
		    printf("prviput eto ga\n");
	    }else{
		    printf("nesto ne stima\n");
	    }
/*	    if (gettimeofday(&timeNow, NULL) == 0)
	    {
		    mySecNow = timeNow.tv_sec;
		    myMSecNow = timeNow.tv_usec / 1000;
	    }
	    vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
*/
    }
#endif
#if (DSTAR && (DSTAR3D==0))
    if(DS->SearchPath())
    {
          search_time=DS->izracuni[DS->brojac_izracuna-1]/1000.;
          search_explored=DS->azurirani_polje[DS->brojac_izracuna-1];

          if((path=(DS->getPath()))==NULL){
            printf("Planner> DStar path is NULL!\n");
   	        DS->racunaoupromjeni=0;//for logging the path in WH
            return 2;
          }
          path_length=DS->getPathLength();
#if (DSTAR_REVERSE==0) //this is for logging each time
          if (DS->prviput){
    	      DS->racunaoupromjeni=2;//for logging the path in WH
          
          }else{
    	      DS->racunaoupromjeni=1;//for logging the path in WH
          
          }
#endif
#if (IDEAL_MODEL==0)
        if (path_length<=1){
//          DW->naCilju=true;
        }
#endif

        if ((ori= (ori_point *)malloc(path_length*sizeof(ori_point)))==NULL) return 2;
      for (int i=0; i<path_length; i++){
        if (i<path_length-1)
          DS->arc_cost(path[i+1].x,path[i+1].y,path[i].x,path[i].y);
        ori[i].travcost=DS->c;
        ori[i].h=DS->map[path[i].x][path[i].y].h_cost_int;

      }

 //          printf("Planner> Duzina pronadjenog puta: path_length=%d",path_length);
//           for(int i=0;i<path_length;i++){
//           printf("Planner>   path[%d].x=%d,   path[%d].y=%d", i, path[i].x,i, path[i].y );
//           }
/*	  if (DS->racunaoupromjeni){
		  printf("replanirao\n");
	  }*/
#if DSTAR_REVERSE
		  if (DS->SearchPathReverse()){
			  printf("eto ga\n");
			  int minf=DS->map[DS->GoalRacunac.x][DS->GoalRacunac.y].total_cost_int;
			  if ((DS->racunaoupromjeni) && (DS->map[DS->StartRacunac.x][DS->StartRacunac.y].total_cost_int!=minf)){
			  	printf("StartRacunac ima veci cost. Racunam ponovo.\n");
			  	DS->resetReverse();
			  	if (DS->SearchPathReverse()){
			  		printf("eto ga\n");
			  	}else{
			  		printf("nesto ne stima\n");
			  	}
			  	
			  }
		  }else{
			  printf("nesto ne stima\n");
		  }
		  if((path=(DS->getPathWitkowski()))==NULL){
			  printf("Planner> Putanja witkowski je NULL!\n");
			  return 2;
		  }
		  path_length=DS->getPathLengthWitkowskiSegment();
		  printf("PL path_length=%d\n",path_length);
/*		  if (gettimeofday(&timeNow, NULL) == 0)
		  {
			  mySecNow = timeNow.tv_sec;
			  myMSecNow = timeNow.tv_usec / 1000;
		  }
		  vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
		  printf("PL:SekvIzv> prije pozivanja replaniranja EStar algoritma %d ms\n", vremenska_razlika);
*/
		  
#endif


#if KOMBINACIJA
	  DS->SearchPathWitkowski();
	  if((path=(DS->getPathWitkowski()))==NULL){
		  printf("Planner> Putanja witkowski je NULL!\n");
		  return 2;
	  }
	  path_length=DS->getPathLengthWitkowskiSegment();
#endif
    }else{
        printf("PLMainSequence> DStar has no path!\n");
        return 2;
    }
#endif
#if (DSTAR==0)
    if(DS->SearchPathWitkowski())
{
	if((path=(DS->getPath_backward()))==NULL){
		printf("Planner> Putanja backward je NULL!\n");
		return 2;
	}
// 	DS->getPath_backward();
	DS->getPath_forward();
	path_length=DS->getPathLength_backward();
 //          printf("Planner> Duzina pronadjenog puta: path_length=%d",path_length);
//           for(int i=0;i<path_length;i++){
//           printf("Planner>   path[%d].x=%d,   path[%d].y=%d", i, path[i].x,i, path[i].y );
//           }
}else{
	printf("Planner Sekvenca> Witkowski nema path! \n"); 
	return 2;
}
#endif   
//testing interpolation
#if 0
      if (DS->prviput){
        if ( (logfile = fopen("logger//interpolatedcost","wt")) == NULL ){
	 	      printf("Error! file couldn't be opened.");
	 	      return 0;
	      }
	      double x,y,th,S; 
//	      double xlow=3500., xhigh=3800., ylow=2600., yhigh=2900., thlow=90., thhigh=270.; //interesting part
//	      double xlow=2100., xhigh=2400., ylow=1900., yhigh=2200., thlow=0., thhigh=180.; //near the goal
//	      double xlow=13200., xhigh=13500., ylow=18500., yhigh=18800., thlow=0., thhigh=90.; //interesting lab78
//	      double xlow=5000., xhigh=5300., ylow=11900., yhigh=12200., thlow=-90., thhigh=90.; //interesting lab78
//	      double xlow=5800., xhigh=6100., ylow=2900., yhigh=3200., thlow=45., thhigh=215.; 
//	      double xlow=6300., xhigh=6600., ylow=3700., yhigh=4000., thlow=65., thhigh=215.; //simetrija
//	      double xlow=5500., xhigh=5800., ylow=1200., yhigh=1500., thlow=270., thhigh=90.; 
//	      double xlow=0., xhigh=GM->Map_Dim_X_A, ylow=0., yhigh=GM->Map_Dim_Y_A, thlow=0., thhigh=0.;
//	      double xlow=3300., xhigh=3600., ylow=3300., yhigh=3600., thlow=0., thhigh=90.; //smap
	      double xlow=20900., xhigh=21100., ylow=5900., yhigh=6200., thlow=0., thhigh=90.; //smap
//	      double xlow=900., xhigh=1200., ylow=7700., yhigh=8000., thlow=-90., thhigh=90.;//corridor goal
//	      double xlow=5500., xhigh=5800., ylow=3200., yhigh=3500., thlow=0., thhigh=100.;//corridor entrance passage
//	      double xlow=9100., xhigh=9400., ylow=1400., yhigh=1700., thlow=90., thhigh=190.;//corridor start
	      int numx=100, numy=100, numth=100;
	      FILE *logfilex, *logfiley, *logfileth, *logfileS;
        if ( (logfilex = fopen("logger//interpolatedx","wt")) == NULL || (logfiley = fopen("logger//interpolatedy","wt")) == NULL || (logfileth = fopen("logger//interpolatedth","wt")) == NULL || (logfileS = fopen("logger//interpolatedS","wt")) == NULL){
	 	      printf("Error! file couldn't be opened.");
	 	      return 0;
	      }

        for (int iy=0; iy<numy; iy++){
        for (int ith=0; ith<numth; ith++){
	      for (int ix=0; ix<numx; ix++){
	            
	            th=(ith/((double)(numth))*(thhigh-thlow)+thlow)*M_PI/180.;
	            x=ix/((double)(numx))*(xhigh-xlow)+xlow;
	            y=iy/((double)(numy))*(yhigh-ylow)+ylow;
              S=DW->computeInterpolatedCost(x,y,th);
//              printf("S = %f ",S);
//              S=DW->computeTau(x,y,th);
              if (S<OBSTACLE/COSTSTRAIGHT){
                fprintf(logfile,"%f %f %f %f\n",x,y,th,S);
                
	            }else{
	              S=-1.;
	            }

              if (iy==0 && ith==0) fprintf(logfilex,"%f\n",x);
              if (ix==0 && ith==0) fprintf(logfiley,"%f\n",y);
              if (ix==0 && iy==0) fprintf(logfileth,"%f\n",th);
                
              fprintf(logfileS,"%f ",S);

	          }
	        }
	        fprintf(logfileS,"\n");
	      }
        fprintf(logfileS,"\n");
        fclose(logfile);
        fclose(logfilex);
        fclose(logfiley);
        fclose(logfileth);
        fclose(logfileS);
      }
#endif
//comparing full 3D and ori search
#if 0
//      DS->Goal.x=158; DS->Goal.y=184; DS->Goal.th=100;
if (DS->prviput){
    	if (DS->SearchPathOri()){
    		printf("jupi\n");
    	}
         
      for (int i=0; i<path_length; i++){
        printf("%d. (%d,%d,%d) full3D cost=%d\n",i,path[i].x,path[i].y,path[i].th, DS->map[path[i].x][path[i].y].h_cost_intOri[path[i].th]);
      }
#if 1   
    	if((path=(DS->getPathOri()))==NULL){
            printf("Planner> DStar path is NULL!\n");
            return 2;
      }
      path_length=DS->getPathLength();
      for (int i=0; i<path_length; i++){
        if (i<path_length-1)
          DS->arc_costOri(path[i+1],path[i]);
        ori[i].travcost=DS->c;
        ori[i].h=DS->map[path[i].x][path[i].y].h_cost_intOri[path[i].th];

        printf("%d. (%d,%d,%d) full3D cost=%d\n",i,path[i].x,path[i].y,path[i].th, DS->map[path[i].x][path[i].y].h_cost_intOri[path[i].th]);
      }
      
#endif
    int maxOri=OrientationIntervals::getMaxOrientation();
    int h, cnt=0, freecnt=0, maxdiff=0, minLo;
    double maxreldiff=0, tempreldiff, avereldiff=0;
    for (int i=0; i<star_size_x; i++){
      for (int j=0; j<star_size_y; j++){
        for (int k=0; k<maxOri+1; k++){
        	IntPose temp(i,j,k); 
       	  DStarSearchNode* currentNode = cspace->getDStarSearchNode(temp.x,temp.y,temp.theta);
          if (currentNode==NULL) {
            h=OBSTACLE;
          } else{ 

       	  DStarSearchNode* nextNode = currentNode->getNext();
       	  minLo=0; 
          if (nextNode!=NULL || (i==global_goal_i.x && j==global_goal_i.y)){
  OrientationIntervals::computeOrientationDistanceInInterval((currentNode)->getOrientationInterval(),(currentNode)->getDesiredOrientation(),temp.theta,minLo); 
            }
  
            h=currentNode->h + COSTROTATION*(minLo);
//            if (currentNode->getDesiredOrientation()!=temp.theta) continue; 
            freecnt++;
          } 
          if (DS->map[temp.x][temp.y].h_cost_intOri[temp.theta]!=h){
            cnt++;
            printf("temp=(%d,%d,%d) full3D cost=%d, ori cost=%d,   diff=%d\n",i,j,k,DS->map[temp.x][temp.y].h_cost_intOri[temp.theta],h, h-DS->map[temp.x][temp.y].h_cost_intOri[temp.theta]);
            tempreldiff=abs(DS->map[temp.x][temp.y].h_cost_intOri[temp.theta]-h)/(double)DS->map[temp.x][temp.y].h_cost_intOri[temp.theta];

            avereldiff+=tempreldiff;

            if (abs(DS->map[temp.x][temp.y].h_cost_intOri[temp.theta]-h)>maxdiff)
              maxdiff=abs(DS->map[temp.x][temp.y].h_cost_intOri[temp.theta]-h);
            if (tempreldiff>maxreldiff)
              maxreldiff=tempreldiff;
          }
        }
      }
    }
    printf("number of nodes with different values %d, out of total non obstacle nodes %d, maxdiff=%d, maxreldiff=%f, avereldiff=%f\n",cnt,freecnt,maxdiff,maxreldiff,(double)avereldiff/(double)cnt);
}
#endif

    //path se nalazi u planneru i to polje integera
    if(!IntToRealPath(GM->Map_Dim_X,GM->Map_Dim_Y, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM))
      {
        printf("PlannerMainSequence> Failed path conversion from int to real!\n");
        return 2;
      }
       return 1;
}


int Planner::RealToInt(R_point &real_point, I_point &int_point, double map_size_x, double map_size_y, double map_home_x, double map_home_y, double cell_dim)
{
  double x,y;
  x=real_point.x-map_home_x;
  y=real_point.y-map_home_y;
 if((x>=0.0)&&(x<=map_size_x)&&(y>=0.0)&&(y<=map_size_y)){
      
      int_point.x=(int)floor(x/cell_dim);
      int_point.y=(int)floor(y/cell_dim);
    return 1;
  }else{
    printf("Planner RealToInt> Banana-- x=%f, y=%f, map_size_x=%f, map_size_y=%f", x, y, map_size_x, map_size_y);
    return 2;
  }
}

int Planner::RealToReal(R_point &real_point, R_point &r_point, double map_size_x, double map_size_y, double map_home_x, double map_home_y, double cell_dim)
{
  double x,y;
  x=real_point.x-map_home_x;
  y=real_point.y-map_home_y;
 if((x>=0.0)&&(x<=map_size_x)&&(y>=0.0)&&(y<=map_size_y)){
      
      r_point.x=(x/cell_dim)-0.5;
      r_point.y=(y/cell_dim)-0.5;
    return 1;
  }else{
    printf("Planner RealToInt> Banana-- x=%f, y=%f, map_size_x=%f, map_size_y=%f", x, y, map_size_x, map_size_y);
    return 2;
  }
}

int Planner::IntToReal(I_point &int_point, R_point &real_point, int map_size_x, int map_size_y, double map_home_x, double map_home_y, double cell_dim)
{
  //STAVILI SMO ELEMENT NA SREDINU CELIJE!!!!
	if((int_point.x>=0)&&(int_point.x<map_size_x)&&(int_point.y>=0)&&(int_point.y<map_size_y)){
      real_point.x=map_home_x+int_point.x*cell_dim+0.5*cell_dim;
      real_point.y=map_home_y+int_point.y*cell_dim+0.5*cell_dim;
#if RECTANGULAR
      real_point.th=cspace->mapToWorldTheta(int_point.th);
#else
      real_point.th=int_point.th*M_PI/180.;
#endif
    return 1;
  }else{
    printf("Planner IntToRealPath > IntToReal conversion failed!\n");
    return 2;
  }
}

int Planner::IntToRealPath(int map_size_x, int map_size_y, double map_home_x, double map_home_y, double cell_dim)
{

    I_point i_point_temp;
//     R_point razlika;
    //drugim rijecima, rezerviramo samo za max memorije i ne prelazimo preko toga
    if(path_length>path_length_max){
    	//reallociramo realnu putanju
    	path_r=(R_point*)realloc(path_r,sizeof(R_point)*path_length);
	path_length_max=path_length;
     }
    for(int i=0;i<path_length;i++){
      i_point_temp=path[i];
    if(!IntToReal(i_point_temp, path_r[i], map_size_x, map_size_y,map_home_x, map_home_y, cell_dim))
    {
       printf("Planner IntToRealPath > element path[%d].x=%d , path[%d].y=%d", i, path[i].x,i, path[i].y);
       printf("Planner IntToRealPath > IntToReal conversion failed!");
      return 2;
    }
#if (RECTANGULAR==0)
    if (i==path_length-1) {
      path_r[i].th=WH->global_goal_workhorse.th;
    }
#endif
  }
    return 1;
}

void Planner::Puni(int i, int j, int star_size_x, int star_size_y, int robot_mask, int cost_mask){
	int index_x, index_y, cost;//i cost maska
	I_point temp;
	DStarCell **star_map=DS->GetMap();
	  //dodavanje ekstra prepreka
#if 0
  star_map[13][24].prepreka_bool=true;
  star_map[13][24].traversal_cost=OBSTACLE;
  star_map[14][24].prepreka_bool=true;
  star_map[14][24].traversal_cost=OBSTACLE;
  star_map[14][26].prepreka_bool=true;
  star_map[14][26].traversal_cost=OBSTACLE;
  star_map[15][26].prepreka_bool=true;
  star_map[15][26].traversal_cost=OBSTACLE;
  for(int i=0; i<7; i++){
    star_map[12][22+i].prepreka_bool=true;
    star_map[12][22+i].traversal_cost=OBSTACLE;
    star_map[16][22+i].prepreka_bool=true;
    star_map[16][22+i].traversal_cost=OBSTACLE;
  }
  for(int i=0; i<3; i++){
    star_map[13+i][22].prepreka_bool=true;
    star_map[13+i][22].traversal_cost=OBSTACLE;
    star_map[13+i][28].prepreka_bool=true;
    star_map[13+i][28].traversal_cost=OBSTACLE;
  }
#endif

	if (!((star_map[i][j].prepreka_bool==true)&&(star_map[i][j]._maska.x==i)&&(star_map[i][j]._maska.y==j))) {//ako to nije znaci da je oko ove prepreke vec radjena maska prepreke
		star_map[i][j]._maska.x=i;
		star_map[i][j]._maska.y=j;//ova je glavna
		for(int k=-robot_mask-cost_mask;k<=robot_mask+cost_mask;k++){
			for(int l=-robot_mask-cost_mask;l<=robot_mask+cost_mask;l++){
				temp.x=i+k; temp.y=j+l;
				if((temp.x>=0)&& (temp.x<star_size_x)&&(temp.y>=0)&& (temp.y<star_size_y)){
					if (star_map[temp.x][temp.y].prepreka_bool==false){//ovaj uvjet se pita tu gore jer se maska i cost maska radi samo za slobodna polja
//						if (((abs(k)<=robot_mask-1)&&(abs(l)<=robot_mask-1))||(abs(k)+abs(l)==robot_mask)) {//za robotovu masku...
//						if ((abs(k)<=robot_mask)&&(abs(l)<=robot_mask)){//za robotovu masku...
						if (0||((abs(k)<=robot_mask)&&(abs(l)<=robot_mask)&&(abs(k)+abs(l)<2*robot_mask))){//za robotovu masku...

							//jos jedan uvjet o udaljenosti prepreke od centra polja, ruzno je jer koristim iz GM klase a ne preko ulaznih argumenata ali ne da mi se
							double udaljenost=0.;
							if (0){
								udaljenost=sqrt((i-temp.x)*(i-temp.x)+(j-temp.y)*(j-temp.y))*GM->Map_Cell_Size;
							}
							if ((udaljenost<RR+50.)) //50 za ushape i S, 10 za corridor
							{
							star_map[temp.x][temp.y].prepreka_bool=true;
							star_map[temp.x][temp.y].traversal_cost=OBSTACLE;
							star_map[temp.x][temp.y].traversal_cost_stari=OBSTACLE;//tu odmah pisem starog jer bu se ovaj ubacio zbog prepreke
							if ((DS->prviput==0) && (star_map[temp.x][temp.y].promjena!=2))
							{
// 								DS->listapromjena[DS->NumElemPromjena]=temp;
// 								DS->NumElemPromjena++;
								DS->listapunjenja[DS->NumElemPunjenja]=temp;
								DS->NumElemPunjenja++;
								DS->promjena=1;
								star_map[temp.x][temp.y].promjena=2; //ali cemo mu promijeniti oznaku s praznjenja na punjenje
							}
							star_map[temp.x][temp.y].preprekaokolo++; //prva prepreka u robot maski, ima samo jedna prepreka u robot maski oko ovog polja ili je to bas to polje
							star_map[temp.x][temp.y].preprekablizu++; //unutar robot+cost maske
							}else{
#if (DEBEL<2)
  							cost=EMPTYC+COST_MASK+DEBEL;
#else
  							cost=EMPTYC*pow(DEBEL,(COST_MASK-std::min(COST_MASK,int((udaljenost-RR-50.)/GM->Map_Cell_Size))));//50 za ushape i s, a 10 za coridor
#endif
  							if (cost>star_map[temp.x][temp.y].traversal_cost) {
  								star_map[temp.x][temp.y].traversal_cost=cost;
  							  star_map[temp.x][temp.y].traversal_cost_stari=cost;
  							}
								star_map[temp.x][temp.y].preprekablizu++; //unutar robot+cost maske
							}
 // 2 oznacuje promjenu na prepreku, a 1 promjenu na prazno; 0 - nema prepreke
							
						} else {//izvan robotove maske a unutar cost maske
							index_x=std::max((abs(k)-robot_mask-1), 0);
							index_y=std::max((abs(l)-robot_mask-1), 0);
							if (1&&(abs(k)>robot_mask)&&(abs(l)>robot_mask)) {//za dijagonalne drugacije
								index_x=std::max((abs(k)-robot_mask+abs(k)-robot_mask-1-abs(abs(k)-abs(l))), 0);
								index_y=std::max((abs(l)-robot_mask+abs(l)-robot_mask-1-abs(abs(k)-abs(l))), 0);
							}
	//pri tome uzimamo max od x,y smjera da se lijepo poslozi
#if (DEBEL<2)
							cost=EMPTYC+COST_MASK-std::max(index_x,index_y)+DEBEL;
#else
//							cost=EMPTYC+COST_MASK*DEBEL-DEBEL*std::max(index_x,index_y);
							cost=EMPTYC*pow(DEBEL,(COST_MASK-std::max(index_x,index_y)))+LOW_COST;
#endif
							if (cost>star_map[temp.x][temp.y].traversal_cost) {
								star_map[temp.x][temp.y].traversal_cost=cost;
								if (DS->prviput){
									star_map[temp.x][temp.y].traversal_cost_stari=cost;//tu postavlja i stari u inicijalnom slucaju
								}
							}
							star_map[temp.x][temp.y].preprekablizu++;//koliko ih ima blizu, dalje od robot maske, ali unutar costmaske
							if ((DS->prviput==0)&&(star_map[temp.x][temp.y].traversal_cost!=star_map[temp.x][temp.y].traversal_cost_stari)&&(star_map[temp.x][temp.y].promjena!=2)&&(star_map[temp.x][temp.y].promjena!=1))
							{//gledamo i promjenu costmaske, stavljamo to na punjenje (tak je svejedno kad i jedne i druge isto ubacujem u d* mapu)
								//moguce je da ce se vise puta mijenjati traversal_cost pa treba provjeriti u DStar.cpp da li je razlicit
								DS->listacostmaska[DS->NumElemCostmaska]=temp;
								DS->NumElemCostmaska++;
								DS->promjena=1;
								star_map[temp.x][temp.y].promjena=2; //ali cemo mu promijeniti oznaku s praznjenja na punjenje
							}
						}//izvan robotove maske a unutar cost maske
					}//prepreka false
					else {//ako je prepreka true naznaci preklapanje u robotovoj maski
//						if (((abs(k)<=robot_mask-1)&&(abs(l)<=robot_mask-1))||(abs(k)+abs(l)==robot_mask)) {
						if (0||(abs(k)<=robot_mask)&&(abs(l)<=robot_mask)&&(abs(k)+abs(l)<2*robot_mask)){
//						if ((abs(k)<=robot_mask)&&(abs(l)<=robot_mask)){
							star_map[temp.x][temp.y].preprekaokolo++;//broji koliko ima prepreka okolo
						}
						star_map[temp.x][temp.y].preprekablizu++;//koliko ih ima blizu, unutar cost+robot maske
					}
				}//unutar mape
			}//for
		}//for po maskama
	}//ako nije vec oko te prepreke radjena maska prepreka
}


void Planner::Prazni(int i, int j,I_point *pamtiindekse,int *brojpam, int star_size_x, int star_size_y, int robot_mask, int cost_mask, bool startzauzet){
	I_point temp;
	DStarCell **star_map=DS->GetMap();
	if ((star_map[i][j].prepreka_bool==true)&&(star_map[i][j]._maska.x==i)&&(star_map[i][j]._maska.y==j)) {//samo se glavne brisu
		star_map[i][j]._maska.x=-1*i;
		star_map[i][j]._maska.y=-1*j;//ova je glavna za brisanje
		if (startzauzet)
			GM->Map[i][j].occupancy=0;//i u GM treba obrisati startnu poziciju ako se o njoj radi, a ne smije se brisati ako se radi o pokretnoj
		for(int k=-robot_mask-cost_mask;k<=robot_mask+cost_mask;k++){
			for(int l=-robot_mask-cost_mask;l<=robot_mask+cost_mask;l++){
				temp.x=i+k; temp.y=j+l;
				if((temp.x>=0)&& (temp.x<star_size_x)&&(temp.y>=0)&& (temp.y<star_size_y))
				{
				  //unutar robot maske
					if (0||(abs(k)<=robot_mask)&&(abs(l)<=robot_mask)&&(abs(k)+abs(l)<2*robot_mask))
//					if ((abs(k)<=robot_mask)&&(abs(l)<=robot_mask))
					{ 
						star_map[temp.x][temp.y].preprekaokolo--;
						star_map[temp.x][temp.y].preprekablizu--;
						if (star_map[temp.x][temp.y].prepreka_bool==true) {
							if (star_map[temp.x][temp.y].preprekaokolo==0) {//gledamo prvo one bez preklapanja
						  //ako je maska prepreka bas zbog i,j tog polja onda ga brisemo
								star_map[temp.x][temp.y].prepreka_bool=false;
								star_map[temp.x][temp.y].traversal_cost=EMPTYC;
								star_map[temp.x][temp.y].traversal_cost_stari=EMPTYC;//tu odmah postavlja stari jer je u praznjenje slucaju
								if (star_map[temp.x][temp.y].preprekablizu!=0){
									pamtiindekse[*brojpam]=temp;//pamti indekse polja za koje treba ponovo racunati costmasku
									(*brojpam)++;
								}else{//trazim dal sam ovo polje vec prije upisala u pomocno da ga izbacim van
									for (int z=0;z<*brojpam;z++){
										if ((pamtiindekse[z].x==temp.x)&&(pamtiindekse[z].y==temp.y)){
										pamtiindekse[z]=pamtiindekse[--(*brojpam)];//brise ga iz polja
										break;
										}
									}	  
								}

 // 2 oznacuje promjenu na prepreku, a 1 promjenu na prazno; 0 - nema prepreke
								if ((DS->prviput==0) && (star_map[temp.x][temp.y].promjena!=1))
								{
									star_map[temp.x][temp.y].promjena=1;
									DS->promjena=1; //postavljamo oznaku za promjenu
// 									DS->listapromjena[DS->NumElemPromjena]=temp;
// 									DS->NumElemPromjena++;
									DS->listapraznjenja[DS->NumElemPraznjenja]=temp;
									DS->NumElemPraznjenja++;
								}
							}//bez preklapanja
						  //inace nista
						}//ako je prepreka true
					  //false slucaj ne bi smio postojati! pa to mozes provjeriti
						else {
//							printf("PL something is wrong!\n");
						}
					} //unutar maske...
					else {//izvan robot maske, ali unutar cost maske
						star_map[temp.x][temp.y].preprekablizu--;
						if (star_map[temp.x][temp.y].prepreka_bool==false){ //naravno mora biti slobodno za racunanje costmaske
							if (star_map[temp.x][temp.y].preprekablizu==0){//ako je ova prepreka jedina bila blizu ove onda odmah postavljamo traversal_cost
								star_map[temp.x][temp.y].traversal_cost=EMPTYC;
								if ((DS->prviput==0)&&(star_map[temp.x][temp.y].traversal_cost!=star_map[temp.x][temp.y].traversal_cost_stari)&&(star_map[temp.x][temp.y].promjena!=1)&&(star_map[temp.x][temp.y].promjena!=2))
								{//gledamo i promjenu costmaske, stavljamo to na praznjenje (tak je svejedno kad i jedne i druge isto ubacujem u d* mapu)
								//moguce je da ce se vise puta mijenjati traversal_cost pa treba provjeriti u DStar.cpp da li je razlicit
									DS->listacostmaska[DS->NumElemCostmaska]=temp;
									DS->NumElemCostmaska++;
									DS->promjena=1;
									star_map[temp.x][temp.y].promjena=2; //ali cemo mu promijeniti oznaku s praznjenja na punjenje
								}
	//trazim dal sam ovo polje vec prije upisala u pomocno da ga izbacim van
								for (int z=0;z<*brojpam;z++){
									if ((pamtiindekse[z].x==temp.x)&&(pamtiindekse[z].y==temp.y)){
										pamtiindekse[z]=pamtiindekse[--(*brojpam)];//brise ga iz polja
										break;
									}
								}
							}else{
								int duplic=0;
								for (int z=0;z<*brojpam;z++){
									if ((pamtiindekse[z].x==temp.x)&&(pamtiindekse[z].y==temp.y)){
										duplic=1;
										break;
									}
								}
								if (!duplic){
									pamtiindekse[*brojpam]=temp;//pamti indekse polja za koje treba ponovo racunati costmasku
									(*brojpam)++;
								}
							}
						}
					}
				}//unutar mape
			}//for
		}//for
	}//if
/*	if (*brojpam>0){
		printf("mijau00");
	}*/
}


void Planner::PrazniCostMasku(I_point *pamtiindekse, int brojpam, int star_size_x, int star_size_y, int cost_mask){
	DStarCell **star_map=DS->GetMap(); 
	int index_x, index_y, cost;//i cost maska
	I_point temp;
	int temp_m, temp_n;
	for (int k=0;k<brojpam;k++){
		temp.x=pamtiindekse[k].x;
		temp.y=pamtiindekse[k].y;
	
					  //mora biti odgovarajuca vrijednost traversal_costa pa racunam ponovo kao gore costmask samo obrnutim putem, al sve je simetricno pa moze tak
					  //prvo ide inicijalizacija na empti, ali ak nije obstacle, bu ga poravnal kak treba
		star_map[temp.x][temp.y].traversal_cost=EMPTYC;
		for(int kk=-cost_mask;kk<=cost_mask;kk++){
			for(int ll=-cost_mask;ll<=cost_mask;ll++){
				temp_m=temp.x+kk; temp_n=temp.y+ll;
				if((temp_m>=0)&& (temp_m<star_size_x)&&(temp_n>=0)&& (temp_n<star_size_y)) {
					if (star_map[temp_m][temp_n].prepreka_bool==true){
						index_x=std::max((abs(kk)-1), 0);
						index_y=std::max((abs(ll)-1), 0);
						if (1&&(abs(kk)>0)&&(abs(ll)>0)) {//za dijagonalne drugacije
							index_x=std::max((abs(kk)+abs(kk)-1-abs(abs(kk)-abs(ll))), 0);
							index_y=std::max((abs(ll)+abs(ll)-1-abs(abs(kk)-abs(ll))), 0);
						}  
	//pri tome uzimamo max od x,y smjera da se lijepo poslozi
#if (DEBEL<2)
							cost=EMPTYC+COST_MASK-std::max(index_x,index_y)+DEBEL;
#else
//							cost=EMPTYC+COST_MASK*DEBEL-DEBEL*std::max(index_x,index_y);
							cost=EMPTYC*pow(DEBEL,(COST_MASK-std::max(index_x,index_y)))+LOW_COST;
#endif
						star_map[temp.x][temp.y].traversal_cost=std::max(star_map[temp.x][temp.y].traversal_cost,cost);
						if ((DS->prviput==0)&&(star_map[temp.x][temp.y].traversal_cost!=star_map[temp.x][temp.y].traversal_cost_stari)&&(star_map[temp.x][temp.y].promjena!=1)&&(star_map[temp.x][temp.y].promjena!=2))
						{//gledamo i promjenu costmaske, stavljamo to na praznjenje (tak je svejedno kad i jedne i druge isto ubacujem u d* mapu)
								//moguce je da ce se vise puta mijenjati traversal_cost pa treba provjeriti u DStar.cpp da li je razlicit
							DS->listacostmaska[DS->NumElemCostmaska]=temp;
							DS->NumElemCostmaska++;
							DS->promjena=1;
							star_map[temp.x][temp.y].promjena=2; //ali cemo mu promijeniti oznaku s praznjenja na punjenje
						}

					}
				}
			}
		}
	}
}
