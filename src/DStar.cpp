#include "DStar.h"  //Params.h
#include "GridMap.h"

extern GridMap *GM;
#if RECTANGULAR
#include <cspacevoronoi.h>
extern CSpaceVoronoi *cspace;
#endif






int DStar::Init( int start_x, int start_y, int goal_x, int goal_y)
{

	racunaoupromjeni=0;
    time_stamp_counter++;       //azuriranje trenutnog time_stampa
//     printf("DStar::Init> time_stamp_counter=%d\n",time_stamp_counter);
    Start.x=start_x; Start.y=start_y;         //indexi cilja
    Goal.x=goal_x; Goal.y=goal_y;         //indexi cilja
   if(IsValid( start_x, start_y )==0){       //pozicija nije u mapi
    printf("DStar Init> Start out of map!\n");
    return 0;
    }
    travcost=map[Start.x][Start.y].traversal_cost;
#if RECTANGULAR
    IntPose pointX((Start.x),(Start.y),Start.th);
    travcost3d = std::max(1., (MAXDISTANCE - floor(cspace->getDistanceInCells(pointX))));
#endif
    if(IsValid( start_x, start_y )==2){       //pozicija je zauzeta
		printf("DStar Init> Start occupied!\n");
		Start_zauzet=true;
// 		pipodjednom=true;//i tu ga stavljam da vidim sto ce reci taj praznostart
    return 2;
    }
    if(IsValid( goal_x, goal_y )==0){       //pozicija nije u mapi
		printf("DStar Init> Goal (%d,%d) out of map!\n",goal_x,goal_y);
    return 0;
    }
    if(IsValid( goal_x, goal_y )==2){       //pozicija je zauzeta
	    printf("DStar Init> Goal (%d,%d) occupied!\n",goal_x,goal_y);
    return 2;
    }
   return 1;
}

int DStar::InitOri( I_point start, I_point goal)
{
    Start.x=start.x; Start.y=start.y; Start.th=start.th;         //indexi cilja
    Goal.x=goal.x; Goal.y=goal.y; Goal.th=goal.th;         //indexi cilja
#if RECTANGULAR
		OrientationIntervals ints;
    		ints= cspace->getAdmissibleOrientationStructure(Goal.x,Goal.y);
    		std::cout<<"intervals goal: "<<ints.print()<<std::endl;
    		ints= cspace->getAdmissibleOrientationStructure(Start.x,Start.y);
    		std::cout<<"intervals start: "<<ints.print()<<std::endl;
    		maxOri=ints.getMaxOrientation();
    		if (maxOri>MAXORI) printf("segm.fault will be because we don't have enough reserved space\n");
   if(IsValidOri( start )==0){       //pozicija nije u mapi
    printf("DStar Init> Start (%d,%d,%d) out of map!\n",start.x,start.y,start.th);
    return 0;
    }
    travcost=map[Start.x][Start.y].traversal_cost;
    IntPose pointX((Start.x),(Start.y),Start.th);
    travcost3d = std::max(1., (MAXDISTANCE - floor(cspace->getDistanceInCells(pointX))));
#endif
    if(IsValidOri( start )==2){       //pozicija je zauzeta
		printf("DStar Init> Start (%d,%d,%d) occupied!\n",start.x,start.y,start.th);
		Start_zauzet=true;
// 		pipodjednom=true;//i tu ga stavljam da vidim sto ce reci taj praznostart
    return 2;
    }
    if(IsValidOri( goal )==0){       //pozicija nije u mapi
		printf("DStar Init> Goal (%d,%d,%d) out of map!\n",goal.x,goal.y,goal.th);
    return 0;
    }
    if(IsValidOri( goal )==2){       //pozicija je zauzeta
	    printf("DStar Init> Goal (%d,%d,%d) occupied!\n",goal.x,goal.y,goal.th);
    return 2;
    }
   return 1;
}

int DStar::howmanyOriCollides( I_point X)
{
  int howmuch=OBSTACLE;
#if RECTANGULAR
   if(IsValidOri( X )==1){ 
    		std::vector<OrientationInterval> connectedIntervals;
    		connectedIntervals=cspace->getAdmissibleOrientations(X.x,X.y);
    		howmuch=maxOri+1-OrientationIntervals::getIntervalSize(connectedIntervals[0]);
//    		printf("intervalsize %d\n",OrientationIntervals::getIntervalSize(connectedIntervals[0]));
//    DStarSearchNode* Node=cspace->getDStarSearchNode(X.x,X.y,X.th);
//    if (Node!=NULL){
//    OrientationInterval interval = (Node)->getOrientationInterval();
//    howmuch=maxOri+1-OrientationIntervals::getIntervalSize(interval);
//    }
    }
#endif
   return howmuch;
}

bool DStar::SearchPathReverse(){
	int Goal_f, Goal_h, f_old, h_old, k_old, g_old, Start_h;
	if ( (Goal.x==Start.x) && (Goal.y==Start.y) )
	{
		printf("DS> goal==start! finished!\n");
		return true;
	}
	if (gettimeofday(&timeStart, NULL) == 0)
	{
		mySecStart = timeStart.tv_sec;
		myMSecStart = timeStart.tv_usec / 1000;
	}
	if (prviput){
		NumElemListaReverse=0;
		insertNodeReverse(StartRacunac,0);
		watchdog_counter=0;
		azurirani_reverse=0;
		if (racunaoupromjeni){
			prviput=0;//mali trik za samo RD* ponovni search i to neiscrpno
			Start_h=map[StartRacunac.x][StartRacunac.y].h_cost_int;
		}else{
			GoalRacunac=Goal;
		}
		printf("initial: GoalRacunac=(%d,%d), StartRacunac=(%d,%d), Goal=(%d,%d), Start=(%d,%d)\n",GoalRacunac.x,GoalRacunac.y, StartRacunac.x, StartRacunac.y,Goal.x,Goal.y, Start.x, Start.y);
	}
//	else
	if (prviput==0)
	{
		if (racunaoupromjeni==0){
			printf("SearchPathReverse: nema promjene\n");
			return true;
		}
		//tu izbrisati sve koji imaju cost manji od StartRacunca
		Start_h=map[StartRacunac.x][StartRacunac.y].h_cost_int;
// 		GoalRacunac=Goal;//proba
		Goal_f = map[GoalRacunac.x][GoalRacunac.y].h_cost_int_reverse+map[GoalRacunac.x][GoalRacunac.y].h_cost_int;
#if BEZ_HEU && 1 //patak
		Goal_f=0;
#endif
		Goal_h = map[GoalRacunac.x][GoalRacunac.y].h_cost_int_reverse;
		printf("GoalRacunac=(%d,%d), Goal_f=%d, Goal_h=%d\n",GoalRacunac.x,GoalRacunac.y,Goal_f,Goal_h);
		printf("broj elemenata na listi reverse %d, h_cost od StartRacunca je %d\n",NumElemListaReverse, Start_h);
		// 		ispisi(glava_reverse);
#if 1
// 		if (!(brisi_do_costa(&glava_reverse, Start_h, Goal_h))){//brisanje iz prave liste
		if (!(brisi_do_costa(&glava_reverse, Start_h))){//brisanje iz prave liste
			if ( (logfile = fopen("komb","a")) == NULL )
				printf("Error! komb file couldn't be opened.");
			else{
				fprintf(logfile,"SearchPath> nema trazenog elementa u pravoj listi\n");
				fclose(logfile);
			}
		}
#endif
		printf("broj elemenata na listi reverse %d\n",NumElemListaReverse);
		// 		ispisi(glava_reverse);
		
	}

	while (NumElemListaReverse != 0)
	{
		MinCostElemLista=(glava_reverse)->element;
// 		printf("watchdog_counter=%d, MinCostElemLista=(%d,%d)\n",watchdog_counter,MinCostElemLista.x,MinCostElemLista.y);
		if (prviput==0){
			k_old = map[MinCostElemLista.x][MinCostElemLista.y].k_cost_int_reverse;
			h_old = map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int_reverse;
			g_old = map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int;
			f_old = k_old+g_old;
#if BEZ_HEU && 1 //patak
			f_old=0;
#endif
			if (((GoalRacunac.x==MinCostElemLista.x) && (GoalRacunac.y==MinCostElemLista.y)))
			{
				Goal_f = h_old + g_old ;
#if BEZ_HEU && 1 //patak
				Goal_f=0;
#endif
				Goal_h = h_old;
			}
#if 1 //ovo nemam opisano u clanku ali je korisno kod replaniranja da ne ide van kruga od D* puta kolko ima od Starta do Goala
			if ((map[MinCostElemLista.x][MinCostElemLista.y].k_cost_int>Start_h)){
	/*			if ((map[MinCostElemLista.x][MinCostElemLista.y].k_cost_int>Start_h)||(!(LESSEQ((k_old+pathCostEstimate(MinCostElemLista,Goal)),k_old,(Goal_h+map[GoalRacunac.x][GoalRacunac.y].h_cost_int),Goal_h)))){*/
// 				printf("veci k ima (%d,%d), ne treba racunati\n",MinCostElemLista.x,MinCostElemLista.y);
// 				printf("broj elemenata na listi reverse %d, h_cost od StartRacunca je %d\n",NumElemListaReverse, Start_h);
				// 		ispisi(glava_reverse);
// 				if (!(brisi_do_costa(&glava_reverse, Start_h, Goal_h))){//brisanje iz prave liste
				if (!(brisi_do_costa(&glava_reverse, Start_h))){
					if ( (logfile = fopen("komb","a")) == NULL )
						printf("Error! komb file couldn't be opened.");
					else{
						fprintf(logfile,"SearchPath> nema trazenog elementa u pravoj listi\n");
						fclose(logfile);
					}
				}
 				printf("broj elemenata na listi reverse %d\n",NumElemListaReverse);
				continue;
			}//provjera za k
#endif
			
// 			printf("f_old=%d, k_old=%d, Goal_f=%d, Goal_h=%d, h_old=%d\n",f_old,k_old, Goal_f, Goal_h, h_old);
if(1){//tu sam stavila 0 kad sam htjela uvijek iscrpno patak
			if ((!(LESSEQ(f_old,k_old,Goal_f,Goal_h))))//&&(k_old==h_old))
			{
				if (gettimeofday(&timeNow, NULL) == 0)
				{
					mySecNow = timeNow.tv_sec;
					myMSecNow = timeNow.tv_usec / 1000;
				}
				vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
				if (watchdog_counter || 1){
					printf("DStar SearchPathReverse> trajanje algoritma kod promjene = %d ms, watchdog=%d, broj azuriranih=%d\n",vremenska_razlika,watchdog_counter,azurirani_reverse);
					printf("DS> broj cvorova na listi:%d\n",NumElemListaReverse);
					if ( (logfile = fopen("komb","a")) == NULL )
						printf("Error! komb file couldn't be opened.");
					else{
						fprintf(logfile,"\nDStarReverse replaniranje: trajanje %d ms, azurirani=%d, broj cvorova na listi %d, broj iteracija %d, GoalRacunac=(%d,%d), f=%d\n", vremenska_razlika, azurirani_reverse, NumElemListaReverse, watchdog_counter,GoalRacunac.x,GoalRacunac.y, map[GoalRacunac.x][GoalRacunac.y].total_cost_int);
						fclose(logfile);
					}
				}
				izracuniWitkowski[brojac_izracuna-1]=vremenska_razlika;
				broj_iteracijaWitkowski[brojac_izracuna-1]=watchdog_counter;
				broj_cvorova_na_listi_backward[brojac_izracuna-1]=NumElemListaReverse;
				azurirani_backward_polje[brojac_izracuna-1]=azurirani_reverse;

				watchdog_counter=0;
				azurirani_reverse=0;
				return true;
			}
}
		}
		processStateReverse();
		++watchdog_counter;
/*
		if (watchdog_counter== 20000){//to je otprilike 100 ms
			using namespace PlayerCc;
			PlayerClient robot("localhost", 6665);
			Position2dProxy pp(&robot, 0);
			pp.SetSpeed( 0., 0. ); //ove brzine se postavljaju u WH
			printf("uspio sam postaviti brzinu 0!\n");
		}*/
		if (watchdog_counter > 20*MapSizeX*MapSizeY)
		{
			printf("Previse posjecenih cvorova!!!!\n");
			return false;
		}
	}   //od while
	if (gettimeofday(&timeNow, NULL) == 0)
	{
		mySecNow = timeNow.tv_sec;
		myMSecNow = timeNow.tv_usec / 1000;
	}
	vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
	if (prviput){
		printf("DStar SearchPathReverse> trajanje algoritma kod iscrpnog pretrazivanja = %d ms, watchdog=%d, broj azuriranih=%d\n",vremenska_razlika,watchdog_counter,azurirani_reverse);
		if ( (logfile = fopen("komb","a")) == NULL )
			printf("Error! komb file couldn't be opened.");
		else{
			fprintf(logfile,"\nDStarReverse inicijalno: trajanje %d ms, azurirani=%d, broj cvorova na listi %d, broj iteracija %d, GoalRacunac=(%d,%d), f=%d\n", vremenska_razlika, azurirani_reverse, NumElemListaReverse, watchdog_counter,GoalRacunac.x,GoalRacunac.y, map[GoalRacunac.x][GoalRacunac.y].total_cost_int);
			fclose(logfile);
		}
		izracuniWitkowski[brojac_izracuna]=vremenska_razlika;
		broj_iteracijaWitkowski[brojac_izracuna]=watchdog_counter;
		broj_cvorova_na_listi_backward[brojac_izracuna]=NumElemListaReverse;
		azurirani_backward_polje[brojac_izracuna]=azurirani_reverse;
// 		prviput=0; //nema vise prvog puta
	}else if (watchdog_counter){
		printf("DStar SearchPathReverse> trajanje algoritma kod iscrpnog replaniranja = %d ms, watchdog=%d, broj azuriranih=%d\n",vremenska_razlika,watchdog_counter,azurirani_reverse);
		if ( (logfile = fopen("komb","a")) == NULL )
			printf("Error! komb file couldn't be opened.");
		else{
			fprintf(logfile,"\nDStarReverse iscrpno replaniranje: trajanje %d ms, azurirani=%d, broj cvorova na listi %d, broj iteracija %d, GoalRacunac=(%d,%d), f=%d\n", vremenska_razlika, azurirani_reverse, NumElemListaReverse, watchdog_counter,GoalRacunac.x,GoalRacunac.y, map[GoalRacunac.x][GoalRacunac.y].total_cost_int);
			fclose(logfile);
		}
		izracuniWitkowski[brojac_izracuna-1]=vremenska_razlika;
		broj_iteracijaWitkowski[brojac_izracuna-1]=watchdog_counter;
		broj_cvorova_na_listi_backward[brojac_izracuna-1]=NumElemListaReverse;
		azurirani_backward_polje[brojac_izracuna-1]=azurirani_reverse;
	}
	watchdog_counter=0;
	azurirani_reverse=0;
	return true;
}

bool DStar::SearchPathOri(){
	if (gettimeofday(&timeStart, NULL) == 0)
	{
		mySecStart = timeStart.tv_sec;
		myMSecStart = timeStart.tv_usec / 1000;
	}
	if (prviput){
    queue.clear();
		NumElemLista=0;
		insertNodeOri(Goal,0);
		watchdog_counter=0;
		azurirani=0;
	}
  while (queue.empty()==false)
  {
    I_point* n = queue.pop(); 
    MinCostElemLista.x = n->x;
    MinCostElemLista.y = n->y;
    MinCostElemLista.th = n->th;
    if(map[MinCostElemLista.x][MinCostElemLista.y].tagOri[MinCostElemLista.th] != OPEN){
      NumElemLista--;
      continue;
    }else{
		  processStateOri();
	    ++watchdog_counter;
    }
  }

//	while (NumElemLista != 0)
//	{
//		MinCostElemLista=(glava)->element;
//		processStateOri();
//		++watchdog_counter;
//		if (watchdog_counter > 2*maxOri*MapSizeX*MapSizeY)
//		{
//			printf("Too many explored nodes!!!!\n");
//			PathExists=false;
//			return false;
//		}
//	}   //od while

	if (gettimeofday(&timeNow, NULL) == 0)
	{
		mySecNow = timeNow.tv_sec;
		myMSecNow = timeNow.tv_usec / 1000;
	}
	vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
	if (prviput){
		printf("DStar SearchPathOri> exhaustive search = %d ms, watchdog=%d, No explored nodes=%d, No nodes on list %d \n",vremenska_razlika,watchdog_counter,azurirani, NumElemLista);
	}
  izracuni[brojac_izracuna]=vremenska_razlika;
  broj_iteracija[brojac_izracuna]=watchdog_counter;
  broj_cvorova_na_listi[brojac_izracuna]=NumElemLista;
  azurirani_polje[brojac_izracuna]=azurirani;
  brojac_izracuna++;
	watchdog_counter=0;
	azurirani=0;
	PathExists=true;
	return true;
}


bool    DStar::SearchPath()
{
	int f_old, f_biased_old, k_old, g_old, h_old;
  I_point element; //onaj koji ima promjenu
#if (DSTAR_REVERSE)
  StartRacunac=Start;//stavljam pravi start
#endif
//inicijalno pretrazivanje, od pocetka
  	if ((prviput) && (watchdog_counter==0))
	{
#if USEBUCKET
    queue.clear();
#endif
		StartCurr.x=Start.x;//pocetna vrijednost za racunanje dcurr-a
		StartCurr.y=Start.y;
		StartRacunac=Start;
    NumElemLista=0;
	max_broj_cvorova=0;//resetiram
	azurirani=0;
	watchdog_counter++;  //zbog njega se racuna
	prviStart=Start;//zapamti prvi start
    insertNode(Goal,0);
    printf("map[Goal.x][Goal.y].total_cost_int=%d, heuristics=%d, NumElemLista=%d\n",map[Goal.x][Goal.y].total_cost_int,map[Goal.x][Goal.y].g_cost_int, NumElemLista);
	}  //if prviput
  if ( ((promjena)||(watchdog_counter!=0)))
  {
	  map[StartRacunac.x][StartRacunac.y].g_cost_int=pathCostEstimate(StartRacunac,StartRacunac);//ako je slucajno sletio s proracunatog g-a
#if DSTAR_REVERSE
	  map[StartRacunac.x][StartRacunac.y].g_cost_int=map[StartRacunac.x][StartRacunac.y].k_cost_int_reverse;
#endif
	  Robot_f = map[StartRacunac.x][StartRacunac.y].h_cost_int+map[StartRacunac.x][StartRacunac.y].g_cost_int;//to sam dodala zbog praviStart
  Robot_f_biased = Robot_f;//gledam kak je bez dcurra + dcurr+pathCostEstimate(StartCurr,Start);//moram ovak buduci da se tek dolje racuna dcurr
#if BEZ_HEU
  Robot_f=0;
  Robot_f_biased=0;
#endif
  Robot_h = map[StartRacunac.x][StartRacunac.y].h_cost_int;
	  dcurr=0;
 }
  if (pipodjednom)
	  printf("Robot_f=%d, Robot_h=%d\n",Robot_f, Robot_h);

//-----------------------------------------------------------------------------------
//inserting the changed nodes
    if ((promjena)&&(prviput==0)) //promjena je postavljena u Planneru na 1 
    {
//	    if ((NumElemPunjenja>0)&&(NumElemPraznjenja>0)){//bool praznjenje oznacava prvi prolaz kroz algoritam za praznjenje kada je true
// 		    printf("DStar::SearchPath> punjenje i praznjenje odjednom\n");
//	    }
		    //potrebno je napraviti dva odvojena prolaza kroz algoritam: za praznjenje pa za punjenje
		//a dodan je i treci, za costmasku
	    for(int i=0; (i<NumElemPraznjenja); i++)
      {
        element.x = listapraznjenja[i].x;
	element.y = listapraznjenja[i].y;
	if ((map[element.x][element.y].promjena == 1) || 1)  // 1 oznacava promjenu na slobodno
	{//a pitamo za svaki slucaj da nije doletio neki za punjenje, sto nece biti slucaj al eto
		if ((map[element.x][element.y].tag==CLOSED)) //&&(map[element.x][element.y].h_cost_int<OBSTACLE))//injega
		{
			if (pipodjednom)
				printf("main empty (%d,%d)\n",element.x,element.y);
			insertNode(element,map[element.x][element.y].h_cost_int);
			watchdog_counter++;
		}  //if
#if DSTAR_REVERSE
		if ((map[element.x][element.y].tag_reverse==CLOSED))//&&(map[element.x][element.y].h_cost_int_reverse<OBSTACLE))
		{
			insertNodeReverse(element,map[element.x][element.y].h_cost_int_reverse);
// 			map[element.x][element.y].time_stamp_forward--;
		}  //if
#endif
#ifdef DSTAR_EIGHT_CONNECTED 
		for ( int d = 0; d < 8; d++ ) //for all neighbors
#else
		for ( int d = 0; d < 4; d++ ) //for all neighbors
#endif
          {
            I_point point;
      //odredjivanje pozicije novog susjeda
            point.x=element.x+xofs[d];
            point.y=element.y+yofs[d];
	    if ( IsValid( point.x, point.y )!=0 )//moraju imati i prepreku u sebi
            {
		    if ((map[point.x][point.y].tag==CLOSED))//&&(map[point.x][point.y].h_cost_int<OBSTACLE))//ciste susjede ubacujemo
		    {
                if (pipodjednom)
			printf("%d. neighbor of empty (%d,%d)\n",d,point.x,point.y);
		insertNode(point,map[point.x][point.y].h_cost_int);
                watchdog_counter++;
              }  //if
#if DSTAR_REVERSE
	      if ((map[point.x][point.y].tag_reverse==CLOSED)&&(map[point.x][point.y].h_cost_int_reverse<OBSTACLE))
	      {
		      insertNodeReverse(point,map[point.x][point.y].h_cost_int_reverse);
// 		      map[point.x][point.y].time_stamp_forward--;
	      }  //if
#endif
            }    //if
          } //for po susjedima
	}//if za svaki slucaj
	else{
		//da vidim sto je to zalutalo
// 		printf("DStar::SearchPath> nije praznjenje (1) neg (%d) za element (%d,%d)\n",map[element.x][element.y].promjena,element.x,element.y);
		if ( (logfile = fopen("komb","a")) == NULL )
			printf("Error! komb file couldn't be opened.");
		else{
			fprintf(logfile,"SearchPath> nije praznjenje (1) neg (%d) za element (%d,%d)\n",map[element.x][element.y].promjena,element.x,element.y);
			fclose(logfile);
		}
	}
      }//for po praznjenjima
//      printf("change to free\n");
//      logfile=fopen("slobodno.txt","w");
      for(int i=0; (i<NumElemPraznjenja); i++){//jos jednom da azuriramo promjenu
 	      map[listapraznjenja[i].x][listapraznjenja[i].y].promjena=11;
//	      fprintf(logfile,"%d\n%d\n",listapraznjenja[i].x,listapraznjenja[i].y);
      }
//      printf("\n");
//      fclose(logfile);
	for(int i=0; (i<NumElemPunjenja); i++)
      {
	      element.x = listapunjenja[i].x;
	      element.y = listapunjenja[i].y;
	      if ((map[element.x][element.y].promjena == 2)|| 1)   //2 oznacava promjenu na zauzeto
	      {//a pitamo za svaki slucaj da se ne obrise start onda bi ovaj imao 1
		      if ( (map[element.x][element.y].tag==CLOSED))// && (map[element.x][element.y].h_cost_int<OBSTACLE) ) //injega
		      {
			      if (pipodjednom)
				      printf("main occupied (%d,%d)\n",element.x,element.y);
			      insertNode(element,map[element.x][element.y].h_cost_int);
			      watchdog_counter++;
		      }  //if
#if DSTAR_REVERSE
		      if ((map[element.x][element.y].tag_reverse==CLOSED)&&(map[element.x][element.y].h_cost_int_reverse<OBSTACLE))
		      {
			      insertNodeReverse(element,map[element.x][element.y].h_cost_int_reverse);
// 			      map[element.x][element.y].time_stamp_forward--;
		      }  //if
#endif
#ifdef DSTAR_EIGHT_CONNECTED 
		for ( int d = 0; d < 8; d++ ) //for all neighbors
#else
		for ( int d = 0; d < 4; d++ ) //for all neighbors
#endif
		      {
			      I_point point;

      //odredjivanje pozicije novog susjeda
			      point.x=element.x+xofs[d];
			      point.y=element.y+yofs[d];
			      if ( IsValid( point.x, point.y )!=0 )//moraju imati i prepreku u sebi
			      {
				      if ((map[point.x][point.y].tag==CLOSED)) //&&(map[point.x][point.y].h_cost_int<OBSTACLE))//ciste susjede ubacujemo
				//if ((map[point.x][point.y].tag==CLOSED)&&(map[point.x][point.y].promjena!=2))//ciste susjede ubacujemo
				      {
					      if (pipodjednom)
						      printf("%d. neighbor of occupied (%d,%d)\n",d,point.x,point.y);
					      insertNode(point,map[point.x][point.y].h_cost_int);
					      watchdog_counter++;
				      }
#if DSTAR_REVERSE
				      if ((map[point.x][point.y].tag_reverse==CLOSED)&&(map[point.x][point.y].h_cost_int_reverse<OBSTACLE))
				      {
					      insertNodeReverse(point,map[point.x][point.y].h_cost_int_reverse);
// 					      map[point.x][point.y].time_stamp_forward--;
				      }  //if
#endif
			      }//if
		      }//for po susjedima
	      }//if za svaki slucaj
	      else{
		//da vidim sto je to zalutalo
// 		      printf("DStar::SearchPath> nije punjenje (2) neg (%d) za element (%d,%d)\n",map[element.x][element.y].promjena,element.x,element.y);
		      if ( (logfile = fopen("komb","a")) == NULL )
			      printf("Error! komb file couldn't be opened.");
		      else{
				  fprintf(logfile,"SearchPath> nije punjenje (2) neg (%d) za element (%d,%d) traversal cost=%d\n",map[element.x][element.y].promjena,element.x,element.y,map[element.x][element.y].traversal_cost);
			      fclose(logfile);
		      }
	      }
      }   //for po punjenjima
//      printf("change to occupied\n");
//      logfile=fopen("zauzeto.txt","w");
      for(int i=0; (i<NumElemPunjenja); i++){//jos jednom da azuriramo promjenu
	      map[listapunjenja[i].x][listapunjenja[i].y].promjena=22;
//	      fprintf(logfile,"%d\n%d\n",listapunjenja[i].x,listapunjenja[i].y);
      }
//      printf("\n");
//      fclose(logfile);
#if (NO_COSTMASK==0)
	  for(int i=0; (i<NumElemCostmaska); i++)
	  {
		  element.x = listacostmaska[i].x;
		  element.y = listacostmaska[i].y;
		  if ((map[element.x][element.y].traversal_cost!=map[element.x][element.y].traversal_cost_stari))
		  {//a pitamo za svaki slucaj da nije doletio neki za punjenje, sto nece biti slucaj al eto
			  if ((map[element.x][element.y].tag==CLOSED)) //&&(map[element.x][element.y].h_cost_int<OBSTACLE))//injega
			  {
				  if (pipodjednom)
					  printf("main of costmask change (%d,%d)\n",element.x,element.y);
				  insertNode(element,map[element.x][element.y].h_cost_int);
				  watchdog_counter++;
			  }  //if
#if DSTAR_REVERSE
			  if ((map[element.x][element.y].tag_reverse==CLOSED)&&(map[element.x][element.y].h_cost_int_reverse<OBSTACLE))
			  {
				  insertNodeReverse(element,map[element.x][element.y].h_cost_int_reverse);
// 				  map[element.x][element.y].time_stamp_forward--;
			  }  //if
#endif
#ifdef DSTAR_EIGHT_CONNECTED 
		for ( int d = 0; d < 8; d++ ) //for all neighbors
#else
		for ( int d = 0; d < 4; d++ ) //for all neighbors
#endif
			  {
				  I_point point;

      //odredjivanje pozicije novog susjeda
				  point.x=element.x+xofs[d];
				  point.y=element.y+yofs[d];
				  if ( IsValid( point.x, point.y )!=0 )//moraju imati i prepreku u sebi
				  {
					  if ((map[point.x][point.y].tag==CLOSED)) //&&(map[point.x][point.y].h_cost_int<OBSTACLE))//ciste susjede ubacujemo
					  {
						  if (pipodjednom)
							  printf("%d. neighbor of costmask change (%d,%d)\n",d,point.x,point.y);
						  insertNode(point,map[point.x][point.y].h_cost_int);
						  watchdog_counter++;
					  }  //if
#if DSTAR_REVERSE
					  if ((map[point.x][point.y].tag_reverse==CLOSED)&&(map[point.x][point.y].h_cost_int_reverse<OBSTACLE))
					  {
						  insertNodeReverse(point,map[point.x][point.y].h_cost_int_reverse);
// 						  map[point.x][point.y].time_stamp_forward--;
					  }  //if
#endif
				  }    //if
			  } //for po susjedima
		  }//if za svaki slucaj
	  }//for po promjenama costmaske
#endif //od NO_COSTMASK
//	  printf("change of cost\n");
	  for(int i=0; (i<NumElemCostmaska); i++){//jos jednom da azuriramo promjenu
		  I_point element=listacostmaska[i];
		  if  (map[element.x][element.y].traversal_cost!=map[element.x][element.y].traversal_cost_stari){
			  map[element.x][element.y].promjena=55;//novi broj nigdje ne koristen, to je za slucaj da oba algoritma odjednom vrtimo
			  map[element.x][element.y].traversal_cost_stari=map[element.x][element.y].traversal_cost;
//			  printf("(%d,%d) ",element.x,element.y);
		  }else{
			  //oznacimo da se radilo o istima
			  map[element.x][element.y].promjena=11;
		  }
	  }
	  printf("\n");
	  //      NumElemPromjena=0;//to se radi u Planneru ali zbog debuganja moram tu napisati, a sad bum u moj.cc to napisala da tam mogu crtati
    }
//------------------------------------------------------------

if (0 && brojac_izracuna==0){

    negative_cell.x=Start.x+10;
    negative_cell.y=Start.y+10;
    //element.x=Start.x + 30;
        //element.y=Start.y - 5;
    map[negative_cell.x][negative_cell.y].traversal_cost=-100;
    //map[element.x][element.y].traversal_cost=-100;
    if ( (map[negative_cell.x][negative_cell.y].tag==CLOSED))// && (map[element.x][element.y].h_cost_int<OBSTACLE) ) //injega
    		      {

    			      insertNode(negative_cell,map[negative_cell.x][negative_cell.y].h_cost_int);
    			      watchdog_counter++;
    		      }

   /* if ( (map[element.x][element.y].tag==CLOSED))// && (map[element.x][element.y].h_cost_int<OBSTACLE) ) //injega
        		      {

        			      insertNode(element,map[element.x][element.y].h_cost_int);
        			      watchdog_counter++;
        		      }
*/

}
if (prviput==1){
	map[negative_cell.x][negative_cell.y].traversal_cost=-100;
}
//if (negative_cell.x!=-1)
//  map[negative_cell.x][negative_cell.y].traversal_cost=-100;

if (Start - negative_cell<=1){
	map[negative_cell.x][negative_cell.y].traversal_cost=2;
	if ( (map[negative_cell.x][negative_cell.y].tag==CLOSED))// && (map[element.x][element.y].h_cost_int<OBSTACLE) ) //injega
	    		      {

	    			      insertNode(negative_cell,map[negative_cell.x][negative_cell.y].h_cost_int);
	    			      watchdog_counter++;
	    		      }
//	    		      negative_cell.x=-1;
//	    		      negative_cell.y=-1;
}

blacklist.clear();
//all changed nodes are inserted    
    
//tu sam stavila pocetak vremena
if (gettimeofday(&timeStart, NULL) == 0)
{
	mySecStart = timeStart.tv_sec;
	myMSecStart = timeStart.tv_usec / 1000;
}

   if ( watchdog_counter!=0)
  {  
	  watchdog_counter=0;//reset
	  while (NumElemLista != 0)
    {
		max_broj_cvorova=std::max(max_broj_cvorova,NumElemLista);
#if USEBUCKET
    I_point* n = queue.pop(); 
    MinCostElemLista.x = n->x;
    MinCostElemLista.y = n->y;
    MinCostElemLista.th = n->th;
    if(map[MinCostElemLista.x][MinCostElemLista.y].tag != OPEN){
      NumElemLista--;
      continue;
    }
#else
	  MinCostElemLista=(glava)->element;
#endif
#if BEZ_HEU
      f_old=0;
      f_biased_old=0;
#else
	f_biased_old = map[MinCostElemLista.x][MinCostElemLista.y].total_cost_int_biased;
	f_old = map[MinCostElemLista.x][MinCostElemLista.y].total_cost_int;
#endif
	  k_old = map[MinCostElemLista.x][MinCostElemLista.y].k_cost_int;
	  if (k_old>OBSTACLE){
	    break;
	  }
      h_old = map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int;
      g_old = map[MinCostElemLista.x][MinCostElemLista.y].g_cost_int;
    //ako je najbolji na startu (tj trenutnoj poziciji)
	  if (((StartRacunac.x==MinCostElemLista.x) && (StartRacunac.y==MinCostElemLista.y)))	  {
		  Robot_f = h_old + g_old ;
		  Robot_h = h_old;
        Robot_f_biased = Robot_f + dcurr;
#if BEZ_HEU
	Robot_f=0;
	Robot_f_biased=0;
#endif
	if (pipodjednom)
		printf("the best: Robot_f=%d, Robot_h=%d\n",Robot_f, Robot_h);
		//printf("DStar SearchPath::najbolji> Robot_f=%d Robot_h=%d k=%d\n", Robot_f, Robot_h, k_old);
#if ISCRPNI
	if (0)
#else
		if (prviput==1) //samo inicijalni put
#endif
		{
			PathExists=true;
				if (gettimeofday(&timeNow, NULL) == 0)
				{
					mySecNow = timeNow.tv_sec;
					myMSecNow = timeNow.tv_usec / 1000;
				}
				vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
				izracuni[brojac_izracuna]=vremenska_razlika;
				broj_iteracija[brojac_izracuna]=watchdog_counter;
				broj_cvorova_na_listi[brojac_izracuna]=NumElemLista;
				max_broj_cvorova_na_listi[brojac_izracuna]=max_broj_cvorova;
				azurirani_polje[brojac_izracuna]=azurirani;
    //ovo je za iscrtavanje
				f_putanje[brojac_izracuna]=Robot_f;
				f_biased_putanje[brojac_izracuna]=Robot_f_biased;
				h_putanje[brojac_izracuna]=Robot_h;
				printf("DStar SearchPath> initial search time = %d ms, w=%d NumElemList=%d Robot_f=%d Robot_h=%d, no. iterations=%d\n",vremenska_razlika,watchdog_counter,NumElemLista, Robot_f, Robot_h,brojac_izracuna);
// 				printf("DS> broj azuriranih cvorova:%d\n",azurirani);
				if ( (logfile = fopen("komb","a")) == NULL )
					printf("Error! komb file couldn't be opened.");
				else{
					fprintf(logfile,"\nDStar: trajanje inic. alg. %d ms, azurirani=%d, broj cvorova na listi %d, a maksimalno %d,broj iteracija %d, StartRacunac=(%d,%d), f=%d\n", vremenska_razlika, azurirani, NumElemLista, max_broj_cvorova,watchdog_counter,StartRacunac.x,StartRacunac.y, map[StartRacunac.x][StartRacunac.y].total_cost_int);
					fclose(logfile);
				}
				brojac_izracuna++;

				watchdog_counter=0; //vracamo ga na 0
				prviput=0; //nema vise prvog puta
				inicijalni_put=1; //za cuvanje cijelog patha
				max_broj_cvorova=0;//resetiram
				azurirani=0;
				return true;
//			}
		}

      }

    //exhaustive search 0 instead of 1
	  if (1&&(!(LESSEQ(f_old,k_old,Robot_f,Robot_h)))&&(prviput==0)&&(Robot_h<OBSTACLE))
      {
	      PathExists=true;
        if (gettimeofday(&timeNow, NULL) == 0)
        {
	        mySecNow = timeNow.tv_sec;
	        myMSecNow = timeNow.tv_usec / 1000;
        }
		    vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
		    izracuni[brojac_izracuna]=vremenska_razlika;
		    broj_iteracija[brojac_izracuna]=watchdog_counter;
		    broj_cvorova_na_listi[brojac_izracuna]=NumElemLista;
		    max_broj_cvorova_na_listi[brojac_izracuna]=max_broj_cvorova;
			azurirani_polje[brojac_izracuna]=azurirani;
	    //ovo je za iscrtavanje
		    f_putanje[brojac_izracuna]=Robot_f;
		    f_biased_putanje[brojac_izracuna]=Robot_f_biased;
		    h_putanje[brojac_izracuna]=Robot_h;

		    printf("DStar SearchPath> replanning time = %d ms, watchdog=%d, new cg %d\n",vremenska_razlika,watchdog_counter,novoTeziste);
		printf("DS> no. explored nodes: %d, no. nodes on the list: %d\n",azurirani,NumElemLista);
		brojac_izracuna++;
		racunaoupromjeni=1;
		StartCurr=StartRacunac;
		
		if ( (logfile = fopen("komb","a")) == NULL )
			printf("Error! komb file couldn't be opened.");
		else{
			fprintf(logfile,"\nDStar: trajanje %d ms, azurirani=%d, broj cvorova na listi %d, a maksimalno %d,broj iteracija %d, StartRacunac=(%d,%d), f=%d, brojac_izracuna=%d\n", vremenska_razlika, azurirani, NumElemLista, max_broj_cvorova,watchdog_counter,StartRacunac.x,StartRacunac.y, map[StartRacunac.x][StartRacunac.y].total_cost_int,brojac_izracuna);
			fclose(logfile);
		}

        watchdog_counter=0;
	max_broj_cvorova=0;//resetiram
	azurirani=0;
	pipodjednom=false;//tu ga resetiram
	return true;
	    }




      processState();
      ++watchdog_counter;
      
      if (watchdog_counter > 20*MapSizeX*MapSizeY)
      {
        PathExists=false;
		PathLength=0;
		printf("DStar SearchPath>   Too many explored nodes!!!!--------time_stamp_counter=%d k_old=%d\n",time_stamp_counter,k_old);
        return false;
      }

    }   //od while

 if (blacklist.size()>0) {
	  //adjust costs for blacklist

	    //find the best neighbor pointing to the negative cell

	    int min_cell=map[blacklist.front().x + xofs[0]] [blacklist.front().y + yofs[0]].k_cost_int;
	    I_point min_cell_around_neg;

	    min_cell_around_neg.x=blacklist.front().x + xofs[0];
	    min_cell_around_neg.y=(blacklist.front().y + yofs[0]);

	    #ifdef DSTAR_EIGHT_CONNECTED
	  		for ( int i = 1; i < 8; i++ ) //for all neighbors
	  #else
	  		for ( int i = 1; i < 4; i++ ) //for all neighbors
	  #endif

	  		{
	  //  for (int i=0;i<8;i++){

	              if (map[blacklist.front().x + xofs[i]] [blacklist.front().y + yofs[i]].k_cost_int<min_cell)

	  	  {
	  		  min_cell=map[blacklist.front().x + xofs[i]] [blacklist.front().y + yofs[i]].k_cost_int;
	            min_cell_around_neg.x=blacklist.front().x + xofs[i];
	            min_cell_around_neg.y=(blacklist.front().y + yofs[i]);
	  	  }

	    }



	  		int cost_travel=map[blacklist[0].x][blacklist[0].y].traversal_cost;

	  		if ((blacklist[0].x - min_cell_around_neg.x!=0)  && (blacklist[0].y - min_cell_around_neg.y !=0)) {
	  			  map[blacklist[0].x] [blacklist[0].y].k_cost_int=min_cell + COSTDIAGONAL*cost_travel;
	  		      map[blacklist[0].x] [blacklist[0].y].h_cost_int=map[blacklist[0].x] [blacklist[0].y].k_cost_int;
	  		}
	  		else{
	  			map[blacklist[0].x] [blacklist[0].y].k_cost_int=min_cell + COSTSTRAIGHT*cost_travel;;
	  			map[blacklist[0].x] [blacklist[0].y].h_cost_int=map[blacklist[0].x] [blacklist[0].y].k_cost_int;
	  		}

	  		for (int i=1; i<blacklist.size();i++) {
	  			cost_travel=map[blacklist[i].x][blacklist[i].y].traversal_cost;

	  			if ((blacklist[i].x - blacklist[i-1].x!=0)  && (blacklist[i].y - blacklist[i-1].y!=0)) {
	  	  map[blacklist[i].x] [blacklist[i].y].k_cost_int=map[blacklist[i-1].x] [blacklist[i-1].y].k_cost_int - COSTDIAGONAL*cost_travel;;
	        map[blacklist[i].x] [blacklist[i].y].h_cost_int=map[blacklist[i].x] [blacklist[i].y].k_cost_int;
	  		}

	  		else {
	      	  map[blacklist[i].x] [blacklist[i].y].k_cost_int=map[blacklist[i-1].x] [blacklist[i-1].y].k_cost_int - COSTSTRAIGHT*cost_travel;;
	      	        map[blacklist[i].x] [blacklist[i].y].h_cost_int=map[blacklist[i].x] [blacklist[i].y].k_cost_int;
	        }
	        printf("adjust blacklist (%d,%d) h %d\n",blacklist[i-1].x, blacklist[i-1].y, map[blacklist[i-1].x] [blacklist[i-1].y].k_cost_int);
	    }

blacklist.clear();

 }

    printf("DStar SearchPath> empty list!!!!\n");
  } 
  
  
  PathExists=true;//zelim uvecavati brojac samo kad nesto radi a ne svaki ciklus
  if (gettimeofday(&timeNow, NULL) == 0)
  {
    mySecNow = timeNow.tv_sec;
	  myMSecNow = timeNow.tv_usec / 1000;
  }
  vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
  izracuni[brojac_izracuna]=vremenska_razlika;
  broj_iteracija[brojac_izracuna]=watchdog_counter;
  broj_cvorova_na_listi[brojac_izracuna]=NumElemLista;
  max_broj_cvorova_na_listi[brojac_izracuna]=max_broj_cvorova;
  azurirani_polje[brojac_izracuna]=azurirani;
      //ovo je za iscrtavanje
  f_putanje[brojac_izracuna]=Robot_f;
  f_biased_putanje[brojac_izracuna]=Robot_f_biased;
  h_putanje[brojac_izracuna]=Robot_h;
	if (prviput){
		printf("DStar SearchPath> exhaustive search = %d ms, watchdog=%d, No explored nodes=%d, No nodes on list %d \n",vremenska_razlika,watchdog_counter,azurirani, NumElemLista);
	}
  if (prviput){
	  if ( (logfile = fopen("komb","a")) == NULL )
		  printf("Error! komb file couldn't be opened.");
	  else{
		  fprintf(logfile,"\nDStar: trajanje iscrpnog inic. alg. %d ms, azurirani=%d, broj cvorova na listi %d, a maksimalno %d,broj iteracija %d, StartRacunac=(%d,%d), f=%d\n", vremenska_razlika, azurirani, NumElemLista, max_broj_cvorova,watchdog_counter,StartRacunac.x,StartRacunac.y, map[StartRacunac.x][StartRacunac.y].total_cost_int);
		  fclose(logfile);
	  }
//	  prviput=0;
	  inicijalni_put=1; //za cuvanje cijelog patha
  }
  if ((promjena)&&(watchdog_counter!=0)){
	  if ( (logfile = fopen("komb","a")) == NULL )
		  printf("Error! komb file couldn't be opened.");
	  else{
		  fprintf(logfile,"\nDStar replaniranje: trajanje iscrpnog alg. %d ms, azurirani=%d, broj cvorova na listi %d, a maksimalno %d,broj iteracija %d, StartRacunac=(%d,%d), f=%d\n", vremenska_razlika, azurirani, NumElemLista, max_broj_cvorova,watchdog_counter,StartRacunac.x,StartRacunac.y, map[StartRacunac.x][StartRacunac.y].total_cost_int);
		  fclose(logfile);
	  }
	  racunaoupromjeni=1;
	  StartCurr=StartRacunac;
	  printf("DStar replanning: exhaustive search time %d ms, explored nodes=%d, no. nodes on list %d, max nodes on list %d, no. iterations %d, StartRacunac=(%d,%d), f=%d\n", vremenska_razlika, azurirani, NumElemLista, max_broj_cvorova,watchdog_counter,StartRacunac.x,StartRacunac.y, map[StartRacunac.x][StartRacunac.y].total_cost_int);
  }

  watchdog_counter=0;
  max_broj_cvorova=0;//resetiram
  azurirani=0;

  brojac_izracuna++;





  return true;
}



I_point* DStar::getPath(){
	
	PathLength=0;
	if(!PathExists)
  {
    printf("DStar getPath> Path does not exist!!!");
    return NULL;
  }
  if ( (Goal.x==Start.x) && (Goal.y==Start.y) )
  {
	  printf("DS getPath> goal==start! finished!\n");
	  PathLength=1;
	  path[0]=Start;
	  return path;
  }
  //inicijalizacija
  bliziPut.x=-1;bliziPut.y=-1;
  najboljiPut[0]=StartRacunac;
  stariPut[0].x=pathstari[0].x;stariPut[0].y=pathstari[0].y;
  //ako treba inicijalizirati path
  I_point temp1,temp2,pocetni;
  I_point trazipetlju[10];//tu ce se pamtiti zadnjih 10 polja puta iz susjeda. ako se neki ponavlja onda javi da je petlja i izadji van iz tog susjeda. cirkularno polje
  I_point pomocni[10];
  int dobar,pocetnarazlika;//,trenutnarazlika;
  int malid,malip;//total_cost traversal_cost,
//   total_cost=0;//nepotrebno je racunati cost kad to ocitamo iz mape (h_cost_int)
  int problemcek=0;
  int tempPathLength, tempPathCost;//traziBlizeg=0,
  //prvo gledamo da li je najbolji put bliski starom putu i tek onda ga provjeravamo
  PathCost=map[StartRacunac.x][StartRacunac.y].h_cost_int;
	  PathLength=PathLengthStari;
	  petlja.x=-1;   petlja.y=-1;
	  prepreka.x=-1;   prepreka.y=-1;
	  nemanext.x=-1;  nemanext.y=-1;
	  temp1.x=StartRacunac.x; temp1.y=StartRacunac.y;
	  if ((PathCost<OBSTACLE)){//||(inicijalni_put)
  }else{
	  printf("DS::getPath()> PathCost=%d last good StartCurr=(%d,%d)\n",PathCost,StartCurr.x,StartCurr.y);
	  StartRacunac=StartCurr;
	  temp1=StartRacunac;
  }
  //tu provjeriti da li je novi put slican starom i ako nije onda gledati susjedne puteve
  if (!inicijalni_put) {//ovaj dio cijeli nis ne radi zbog 0 u uvjetu
// 	  printf("PathLengthStari=%d\n",PathLengthStari);
	  dobar=1;//da li se poklapa, inicijalno se svaki put poklapa koji je kraci od 10 polja

  }

  PathLength=1;
  prepreka.x=-1;prepreka.y=-1;//inicijalizacija ako nema prepreke na putu
  nemanext.x=-1;nemanext.y=-1;//inicijalizacija ako nema prepreke na putu
	  //prvo provjeravamo dal je put ok i kolko je dugacak
  malid=pathCostEstimateDiagonalStraight(StartRacunac,Start);
  malip=std::max( abs( StartRacunac.x-Start.x ), abs( StartRacunac.y-Start.y ) );
  pocetni=temp1;
//   printf("pocetni=(%d,%d), temp1=(%d,%d)\n",pocetni.x,pocetni.y,temp1.x,temp1.y);
  while((temp1.x!=Goal.x)||(temp1.y!=Goal.y))
  {
	  if ( IsValid( temp1.x, temp1.y )!=1 )
	  {
		  printf("DStar> out of map or obstacle!!! temp1.x=%d, temp1.y=%d, Start.x=%d, Start.y=%d\n", temp1.x,temp1.y,Start.x,Start.y);
		  printf("DStar> Map dimensions: MapSizeX=%d, MapSizeY=%d\n", MapSizeX, MapSizeY);
		  printf("DStar> no. iterations: watchdog_counter=%d, time_stamp_counter=%d, calculations counter=%d\n",watchdog_counter,time_stamp_counter,brojac_izracuna);
		  if ( IsValid( temp1.x, temp1.y )==2 ) {
			  prepreka.x=temp1.x;
			  prepreka.y=temp1.y;
			  if ( (logfile = fopen("komb","a")) == NULL )
				  printf("Error! komb file couldn't be opened.");
			  else{
				  fprintf(logfile,"getPath> prepreka=(%d,%d)\n",prepreka.x,prepreka.y);
				  fclose(logfile);
			  }
		  }
		  problemcek=2;
		  //return NULL;
		  break;
	  }
	  else
	  {
		  temp2.x=map[temp1.x][temp1.y]._next.x;
		  temp2.y=map[temp1.x][temp1.y]._next.y;
		  if ((temp2.x==-1)||(temp2.y==-1)) {
			  printf("no next\n");
			  nemanext=temp1;
			  //return NULL;
			  problemcek=3;
			  break;
		  }
		  if (PathLength<=10){
			  trazipetlju[PathLength-1].x=temp1.x;
			  trazipetlju[PathLength-1].y=temp1.y;
			  for (int i=0;i<PathLength;i++){
				  if ((temp2.x==trazipetlju[i].x)&&(temp2.y==trazipetlju[i].y)){
					  printf("loop in the path! path length %d\n",PathLength);
					  //return NULL;
					  petlja=temp2;
					  if ( (logfile = fopen("komb","a")) == NULL )
						  printf("Error! komb file couldn't be opened.");
					  else{
						  fprintf(logfile,"getPath> loop=(%d,%d)\n",petlja.x,petlja.y);
						  fclose(logfile);
					  }
					  problemcek=1;
					  break;
				  }
			  }
			  if (problemcek==1)
				  break;
		  }else{
			  trazipetlju[(PathLength % 10)].x=temp1.x;
			  trazipetlju[(PathLength % 10)].y=temp1.y;
			  for (int i=0;i<10;i++){
				  if ((temp2.x==trazipetlju[i].x)&&(temp2.y==trazipetlju[i].y)){
					  printf("loop in the path! path length %d\n",PathLength);
					  petlja=temp2;
					  if ( (logfile = fopen("komb","a")) == NULL )
						  printf("Error! komb file couldn't be opened.");
					  else{
						  fprintf(logfile,"getPath> petlja=(%d,%d)\n",petlja.x,petlja.y);
						  fclose(logfile);
					  }
					  problemcek=1;
					  break;
					  //return NULL;
				  }
			  }
			  if (problemcek==1)
				  break;
		  }
		  temp1.x=temp2.x;
		  temp1.y=temp2.y;
		  PathLength++;
		  if (PathLength>MapSizeX*MapSizeY)
		  {
			  printf("loop! time_stamp_counter=%d\n",time_stamp_counter);
			  if (watchdog_counter==0)
				  watchdog_counter=1;
			  problemcek=1;
			  break;
			  //return NULL;
		  }
	  }
  }
  cijena_puta[(brojac_izracuna-1)]=PathCost;//total_cost;
  duljina_puta[(brojac_izracuna-1)]=PathLength;

  if (problemcek){
	  printf("problem=%d\n",problemcek);
		return NULL;
  }
  temp1=pocetni;
//   printf("pocetni=(%d,%d), temp1=(%d,%d)\n",pocetni.x,pocetni.y,temp1.x,temp1.y);
#if KOMBINACIJA
  int smjer;//to je d, ima ukupno 8 smjerova
  int smjer1, smjer2=-1;
  int path_flag=0;//do 4 idemo defaultno
  int path_segment=4;
//   int los_cell_num=30; //probala sam min_path umjesto los
  int index_old=0, index_new=0;
  int index2;
  int racunao=0;//provjera racunanja stalno istog puta zbog zauzete startne pozicije
  I_point point;
  goal_wit=Goal;
  printf("DS> Goal=(%d,%d), goal_wit_old=(%d,%d)\n", Goal.x, Goal.y, goal_wit_old.x, goal_wit_old.y);
#endif
  for(int i=0;i<PathLength;i++)
  {
    temp1.th=0;
    path[i]=temp1;
    temp1.x=map[path[i].x][path[i].y]._next.x;
    temp1.y=map[path[i].x][path[i].y]._next.y;
    if (temp1.x==-1){
      path[i].th = Goal.th;
    }else{
      path[i].th=(int) (atan2((temp1.y-path[i].y),(temp1.x-path[i].x))*180/M_PI);
    }
#if KOMBINACIJA
    if ((path[i].x==goal_wit_old.x) && (path[i].y==goal_wit_old.y)){
	    index_old=i;
    }
    if (index_new!=0){
// 	    printf("DS> index_old (%d) index_new (%d)\n", index_old, index_new);
	    if (index_old>index_new){
		    goal_wit=goal_wit_old;
		    index_new=0;
		    printf("DS> index_old (%d)>index_new (%d)\n", index_old, index_new);
	    }
    }
    if (dobar){
	    if ((abs(pathstari[i].x-path[i].x)>0)||(abs(pathstari[i].y-path[i].y)>0))
		    racunao=1;
	    if ((i==PathLength-1)&&(racunao==0))
		    racunaoupromjeni=0;
    }
    if (path_flag<path_segment)
    for(int d=0;d<8;d++){
	    point.x=path[i].x+xofs[d];
	    point.y=path[i].y+yofs[d];
	    if ((point.x==temp1.x)&&(point.y==temp1.y)){
		    if (i==0){
			    smjer1=d;
			    smjer=d;
			    printf("DS> smjer1=%d\n",d);
		    }else if (d!=smjer){
			    if (smjer2==-1){
				    smjer2=d;
				    smjer=d;
				    index2=i;//pocetak drugog smjera
				    printf("DS> smjer2=%d, pocetak=%d\n",d,index2);
			    }else{
				    smjer=d;
				    //tu bi iso jos jedan uvjet, ako je 6 okolnjih prepreka onda isto ovo
				    if ((smjer!=smjer1)&&(smjer!=smjer2)){
						printf("DS> smjer=%d, ide od prethodnog pocetka=%d\n",d,index2);
						for (int j=index2; j<i; j++){
							for(int dd=0;dd<8;dd++){
								point.x=path[j].x+xofs[dd];
								point.y=path[j].y+yofs[dd];
								if(IsValid(point.x,point.y)==2){
									if (j>=los_cell_num){
										goal_wit=path[j];
										index_new=j;
										path_flag=path_segment;//uvjet za zaustavljanje
										printf("DS> nasao blizu prepreke prvog j=%d\n",j);
									}
									break;
								}
							}
							if (index_new==j){
								break;
							}
						}
						if (path_flag!=path_segment){//nije nasao
							smjer1=smjer2;
							smjer2=d;
							index2=i;
							printf("DS> unutar los. smjer1=%d, smjer2=%d. pocetak=%d\n",smjer1,smjer2,index2);
						}

				    }else{
					    index2=i;//neki pocetak segmenta je
				    }
			    }
		    }
		    break;
	    }
    }
#endif
    if (inicijalni_put)
    {
	    pathinicijalni[i]=path[i];//uopce ne koristim taj nego iz workhorsa, al nek se nadje
    }
	if (i<100){
// 		stariPut[i].x=pathstari[i].x;
// 		stariPut[i].y=pathstari[i].y;
		stariPut[i]=pathstari[i];
	}
    pathstari[i]=path[i];//pamtim za sljedeci ciklus
/*    if (i<3) {
	    printf("pathstari[%d]=(%d,%d) ",i,pathstari[i].x,pathstari[i].y);
    }*/
  }
  PathLengthStari=PathLength;
#if DSTAR_REVERSE
  int goalseg_index=0, nasao_S_tocku=0, brojac_prepreka=0;
  int smjer=-1;//to je d, ima ukupno 8 smjerova
  int smjer1=-1, smjer2=-1;
  int index2, iprva=-1, kraj_segmenta=-1;
  I_point point;
  I_point p1,p2,p3,x2;
  int dx,sd1,sd2;
  int cost_to_go=0;
  goal_index=0;
  radiponovo=0;

  if ((inicijalni_put==0)&&(racunaoupromjeni)){//ovo je puno bolji nacin provjeravanja jel novi put na starom
	  int windex=0;
	  for(int i=PathLength-1;i>=0;i--)
	  {
		  if ((path[i].x==w0.x)&&(path[i].y==w0.y)){
			  windex=i;//moze ga ne naci na novom putu, al to znaci da je goal_index dalje pa je ok
		  }
		  if (map[path[i].x][path[i].y].path_counter==old_path_counter){
			  GoalRacunac=path[i];
			  goal_index=i;
			  cost_to_go=map[path[i].x][path[i].y].h_cost_int;
		  }else{
			  break;
		  }
	  }
	  printf("GoalRacunac na starom D* i novom D* putu (%d,%d), goal_index=%d, windex=%d, w0=(%d,%d)\n",GoalRacunac.x,GoalRacunac.y,goal_index,windex,w0.x,w0.y);
	  if (goal_index<windex){
		  goal_index=windex; //ako je to slucaj trebalo bi i GoalRacunac pomaknuti
		  GoalRacunac=w0;//dodala, prije nije bilo, mozda rijesi problem s novim w0
		  cost_to_go=map[w0.x][w0.y].h_cost_int;//ovo je falilo
	  }
	  if (goal_index>=6){//pisalo 0 6 zbog odmaka od puta, pa cemo vidjeti
		  for(int i=1;i<PathLengthWitkowskiSegment;i++)
		  {
			  if (((map[path_witkowski_segment[i].x][path_witkowski_segment[i].y].path_counter==old_path_counter)||1)&&(map[path_witkowski_segment[i].x][path_witkowski_segment[i].y].h_cost_int<=cost_to_go)){
				  GoalRacunac=path_witkowski_segment[i];
				  goalseg_index=i;//zapamti indeks na W putu
				  break;
			  }
		  }
		  printf("GoalRacunac na starom W i starom D* putu (%d,%d), goalseg_index=%d\n",GoalRacunac.x,GoalRacunac.y,goalseg_index);
		  radiponovo=1;//zastavica za racunanje najkraceg euklidskog puta
	  }else{
		  racunaoupromjeni=0;//nema promjene puta
		  radiponovo=0;//treba staviti to da ne radi bez veze, ovak svaki put radi
	  }
  }else if (inicijalni_put){
	  radiponovo=1;//da prvi put napravi
  }
  for(int i=0;i<PathLength;i++)
  {
	  map[path[i].x][path[i].y].path_counter=time_stamp_counter;
	  //ovo izbacujem van
	  if ((nasao_S_tocku!=1)&&(goal_index>=0)&&(racunaoupromjeni)){
		  temp1.x=map[path[i].x][path[i].y]._next.x;
		  temp1.y=map[path[i].x][path[i].y]._next.y;
		  for(int d=0;d<8;d++){
			  point.x=path[i].x+xofs[d];
			  point.y=path[i].y+yofs[d];
			  if ((point.x==temp1.x)&&(point.y==temp1.y)){
				  //dodan novi dio za S tocku
				  if ((smjer==d)){
					  if (i>=goal_index){
					  if (sd1!=-1){
						  point.x=path[i].x+xofs[sd1];
						  point.y=path[i].y+yofs[sd1];
						  if (IsValid(point.x,point.y)){
// #if COST_MASK
						  if(map[point.x][point.y].traversal_cost>EMPTYC)
/*#else
							  if(IsValid(point.x,point.y)==2)
#endif*/
						  {
							  brojac_prepreka++;
							  if (brojac_prepreka==1){
								  iprva=i;
							  }
							  sd1=-1;
						  }
						  }
					  }
					  if (sd2!=-1){
						  point.x=path[i].x+xofs[sd2];
						  point.y=path[i].y+yofs[sd2];
						  if (IsValid(point.x,point.y)){
// #if COST_MASK
						  if(map[point.x][point.y].traversal_cost>EMPTYC)
// #else
// 							  if(IsValid(point.x,point.y)==2)
// #endif
						  {
							  brojac_prepreka++;
							  if (brojac_prepreka==1){
								  iprva=i;
							  }
							  sd2=-1;
						  }
						  }
					  }
					  }
					  if ((brojac_prepreka>=2)&&(i>=goal_index))
					  {
// 						  GoalRacunac=path[i];
						  GoalRacunac=path[iprva];
						  nasao_S_tocku=2;//uvjet za zaustavljanje
// 						  goal_index=i;
						  goal_index=iprva;
						  printf("DS> nasao S tocku s preprekama s obje strane goal_index=%d\n",goal_index);
// 						  cost_to_go=map[path[i].x][path[i].y].h_cost_int;
						  cost_to_go=map[path[iprva].x][path[iprva].y].h_cost_int;
					  }
				  }else{
					  brojac_prepreka=0;
					  if ((d==3)||(d==4)){
						  sd1=1;
						  sd2=6;
					  }else if ((d==1)||(d==6)){
						  sd1=3;
						  sd2=4;
					  }else if ((d==2)||(d==5)){
						  sd1=0;
						  sd2=7;
					  }else if ((d==0)||(d==7)){
						  sd1=2;
						  sd2=5;
					  }
					  if (i>=goal_index){
					  point.x=path[i].x+xofs[sd1];
					  point.y=path[i].y+yofs[sd1];
					  if (IsValid(point.x,point.y)){
// #if COST_MASK
					  if(map[point.x][point.y].traversal_cost>EMPTYC)
// #else
// 						  if(IsValid(point.x,point.y)==2)
// #endif
					  {
						  brojac_prepreka++;
						  iprva=i;
						  sd1=-1;
					  }
					  }
					  point.x=path[i].x+xofs[sd2];
					  point.y=path[i].y+yofs[sd2];
					  if (IsValid(point.x,point.y)){
// #if COST_MASK
					  if(map[point.x][point.y].traversal_cost>EMPTYC)
// #else
// 						  if(IsValid(point.x,point.y)==2)
// #endif
					  {
						  brojac_prepreka++;
						  if (brojac_prepreka==1)
							  iprva=i;
						  sd2=-1;
					  }
					  }
					  }
				  }

				  if (smjer1==-1){
					  smjer1=d;
					  smjer=d;
					  printf("DS> smjer1=%d\n",d);
				  }else if (d!=smjer){
					  if ((smjer2==-1)){//zakomentiravam stari nacin za S tocku
						  smjer2=d;
						  smjer=d;
						  index2=i;//pocetak drugog smjera
						  printf("DS> smjer2=%d, pocetak=%d\n",d,index2);
					  }
					  else{
						  smjer=d;
				    //tu bi iso jos jedan uvjet, ako je 6 okolnjih prepreka onda isto ovo
						  if (((smjer!=smjer1)&&(smjer!=smjer2))||(brojac_prepreka==6)){
							printf("DS> smjer=%d, ide od prethodnog pocetka=%d\n",d,index2);
							if (abs(path[index2-1].x-path[index2].x)+abs(path[index2-1].y-path[index2].y)==1){
								p1=path[index2-1];
								p2=path[index2];
								p3=path[index2+1];
							}else{
								p1=path[index2+1];
								p2=path[index2];
								p3=path[index2-1];
							}
							if (p1.x==p2.x){
								x2.x=p3.x;
								x2.y=p2.y;
							}else{
								x2.x=p2.x;
								x2.y=p3.y;
							}
							for(int dd=0;dd<8;dd++){
								point.x=path[index2].x+xofs[dd];
								point.y=path[index2].y+yofs[dd];
								if ((x2.x==point.x)&&(x2.y==point.y)){
									dx=dd;
								}
							}
							for (int j=index2; j<i; j++){
								if (j>=goal_index){
									brojac_prepreka=0;
									point.x=path[j].x+xofs[dx];
									point.y=path[j].y+yofs[dx];
// #if COST_MASK
									if(map[point.x][point.y].traversal_cost>EMPTYC)
// #else
// 										if(IsValid(point.x,point.y)==2)
// #endif
									{
										brojac_prepreka=2;//da pase s donjim uvjetom
									}

									  if (brojac_prepreka>=2)
									  {
										GoalRacunac=path[j];
										nasao_S_tocku=1;//uvjet za zaustavljanje
										goal_index=j;
										kraj_segmenta=i;
										printf("DS> nasao S tocku blizu prepreke prvu goal_index=%d\n",goal_index);
										cost_to_go=map[path[j].x][path[j].y].h_cost_int;
										break;
									  }
								}
							}
							if ((nasao_S_tocku!=1)&&((smjer!=smjer1)&&(smjer!=smjer2)))
							{
								  smjer1=smjer2;
								  smjer2=smjer;
								  nasao_S_tocku=0;
								  index2=i;//pocetak drugog smjera
								  printf("DS> unutar novog puta: smjer1=%d, smjer2=%d, pocetak=%d\n",smjer1,smjer2,index2);
							}
						  }else if ((smjer==smjer1)||(smjer==smjer2)){
							  index2=i;//uvijek ce to bit smjer1 zbog ovog dolje, i sad je sve glupo
							  if (smjer==smjer1){
								  smjer1=smjer2;
								  smjer2=smjer;
							  }
							  if (nasao_S_tocku==2)
								  nasao_S_tocku=1;
							  printf("DS> smjer1=%d, smjer2=%d, pocetak=%d\n",smjer1,smjer2,index2);
						  }
					  }
				  }//if d!=smjer
				  break;
			  }
		  }
	  }
  }
  old_path_counter=time_stamp_counter;
  if (nasao_S_tocku!=0){
	  int j=0;
	  int flagic=1;
	  double orientation;
	  I_point point;
	  R_point rpoint;
	  int brpom, indeks_nastavka=-1,pomak;
	  for (int i=goalseg_index;i<PathLengthWitkowskiSegment;i++){
		  if (flagic){
			  printf("h(GoalRacunac)=%d, h(W[%d])=%d\n",cost_to_go,i,map[path_witkowski_segment[i].x][path_witkowski_segment[i].y].h_cost_int);
			  if ((map[path_witkowski_segment[i].x][path_witkowski_segment[i].y].h_cost_int<=cost_to_go) && ((path_witkowski_segment[i].x!=GoalRacunac.x) || (path_witkowski_segment[i].y!=GoalRacunac.y))){
				  //provjera vizibilitija ako GoalRacunac nije S tocka na starom W
				  if (((path_witkowski_segment[i-1].x!=GoalRacunac.x) || (path_witkowski_segment[i-1].y!=GoalRacunac.y))&&((path_witkowski_segment[i].x!=path[kraj_segmenta].x) || (path_witkowski_segment[i].y!=path[kraj_segmenta].y))){
					  
				  orientation=atan2((path_witkowski_segment[i].y-GoalRacunac.y),(path_witkowski_segment[i].x-GoalRacunac.x));
				  point.x=GoalRacunac.x;
				  point.y=GoalRacunac.y;
				  rpoint.x=(point.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2.)+0.5*GM->Map_Cell_Size*cos(orientation);
				  rpoint.y=(point.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2.)+0.5*GM->Map_Cell_Size*sin(orientation);
				  GM->check_point(rpoint);
				  brpom=0;
				  point=GM->cell_point_temp;
				  while ((abs(point.x-path_witkowski_segment[i].x)>1) || (abs(point.y-path_witkowski_segment[i].y)>1)){
					  brpom++;
					  point.x=GoalRacunac.x;
					  point.y=GoalRacunac.y;
					  rpoint.x=(point.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2.)+brpom*0.5*GM->Map_Cell_Size*cos(orientation);
					  rpoint.y=(point.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2.)+brpom*0.5*GM->Map_Cell_Size*sin(orientation);
					  GM->check_point(rpoint);
					  point=GM->cell_point_temp;
					  if (map[point.x][point.y].traversal_cost!=EMPTYC)
					{
						if (kraj_segmenta!=-1){
						//ako nije vidljiv
						printf("nije vidljiv, ubacujem s puta od D* (%d,%d)\n",path[kraj_segmenta].x,path[kraj_segmenta].y);
						path_witkowski_segment_old[j]=path[kraj_segmenta];
						j++;
						}else{
							indeks_nastavka=i+1;
							while(map[path_witkowski_segment[indeks_nastavka].x][path_witkowski_segment[indeks_nastavka].y].path_counter!=old_path_counter){
								indeks_nastavka++;
							}
							printf("nije vidljiv, ubacujem dio puta od D* do indeksa %d\n",indeks_nastavka);
							pomak=1;
							while ((path_witkowski_segment[indeks_nastavka].x!=path[goal_index+pomak].x) || (path_witkowski_segment[indeks_nastavka].y!=path[goal_index+pomak].y)){
								path_witkowski_segment_old[j]=path[goal_index+pomak];
								j++;
								pomak++;
							}
						}
						break;
					}
				  }
			  }
				  //do tud sam dodala
				  if (indeks_nastavka==-1){
				  printf("ide nastavak puta W (%d,%d)\n",path_witkowski_segment[i].x,path_witkowski_segment[i].y);
				  path_witkowski_segment_old[j]=path_witkowski_segment[i];
				  j++;
				  }
				  flagic=0;
			  }
		  }else{
			  if ((i>=indeks_nastavka)){
			  path_witkowski_segment_old[j]=path_witkowski_segment[i];
			  j++;
			  }
		  }
	  }
	  PathLengthWitkowskiSegmentOld=j;
	  printf("S tocka je GoalRacunac=(%d,%d), W_old[0]=(%d,%d)\n", GoalRacunac.x, GoalRacunac.y, path_witkowski_segment_old[0].x,path_witkowski_segment_old[0].y);
  }else{//tu bi trebao biti pravi cilj, a ono neki drugi pa ne valja
	  if (racunaoupromjeni){//to dodajem
	  PathLengthWitkowskiSegmentOld=0;
	  printf("S tocka je GoalRacunac=(%d,%d)=Goal\n", GoalRacunac.x, GoalRacunac.y);
	  goal_index=PathLength-1;//kljucno za rjesavanje segm. faulta nize
	  if ((Goal.x!=GoalRacunac.x)||(Goal.y!=GoalRacunac.y)){
		  printf("nije jednak cilju!\n");
		  GoalRacunac=Goal;
	  }
	  }
  }
#else //od daleko gore DSTAR_REVERSE
	radiponovo=0;
  if ((inicijalni_put==0)&&(racunaoupromjeni)){//ovo je puno bolji nacin provjeravanja jel novi put na starom
	  int windex=0;
	  for(int i=PathLength-1;i>=0;i--)
	  {
		  if (map[path[i].x][path[i].y].path_counter!=old_path_counter){
		  	  printf("old path and new path differ at index %d of the new path\n",i);
		  	  radiponovo=1;
			  break;
		  }
	  }
  }
  for(int i=0;i<PathLength;i++)
  {
	  map[path[i].x][path[i].y].path_counter=time_stamp_counter;
  } 
  old_path_counter=time_stamp_counter;
  if (radiponovo==0) racunaoupromjeni=0; 
	
#endif
if ((radiponovo) || (inicijalni_put)){
  double duljina_ds_puta=0.;
  int traversalcostpath=travcost;
 printf("cijena 2D puta je %d\n",map[Start.x][Start.y].h_cost_int);
	  if ( (logfile = fopen("logger//traversalcostpath","wt")) == NULL ){
	 	  printf("Error! file couldn't be opened.");
	  }else{
	  		fprintf(logfile,"%d %d\n",traversalcostpath, map[Start.x][Start.y].h_cost_int);

  for (int i=1; i<PathLength; i++){
    traversalcostpath = map[path[i].x][path[i].y].traversal_cost;
    fprintf(logfile,"%d %d\n",traversalcostpath, map[path[i].x][path[i].y].h_cost_int);

    duljina_ds_puta+=sqrt((path[i].x-path[i-1].x)*(path[i].x-path[i-1].x)+(path[i].y-path[i-1].y)*(path[i].y-path[i-1].y))*(CELL_DIM)*0.001;
  }
	 	  fclose(logfile);
	  }
  if ( (logfile = fopen("komb","a")) == NULL )
    printf("Error! komb file couldn't be opened.");
  else{
    fprintf(logfile,"DS::getPath() duljina putanje [m]=%f\n", duljina_ds_puta);
    fclose(logfile);
  }
  duljina_puta_um[(brojac_izracuna-1)]=duljina_ds_puta;
}

  //printf("\n");
  if (inicijalni_put)
  {
    inicijalni_put=0;
    PathLengthinicijalni=PathLength;
  }
  //Start.x=temp2.x; Start.y=temp2.y;//vracam stari start
  if (brojac_izracuna%(MapSizeX*MapSizeY)==0){
	  if(((azurirani_polje = (int *)realloc( azurirani_polje, (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((izracuni = (int *)realloc( izracuni, (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&(( f_biased_putanje= (int *)realloc( f_biased_putanje, (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((f_putanje = (int *)realloc(f_putanje , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((h_putanje = (int *)realloc( h_putanje, (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&(( broj_iteracija= (int *)realloc(broj_iteracija , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((broj_cvorova_na_listi = (int *)realloc(broj_cvorova_na_listi , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((max_broj_cvorova_na_listi = (int *)realloc(max_broj_cvorova_na_listi , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((cijena_puta = (int *)realloc(cijena_puta , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((duljina_puta = (int *)realloc(duljina_puta , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((duljina_puta_um = (double *)realloc(duljina_puta_um , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(double)))!=NULL)){
		  printf("DStar::getPath> REALLOC succeeded\n");
	  }else{
		  printf("DStar::getPath> Error allocating memory REALLOC!!!\n");
		  return NULL;
	  }
#if KOMBINACIJA || DSTAR_REVERSE
if(((azurirani_backward_polje = (int *)realloc( azurirani_backward_polje, (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((azurirani_forward_polje = (int *)realloc(azurirani_forward_polje , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((izracuniWitkowski = (int *)realloc(izracuniWitkowski , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&(( broj_iteracijaWitkowski= (int *)realloc(broj_iteracijaWitkowski , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((broj_cvorova_na_listi_forward = (int *)realloc(broj_cvorova_na_listi_forward , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((max_broj_cvorova_na_listi_forward = (int *)realloc(max_broj_cvorova_na_listi_forward , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((broj_cvorova_na_listi_backward = (int *)realloc( broj_cvorova_na_listi_backward, (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((max_broj_cvorova_na_listi_backward = (int *)realloc(max_broj_cvorova_na_listi_backward , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((duljina_wit_puta_um = (double *)realloc(duljina_wit_puta_um , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(double)))!=NULL)){
		  printf("DStar::getPath witkowski> REALLOC succeeded\n");
	  }else{
		  printf("DStar::getPath witkowski> Error allocating memory REALLOC!!!\n");
		  return NULL;
	  }
#endif
  }
  return path;
}

void DStar::processState() {
	if (pipodjednom) pipodjednom=false;
	int f_val, k_val, h_val;
	//I_point element pokazuje na neku celiju u mapi!!!
	//prosirenje cvora na susjedne u svih 8 smjerova
	I_point point;
	int h_point, f_point;
#if (USEBUCKET==0)
	if (!(brisi(&glava, MinCostElemLista))){//brisanje iz prave liste
		if ( (logfile = fopen("komb","a")) == NULL )
			printf("Error! komb file couldn't be opened.");
		else{
			fprintf(logfile,"SearchPath> nema trazenog elementa u pravoj listi\n");
			fclose(logfile);
		}
	}
#endif
	map[MinCostElemLista.x][MinCostElemLista.y].tag=CLOSED;
	NumElemLista--;
	f_val = map[MinCostElemLista.x][MinCostElemLista.y].total_cost_int;    //vektor najboljeg <f_val, k_val>
#if BEZ_HEU
	f_val=0;
#endif
	k_val = map[MinCostElemLista.x][MinCostElemLista.y].k_cost_int;
	h_val = map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int;
//avoiding overflow
	if (h_val>OBSTACLE) h_val=OBSTACLE+1.;
	if (k_val>OBSTACLE) k_val=OBSTACLE+1.;
	if (f_val>OBSTACLE) f_val=OBSTACLE+1.;




	  //blacklist
	// if new cell has a negative cost

	if (map[MinCostElemLista.x][MinCostElemLista.y].traversal_cost<0 && (k_val==h_val)){
			  //generate blacklist
		I_point element;
		element=MinCostElemLista;

		blacklist.push_back(MinCostElemLista); // push negative cell to the blacklist

	//add on the list path from the negative cell to the goal
		while  ( (element.x!=Goal.x || element.y!=Goal.y) &&  map[element.x][element.y].k_cost_int == map[element.x][element.y].h_cost_int )
		{

			if (map[element.x][element.y]._next.x!=-1 && map[element.x][element.y]._next.y!=-1)
			{
				element=map[element.x][element.y]._next;
        printf("blacklist element %d %d\n",element.x,element.y);
		    if (!(std::find(blacklist.begin(), blacklist.end(), element)==blacklist.end() && !(element==blacklist.back()) )){
		      printf("blacklist ponavljanje elementa!\n");
		      break;
		    }
		    blacklist.push_back(element);   // push elements on a blacklist
 			}
			else
			{
				std::cout << "pointer with forbidden value" << std::endl;
        break;
			}

		  }
	}







	// RAISE STANJE
	if (1 || (k_val < h_val) )//tu najbolji nije prepreka
	{
		//printf("raise");
#ifdef DSTAR_EIGHT_CONNECTED 
		for ( int d = 0; d < 8; d++ ) //for all neighbors
#else
		for ( int d = 0; d < 4; d++ ) //for all neighbors
#endif
		{
      //odredjivanje pozicije novog susjeda
      point.x=MinCostElemLista.x+xofs[d];
      point.y=MinCostElemLista.y+yofs[d];
      if ( IsValid( point.x, point.y )==1 )  //ne gleda susjede s preprekom u sebi
      {
		  h_point = map[point.x][point.y].h_cost_int;
#if DSTAR_REVERSE
		  f_point=h_point + map[point.x][point.y].k_cost_int_reverse;
#else
		f_point = h_point + pathCostEstimate( point, StartRacunac );
#endif
#if BEZ_HEU
		  f_point=0;
#endif
//avoiding overflow
	if (h_point>OBSTACLE) h_point=OBSTACLE+1.;
	if (f_point>OBSTACLE) f_point=OBSTACLE+1.;

        //postavljen cost c izmedju najboljeg stanja i susjeda
        arc_cost( point.x, point.y, MinCostElemLista.x, MinCostElemLista.y);
	if ( (map[point.x][point.y].tag!=NEW) && (LESSEQ(f_point, h_point, f_val, k_val))
			&& (h_val > h_point + c*travCost[d])&& (c<OBSTACLE)&& ((h_point<OBSTACLE)||(h_val<OBSTACLE))
			&& (std::find(blacklist.begin(), blacklist.end(), point)==blacklist.end() && !(point==blacklist.back()) )
			&& (map[point.x][point.y].traversal_cost>=0)
			)//


	{

		  

			
	map[MinCostElemLista.x][MinCostElemLista.y]._next.x = point.x;
          map[MinCostElemLista.x][MinCostElemLista.y]._next.y = point.y;
          map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int = h_point + c*travCost[d];
          h_val = map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int;

//avoiding overflow
       	if (h_val>OBSTACLE) h_val=OBSTACLE+1.;

#if DSTAR_REVERSE
	  map[MinCostElemLista.x][MinCostElemLista.y].total_cost_int = map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int_reverse + map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int;
#endif

        }
      }
    }
  }
	// LOWER STANJE
  if (k_val == h_val)//tu najbolji moze bit prepreka samo u slucaju da je P->N i hovi su razliciti ili da je P NEW
	{
#ifdef DSTAR_EIGHT_CONNECTED 
		for ( int d = 0; d < 8; d++ ) //for all neighbors
#else
		for ( int d = 0; d < 4; d++ ) //for all neighbors
#endif
	{
      //odredjivanje pozicije novog susjeda
      point.x=MinCostElemLista.x+xofs[d];
      point.y=MinCostElemLista.y+yofs[d];
	  if ( IsValid( point.x, point.y )!=0 )//==1
      {
        h_point = map[point.x][point.y].h_cost_int;
        //avoiding overflow
	if (h_point>OBSTACLE) h_point=OBSTACLE+1.;

//postavljen cost c izmedju najboljeg stanja i susjeda
	arc_cost( MinCostElemLista.x, MinCostElemLista.y, point.x, point.y);//prvi nadodani uvjet je da ne ode u dedend bezveze
	if ( ((std::find(blacklist.begin(), blacklist.end(), point)==blacklist.end() && !(point==blacklist.back()) ))
	  && ( ((map[point.x][point.y].tag==NEW)&&(0||c<OBSTACLE)&&(0||h_val<OBSTACLE)) || ( ((map[point.x][point.y]._next.x == MinCostElemLista.x) && (map[point.x][point.y]._next.y == MinCostElemLista.y)) && (h_point != h_val + c*travCost[d]) && (1||(h_point<OBSTACLE)||(h_val<OBSTACLE)) && (1||(h_point<OBSTACLE)||(c<OBSTACLE))) || ( ((map[point.x][point.y]._next.x != MinCostElemLista.x) || (map[point.x][point.y]._next.y != MinCostElemLista.y)) && (h_point > h_val + c*travCost[d]) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE))))
	)
	{
			if ((h_val>OBSTACLE)&&(k_val>OBSTACLE)){
				printf("fat LOWER h_val=%d k_val=%d c*travCost[%d]=%d\n",h_val,k_val,d,c*travCost[d]);
				pipodjednom=true;
			}

			map[point.x][point.y]._next.x = MinCostElemLista.x;
					map[point.x][point.y]._next.y = MinCostElemLista.y;
					insertNode(point, h_val + c*travCost[d]);
		}
	  }
	} //for susjedi
	}
  else        //opet RAISE STANJE
  {//tu N moze bit prepreka ako je P NEW ili P->N s razlicitim hovima
		//printf("opet raise");
	  if (k_val>h_val){
		  printf("exists k>h\n");
		  pipodjednom=true;
	  }
#ifdef DSTAR_EIGHT_CONNECTED 
		for ( int d = 0; d < 8; d++ ) //for all neighbors
#else
		for ( int d = 0; d < 4; d++ ) //for all neighbors
#endif
		{
      //odredjivanje pozicije novog susjeda
      point.x=MinCostElemLista.x+xofs[d];
      point.y=MinCostElemLista.y+yofs[d];
	  if ( IsValid( point.x, point.y )!=0 )//==1
      {
        h_point = map[point.x][point.y].h_cost_int;
#if DSTAR_REVERSE
	f_point=h_point + map[point.x][point.y].k_cost_int_reverse;
#else
	f_point = h_point + pathCostEstimate( point, StartRacunac );
#endif
#if BEZ_HEU
	f_point=0;
#endif
//avoiding overflow
	if (h_point>OBSTACLE) h_point=OBSTACLE+1.;
	if (f_point>OBSTACLE) f_point=OBSTACLE+1.;

        //postavlja c za prijelaz izmedju susjeda i najboljeg
        arc_cost( MinCostElemLista.x, MinCostElemLista.y, point.x, point.y);
	if ( ((std::find(blacklist.begin(), blacklist.end(), point)==blacklist.end() && !(point==blacklist.back()) ))
	   && ( ((map[point.x][point.y].tag==NEW)&&(0||c<OBSTACLE)&&(0||h_val<OBSTACLE)) || ( ((map[point.x][point.y]._next.x == MinCostElemLista.x) && (map[point.x][point.y]._next.y == MinCostElemLista.y)) && (h_point != h_val + c*travCost[d]) && (1||(h_point<OBSTACLE)||(h_val<OBSTACLE)) && (1||(h_point<OBSTACLE)||(c<OBSTACLE))) )
	)
	{

			map[point.x][point.y]._next.x = MinCostElemLista.x;
					map[point.x][point.y]._next.y = MinCostElemLista.y;
					insertNode(point, h_val + c*travCost[d]);
					//printf("opet raise obradjen");
		}
		else
        {
		if ( ((std::find(blacklist.begin(), blacklist.end(), MinCostElemLista)==blacklist.end() && !(MinCostElemLista==blacklist.back()) ))
			&& ( ((map[point.x][point.y]._next.x != MinCostElemLista.x) || (map[point.x][point.y]._next.y != MinCostElemLista.y)) && (h_point > h_val + c*travCost[d]) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && (map[MinCostElemLista.x][MinCostElemLista.y].tag == CLOSED))
		 )
				{

			 insertNode(MinCostElemLista, h_val);
	    //printf("opet raise obradjen");
		 }
		else
        {
        arc_cost( point.x,point.y,MinCostElemLista.x, MinCostElemLista.y);
		if ( ((std::find(blacklist.begin(), blacklist.end(), point)==blacklist.end() && !(point==blacklist.back()) ))
		   && ( ((map[point.x][point.y]._next.x != MinCostElemLista.x) || (map[point.x][point.y]._next.y != MinCostElemLista.y)) && (h_val > h_point + c*travCost[d]) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && (map[point.x][point.y].tag == CLOSED) && LESS(f_val, k_val, f_point, h_point))
        )
		{

		      insertNode(point, h_point);

	      }
          }
			  } //else
			} //isvalid
		} //for
	}   //else

}

void DStar::processStateOri() {

//  if (MinCostElemLista.x==74 && MinCostElemLista.y==35 && MinCostElemLista.th==36){
//    printf("node\n");
//  }

	int k_val, h_val;
	I_point point;
	int h_point;
//	if (!(brisi(&glava, MinCostElemLista))){//brisanje iz prave liste
//		if ( (logfile = fopen("komb","a")) == NULL )
//			printf("Error! komb file couldn't be opened.");
//		else{
//			fprintf(logfile,"SearchPath> nema trazenog elementa u pravoj listi\n");
//			fclose(logfile);
//		}
//	}
	map[MinCostElemLista.x][MinCostElemLista.y].tagOri[MinCostElemLista.th]=CLOSED;
	NumElemLista--;
	k_val = map[MinCostElemLista.x][MinCostElemLista.y].k_cost_intOri[MinCostElemLista.th];
	h_val = map[MinCostElemLista.x][MinCostElemLista.y].h_cost_intOri[MinCostElemLista.th];
//avoiding overflow
	if (h_val>OBSTACLE) h_val=OBSTACLE+1.;
	if (k_val>OBSTACLE) k_val=OBSTACLE+1.;

	// RAISE STANJE
	if (k_val < h_val)//tu najbolji nije prepreka
	{
		//printf("raise");
#ifdef DSTAR_EIGHT_CONNECTED 
		for ( int d = 0; d < 10; d++ ) //for all neighbors
#else
		for ( int d = 0; d < 6; d++ ) //for all neighbors
#endif
		{
      //determine neighbor
      point.x=MinCostElemLista.x+xofsOri[d];
      point.y=MinCostElemLista.y+yofsOri[d];
      point.th=MinCostElemLista.th+thofsOri[d];
      if (point.th<0) point.th=maxOri;
      if (point.th>maxOri) point.th=0;

      if ( IsValidOri( point )==1 )  //ne gleda susjede s preprekom u sebi
      {
		  h_point = map[point.x][point.y].h_cost_intOri[point.th];
//avoiding overflow
	if (h_point>OBSTACLE) h_point=OBSTACLE+1.;

        //postavljen cost c izmedju najboljeg stanja i susjeda
        arc_costOri( point, MinCostElemLista);
        //ugli hardcode, don't change order of transitions otherwise it won't work
        if ((c<OBSTACLE) && (d==4 || d==5)) c=1;
	if ( (map[point.x][point.y].tagOri[point.th]!=NEW) && (h_point<=k_val) && (h_val > h_point + c*travCostOri[d])&& (c<OBSTACLE)&& ((h_point<OBSTACLE)||(h_val<OBSTACLE)))// 
		{

	        map[MinCostElemLista.x][MinCostElemLista.y]._nextOri[MinCostElemLista.th] = point;
          h_val = h_point + c*travCostOri[d];
          map[MinCostElemLista.x][MinCostElemLista.y].h_cost_intOri[MinCostElemLista.th] = h_val;

//avoiding overflow
       	if (h_val>OBSTACLE) h_val=OBSTACLE+1.;

        }
      }
    }
  }
	// LOWER STATE
  if (k_val == h_val)//tu najbolji moze bit prepreka samo u slucaju da je P->N i hovi su razliciti ili da je P NEW
	{
#ifdef DSTAR_EIGHT_CONNECTED
		for ( int d = 0; d < 10; d++ ) //for all neighbors
#else
		for ( int d = 0; d < 6; d++ ) //for all neighbors
#endif
	{
      //determine neighbor
      point.x=MinCostElemLista.x+xofsOri[d];
      point.y=MinCostElemLista.y+yofsOri[d];
      point.th=MinCostElemLista.th+thofsOri[d];
      if (point.th<0) point.th=maxOri;
      if (point.th>maxOri) point.th=0;
	  if ( IsValidOri( point )!=0 )//==1
      {
		  h_point = map[point.x][point.y].h_cost_intOri[point.th];
        //avoiding overflow
	if (h_point>OBSTACLE) h_point=OBSTACLE+1.;

//postavljen cost c izmedju najboljeg stanja i susjeda
        arc_costOri( MinCostElemLista, point);
        //ugli hardcode, don't change order of transitions otherwise it won't work
        if ((c<OBSTACLE) && (d==4 || d==5)) c=1;
	if ( ((map[point.x][point.y].tagOri[point.th]==NEW)&&(c<OBSTACLE)&&(h_val<OBSTACLE)) || ( ((map[point.x][point.y]._nextOri[point.th].x == MinCostElemLista.x) && (map[point.x][point.y]._nextOri[point.th].y == MinCostElemLista.y) && (map[point.x][point.y]._nextOri[point.th].th == MinCostElemLista.th)) && (h_point != h_val + c*travCostOri[d]) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && ((h_point<OBSTACLE)||(c<OBSTACLE))) || ( ((map[point.x][point.y]._nextOri[point.th].x != MinCostElemLista.x) || (map[point.x][point.y]._nextOri[point.th].y != MinCostElemLista.y) || (map[point.x][point.y]._nextOri[point.th].th != MinCostElemLista.th)) && (h_point > h_val + c*travCostOri[d]) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE))) )
		{
			if ((h_val>OBSTACLE)&&(k_val>OBSTACLE)){
				printf("fat LOWER\n");
			}

			map[point.x][point.y]._nextOri[point.th] = MinCostElemLista;
			insertNodeOri(point, h_val + c*travCostOri[d]);
		}
	  }
	} //for susjedi
	}
  else        //opet RAISE STANJE
  {//tu N moze bit prepreka ako je P NEW ili P->N s razlicitim hovima
		//printf("opet raise");
	  if (k_val>h_val){
		  printf("exists k>h\n");
	  }
#ifdef DSTAR_EIGHT_CONNECTED
		for ( int d = 0; d < 10; d++ ) //for all neighbors
#else
		for ( int d = 0; d < 6; d++ ) //for all neighbors
#endif
		{
      //determine neighbor
      point.x=MinCostElemLista.x+xofsOri[d];
      point.y=MinCostElemLista.y+yofsOri[d];
      point.th=MinCostElemLista.th+thofsOri[d];
      if (point.th<0) point.th=maxOri;
      if (point.th>maxOri) point.th=0;
	  if ( IsValidOri( point )!=0 )//==1
      {
		  h_point = map[point.x][point.y].h_cost_intOri[point.th];
//avoiding overflow
	if (h_point>OBSTACLE) h_point=OBSTACLE+1.;

        //postavlja c for transition from best to neighbor
        arc_costOri( point, MinCostElemLista);
        //ugli hardcode, don't change order of transitions otherwise it won't work
        if ((c<OBSTACLE) && (d==4 || d==5)) c=1;
        int hc=c;
        //postavlja c za prijelaz izmedju susjeda i najboljeg
        arc_costOri( MinCostElemLista, point);
        //ugli hardcode, don't change order of transitions otherwise it won't work
        if ((c<OBSTACLE) && (d==4 || d==5)) c=1;
	if ( ((map[point.x][point.y].tagOri[point.th]==NEW)&&(c<OBSTACLE)&&(h_val<OBSTACLE)) || ( ((map[point.x][point.y]._nextOri[point.th].x == MinCostElemLista.x) && (map[point.x][point.y]._nextOri[point.th].y == MinCostElemLista.y) && (map[point.x][point.y]._nextOri[point.th].th == MinCostElemLista.th)) && (h_point != h_val + c*travCostOri[d]) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && ((h_point<OBSTACLE)||(c<OBSTACLE))) )
		{

			map[point.x][point.y]._nextOri[point.th] = MinCostElemLista;
			insertNodeOri(point, h_val + c*travCostOri[d]);
		}
		else
        {
		if ( ((map[point.x][point.y]._nextOri[point.th].x != MinCostElemLista.x) || (map[point.x][point.y]._nextOri[point.th].y != MinCostElemLista.y) || (map[point.x][point.y]._nextOri[point.th].th != MinCostElemLista.th)) && (h_point > h_val + c*travCostOri[d]) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && (map[MinCostElemLista.x][MinCostElemLista.y].tagOri[MinCostElemLista.th] == CLOSED))
		 {

			 insertNodeOri(MinCostElemLista, h_val);
	    //printf("opet raise obradjen");
		 }
		else
        {
		if ( ((map[point.x][point.y]._nextOri[point.th].x != MinCostElemLista.x) || (map[point.x][point.y]._nextOri[point.th].y != MinCostElemLista.y) || (map[point.x][point.y]._nextOri[point.th].th != MinCostElemLista.th)) && (h_val > h_point + hc*travCostOri[d]) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && (map[point.x][point.y].tagOri[point.th] == CLOSED) && (k_val<h_point))
              {

		      insertNodeOri(point, h_point);

	      }
          }
			  } //else
			} //isvalid
		} //for
	}   //else

}


void DStar::processStateReverse() {
	int f_val, k_val, h_val;
	//I_point element pokazuje na neku celiju u mapi!!!
	//prosirenje cvora na susjedne u svih 8 smjerova
	I_point point;
	int h_point, f_point;
	if (!(brisi(&glava_reverse, MinCostElemLista))){//brisanje iz prave liste
		if ( (logfile = fopen("komb","a")) == NULL )
			printf("Error! komb file couldn't be opened.");
		else{
			fprintf(logfile,"SearchPath> nema trazenog elementa u pravoj listi\n");
			fclose(logfile);
		}
	}
	map[MinCostElemLista.x][MinCostElemLista.y].tag_reverse=CLOSED;
	NumElemListaReverse--;
	k_val = map[MinCostElemLista.x][MinCostElemLista.y].k_cost_int_reverse;
	h_val = map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int_reverse;
	f_val = k_val + map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int;    //vektor najboljeg <f_val, k_val>
#if BEZ_HEU && 1
	f_val=0;
#endif
	// RAISE STANJE
	if (k_val < h_val)//tu najbolji nije prepreka
	{
		//printf("raise");
		for ( int d = 0; d < 8; d++ )
		{
      //odredjivanje pozicije novog susjeda
			point.x=MinCostElemLista.x+xofs[d];
			point.y=MinCostElemLista.y+yofs[d];
			if ( IsValid( point.x, point.y )==1 )  //ne gleda susjede s preprekom u sebi
			{
				h_point = map[point.x][point.y].h_cost_int_reverse;
				f_point = map[point.x][point.y].total_cost_int;
#if BEZ_HEU && 1
				f_point=0;
#endif
				//postavljen cost c izmedju najboljeg stanja i susjeda
				arc_cost( point.x, point.y, MinCostElemLista.x, MinCostElemLista.y);
				if ( (map[point.x][point.y].tag_reverse!=NEW) && (LESSEQ(f_point, h_point, f_val, k_val)) && (h_val > h_point + c*travCost[d])&& (c<OBSTACLE)&& ((h_point<OBSTACLE)||(h_val<OBSTACLE)))//
				{
					map[MinCostElemLista.x][MinCostElemLista.y]._next_reverse.x = point.x;
					map[MinCostElemLista.x][MinCostElemLista.y]._next_reverse.y = point.y;
					map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int_reverse = h_point + c*travCost[d];
					map[MinCostElemLista.x][MinCostElemLista.y].total_cost_int = map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int_reverse + map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int;
					h_val = map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int_reverse;
				}
			}
		}
	}
	// LOWER STANJE
	if (k_val == h_val)//tu najbolji moze bit prepreka samo u slucaju da je P->N i hovi su razliciti ili da je P NEW
	{
		for ( int d = 0; d < 8; d++ )
		{
      //odredjivanje pozicije novog susjeda
			point.x=MinCostElemLista.x+xofs[d];
			point.y=MinCostElemLista.y+yofs[d];
			if ( IsValid( point.x, point.y )!=0 )//==1
			{
				h_point = map[point.x][point.y].h_cost_int_reverse;
//postavljen cost c izmedju najboljeg stanja i susjeda
				arc_cost( MinCostElemLista.x, MinCostElemLista.y, point.x, point.y);//prvi nadodani uvjet je da ne ode u dedend bezveze
				if ( ((map[point.x][point.y].tag_reverse==NEW)&&(c<OBSTACLE)&&(h_val<OBSTACLE)) || ( ((map[point.x][point.y]._next_reverse.x == MinCostElemLista.x) && (map[point.x][point.y]._next_reverse.y == MinCostElemLista.y)) && (h_point != h_val + c*travCost[d]) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && ((h_point<OBSTACLE)||(c<OBSTACLE))) || ( ((map[point.x][point.y]._next_reverse.x != MinCostElemLista.x) || (map[point.x][point.y]._next_reverse.y != MinCostElemLista.y)) && (h_point > h_val + c*travCost[d]) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE))) )
				{
					if ((h_val>OBSTACLE)&&(k_val>OBSTACLE)){
						printf("fat LOWER\n");
					}
			
					map[point.x][point.y]._next_reverse.x = MinCostElemLista.x;
					map[point.x][point.y]._next_reverse.y = MinCostElemLista.y;
					insertNodeReverse(point, h_val + c*travCost[d]);
				}
			}
		} //for susjedi
	}
	else        //opet RAISE STANJE
	{//tu N moze bit prepreka ako je P NEW ili P->N s razlicitim hovima
		//printf("opet raise");
		if (k_val>h_val){
			printf("reverse: exists k>h\n");
		}
		for ( int d = 0; d < 8; d++ )
		{
      //odredjivanje pozicije novog susjeda
			point.x=MinCostElemLista.x+xofs[d];
			point.y=MinCostElemLista.y+yofs[d];
			if ( IsValid( point.x, point.y )!=0 )//==1
			{
				h_point = map[point.x][point.y].h_cost_int_reverse;
				f_point = map[point.x][point.y].total_cost_int;
#if BEZ_HEU && 1
				f_point=0;
#endif
        //postavlja c za prijelaz izmedju susjeda i najboljeg
				arc_cost( MinCostElemLista.x, MinCostElemLista.y, point.x, point.y);
				if ( ((map[point.x][point.y].tag_reverse==NEW)&&(c<OBSTACLE)&&(h_val<OBSTACLE)) || ( ((map[point.x][point.y]._next_reverse.x == MinCostElemLista.x) && (map[point.x][point.y]._next_reverse.y == MinCostElemLista.y)) && (h_point != h_val + c*travCost[d]) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && ((h_point<OBSTACLE)||(c<OBSTACLE))) )
				{
					map[point.x][point.y]._next_reverse.x = MinCostElemLista.x;
					map[point.x][point.y]._next_reverse.y = MinCostElemLista.y;
					insertNodeReverse(point, h_val + c*travCost[d]);
				}
				else
				{
					if ( ((map[point.x][point.y]._next_reverse.x != MinCostElemLista.x) || (map[point.x][point.y]._next_reverse.y != MinCostElemLista.y)) && (h_point > h_val + c*travCost[d]) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && (map[MinCostElemLista.x][MinCostElemLista.y].tag_reverse == CLOSED))
					{
						insertNodeReverse(MinCostElemLista, h_val);
					}
					else
					{
						if ( ((map[point.x][point.y]._next_reverse.x != MinCostElemLista.x) || (map[point.x][point.y]._next_reverse.y != MinCostElemLista.y)) && (h_val > h_point + c*travCost[d]) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && (map[point.x][point.y].tag_reverse == CLOSED) && LESS(f_val, k_val, f_point, h_point))
						{
							insertNodeReverse(point, h_point);
						}
					}
				} //else
			} //isvalid
		} //for
	}   //else

}


void    DStar::insertNode( I_point element, int h_new )
{
  if (map[element.x][element.y].tag==NEW)	//NEW (nikad bio na listi)
  {
		map[element.x][element.y].k_cost_int = h_new;
  }
	else if (map[element.x][element.y].tag==OPEN)	//ako je OPEN
	{
		map[element.x][element.y].k_cost_int = std::min(map[element.x][element.y].k_cost_int, h_new);
#if (USEBUCKET==0)
	if (!(brisi(&glava, element))){//brisanje iz prave liste
		if ( (logfile = fopen("komb","a")) == NULL )
			printf("Error! komb file couldn't be opened.");
		else{
			fprintf(logfile,"insertNode> nema trazenog elementa u pravoj listi\n");
			fclose(logfile);
		}
	}
	NumElemLista--;
#endif
  }
	else 	// ako je CLOSED
  {
	  map[element.x][element.y].k_cost_int = std::min(map[element.x][element.y].h_cost_int, h_new);
  }
  map[element.x][element.y].h_cost_int = h_new;
  if ((map[element.x][element.y].k_cost_int>OBSTACLE)&&(h_new>OBSTACLE)){
	  printf("fat both (%d,%d)\n",element.x,element.y);
	  pipodjednom=true;
  }
  map[element.x][element.y].time_stamp = time_stamp_counter;
#if DSTAR_REVERSE
#if (BEZ_HEU==0)
	map[element.x][element.y].g_cost_int=map[element.x][element.y].h_cost_int_reverse;
#endif
  map[element.x][element.y].total_cost_int = map[element.x][element.y].h_cost_int_reverse + map[element.x][element.y].h_cost_int;
  
#else
map[element.x][element.y].g_cost_int = pathCostEstimate( element, StartRacunac );
  map[element.x][element.y].total_cost_int = map[element.x][element.y].k_cost_int + map[element.x][element.y].g_cost_int;
  map[element.x][element.y].total_cost_int_biased = map[element.x][element.y].total_cost_int + dcurr;
#endif
  map[element.x][element.y].tag = OPEN;
  map[element.x][element.y].skupine = 0;
//   lista[NumElemLista]=element;	//tu se dodaje stanje
  NumElemLista++;     //povecali smo brojac za listu
  azurirani++;
#if USEBUCKET
  I_point* newelement;
  newelement = (I_point *) malloc(sizeof(I_point));
  newelement->x=element.x;
  newelement->y=element.y;
  newelement->th=element.th;
  queue.push(map[newelement->x][newelement->y].k_cost_int, newelement);
#else
  if (!(dodaj (&glava, element))){//dodavanje u pravu listu
	  if ( (logfile = fopen("komb","a")) == NULL )
		  printf("Error! komb file couldn't be opened.");
	  else{
		  fprintf(logfile,"insertNode> nema mjesta za stvaranje novog cvora\n");
		  fclose(logfile);
	  }
  }
#endif
/*  ispisi (glava);
  ispisiListu();*/
#if ((DSTAR_REVERSE==1)&&(BEZ_HEU==0)) //bio patak sa ilijem 1
  if (map[element.x][element.y].tag_reverse==OPEN){
	  if (!(brisi(&glava_reverse, element))){//brisanje iz prave liste
		  if ( (logfile = fopen("komb","a")) == NULL )
			  printf("Error! komb file couldn't be opened.");
		  else{
			  fprintf(logfile,"insertNode> nema trazenog elementa u pravoj listi\n");
			  fclose(logfile);
		  }
	  }
	  if (!(dodaj_reverse (&glava_reverse, element))){//dodavanje u pravu listu
		  if ( (logfile = fopen("komb","a")) == NULL )
			  printf("Error! komb file couldn't be opened.");
		  else{
			  fprintf(logfile,"insertNode> nema mjesta za stvaranje novog cvora\n");
			  fclose(logfile);
		  }
	  }
  }
#endif
}

void    DStar::insertNodeOri( I_point element, int h_new )
{
  if (map[element.x][element.y].tagOri[element.th]==NEW)	//NEW (nikad bio na listi)
  {
		map[element.x][element.y].k_cost_intOri[element.th] = h_new;
  }
	else if (map[element.x][element.y].tagOri[element.th]==OPEN)	//ako je OPEN
	{
		map[element.x][element.y].k_cost_intOri[element.th] = std::min(map[element.x][element.y].k_cost_intOri[element.th], h_new);
//	if (!(brisi(&glava, element))){//brisanje iz prave liste
//		if ( (logfile = fopen("komb","a")) == NULL )
//			printf("Error! komb file couldn't be opened.");
//		else{
//			fprintf(logfile,"insertNode> nema trazenog elementa u pravoj listi\n");
//			fclose(logfile);
//		}
//	}
//	NumElemLista--;
  }
	else 	// ako je CLOSED
  {
	  map[element.x][element.y].k_cost_intOri[element.th] = std::min(map[element.x][element.y].h_cost_intOri[element.th], h_new);
  }
  map[element.x][element.y].h_cost_intOri[element.th] = h_new;
  if ((map[element.x][element.y].k_cost_intOri[element.th]>OBSTACLE)&&(h_new>OBSTACLE)){
	  printf("fat both (%d,%d)\n",element.x,element.y);
  }
  map[element.x][element.y].tagOri[element.th] = OPEN;

  NumElemLista++;     //povecali smo brojac za listu
  azurirani++;

  I_point* newelement;
  newelement = (I_point *) malloc(sizeof(I_point));
  newelement->x=element.x;
  newelement->y=element.y;
  newelement->th=element.th;
  queue.push(map[newelement->x][newelement->y].k_cost_intOri[newelement->th], newelement);


//  if (!(dodajOri (&glava, element))){//dodavanje u pravu listu
//	  if ( (logfile = fopen("komb","a")) == NULL )
//		  printf("Error! komb file couldn't be opened.");
//	  else{
//		  fprintf(logfile,"insertNode> nema mjesta za stvaranje novog cvora\n");
//		  fclose(logfile);
//	  }
//  }
}


void    DStar::insertNodeReverse( I_point element, int h_new )
{
	if (map[element.x][element.y].tag_reverse==NEW)	//NEW (nikad bio na listi)
	{
		map[element.x][element.y].k_cost_int_reverse = h_new;
	}
	else if (map[element.x][element.y].tag_reverse==OPEN)	//ako je OPEN
	{
		map[element.x][element.y].k_cost_int_reverse = std::min(map[element.x][element.y].k_cost_int_reverse, h_new);
		if (!(brisi(&glava_reverse, element))){//brisanje iz prave liste
			if ( (logfile = fopen("komb","a")) == NULL )
				printf("Error! komb file couldn't be opened.");
			else{
				fprintf(logfile,"insertNode> nema trazenog elementa u pravoj listi\n");
				fclose(logfile);
			}
		}
		NumElemListaReverse--;
	}
	else 	// ako je CLOSED
	{
		map[element.x][element.y].k_cost_int_reverse = std::min(map[element.x][element.y].h_cost_int_reverse, h_new);
	}
	map[element.x][element.y].h_cost_int_reverse = h_new;
	if ((map[element.x][element.y].k_cost_int_reverse>OBSTACLE)&&(h_new>OBSTACLE)){
		printf("tu su oba debela (%d,%d)\n",element.x,element.y);
	}
	map[element.x][element.y].total_cost_int = map[element.x][element.y].h_cost_int_reverse + map[element.x][element.y].h_cost_int;
	map[element.x][element.y].tag_reverse = OPEN;
	map[element.x][element.y].time_stamp_forward=time_stamp_counter;
	map[element.x][element.y].skupine=0;
	NumElemListaReverse++;     //povecali smo brojac za listu
	azurirani_reverse++;
	if (!(dodaj_reverse (&glava_reverse, element))){//dodavanje u pravu listu
		if ( (logfile = fopen("komb","a")) == NULL )
			printf("Error! komb file couldn't be opened.");
		else{
			fprintf(logfile,"insertNode> nema mjesta za stvaranje novog cvora\n");
			fclose(logfile);
		}
	}
#if ((BEZ_HEU==0))
	if (map[element.x][element.y].tag==OPEN){
		map[element.x][element.y].g_cost_int=map[element.x][element.y].k_cost_int_reverse;
		if (!(brisi(&glava, element))){//brisanje iz prave liste
			if ( (logfile = fopen("komb","a")) == NULL )
				printf("Error! komb file couldn't be opened.");
			else{
				fprintf(logfile,"insertNode> nema trazenog elementa u pravoj listi\n");
				fclose(logfile);
			}
		}
		if (!(dodaj (&glava, element))){//dodavanje u pravu listu
			if ( (logfile = fopen("komb","a")) == NULL )
				printf("Error! komb file couldn't be opened.");
			else{
				fprintf(logfile,"insertNode> nema mjesta za stvaranje novog cvora\n");
				fclose(logfile);
			}
		}
	}
#endif
}

//POSTAVLJANJE COSTOVA PRIJELAZA IZ STANJA Y U X
      //zadnja verzija---c(x,y)=c(y,x) i to veci broj uvijek gledano iz cost mape
void DStar::arc_cost(int X_cell_x, int X_cell_y, int Y_cell_x, int Y_cell_y)
{
  c=map[ X_cell_x ][ X_cell_y ].traversal_cost;
	//c = std::max(map[ X_cell_x ][ X_cell_y ].traversal_cost, map[Y_cell_x ][ Y_cell_y ].traversal_cost);
//  c = (map[ X_cell_x ][ X_cell_y ].traversal_cost, map[Y_cell_x ][ Y_cell_y ].traversal_cost)/2; //no nice interpolation properties, it must be max
  if ((map[ X_cell_x ][ X_cell_y ].cspace_occupied==true)||(map[Y_cell_x ][ Y_cell_y ].cspace_occupied==true)||(map[ X_cell_x ][ X_cell_y ].prepreka_bool==true)||(map[Y_cell_x ][ Y_cell_y ].prepreka_bool==true)){
  	c=OBSTACLE;
  }
#if NO_COSTMASK
	if (c<OBSTACLE)
		c=EMPTYC;
#endif
}

void DStar::arc_costOri(I_point X, I_point Y)
{
  float costX, costY;
  

      IntPose pointX((X.x),(X.y),X.th);
      IntPose pointY((Y.x),(Y.y),Y.th);
#if USE3DCOST
  costX=cspace->getDistanceInCells(pointX);
  costY=cspace->getDistanceInCells(pointY);
  c=std::max(std::max(1.,(MAXDISTANCE-floor(costX))),std::max(1.,(MAXDISTANCE-floor(costY))));
  
//  printf("costX=%f costY=%f c=%d traversal_cost of X=%d, of Y=%d\n",costX,costY,c,map[ X.x ][ X.y ].traversal_cost, map[Y.x ][ Y.y ].traversal_cost);
#else
  c = std::max(map[ X.x ][ X.y ].traversal_cost, map[Y.x ][ Y.y ].traversal_cost);
  
  
#endif
	
#if RECTANGULAR
	if ((cspace->checkCollision(pointX))||(cspace->checkCollision(pointY))){
        	c=OBSTACLE;
        }else
#endif
        {

#if DSTAR3DORI || TEST3DSEARCH
  DStarSearchNode* xNode = cspace->getDStarSearchNode(pointX.x,pointX.y,pointX.theta);
  DStarSearchNode* yNode = cspace->getDStarSearchNode(pointY.x,pointY.y,pointY.theta);
  
  std::vector<OrientationInterval> intersects;
  OrientationInterval intersection;
  OrientationIntervals::computeIntersection(xNode->getOrientationInterval(), yNode->getOrientationInterval(), intersects);
  if (OrientationIntervals::containsOrientation(intersects[0],pointY.theta))// || OrientationIntervals::containsOrientation(intersects[0],pointX.theta))
  {
    intersection=intersects[0];
  }else{
    intersection=intersects[1];
  }
  int minUp,minLo;
      if (OrientationIntervals::getIntervalSize(intersection) < maxOri + 1){
        OrientationIntervals::computeOrientationDistanceInInterval(intersection,intersection.lower,pointY.theta,minLo);
        OrientationIntervals::computeOrientationDistanceInInterval(intersection,intersection.upper,pointY.theta,minUp);
        c = maxOri+1 - 2*std::min(minLo,minUp);
      }else{
        c=1;
      }
#endif        
        
        }
#if NO_COSTMASK
	if (c<OBSTACLE)
		c=EMPTYC;
#endif
}

    
//PROVJERA VALJANOSTI POZICIJE U MAPI
int		DStar::IsValid(int x, int y )
{
    if ( x < 0 || x >= MapSizeX || y < 0 || y >= MapSizeY)
				return 0;
      //ako je tocka unutar mape ali je zauzeta!!!!
      if ((map[ x ][ y ].prepreka_bool == true) || (map[ x ][ y ].cspace_occupied == true)){
        return 2;
        }
			return 1;
}

int DStar::IsValidOri(I_point X )
{
    if ( X.x < 0 || X.x >= MapSizeX || X.y < 0 || X.y >= MapSizeY )
				return 0;
      //ako je tocka unutar mape ali je zauzeta!!!!
#if RECTANGULAR
      IntPose checkme((X.x),(X.y),X.th);
	
	if (cspace->checkCollision(checkme)){
        return 2;
        }
#endif
			return 1;
}

int DStar::PathCostOri(I_point X )
{
  int pathcost=OBSTACLE;
    if ( X.x < 0 || X.x >= MapSizeX || X.y < 0 || X.y >= MapSizeY )
				return pathcost;
      //ako je tocka unutar mape ali je zauzeta!!!!
#if RECTANGULAR
      IntPose checkme((X.x),(X.y),X.th);
	if (cspace->checkCollision(checkme)){
        return pathcost;
        }
	
  DStarSearchNode* xNode = cspace->getDStarSearchNode(checkme.x,checkme.y,checkme.theta);
  if (xNode!=NULL) pathcost=xNode->h;
#endif
			return pathcost;
}

int    DStar::pathCostEstimate( I_point P1, I_point P2 )
{
#if (DSTAR_REVERSE==0)
	P2=prviStart;
#endif
#if BEZ_HEU
	return 0;
#else
            // variante 1
           int straight = std::max( abs( P1.x-P2.x ), abs( P1.y-P2.y ) );
            int diagonal = std::min( abs( P1.x-P2.x  ), abs( P1.y-P2.y ) );
            return ( diagonal * COSTDIAGONAL + ( straight - diagonal ) * COSTSTRAIGHT );
#endif
            // variante 2
        //    return min( abs( P1.x-P2.x ), abs( P1.y-P2.y ) ) * COSTSTRAIGHT * c;
            // variante 3
        //    return int(sqrt( ( P1.x-P2.x ) * ( P1.x-P2.x ) + ( P1.y-P2.y ) * ( P1.y-P2.y ) ) * COSTSTRAIGHT);
            // triviale variante
//            return 0;
}

int    DStar::pathCostEstimateDiagonalStraight( I_point P1, I_point P2 )
{
            // variante 1
	int straight = std::max( abs( P1.x-P2.x ), abs( P1.y-P2.y ) );
	int diagonal = std::min( abs( P1.x-P2.x  ), abs( P1.y-P2.y ) );
	return ( diagonal * COSTDIAGONAL + ( straight - diagonal ) * COSTSTRAIGHT );
}

//tu sad dodajem funkcije za pravu listu
// Dodavanje u listu sortiranu po rastucoj vrijednosti elementa
// vraca 1 ako uspije, inace 0
int DStar::dodaj (atom **glavap, I_point element) {
	atom *novi, *p;
	int el_fb,el_f,el_k,fb=0,f=0,k=0;
	I_point temp;
	if ((novi = (atom *) malloc(sizeof(atom))) == NULL) 
		return 0;
	novi->element = element;
	novi->sljed = NULL;
	el_k=map[element.x][element.y].k_cost_int;
#if BEZ_HEU
	el_f=0;
	el_fb=0;
#else
	el_fb=map[element.x][element.y].total_cost_int_biased;
	el_f=map[element.x][element.y].total_cost_int;
#endif
	if (*glavap != NULL){
		temp=(*glavap)->element;
		k=map[temp.x][temp.y].k_cost_int;
#if BEZ_HEU
		f=0;
		fb=0;
#else
		fb=map[temp.x][temp.y].total_cost_int_biased;
		f=map[temp.x][temp.y].total_cost_int;
#endif
	}
	if (*glavap == NULL || (LESS3(el_fb,el_f,el_k,fb,f,k))) {//(*glavap)->element >= element
    // Dodavanje na pocetak liste
		novi->sljed = *glavap;
		*glavap = novi;
	} else {
	// Dodavanje iza postojeceg elementa kad:
	// a) postojeci atom nema sljedeceg
	// b) element u sljedecem cvoru je veci ili jednak novome
		for (p = *glavap; p->sljed ; p = p->sljed){//;&& (p->sljed)->element < element, a ako to nije treba brejkat
			temp=(p->sljed)->element;//oprez! ako je strogo < onda jednaki elementi koji dolaze novi idu ispred, a ako je <= onda idu iza
			k=map[temp.x][temp.y].k_cost_int;
#if BEZ_HEU
			f=0;
			fb=0;
#else
			fb=map[temp.x][temp.y].total_cost_int_biased;//potrebno je usporediti ta dva slucaja!
			f=map[temp.x][temp.y].total_cost_int;

#endif
			if (LESS3(el_fb,el_f,el_k,fb,f,k))//  strogo <   LESSEQ3(el_fb,el_f,el_k,fb,f,k), <=  !LESSEQ3(fb,f,k,el_fb,el_f,el_k)
				break;
		}
		novi->sljed = p->sljed;
		p->sljed = novi;
	}
// 	NumAtom++;
	return 1;
}

int DStar::dodajOri (atom **glavap, I_point element) {
	atom *novi, *p;
	int el_k,k=0;
	I_point temp;
	if ((novi = (atom *) malloc(sizeof(atom))) == NULL) 
		return 0;
	novi->element = element;
	el_k=map[element.x][element.y].k_cost_intOri[element.th];
	if (*glavap != NULL){
		temp=(*glavap)->element;
		k=map[temp.x][temp.y].k_cost_intOri[temp.th];
	}
	if (*glavap == NULL || (el_k<=k)) {//(*glavap)->element >= element
    // Dodavanje na pocetak liste
		novi->sljed = *glavap;
		*glavap = novi;
	} else {
	// Dodavanje iza postojeceg elementa kad:
	// a) postojeci atom nema sljedeceg
	// b) element u sljedecem cvoru je veci ili jednak novome
		for (p = *glavap; p->sljed ; p = p->sljed){//;&& (p->sljed)->element < element, a ako to nije treba brejkat
			temp=(p->sljed)->element;//oprez! ako je strogo < onda jednaki elementi koji dolaze novi idu ispred, a ako je <= onda idu iza
			k=map[temp.x][temp.y].k_cost_intOri[temp.th];
			if ((el_k<=k))//  strogo <   LESSEQ3(el_fb,el_f,el_k,fb,f,k), <=  !LESSEQ3(fb,f,k,el_fb,el_f,el_k)
				break;
		}
		novi->sljed = p->sljed;
		p->sljed = novi;
	}
// 	NumAtom++;
	return 1;
}


int DStar::dodaj_reverse (atom **glavap, I_point element) {
	atom *novi, *p;
	int el_fb,el_f,el_k,fb=0,f=0,k=0;
	I_point temp;
	if ((novi = (atom *) malloc(sizeof(atom))) == NULL)
		return 0;
	novi->element = element;
	el_fb=0;
	el_k=map[element.x][element.y].k_cost_int_reverse;
#if BEZ_HEU && 1
	el_f=0;
#else
	if (prviput){
		el_f=0;
	}else{
		el_f=el_k+map[element.x][element.y].h_cost_int;
	}
#endif
	if (*glavap != NULL){
		temp=(*glavap)->element;
		fb=0;
		k=map[temp.x][temp.y].k_cost_int_reverse;
#if BEZ_HEU && 1
		f=0;
#else
		if (prviput){
			f=0;
		}else{
			f=k+map[temp.x][temp.y].h_cost_int;
		}
#endif
	}
	if (*glavap == NULL || (LESSEQ3(el_fb,el_f,el_k,fb,f,k))) {//(*glavap)->element >= element
    // Dodavanje na pocetak liste
		novi->sljed = *glavap;
		*glavap = novi;
	} else {
	// Dodavanje iza postojeceg elementa kad:
	// a) postojeci atom nema sljedeceg
	// b) element u sljedecem cvoru je veci ili jednak novome
		for (p = *glavap; p->sljed ; p = p->sljed){//;&& (p->sljed)->element < element, a ako to nije treba brejkat
			temp=(p->sljed)->element;//oprez! ako je strogo < onda jednaki elementi koji dolaze novi idu ispred, a ako je <= onda idu iza
			fb=0;//potrebno je usporediti ta dva slucaja!
			k=map[temp.x][temp.y].k_cost_int_reverse;
#if BEZ_HEU && 1
			f=0;
#else
			if (prviput){
				f=0;
			}else{
				f=k+map[temp.x][temp.y].h_cost_int;
			}
#endif
			if (LESSEQ3(el_fb,el_f,el_k,fb,f,k))//  strogo <   LESSEQ3(el_fb,el_f,el_k,fb,f,k), <=  !LESSEQ3(fb,f,k,el_fb,el_f,el_k)
				break;
		}
		novi->sljed = p->sljed;
		p->sljed = novi;
	}
	return 1;
}

// trazenje elementa liste
// vraca pokazivac na trazeni element ili NULL ako ga ne nadje
atom *DStar::trazi_reverse (atom *glava, int stamp) {
	atom *p;
	for (p = glava; p != NULL; p = p->sljed) {
/*		if (p ->element == element)
			return p;*/
		if (map[p->element.x][p->element.y].time_stamp_forward!=stamp)
			return p;
	}
	return NULL;
}

// trazenje elementa liste
// vraca pokazivac na trazeni element ili NULL ako ga ne nadje
atom *DStar::trazi (atom *glava, int stamp) {
	atom *p;
	for (p = glava; p != NULL; p = p->sljed) {
/*		if (p ->element == element)
		return p;*/
		if (map[p->element.x][p->element.y].time_stamp!=stamp)
			return p;
	}
	return NULL;
}

// Brisanje elementa liste po kljucu
// Objedinjuje trazenje i brisanje
int DStar::brisi (atom **glavap, I_point element) {
	atom *p;
#if DSTAR3D
	for (; *glavap && (((*glavap)->element.x != element.x)||((*glavap)->element.y != element.y)||((*glavap)->element.th != element.th)); glavap = &((*glavap)->sljed));
#else
	for (; *glavap && (((*glavap)->element.x != element.x)||((*glavap)->element.y != element.y)); glavap = &((*glavap)->sljed));
#endif
	if (*glavap) {
		p = *glavap;
// 		map[p->element.x][p->element.y].tag=CLOSED;
		*glavap = (*glavap)->sljed;
		free (p);
// 		NumAtom--;
// 		NumElemLista--;
		return 1;
	} else {
		return 0;
	}
}

int DStar::brisi_do_costa (atom **glavap, int h_cost) {//, int gh_cost) {
	atom *p;
	while (*glavap){
		p=*glavap;
// 		if ((map[(p->element.x)][(p->element.y)].k_cost_int>h_cost) ||(!(LESSEQ((map[(p->element.x)][(p->element.y)].k_cost_int_reverse+pathCostEstimate(p->element,Goal)),map[(p->element.x)][(p->element.y)].k_cost_int_reverse,(gh_cost+map[GoalRacunac.x][GoalRacunac.y].h_cost_int),gh_cost)))){
	if ((map[(p->element.x)][(p->element.y)].k_cost_int>h_cost)){
// 			printf("lista reverse: veci cost ima element (%d,%d): f=%d, k=%d, Gf=%d, Gh=%d\n",(p)->element.x,(p)->element.y,(map[(p->element.x)][(p->element.y)].k_cost_int_reverse+pathCostEstimate(p->element,Goal)),map[(p->element.x)][(p->element.y)].k_cost_int_reverse,(gh_cost+map[GoalRacunac.x][GoalRacunac.y].h_cost_int),gh_cost);
// 			printf("lista reverse: veci cost ima element (%d,%d): k=%d, h=%d\n",(p)->element.x,(p)->element.y,map[(p->element.x)][(p->element.y)].k_cost_int,(h_cost));
			map[p->element.x][p->element.y].tag_reverse=CLOSED;
			*glavap = (*glavap)->sljed;//nema vise ovog nego je to sad sljedeci, u slucaju zadnjeg to je null, a ovaj ispod jos jednom to radi pa ne valja
// 			glavap=&((*glavap)->sljed);
			glavap=&((*glavap));
			free(p);
			NumElemListaReverse--;
		}else{
			glavap=&((*glavap)->sljed);
		}
	}
	return 1;
}

void DStar::ispisi (atom *glava) {
	 atom *p;
	 for (p = glava; p != NULL; p = p->sljed) {
		 printf ("Na adresi %p je (%d,%d) koji gleda na %p\n",p, p->element.x,p->element.y, p->sljed);
// 		 printf ("prava  (%d,%d) fb=%d, f=%d, k=%d\n", p->element.x,p->element.y, map[p->element.x][p->element.y].total_cost_int_biased,map[p->element.x][p->element.y].total_cost_int,map[p->element.x][p->element.y].k_cost_int);
	 }
 }


 //ovo je za Witkowski algoritam, brojace i postavljanje na closed moram pisati izvan jer imam vise takvih lista
 // dodaje element u red, vraca 1 ako uspije, inace 0 
 //tu je nekakva greska, buni se o konverziji void pointera...
int DStar::DodajURed (I_point element, atom **ulaz, atom **izlaz) {
	 atom *novi;
	 if ((novi = (atom *) malloc(sizeof(atom))) == NULL) 
		 return 0;
	 novi->element = element;
	 novi->sljed = NULL;
	 if (*izlaz == NULL) {
		 *izlaz = novi;// ako je red bio prazan
	 } else {
		 (*ulaz)->sljed = novi;	// inace, stavi na kraj
	 }
	 *ulaz = novi;			// zapamti zadnjeg
	 return 1;
 }
 int DStar::DodajURedSortirano (I_point element, atom **ulaz, atom **izlaz, int smjer) {
	 atom *novi, *p;
	 int el_f,el_k,f=0,k=0;
	 I_point temp;
	 if ((novi = (atom *) malloc(sizeof(atom))) == NULL) 
		 return 0;
	 novi->element = element;
	 novi->sljed = NULL;
	 el_f=map[element.x][element.y].cost_sum;
	 if (smjer==1){
		 el_k=map[element.x][element.y].cost_k_forward;
	 }else{
		 el_k=map[element.x][element.y].cost_k_backward;
	 }
	 if (el_f>OBSTACLE){
		 el_f=OBSTACLE;
	 }
	 if (*izlaz != NULL){
		 temp=(*izlaz)->element;
		 f=map[temp.x][temp.y].cost_sum;
		 if (smjer==1){
			 k=map[temp.x][temp.y].cost_k_forward;
		 }else{
			 k=map[temp.x][temp.y].cost_k_backward;
		 }
		 if (f>OBSTACLE){
			 f=OBSTACLE;
		 }
	 }else{
		 *ulaz = novi;			// zapamti zadnjeg
	 }
	 if ((*izlaz == NULL)|| (LESSEQ(el_f,el_k,f,k))) {
		 novi->sljed = *izlaz;
		 *izlaz = novi;// ako je red bio prazan
	 } else {
		 for (p = *izlaz; p->sljed ; p = p->sljed){//;&& (p->sljed)->element < element, a ako to nije treba brejkat
			 temp=(p->sljed)->element;//oprez! ako je strogo < onda jednaki elementi koji dolaze novi idu ispred, a ako je <= onda idu iza
			 f=map[temp.x][temp.y].cost_sum;
			 if (smjer==1){
				 k=map[temp.x][temp.y].cost_k_forward;
			 }else{
				 k=map[temp.x][temp.y].cost_k_backward;
			 }
			 if (f>OBSTACLE){
				 f=OBSTACLE;
			 }
			 if (LESSEQ(el_f,el_k,f,k))//  strogo <   LESSEQ3(el_fb,el_f,el_k,fb,f,k), <=  !LESSEQ3(fb,f,k,el_fb,el_f,el_k)
				 break;
		 }
		 if (!(p->sljed)){
			 *ulaz = novi;			// zapamti zadnjeg
		 }
		 novi->sljed = p->sljed;
		 p->sljed = novi;
	 }
/*	 for (p = *izlaz; p->sljed; p = p->sljed) {
		 temp=(p->sljed)->element;
		 el_f=map[p->element.x][p->element.y].cost_sum;
		 if (smjer==1){
			 el_k=map[p->element.x][p->element.y].cost_k_forward;
		 }else{
			 el_k=map[p->element.x][p->element.y].cost_k_backward;
		 }
		 if (el_f>OBSTACLE){
			 el_f=OBSTACLE;
		 }
		 f=map[temp.x][temp.y].cost_sum;
		 if (smjer==1){
			 k=map[temp.x][temp.y].cost_k_forward;
		 }else{
			 k=map[temp.x][temp.y].cost_k_backward;
		 }
		 if (f>OBSTACLE){
			 f=OBSTACLE;
		 }
		 if (!(LESSEQ(el_f,el_k,f,k))){
			 printf("nije uzlazni niz\n");
			 ispisiRed(*izlaz);
			 break;
		 }
	 } */
	 return 1;
 }
 // uklanja element iz reda, vraca 1 ako uspije, inace 0 
 int DStar::SkiniIzReda (atom **ulaz, atom **izlaz) {
	 atom *stari;
	 if (*izlaz) {						// ako red nije prazan 
		 stari = *izlaz;					// zapamti trenutni izlaz
		 *izlaz = (*izlaz)->sljed;		// novi izlaz
		 free (stari);					// oslobodi memoriju skinutog
		 if (*izlaz == NULL){
		 	*ulaz = NULL; // prazan red
			 printf("prazan red, NumElemLista_forward=%d, NumElemLista_backward=%d\n",NumElemLista_forward,NumElemLista_backward);
		 }
		 return 1;
	 }
	 return 0;
 }
// Brisanje elementa liste po kljucu - to mi treba za one koji su OPEN da ih premjestim
// Objedinjuje trazenje i brisanje
 //postavljanje na closed moram radit vani i umanjivanje brojaca elemenata
 int DStar::brisiIzReda (I_point element, atom **ulaz, atom **izlaz) {
	 atom *p;//u for petlji idemo do predzadnjeg
	 for (; (((*izlaz)->sljed)->sljed) && (((*izlaz)->element.x != element.x)||((*izlaz)->element.y != element.y)); izlaz = &((*izlaz)->sljed));
	 if (((*izlaz)->element.x == element.x)&&((*izlaz)->element.y == element.y)) {//ako je ispred predzadnjeg ili predzadnji (mogu oba uvjeta biti neispunjena)
		 p = *izlaz;
		 *izlaz = (*izlaz)->sljed;
		 free (p);
		 return 1;
	 } else {//inace je zadnji ili ga nema
		 if ((((*izlaz)->sljed)->element.x == element.x)&&(((*izlaz)->sljed)->element.y == element.y)) {
			 *ulaz=*izlaz;//ulaz je predzadnji
			 izlaz=&((*izlaz)->sljed);
			 p = *izlaz;
			 *izlaz = (*izlaz)->sljed;
			 free (p);
// 			 ispisiRed(izlaz_forward);
// 			 ispisiRed(izlaz_backward);
			 return 1;
		 }
	 }
	 return 0;
 }
 //trazim po costu za backward i forward svaki zasebno, smjer odredjuje za koji
 //koristim globalne varijable   Robot_sum_backward,Robot_backward i tako i za forward
 //funkcija radi za dva slucaja, inicijalno racunanje i daljnja racunanja
 //radi i brisanje tog elementa za daljnja racunanja, zakomentirano ako radi kao pravi red
 int DStar::traziCost (atom **ulaz, atom **izlaz, I_point *element, int smjer) {
// 	 atom *p;
	 int f,k, Rf, Rh;
	 if (smjer==1){
		 Rh=map[Goal.x][Goal.y].cost_forward;
		 Rf=Rh+map[Goal.x][Goal.y].cost_backward;
	 }else if (smjer==-1){
		 Rh=map[Start.x][Start.y].cost_backward;
		 Rf=Rh+map[Start.x][Start.y].cost_forward;
	 }
	 if (Rf>OBSTACLE){
		 Rf=OBSTACLE;
	 }
	 for (int i=0; *izlaz; izlaz = &((*izlaz)->sljed),i++) {
		 *element=(*izlaz)->element;
		 f=map[(*element).x][(*element).y].cost_sum;
		 if (smjer==1){
// 			 k=map[(*element).x][(*element).y].cost_k_forward;
			k=map[(*element).x][(*element).y].cost_forward;
		 }else if (smjer==-1){
// 			 k=map[(*element).x][(*element).y].cost_k_backward;
			 k=map[(*element).x][(*element).y].cost_backward;
		 }
		 if (f>OBSTACLE){
			 f=OBSTACLE;
		 }
// 		if (LESSEQ(f,k,Rf,Rh)){
		if (k<=Rh){
		//brisanje iz reda, vraca se element preko pointera pa se njega vani onda umece
/*				 p = *izlaz;
				 *izlaz = (*izlaz)->sljed;
				 free (p);
				 if (*izlaz == NULL) *ulaz = NULL; // prazan red*/
// 			printf("f=%d, k=%d, Rf=%d, Rh=%d\n",f,k,Rf,Rh);
			return i;
		}
	 }
	 return 0;
 }
 
 void DStar::ispisiRed (atom *glava) {
	 atom *p;
	 printf("ispisujem red:\n");
	 for (p = glava; p != NULL; p = p->sljed) {
// 		 printf ("Na adresi %p je (%d,%d) koji gleda na %p\n",p, p->element.x,p->element.y, p->sljed);
		 printf ("(%d,%d) f=%d, kf=%d/kb=%d\n", p->element.x,p->element.y,map[p->element.x][p->element.y].cost_sum,map[p->element.x][p->element.y].cost_forward,map[p->element.x][p->element.y].cost_backward);
	 }
 }

 
void DStar::Free()
{

  for (int i=0; i<MapSizeX; i++){
    free(map[i]);
  }
    free(map);
	atom *slj=NULL;
	while (glava){
		slj=glava;
		glava=glava->sljed;
		free(slj);
// 		printf("mujo\n");
	}
// 	while (brisi (&glava, (glava)->element));
	while (SkiniIzReda (&ulaz_forward, &izlaz_forward));
	while (SkiniIzReda (&ulaz_backward, &izlaz_backward));
	if(path) free(path);
}


void DStar::reset()
{
	dcurr=0; //odmak od startne pozicije tijekom voznje
// 	brojac_izracuna=0;
	brojac_izracunaWitkowski=0;oznakaZaCrtanje=0;
  NumElemLista=0;
  NumElemListaReverse=0;
  NumAtom=0;
  NumElemLista_forward=0;
  NumElemLista_backward=0;
  broj_for=0; broj_bac=0;max_broj_cvorova=0;
  azurirani_forward=0;//inicijalizacija
  azurirani_backward=0;//inicijalizacija
//   NumElemPromjena=0;
  NumElemPunjenja=0;
  NumElemPraznjenja=0;
  pipodjednom=false;
  MinCostElemLista.x=-1;MinCostElemLista.y=-1;
  Start.x=-1; Start.y=-1;
  StartRacunac.x=-1;StartRacunac.y=-1;
  GoalRacunac.x=-1;
  GoalRacunac.y=-1;
  w0.x=-1; w0.y=-1;
  preslik.x=-1.;
  preslik.y=-1.;
  los_cell_num=6;
  StartCurr.x=-1; StartCurr.y=-1;
  Goal.x=-1;Goal.y=-1;
  prepreka.x=-1;prepreka.y=-1;
  nemanext.x=-1;nemanext.y=-1;
  petlja.x=-1;petlja.y=-1;
  time_stamp_counter=0;
  old_path_counter=0;
  Robot_f_biased=OBSTACLE;
  Robot_f=OBSTACLE;
  Robot_h=OBSTACLE;
  prviput=1;
  prviputWitkowski=1;
  inicijalni_put=0;
  promjena=0;
  racunaoupromjeni=0;
  novoTeziste=false;
  Start_zauzet=false;
  watchdog_counter=0;
  PathExists=false;
  PathLength=-1;
  PathLengthStari=1;
  PathLengthinicijalni=-1;
  c=1;
  goal_wit_old.x=-1;
  goal_wit_old.y=-1;
 //sve se resetira osim vrijednosti s preprekama i costovima, to se iz GM-a prepisuje bio on pun ili prazan
  for (int i=0; i<MapSizeX; i++){
    for (int j=0; j<MapSizeY; j++){
      map[i][j]._next.x=-1;
		  map[i][j]._next.y=-1;
		  map[i][j]._next_reverse.x=-1; map[i][j]._next_reverse.y=-1; //sluze kao "NULL" vrijednosti
		  map[i][j]._maska.x=-1;
		  map[i][j]._maska.y=-1;
		  map[i][j].preprekaokolo=0;
		  map[i][j].preprekablizu=0;
		  map[i][j].h_cost_int=OBSTACLE;  //"beskonacni" cost do cilja
		  map[i][j].k_cost_int=OBSTACLE;
		  map[i][j].k_cost_int_reverse=OBSTACLE;
		  map[i][j].h_cost_int_reverse=OBSTACLE;
		  map[i][j].g_cost_int=0;
	  map[i][j].total_cost_int=OBSTACLE;  //total cost
      map[i][j].total_cost_int_biased=OBSTACLE;
	  map[i][j].promjena=0;
      map[i][j].time_stamp=0;
      map[i][j].path_counter=0;
      map[i][j].time_stamp_forward=0;
      map[i][j].tag=NEW;
      map[i][j].tag_reverse=NEW;
      map[i][j].traversal_cost=EMPTYC;
      map[i][j].traversal_cost_stari=EMPTYC;
      map[i][j].prepreka_bool=false;
      map[i][j].cspace_occupied=false;
      
      for (int k=0; k<MAXORI; k++){
      map[i][j].tagOri[k]=NEW;
      map[i][j].h_cost_intOri[k]=OBSTACLE;
      map[i][j].k_cost_intOri[k]=OBSTACLE;
      map[i][j]._nextOri[k].x=-1;
      }

	  //witkowski
	  map[i][j].cost_forward=OBSTACLE;
	  map[i][j].cost_backward=OBSTACLE;
	  map[i][j].cost_sum=OBSTACLE;
	  map[i][j].tag_forward=NEW;
	  map[i][j].tag_backward=NEW;
	  map[i][j]._next_forward.x=-1;
	  map[i][j]._next_forward.y=-1;
	  map[i][j]._next_backward.x=-1;
	  map[i][j]._next_backward.y=-1;
	  map[i][j].skupine=0;
	  map[i][j].cvorovi=0;
	}
  }
  glava = NULL;
  glava_reverse = NULL;
  ulaz_forward = NULL;
  ulaz_backward = NULL;
  izlaz_forward = NULL;
  izlaz_backward = NULL;
  novi_ulaz_forward= NULL;		// glava liste
  novi_ulaz_backward= NULL;		// glava liste
  novi_izlaz_forward= NULL;		// glava liste
  novi_izlaz_backward= NULL;		// glava liste

  if(0)
  for(int i=0; i<(int(MapSizeX)*int(MapSizeY)); i++)
  {
    path[i].x=-1; path[i].y=1;
    pathinicijalni[i].x=-1; pathinicijalni[i].y=-1;
    pathstari[i].x=-1; pathstari[i].y=-1;
// 	  listapromjena[i].x=-1; listapromjena[i].y=-1;
	  izracuni[i]=-1; azurirani_polje[i]=-1;f_biased_putanje[i]=-1; f_putanje[i]=-1; h_putanje[i]=-1;
    broj_iteracija[i]=-1;broj_cvorova_na_listi[i]=-1;max_broj_cvorova_na_listi[i]=-1;cijena_puta[i]=-1;duljina_puta[i]=-1;
	izracuniWitkowski[i]=-1; vremena_skupine[i]=-1; azurirani_forward_polje[i]=-1;azurirani_backward_polje[i]=-1; cijena_puta_forward[i]=-1; duljina_puta_forward[i]=-1;
    broj_iteracijaWitkowski[i]=-1;broj_cvorova_na_listi_forward[i]=-1;max_broj_cvorova_na_listi_forward[i]=-1;cijena_puta_backward[i]=-1;duljina_puta_backward[i]=-1;
    broj_cvorova_na_listi_backward[i]=-1;max_broj_cvorova_na_listi_backward[i]=-1;
  }
  
}

void DStar::resetReverse()
{
	prviput=1;
 //sve se resetira osim vrijednosti s preprekama i costovima, to se iz GM-a prepisuje bio on pun ili prazan
	for (int i=0; i<MapSizeX; i++){
		for (int j=0; j<MapSizeY; j++){
		  	map[i][j].k_cost_int_reverse=OBSTACLE;
		  	map[i][j].h_cost_int_reverse=OBSTACLE;
		  	map[i][j].tag_reverse=NEW;
		  	map[i][j]._next_reverse.x=-1; map[i][j]._next_reverse.y=-1; //sluze kao "NULL" vrijednosti
			map[i][j].skupine=0;
		}
	}
  	glava_reverse = NULL;
}


void DStar::resetWitkowski()
{
// 	brojac_izracunaWitkowski=0;
	NumElemLista_forward=0;
	NumElemLista_backward=0;
	broj_for=0; broj_bac=0;
	azurirani_forward=0;//inicijalizacija
	azurirani_backward=0;//inicijalizacija
	watchdog_counter=0;
	prviputWitkowski=1;
 //sve se resetira osim vrijednosti s preprekama i costovima, to se iz GM-a prepisuje bio on pun ili prazan
	for (int i=0; i<MapSizeX; i++){
		for (int j=0; j<MapSizeY; j++){
			map[i][j].time_stamp_forward=0;
			map[i][j].cost_forward=OBSTACLE;
			map[i][j].cost_backward=OBSTACLE;
			map[i][j].cost_sum=OBSTACLE;
			map[i][j].tag_forward=NEW;
			map[i][j].tag_backward=NEW;
			map[i][j]._next_forward.x=-1;
			map[i][j]._next_forward.y=-1;
			map[i][j]._next_backward.x=-1;
			map[i][j]._next_backward.y=-1;
			map[i][j].skupine=0;
			map[i][j].cvorovi=0;
		}
	}
	ulaz_forward = NULL;
	ulaz_backward = NULL;
	izlaz_forward = NULL;
	izlaz_backward = NULL;  
}

void DStar::traziSusjedniPut(int brojPolja) {
	//mogla bih ubrzati: prvo popuniti total_cost s h_cost_intovima, posortirati i po redu gledati koji je bliski, kad ga nadjem onda provjerim da li je put ok
	I_point temp1,temp2;
// 	I_point trazipetlju[10];//tu ce se pamtiti zadnjih 10 polja puta iz susjeda. ako se neki ponavlja onda javi da je petlja i izadji van iz tog susjeda. cirkularno polje
	I_point pomocni[10];//, pomocniNajbolji[10]; //hocu pamtit pomocni put od 3 polja i najbolji put od 3 polja
	int total_cost[8*brojPolja];//traversal_cost,
	int maksic;//h,
	int duljina_puta[8*brojPolja], preprekailpetlja[8*brojPolja];
	int xpomak[8*brojPolja], ypomak[8*brojPolja], sortd[8*brojPolja];
	int pom,minsortd;
	int dobar;//pocetnarazlika,trenutnarazlika,
	int brojpoljaunaprijed;//broj polja koji moze robot prijeci u jednom ciklusu zaokruzeno na gore
	brojpoljaunaprijed=(int)ceil(V_MAX*STEP/CELL_DIM)+brojPolja;
// 	petlja.x=-1;petlja.y=-1;//polje gdje je petlja
	switch(brojPolja){
		case 1:
			for ( int d = 0; d < 8*brojPolja; d++ ) {
				xpomak[d]=xofs[d];
				ypomak[d]=yofs[d];	
			}			
			break;
		case 2:
			for ( int d = 0; d < 8*brojPolja; d++ ) {
				xpomak[d]=x2ofs[d];
				ypomak[d]=y2ofs[d];	
			}
			break;
		case 3:
			for ( int d = 0; d < 8*brojPolja; d++ ) {
				xpomak[d]=x3ofs[d];
				ypomak[d]=y3ofs[d];	
			}			
			break;
		case 4:
			for ( int d = 0; d < 8*brojPolja; d++ ) {
				xpomak[d]=x4ofs[d];
				ypomak[d]=y4ofs[d];	
			}			
			break;
	}
	for ( int d = 0; d < 8*brojPolja; d++ ) {
		sortd[d]=d;
		//tu to dodajem
// 		temp1.x=StartRacunac.x+xpomak[d];
// 		temp1.y=StartRacunac.y+ypomak[d];
		temp1.x=Start.x+xpomak[d];
		temp1.y=Start.y+ypomak[d];
		duljina_puta[d]=PathLength;
		total_cost[d]=map[temp1.x][temp1.y].h_cost_int;
		preprekailpetlja[d]=0;
		int robot_f,robot_h;
// 		robot_h=map[StartRacunac.x][StartRacunac.y].h_cost_int;
// 		robot_f=robot_h+map[StartRacunac.x][StartRacunac.y].g_cost_int;
		robot_h=map[Start.x][Start.y].h_cost_int;
		robot_f=robot_h+map[Start.x][Start.y].g_cost_int;
		if (!(LESSEQ(map[temp1.x][temp1.y].total_cost_int,map[temp1.x][temp1.y].k_cost_int,robot_f,robot_h))||(total_cost[d]>=OBSTACLE)) {
// 			printf("ne uracunavam susjeda %d jer mu je veci f,k ili h>=OBSTACLE\n",d);
			preprekailpetlja[d]=4;
			duljina_puta[d]=1;
		}else
			if ((prepreka.x!=-1)&&(nemanext.x!=-1)) {
				if ((temp1.x==najboljiPut[brojPolja].x)&&(temp1.y==najboljiPut[brojPolja].y)){
// 			printf("ne uracunavam susjeda %d jer se nalazi na najboljem putu\n",d);
			preprekailpetlja[d]=4;
			duljina_puta[d]=1;
		}else if (brojPolja>1){
			for (int i=0;i<br;i++){
				if ((temp1.x==susjediNebliskih[i].x)&&(temp1.y==susjediNebliskih[i].y)){
// 					printf("ne uracunavam susjeda %d jer je nastavak nebliskog\n",d);
					preprekailpetlja[d]=4;
					duljina_puta[d]=1;
					break;
				}
			}
		} 
		}
		//gledamo da li je u nekom nextu najboljiPut[]
		if ((preprekailpetlja[d]==0)&&(prepreka.x!=-1)&&(nemanext.x!=-1)){
			for (int j=0;((j<(2*brojPolja+1))&&(j<PathLength-5)&&(temp1.x!=-1));j++){
				temp2=map[temp1.x][temp1.y]._next;
			for (int i=7;((i>=brojPolja)&&(j<PathLength-5)&&(temp2.x!=-1));i--){
				if ((temp2.x==najboljiPut[i].x)&&(temp2.y==najboljiPut[i].y)) {
// 					printf("ne uracunavam susjeda %d jer mu je next na najboljem putu\n",d);
					preprekailpetlja[d]=4;
					break;
				}
			}
			if (preprekailpetlja[d]){
				break;
			}
			temp1=temp2;
			}
		}
	}
	br=0;
/*	for ( int d = 0; d < 8*brojPolja; d++ )
	{
      //odredjivanje pozicije novog susjeda
		temp1.x=StartRacunac.x+xpomak[d];
		temp1.y=StartRacunac.y+ypomak[d];
		h=map[temp1.x][temp1.y].h_cost_int;
		printf("nova pozicija=(%d,%d), f=h+g=%d, h=%d, f=k+g=%d, k=%d\n",temp1.x,temp1.y,(map[temp1.x][temp1.y].h_cost_int+map[temp1.x][temp1.y].g_cost_int),map[temp1.x][temp1.y].h_cost_int,map[temp1.x][temp1.y].total_cost_int,map[temp1.x][temp1.y].k_cost_int);
		duljina_puta[d]=1;total_cost[d]=0;preprekailpetlja[d]=0;
	  //prvo provjeravamo dal je put ok i kolko je dugacak
		while((temp1.x!=Goal.x)||(temp1.y!=Goal.y))
		{
			if ( IsValid( temp1.x, temp1.y )!=1 )
			{
				printf("DStar> Odriftali smo iz mape il je mozda prepreka!!! temp1.x=%d, temp1.y=%d, susjed %d.\n", temp1.x,temp1.y,d);
				if ( IsValid( temp1.x, temp1.y )==2 ) {
// 					prepreka.x=temp1.x;
// 					prepreka.y=temp1.y;
// 					if ( (logfile = fopen("komb","a")) == NULL )
// 						printf("Error! komb file couldn't be opened.");
// 					else{
// 						fprintf(logfile,"traziSusjedniPut> susjed %d. prepreka=(%d,%d)\n",d,prepreka.x,prepreka.y);
// 						fclose(logfile);
// 					}
				}
				preprekailpetlja[d]=1;//prepreka
				break;//izlazi iz while petlje van i trazi sljedeceg susjeda
			}
			else
			{
				temp2.x=map[temp1.x][temp1.y]._next.x;
				temp2.y=map[temp1.x][temp1.y]._next.y;
				if ((temp2.x==-1)||(temp2.y==-1)) {
					printf("susjed %d. nema next\n",d);
// 					if ( (logfile = fopen("komb","a")) == NULL )
// 						printf("Error! komb file couldn't be opened.");
// 					else{
// 						fprintf(logfile,"traziSusjedniPut> susjed %d. nema next=(%d,%d)\n",d,temp2.x,temp2.y);
// 						fclose(logfile);
// 					}
					preprekailpetlja[d]=2;//nema next
					break;//izlazi iz while petlje van i trazi sljedeceg susjeda
				}
				if ((temp1.x==temp2.x)||(temp1.y==temp2.y)){
					traversal_cost=COSTSTRAIGHT;
				} else {
					traversal_cost=COSTDIAGONAL;
				}
				arc_cost( temp2.x,temp2.y,temp1.x,temp1.y);
				if (c>=OBSTACLE) {
					printf("susjedni (%d.) put ima OBSTACLE u next-u.\n",d);
// 					if ( (logfile = fopen("komb","a")) == NULL )
// 						printf("Error! komb file couldn't be opened.");
// 					else{
// 						fprintf(logfile,"traziSusjedniPut> susjed %d. prepreka=(%d,%d)\n",d,temp2.x,temp2.y);
// 						fclose(logfile);
// 					}
					preprekailpetlja[d]=1;//prepreka
					break;//izlazi iz while petlje van i trazi sljedeceg susjeda
				}
				total_cost[d]+=traversal_cost*c;//racunam ukupni cost na putu
				if (duljina_puta[d]<=10){
					trazipetlju[duljina_puta[d]-1].x=temp1.x;
					trazipetlju[duljina_puta[d]-1].y=temp1.y;
					for (int i=0;i<duljina_puta[d];i++){
						if ((temp2.x==trazipetlju[i].x)&&(temp2.y==trazipetlju[i].y)){
							printf("jao petlja na susjednom (%d.) putu detektirana pomocu polja trazipetlju. duljina puta je %d\n",d,duljina_puta[d]);
// 							preprekailpetlja[d]=3;//petlja
// 							petlja.x=temp2.x;
// 							petlja.y=temp2.y;
// 							if ( (logfile = fopen("komb","a")) == NULL )
// 								printf("Error! komb file couldn't be opened.");
// 							else{
// 								fprintf(logfile,"traziSusjedniPut> susjed %d. petlja=(%d,%d)\n",d,petlja.x,petlja.y);
// 								fclose(logfile);
// 							}
							break;
						}
					}
				}else{
					trazipetlju[(duljina_puta[d] % 10) -1].x=temp1.x;
					trazipetlju[(duljina_puta[d] % 10) -1].y=temp1.y;
					for (int i=0;i<10;i++){
						if ((temp2.x==trazipetlju[i].x)&&(temp2.y==trazipetlju[i].y)){
							printf("jao petlja na susjednom (%d.) putu detektirana pomocu polja trazipetlju. duljina puta je %d\n",d,duljina_puta[d]);
							preprekailpetlja[d]=3;//petlja
// 							petlja.x=temp2.x;
// 							petlja.y=temp2.y;
// 							if ( (logfile = fopen("komb","a")) == NULL )
// 								printf("Error! komb file couldn't be opened.");
// 							else{
// 								fprintf(logfile,"traziSusjedniPut> susjed %d. petlja=(%d,%d)\n",d,petlja.x,petlja.y);
// 								fclose(logfile);
// 							}
							break;
						}
					}
				}
				if (preprekailpetlja[d]==3){
					break;
				}
				temp1.x=temp2.x;
				temp1.y=temp2.y;
				duljina_puta[d]++;
				if (duljina_puta[d]>MapSizeX*MapSizeY)
				{
					printf("jao petlja na susjednom (%d.) putu! time_stamp_counter=%d\n",d,time_stamp_counter);
					preprekailpetlja[d]=3;//petlja
					break;
				}
			}
		}//while
		if (total_cost[d]!=h){
			printf("nisu istiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii\n");
		}
		printf("(%d.) put, total_cost=%d duljina_puta=(%d)\n",d,total_cost[d],duljina_puta[d]);
	}//for*/
  //sad trazimo najmanji total_cost al ne smije bit s preprekom ili s petljom
	for ( int d = 0; d < 8*brojPolja; d++ )
	{
		for (int dd = d+1; dd<8*brojPolja;dd++) {
			if (((preprekailpetlja[sortd[dd]]==0)&&(total_cost[sortd[dd]]<total_cost[sortd[d]]))||(preprekailpetlja[sortd[d]]!=0)) {
				pom=sortd[d];
				sortd[d]=sortd[dd];
				sortd[dd]=pom;
			}
		}
	}
	minsortd=0;//sortd ima sortirane indekse susjeda tako da prvi clan polja koji je indeks susjeda odgovara najjeftinijem putu
/*	for ( int d = 0; d < 8*brojPolja; d++ )
		printf("sortirani: indeks %d, total_cost %d, preprekailpetlja %d\n",sortd[d],total_cost[sortd[d]],preprekailpetlja[sortd[d]]);*/
	dobar=1;//zbog inicijalnog puta
	if (!inicijalni_put) {
		if (duljina_puta[sortd[minsortd]]>9) {
			dobar=0;  //ako ne udje u while petlju da znam da nema slobodnih puteva
			while ((minsortd<8*brojPolja)&&(preprekailpetlja[sortd[minsortd]]==0)) {
// 				pomocni[0].x=StartRacunac.x+xpomak[sortd[minsortd]];
// 				pomocni[0].y=StartRacunac.y+ypomak[sortd[minsortd]];
				pomocni[0].x=Start.x+xpomak[sortd[minsortd]];
				pomocni[0].y=Start.y+ypomak[sortd[minsortd]];
/*				if (minsortd==0){
					pomocniNajbolji[0].x=pomocni[0].x;
					pomocniNajbolji[0].y=pomocni[0].y;
				}*/
				//printf("DS> susjed %d:\t0. (%d,%d)\n",sortd[minsortd],temp1.x,temp1.y);
// 				pocetnarazlika=(pomocni[0].x-pathstari[0].x)*(pomocni[0].x-pathstari[0].x)+(pomocni[0].y-pathstari[0].y)*(pomocni[0].y-pathstari[0].y);
// 				dobar=1;//da li se poklapa, inicijalno se svaki put poklapa
// 				int istaRazlika=0;
				double kut_prvi,kut_drugi,kut;
				int drugi=0;
				for (int i=1;(i<10)&&(pomocni[i-1].x!=-1);i++) {//i=0 je bilo
					pomocni[i].x=map[pomocni[i-1].x][pomocni[i-1].y]._next.x;
					pomocni[i].y=map[pomocni[i-1].x][pomocni[i-1].y]._next.y;
/*					if (minsortd==0){
						pomocniNajbolji[i].x=pomocni[i].x;
						pomocniNajbolji[i].y=pomocni[i].y;
					}*/
					if (i==1){
						kut_prvi=atan2((pomocni[i].y-pomocni[i-1].y),(pomocni[i].x-pomocni[i-1].x));
						kut_drugi=kut_prvi;//inicijalizacija ako nema drugog
// 						printf("kut_prvi=%f\n",kut_prvi);
						if ((kut_prvi==starikut_prvi)&&(brojPolja<2)){
							dobar=1;
							break;
						}
					}else{
						kut=atan2((pomocni[i].y-pomocni[i-1].y),(pomocni[i].x-pomocni[i-1].x));
						if ((kut!=kut_prvi)&&(!drugi)){
							kut_drugi=kut;
// 							printf("kut_drugi=%f, indeks=%d\n",kut_drugi,i);
							drugi=1;
							if (kut_drugi!=starikut_drugi){
								break;//inace vrtim for petlju do kraja da se zapisu pomocni[9]
							}
						}
					}
	//usporedjuje se prvo polje starog puta s prvim poljem novog puta, pa drugo s drugim itd.
/*					trenutnarazlika=(pomocni[i].x-pathstari[i].x)*(pomocni[i].x-pathstari[i].x)+(pomocni[i].y-pathstari[i].y)*(pomocni[i].y-pathstari[i].y);
					if ((trenutnarazlika==pocetnarazlika)){
						istaRazlika=i-istaRazlika;
						if ((istaRazlika==1)&&(i>1)){
							printf("ista razlika za redom 2 puta\n");
							break;
						}
					}
// 			  if (trenutnarazlika-pocetnarazlika>4)//(trenutnarazlika>4)  //5^2, trenutna razlika nije sqrt	
					if (trenutnarazlika>pocetnarazlika)
					{	
						if (trenutnarazlika>1){
							dobar=0;
							if (br<24){
								susjediNebliskih[br]=map[pomocni[0].x][pomocni[0].y]._next;
								br++;
							}
							minsortd++;
							break;
						}else{
							istaRazlika=0;//to je za odmaknute slicne puteve za 1
							pocetnarazlika=trenutnarazlika;
						}
// 						if (watchdog_counter==3){
// 							printf("DS> ne bliski put, susjed %d, cijena %d, do %d. indeksa: (%d,%d), (%d,%d), (%d,%d), (%d,%d)\n",sortd[minsortd],total_cost[sortd[minsortd]],i,pomocni[0].x,pomocni[0].y,pomocni[1].x,pomocni[1].y,pomocni[2].x,pomocni[2].y,pomocni[3].x,pomocni[3].y);
// 						}
					}*/
				}
				if ((!dobar)&&(kut_drugi==starikut_drugi)) {
					maksic=std::max(abs(pomocni[9].x-pathstari[9].x),abs(pomocni[9].y-pathstari[9].y));
// 					printf("susjed %d. MAX udalj=%d\n",sortd[minsortd],maksic);
					if (maksic<5){
						dobar=1;
					}
				}
				if (!dobar){
					if (br<24){
						susjediNebliskih[br]=map[pomocni[0].x][pomocni[0].y]._next;
						br++;
					}
					minsortd++;
				}
				if (dobar){
// 					if ((minsortd!=0)||(watchdog_counter==3)) {
// 						printf("nije najbolji nego je blizi starom putu, susjed %d, brojPolja %d\n",sortd[minsortd],brojPolja);
// 						printf("DS> Stari put: (%d,%d), (%d,%d), (%d,%d), (%d,%d)\n",pathstari[0].x,pathstari[0].y,pathstari[1].x,pathstari[1].y,pathstari[2].x,pathstari[2].y,pathstari[3].x,pathstari[3].y);
// 						printf("DS> Blizi put, susjed %d, cijena %d: (%d,%d), (%d,%d), (%d,%d), (%d,%d)\n",sortd[minsortd],total_cost[sortd[minsortd]],pomocni[0].x,pomocni[0].y,pomocni[1].x,pomocni[1].y,pomocni[2].x,pomocni[2].y,pomocni[3].x,pomocni[3].y);
// /*						bliziPut.x=pomocni[0].x;bliziPut.y=pomocni[0].y;najboljiPut.x=pomocniNajbolji[0].x;najboljiPut.y=pomocniNajbolji[0].y;oznakaZaCrtanje=1;
// 						stariPut.x=pathstari[0].x;stariPut.y=pathstari[0].y;
// 						if ((pathstari[0].x==bliziPut.x)&&(pathstari[0].y==bliziPut.y))
// 							oznakaZaCrtanje=0;*/
// 						printf("DS> Najbolji put, susjed %d, cijena %d: (%d,%d), (%d,%d), (%d,%d), (%d,%d)\n",sortd[0],total_cost[sortd[0]], pomocniNajbolji[0].x,pomocniNajbolji[0].y,pomocniNajbolji[1].x,pomocniNajbolji[1].y,pomocniNajbolji[2].x,pomocniNajbolji[2].y,pomocniNajbolji[3].x,pomocniNajbolji[3].y);
// /*						if ((total_cost[sortd[0]]<total_cost[sortd[minsortd]]-(EMPTYC+COST_MASK)*pathCostEstimate(pomocniNajbolji[0] ,pomocni[0] ))&&(drziStariPut==false)){
// 							printf("DS> !!!!blizi je skuplji od najboljeg, cijena izmedju blizeg i najboljeg je %d\n",pathCostEstimate(pomocniNajbolji[0] ,pomocni[0] ));
// 							minsortd=0;//uzimam ipak najboljeg
// 						}*/
// 					}else{
// 						printf("najbolji i blizi starom putu, cijena %d\n",total_cost[sortd[0]]);oznakaZaCrtanje=0;
// 					}
// 					printf("blizi starom putu, cijena %d\n",total_cost[sortd[minsortd]]);
					break;//ako je dobro poklapanje novog puta sa starim onda se ne gledaju ostali putevi
				}
			}
			if (!dobar) {
				minsortd=0;//ako nema nikakvih poklapanja sa starim putem onda najboljeg uzmemo
			}
		}
	}
	//printf("minsortd=%d\n",minsortd);
	if (preprekailpetlja[sortd[minsortd]]!=0) {
		printf("DS>brojPolja=%d, svi su losi, trazim malo dalje susjede i to oko starta.\n",brojPolja);
		if (watchdog_counter!=3)
			watchdog_counter=2;//na popravni
/*		if ((petlja.x!=-1)&&(petlja.y!=-1)&&(brojPolja==4)&&(watchdog_counter==2)) {
		  StartRacunac.x=petlja.x;
		  StartRacunac.y=petlja.y;//mijenjam StartRacunac
	  }*/
	} else {
		PathLength=duljina_puta[sortd[minsortd]];
   //getting path from StartRacunac to goal
// 		StartRacunac.x=StartRacunac.x+xpomak[sortd[minsortd]];//mijenjam StartRacunac
// 		StartRacunac.y=StartRacunac.y+ypomak[sortd[minsortd]];
		Start.x=Start.x+xpomak[sortd[minsortd]];//mijenjam Start
		Start.y=Start.y+ypomak[sortd[minsortd]];
		PathCost=total_cost[sortd[minsortd]];
		if ((!dobar)){
			printf("DS> brojPolja=%d, trazim dalje susjede\n",brojPolja);
			watchdog_counter=3;//na popravni s nadom
/*			if (brojPolja==1){
				printf("DS> Najbolji put, susjed %d, cijena %d: (%d,%d), (%d,%d), (%d,%d), (%d,%d)\n",sortd[0],total_cost[sortd[0]], pomocniNajbolji[0].x,pomocniNajbolji[0].y,pomocniNajbolji[1].x,pomocniNajbolji[1].y,pomocniNajbolji[2].x,pomocniNajbolji[2].y,pomocniNajbolji[3].x,pomocniNajbolji[3].y);
				printf("DS> stari put: (%d,%d), (%d,%d), (%d,%d), (%d,%d)\n", pathstari[0].x,pathstari[0].y,pathstari[1].x,pathstari[1].y,pathstari[2].x,pathstari[2].y,pathstari[3].x,pathstari[3].y);
			}*/
		}
		else {
		watchdog_counter=0;
		}
	}

}


bool    DStar::SearchPathWitkowski()
{
	I_point element; //onaj koji ima promjenu
// 	I_point point;  //i njegov susjed
// 	int sum_backward,sum_forward,k_backward,k_forward;//,i_cell,j_cell,cost,k_cost;
// 	int brojac_za_trazicost_backward=0,indeks_za_trazicost_backward=0,brojac_za_trazicost_forward=0,indeks_za_trazicost_forward=0;
  //mjeri vrijeme pocetka WIT algoritma
	if (gettimeofday(&timeStart, NULL) == 0)
	{
		mySecStart = timeStart.tv_sec;
		myMSecStart = timeStart.tv_usec / 1000;
	}
	radiponovo=0;
#if (KOMBINACIJA==0)
	if (promjena){
		for(int i=0; (i<NumElemPraznjenja); i++)
		{
			element.x = listapraznjenja[i].x;
			element.y = listapraznjenja[i].y;
			if ((map[element.x][element.y].promjena == 1))  // 1 oznacava promjenu na slobodno
			{//a pitamo za svaki slucaj da nije doletio neki za punjenje, sto nece biti slucaj al eto
				if ((radiponovo==0)&&(map[element.x][element.y].tag_forward==CLOSED)&&(map[element.x][element.y].tag_backward==CLOSED))
				{
					radiponovo=1;
				}  //if
			}//if za svaki slucaj
			map[listapraznjenja[i].x][listapraznjenja[i].y].promjena=11;//da azuriramo promjenu
		}//for po praznjenjima

		for(int i=0; (i<NumElemPunjenja); i++)
		{
			element.x = listapunjenja[i].x;
			element.y = listapunjenja[i].y;
			if ((map[element.x][element.y].promjena == 2))   //2 oznacava promjenu na zauzeto
			{//a pitamo za svaki slucaj da se ne obrise start onda bi ovaj imao 1
				if ((radiponovo==0)&&(map[element.x][element.y].tag_forward==CLOSED)&&(map[element.x][element.y].tag_backward==CLOSED))
				{
					radiponovo=1;
				}  //if
				map[listapunjenja[i].x][listapunjenja[i].y].promjena=22;
			}   //for po punjenjima
		}
		
		for(int i=0; (i<NumElemCostmaska); i++)
		{
			element.x = listacostmaska[i].x;
			element.y = listacostmaska[i].y;
			if ((map[element.x][element.y].traversal_cost!=map[element.x][element.y].traversal_cost_stari))
			{//a pitamo za svaki slucaj da nije doletio neki za punjenje, sto nece biti slucaj al eto
				if ((radiponovo==0)&&(map[element.x][element.y].tag_forward==CLOSED)&&(map[element.x][element.y].tag_backward==CLOSED)){
					radiponovo=1;
				}  //if
				map[element.x][element.y].promjena=55;//novi broj nigdje ne koristen, to je za slucaj da oba algoritma odjednom vrtimo
			}else{
			  //oznacimo da se radilo o istima
				map[element.x][element.y].promjena=11;
			}
		}//for po promjenama costmaske
		
		if (radiponovo){
			printf("WitkowskiSearchPath: promjena\n");
			resetWitkowski();
			racunaoupromjeni=1;
		}
	} //else if promjena
#else
	if ((racunaoupromjeni)||((goal_wit.x!=goal_wit_old.x)||(goal_wit.y!=goal_wit_old.y))){
		printf("WitkowskiSearchPath: racuna! goal_wit=(%d,%d), goal_wit_old=(%d,%d), racunaoupromjeni=%d\n", goal_wit.x, goal_wit.y, goal_wit_old.x, goal_wit_old.y, racunaoupromjeni);
		resetWitkowski();
		goal_wit_old=goal_wit;
		Goal=goal_wit;
		racunaoupromjeni=1;
	}
#endif
	//inicijalno pretrazivanje, od pocetka
	if ((prviputWitkowski)&&(watchdog_counter==0)){
		printf("DS> pretrazivanje pocinje: Start=(%d,%d)\n", Start.x, Start.y);
		radiponovo=1;//ako je pravi inicijalni izracun
		forward=false;
		backward=false;//zastavice za pretrazivanje do cilja odnosno do starta
		azurirani_forward=0;//inicijalizacija
		azurirani_backward=0;//inicijalizacija
		broj_bac=0;
		broj_for=0;
// 		StartRacunac=Start;
	//forward
		NumElemLista_forward=0;
		insertNodeWitkowski(Start,0,1);//forward==1	
	//backward
		NumElemLista_backward=0;
		insertNodeWitkowski(Goal,0,-1);//backward==-1
		watchdog_counter++;
	}
	
//put se ponovo racuna samo u slucaju promjene ili inicijalno ili ako nije stigao dovrsiti racunanje
	
	if ( watchdog_counter!=0)
	{
		watchdog_counter=0;
		while ((NumElemLista_forward != 0)||(NumElemLista_backward != 0))
		{
			broj_for=std::max(broj_for,NumElemLista_forward);
			broj_bac=std::max(broj_bac,NumElemLista_backward);
			//backward
			element_backward.x=-1;element_forward.x=-1;//inicijalizacija
			if ((!backward) && (NumElemLista_backward>0)){
				element=(izlaz_backward)->element;
				element_backward=element;
// 			if ((Start.x==element_backward.x) && (Start.y==element_backward.y)){
	if ((map[element_backward.x][element_backward.y].cost_backward>map[Start.x][Start.y].cost_backward)&&(traziCost(&ulaz_backward,&izlaz_backward,&element,-1)==0)){
				backward=true;
				printf("backward=true, back %d, for %d, sum %d, next=(%d,%d): back %d \n",map[Start.x][Start.y].cost_backward,map[Start.x][Start.y].cost_forward,map[Start.x][Start.y].cost_sum,map[Start.x][Start.y]._next_backward.x,map[Start.x][Start.y]._next_backward.y,map[Start.x][Start.y+1].cost_backward);
				PathExistsWitkowski=true;
// 				ispisiRed(izlaz_backward);
				if (forward==true){
					if (gettimeofday(&timeNow, NULL) == 0)
					{
						mySecNow = timeNow.tv_sec;
						myMSecNow = timeNow.tv_usec / 1000;
					}
					vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
					izracuniWitkowski[brojac_izracunaWitkowski]=vremenska_razlika;
					broj_iteracijaWitkowski[brojac_izracunaWitkowski]=watchdog_counter;
					broj_cvorova_na_listi_forward[brojac_izracunaWitkowski ]=NumElemLista_forward;
					broj_cvorova_na_listi_backward[brojac_izracunaWitkowski ]=NumElemLista_backward;
					max_broj_cvorova_na_listi_forward[brojac_izracunaWitkowski ]=broj_for;
					max_broj_cvorova_na_listi_backward[brojac_izracunaWitkowski ]=broj_bac;
					azurirani_backward_polje[brojac_izracunaWitkowski ]=azurirani_backward;
					azurirani_forward_polje[brojac_izracunaWitkowski]=azurirani_forward;
					printf("Witkowski SearchPath> trajanje inic. algoritma, backward kasni = %d ms, w=%d N_forward=%d N_backward=%d\n",vremenska_razlika,watchdog_counter,NumElemLista_forward,NumElemLista_backward);
					printf("Witkowski> broj azuriranih cvorova forward=%d, backward=%d\n",azurirani_forward,azurirani_backward);
/*					if ( (logfile = fopen("komb","a")) == NULL )
						printf("Error! komb file couldn't be opened.");
					else{
						fprintf(logfile,"Witkowski: trajanje %d ms, azurirani forward=%d, azurirani backward=%d, na listi forward %d cvorova od max %d usred pretrazivanja, a na listi backward %d cvorova od max %d usred pretrazivanja, broj iteracija %d\t", vremenska_razlika, azurirani_forward, azurirani_backward,NumElemLista_forward,broj_for,NumElemLista_backward,broj_bac,watchdog_counter);
						fclose(logfile);
					}*/
					watchdog_counter=0; //vracamo ga na 0
					prviputWitkowski=0; //nema vise prvog puta
					forward=false;
					backward=false;//zastavice za pretrazivanje do cilja odnosno do starta
					azurirani_forward=0;//inicijalizacija
					azurirani_backward=0;//inicijalizacija
					broj_bac=0;
					broj_for=0;
					brojac_izracunaWitkowski++;
					return true;
				}
			}
			}
			//forward
			if ((!forward) && (NumElemLista_forward>0)){
				element=(izlaz_forward)->element;
				element_forward=element;

// 				if ((Goal.x==element_forward.x) && (Goal.y==element_forward.y)){//trazi_jos_forward
if ((map[element_forward.x][element_forward.y].cost_forward>map[Goal.x][Goal.y].cost_forward)&&(traziCost(&ulaz_forward,&izlaz_forward,&element,1)==0)){
					forward=true;
					printf("forward=true, back %d, for %d, sum %d\n",map[Goal.x][Goal.y].cost_backward,map[Goal.x][Goal.y].cost_forward,map[Goal.x][Goal.y].cost_sum);
					PathExistsWitkowski=true;
// 					ispisiRed(izlaz_forward);
					if (backward==true){
						if (gettimeofday(&timeNow, NULL) == 0)
						{
							mySecNow = timeNow.tv_sec;
							myMSecNow = timeNow.tv_usec / 1000;
						}
						vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
						izracuniWitkowski[brojac_izracunaWitkowski]=vremenska_razlika;
						broj_iteracijaWitkowski[brojac_izracunaWitkowski]=watchdog_counter;
						broj_cvorova_na_listi_forward[brojac_izracunaWitkowski]=NumElemLista_forward;
						broj_cvorova_na_listi_backward[brojac_izracunaWitkowski]=NumElemLista_backward;
						max_broj_cvorova_na_listi_forward[brojac_izracunaWitkowski]=broj_for;
						max_broj_cvorova_na_listi_backward[brojac_izracunaWitkowski]=broj_bac;
						azurirani_backward_polje[brojac_izracunaWitkowski]=azurirani_backward;
						azurirani_forward_polje[brojac_izracunaWitkowski]=azurirani_forward;
						printf("Witkowski SearchPath> trajanje inic. algoritma, forward kasni = %d ms, w=%d N_forward=%d N_backward=%d\n",vremenska_razlika,watchdog_counter,NumElemLista_forward,NumElemLista_backward);
						printf("Witkowski> broj azuriranih cvorova forward=%d, backward=%d\n",azurirani_forward,azurirani_backward);
// 						if ( (logfile = fopen("komb","a")) == NULL )
// 							printf("Error! komb file couldn't be opened.");
// 						else{
// 							fprintf(logfile,"Witkowski: trajanje %d ms, azurirani forward=%d, azurirani backward=%d, na listi forward %d cvorova od max %d usred pretrazivanja, a na listi backward %d cvorova od max %d usred pretrazivanja, broj iteracija %d\t", vremenska_razlika, azurirani_forward, azurirani_backward,NumElemLista_forward,broj_for,NumElemLista_backward,broj_bac,watchdog_counter);
// 						}
// 						fclose(logfile);
						watchdog_counter=0; //vracamo ga na 0
						prviputWitkowski=0; //nema vise prvog puta
						forward=false;
						backward=false;//zastavice za pretrazivanje do cilja odnosno do starta
						azurirani_forward=0;//inicijalizacija
						azurirani_backward=0;//inicijalizacija
						broj_bac=0;
						broj_for=0;
						brojac_izracunaWitkowski++;
						return true;
					}
				}
			}
//      printf("prije processStatea\n");
			processStateWitkowski();
			++watchdog_counter;
			if (pipodjednom){
				printf("tu cu stat\n");
			}
			if ((watchdog_counter>= 2000*MapSizeX*MapSizeY)&&(prviputWitkowski==0))//(std::max(azurirani_forward,azurirani_backward)>= MapSizeX*MapSizeY)
			{
				PathExistsWitkowski=false;
				printf("Witkowski SearchPath>   Previse posjecenih cvorova!!!!---watchdog_counter=%d, azurirani_backward=%d, azurirani_forward=%d, broj cvorova na forward listi=%d, a na backward=%d\n",watchdog_counter,azurirani_backward,azurirani_forward,NumElemLista_forward,NumElemLista_backward);
				watchdog_counter=0;
				forward=false;
				backward=false;//zastavice za pretrazivanje do cilja odnosno do starta
				azurirani_forward=0;//inicijalizacija
				azurirani_backward=0;//inicijalizacija
				broj_bac=0;
				broj_for=0;
				brojac_izracunaWitkowski++;
				return false;
			}
			
		}   //od while

//ak je tu doso a nije izaso prije znaci da nema puta
/*		if (gettimeofday(&timeNow, NULL) == 0)
		{
			mySecNow = timeNow.tv_sec;
			myMSecNow = timeNow.tv_usec / 1000;
		}
		vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);*/
// 		PathExistsWitkowski=false;
		printf("Witkowski SearchPath>   lista prazna!!!!-lista_forward=%d, lista_backward=%d, wd=%d\n",NumElemLista_forward,NumElemLista_backward,watchdog_counter);
		/*		if ( (logfile = fopen("komb","a")) == NULL )
			printf("Error! komb file couldn't be opened.");
		else{
		for (int i=0;i<MapSizeX*MapSizeY;i++){
			if ((NumElemLista_forward==0)&&(lista_forward[i].x!=-1)){
				fprintf(logfile,"SearchPathWitkowski> NumElemLista_forward=0, a na listi ima neceg %d. (%d,%d)\n",i,lista_forward[i].x,lista_forward[i].y);
			}
			if ((NumElemLista_backward==0)&&(lista_backward[i].x!=-1)){
				fprintf(logfile,"SearchPathWitkowski> NumElemLista_backward=0, a na listi ima neceg %d. (%d,%d)\n",i,lista_backward[i].x,lista_backward[i].y);
			}
		}
	fclose(logfile);
		}*/
// 	return false;
	} //od if promjena il prviput il nastavak algoritma ak ga nije stigo prosli ciklus izvrtit

	//svaki ciklus biljezim
	if (gettimeofday(&timeNow, NULL) == 0)
	{
		mySecNow = timeNow.tv_sec;
		myMSecNow = timeNow.tv_usec / 1000;
	}
	vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
	izracuniWitkowski[brojac_izracunaWitkowski]=vremenska_razlika;
	broj_iteracijaWitkowski[brojac_izracunaWitkowski]=watchdog_counter;
	broj_cvorova_na_listi_forward[brojac_izracunaWitkowski]=NumElemLista_forward;
	broj_cvorova_na_listi_backward[brojac_izracunaWitkowski]=NumElemLista_backward;
	max_broj_cvorova_na_listi_forward[brojac_izracunaWitkowski]=broj_for;
	max_broj_cvorova_na_listi_backward[brojac_izracunaWitkowski]=broj_bac;
	azurirani_backward_polje[brojac_izracunaWitkowski]=azurirani_backward;
	azurirani_forward_polje[brojac_izracunaWitkowski]=azurirani_forward;
	watchdog_counter=0;
	forward=false;
	backward=false;//zastavice za pretrazivanje do cilja odnosno do starta
	azurirani_forward=0;//inicijalizacija
	azurirani_backward=0;//inicijalizacija
	broj_bac=0;
	broj_for=0;
	brojac_izracunaWitkowski++;
	PathExistsWitkowski=true;
	return true;
}


 void    DStar::insertNodeWitkowski( I_point element, int costnew ,int smjer)
{
	int dod;
	//forward==1
	if (smjer==1){
// 		ispisiRed(izlaz_forward);
		azurirani_forward++;//brojimo koliko cvorova se azurira po pretrazivanju
// 		if (map[element.x][element.y].tag_forward==NEW)
// 		{
			map[element.x][element.y].cost_forward=costnew;
// 		}else {
// 			map[element.x][element.y].cost_forward=std::min(costnew,map[element.x][element.y].cost_forward);
// 		}
		if (map[element.x][element.y].tag_backward!=NEW){
			map[element.x][element.y].cost_sum=map[element.x][element.y].cost_backward+map[element.x][element.y].cost_forward;
		}
		if ((map[element.x][element.y].tag_forward!=OPEN)){
			dod=DodajURed (element, &ulaz_forward, &izlaz_forward);
			NumElemLista_forward++;
		}
		if (!dod){
			if ( (logfile = fopen("komb","a")) == NULL )
				printf("Error! komb file couldn't be opened.");
			else{
				fprintf(logfile,"insertNodeWitkowski> nema memorije\n");
				fclose(logfile);
			}
		}
		map[element.x][element.y].tag_forward=OPEN;
		map[element.x][element.y].time_stamp_forward=time_stamp_counter;
	}
	else if (smjer==-1){ //backward==-1
		azurirani_backward++;//brojimo koliko cvorova se azurira po pretrazivanju
// 		ispisiRed(izlaz_backward);
// 		if (map[element.x][element.y].tag_backward==NEW)	//NEW (nikad bio na listi)
// 		{
			map[element.x][element.y].cost_backward=costnew;
// 		}
// 		else
// 		{
// 			map[element.x][element.y].cost_backward = std::min(map[element.x][element.y].cost_backward, costnew);
// 		}
		if (map[element.x][element.y].tag_forward!=NEW){
			map[element.x][element.y].cost_sum=map[element.x][element.y].cost_backward+map[element.x][element.y].cost_forward;
		}
		if ((element.x==Start.x)&&(element.y==Start.y)){
			printf("inserto start: back %d, for %d, sum %d",map[element.x][element.y].cost_backward,map[element.x][element.y].cost_forward,map[element.x][element.y].cost_sum);
		}
		if (map[element.x][element.y].tag_backward!=OPEN){
			dod=DodajURed (element, &ulaz_backward, &izlaz_backward);
			NumElemLista_backward++;
		}
		if (!dod){
			if ( (logfile = fopen("komb","a")) == NULL )
				printf("Error! komb file couldn't be opened.");
			else{
				fprintf(logfile,"insertNodeWitkowski> nema memorije\n");
				fclose(logfile);
			}
		}
		map[element.x][element.y].tag_backward=OPEN;
		map[element.x][element.y].time_stamp=time_stamp_counter;
	}
}


 void DStar::processStateWitkowski() {
	 int h_val, h_valfor,dod;
	//I_point element pokazuje na neku celiju u mapi!!!
	//prosirenje cvora na susjedne u svih 8 smjerova
	 I_point point;
	 int h_point,h_pointfor;
	 h_val=-1;h_valfor=-1;//inicijalizacija
	 if (element_backward.x!=-1){
		 //tu ide brisanje
// 		 printf("brisanje prije backward\n");
// 		 ispisiRed(izlaz_backward);
		dod=SkiniIzReda (&ulaz_backward, &izlaz_backward);
		 if (!dod) {
			 printf("Red je prazan\n");
			 if ( (logfile = fopen("komb","a")) == NULL )
				 printf("Error! komb file couldn't be opened.");
			 else{
				 fprintf(logfile,"processStateWitkowski> Red je prazan\n");
				 fclose(logfile);
			 }
		 }
// 		 printf("brisanje poslije backward\n");
// 		 ispisiRed(izlaz_backward);
		 NumElemLista_backward--;
		 map[element_backward.x][element_backward.y].tag_backward=CLOSED;

		h_val = map[element_backward.x][element_backward.y].cost_backward;
	 }
	 if (element_forward.x!=-1){
		 //tu ide brisanje
// 		 printf("brisanje prije forward\n");
// 		 ispisiRed(izlaz_forward);
		dod=SkiniIzReda (&ulaz_forward, &izlaz_forward);
		 if (!dod) {
			printf("Red je prazan\n");
			 if ( (logfile = fopen("komb","a")) == NULL )
				 printf("Error! komb file couldn't be opened.");
			 else{
				 fprintf(logfile,"processStateWitkowski> Red je prazan\n");
				 fclose(logfile);
			 }
		 }
// 		 printf("brisanje poslije forward\n");
// 		 ispisiRed(izlaz_forward);
		 NumElemLista_forward--;
		 map[element_forward.x][element_forward.y].tag_forward=CLOSED;

		h_valfor = map[element_forward.x][element_forward.y].cost_forward;
	 }
		for ( int d = 0; d < 8; d++ )
		 {
			if (h_val!=-1){
     //odredjivanje pozicije novog susjeda
				 point.x=element_backward.x+xofs[d];
				 point.y=element_backward.y+yofs[d];
				 if ( IsValid( point.x, point.y )!=0 )
				 {
					 h_point = map[point.x][point.y].cost_backward;
//postavljen cost c izmedju najboljeg stanja i susjeda
					 arc_cost( element_backward.x, element_backward.y, point.x, point.y);
					 if ( ((map[point.x][point.y].tag_backward==NEW)&&(c<OBSTACLE)&&(h_val<OBSTACLE)) || ((h_point > h_val + c*travCost[d]) && (c<OBSTACLE)) )
					 {
						  //provjeravam nastajanje petlje
						 if ((map[element_backward.x][element_backward.y]._next_backward.x == point.x) && (map[element_backward.x][element_backward.y]._next_backward.y == point.y)){
							 pipodjednom=true;
							 printf("processStateWb> radi se petlja!\n");
						 }
						 map[point.x][point.y]._next_backward.x = element_backward.x;
						 map[point.x][point.y]._next_backward.y = element_backward.y;
						 if ((point.x==Start.x)&&(point.y==Start.y)){
							 printf("insertat ce start i bit ce back %d = h_val %d + c*travCost %d od skinutog (%d,%d)\n",h_val + c*travCost[d],h_val,c*travCost[d],element_backward.x,element_backward.y);
						 }
						 
						 insertNodeWitkowski(point, h_val + c*travCost[d],-1);
// 						 insertNodeWitkowski(point, h_val + (c-1)+travCost[d],-1);
					 }
				 }
			}
			if (h_valfor!=-1){
				 point.x=element_forward.x+xofs[d];
				 point.y=element_forward.y+yofs[d];
				 if ( IsValid( point.x, point.y )!=0 )
				 {
					 h_pointfor = map[point.x][point.y].cost_forward;
//postavljen cost c izmedju najboljeg stanja i susjeda
					 arc_cost( element_forward.x, element_forward.y, point.x, point.y);
					 if ( ((map[point.x][point.y].tag_forward==NEW)&&(c<OBSTACLE)&&(h_val<OBSTACLE)) || ((h_pointfor > h_valfor + c*travCost[d]) && (c<OBSTACLE)) )
					 {
						  //provjeravam nastajanje petlje
						 if ((map[element_forward.x][element_forward.y]._next_forward.x == point.x) && (map[element_forward.x][element_forward.y]._next_forward.y == point.y)){
							 pipodjednom=true;
							 printf("processStateWf> radi se petlja!\n");
						 }
						 map[point.x][point.y]._next_forward.x = element_forward.x;
						 map[point.x][point.y]._next_forward.y = element_forward.y;
						 insertNodeWitkowski(point, h_valfor + c*travCost[d],1);
// 						 insertNodeWitkowski(point, h_valfor + (c-1)+travCost[d],1);
					 }
				 }
			 }
		 } //for susjedi
 }

 I_point* DStar::getPathWitkowski(){
  	 
	 if (gettimeofday(&timeStart, NULL) == 0)
	 {
		 mySecStart = timeStart.tv_sec;
		 myMSecStart = timeStart.tv_usec / 1000;
	 }
#if (DSTAR_REVERSE==0)
	 if(!PathExistsWitkowski)
	 {
		 printf("DStar getPathWitkowski> Path does not exist!!!");
		 return NULL;
	 }
#endif
	 if((radiponovo==0)){
	 
	 if (PathLengthWitkowskiSegment>1){
		 //tu moramo naci put od trenutne pozicije (Start)
		 //prvo provjera jel segment dugacak samo jedno polje
//		 int deltax,deltay,pom1x,pom1y;//,pom2x,pom2y;
		 double deltax,deltay;
		 double pom1x,pom1y;
		 double pomocni;
		 deltax=double(path_witkowski_segment[1].x-path_witkowski_segment[0].x);
		 deltay=double(path_witkowski_segment[1].y-path_witkowski_segment[0].y);
// 		 printf("p[0]=(%d,%d) p[1]=(%d,%d) p[2]=(%d,%d)\n",path_witkowski_segment[0].x,path_witkowski_segment[0].y,path_witkowski_segment[1].x,path_witkowski_segment[1].y,path_witkowski_segment[2].x,path_witkowski_segment[2].y);
//		 pom1x=(Start.x-path_witkowski_segment[0].x);
//		 pom1y=(Start.y-path_witkowski_segment[0].y);
		 pom1x=(start_ri.x-double(path_witkowski_segment[0].x));
		 pom1y=(start_ri.y-double(path_witkowski_segment[0].y));
/*		 if ((abs(deltax)<=1)&&(abs(deltay)<=1)){
			 pom2x=(Start.x-path_witkowski_segment[1].x);
			 pom2y=(Start.y-path_witkowski_segment[1].y);
			 if ((pom1x*pom1x+pom1y*pom1y)>(pom2x*pom2x+pom2y*pom2y)){
				 for(int i=1;i<PathLengthWitkowskiSegment;i++){
					 path_witkowski_segment[i-1]=path_witkowski_segment[i];
				 }
				 PathLengthWitkowskiSegment--;
			 }
		 }else*/
			 pomocni=(pom1x*deltax+pom1y*deltay)/((deltax*deltax+deltay*deltay));
			 //scalar product of two vectors ---> cos of their angle
			 preslik.x=double(path_witkowski_segment[0].x)+pomocni*(deltax);
			 preslik.y=double(path_witkowski_segment[0].y)+pomocni*(deltay);
// 			 printf("pomocni=%f, pom1x=%d, pom1y=%d, deltax=%d, deltay=%d, Start=(%d,%d) preslik (%f,%f)\n",pomocni, pom1x,pom1y,deltax,deltay,Start.x, Start.y,preslik.x,preslik.y);
// 			 pom1x=int(preslik.x);
// 			 pom1y=int(preslik.y);
			 if ((fabs(preslik.x-double(path_witkowski_segment[1].x))) + (fabs(preslik.y-double(path_witkowski_segment[1].y)))<1.5){
				 for(int i=1;i<PathLengthWitkowskiSegment;i++){
					 path_witkowski_segment[i-1]=path_witkowski_segment[i];
				 }
			 	 PathLengthWitkowskiSegment--;
			 }else if (PathLengthWitkowskiSegment>2){
				 //gledamo projekciju na drugi segment
#if 0			 
				 deltax=(path_witkowski_segment[2].x-path_witkowski_segment[1].x);
				 deltay=(path_witkowski_segment[2].y-path_witkowski_segment[1].y);
				 pom1x=(Start.x-path_witkowski_segment[1].x);
				 pom1y=(Start.y-path_witkowski_segment[1].y);
					 pomocni=(pom1x*deltax+pom1y*deltay)/(double(deltax*deltax+deltay*deltay));
					 printf("pomocni=%f, pom1x=%d, pom1y=%d, deltax=%d, deltay=%d, Start=(%d,%d)\n",pomocni, pom1x,pom1y,deltax,deltay,Start.x, Start.y);
					 preslik.x=path_witkowski_segment[1].x+pomocni*double(deltax);
					 preslik.y=path_witkowski_segment[1].y+pomocni*double(deltay);
					 printf("pomocni=%f, pom1x=%d, pom1y=%d, deltax=%d, deltay=%d, Start=(%d,%d) preslik (%f,%f)\n",pomocni, pom1x,pom1y,deltax,deltay,Start.x, Start.y,preslik.x,preslik.y);
					 // 			 pom1x=int(preslik.x);
					 // 			 pom1y=int(preslik.y);
					 if (((fabs(preslik.x-double(path_witkowski_segment[1].x))<=1.) && (fabs(preslik.y-double(path_witkowski_segment[1].y))<=1.))){
						 for(int i=1;i<PathLengthWitkowskiSegment;i++){
							 path_witkowski_segment[i-1]=path_witkowski_segment[i];
						 }
						 PathLengthWitkowskiSegment--;
					 }else
#endif
					 {//ako se radi o D* ficleku onda jos ovo provjeriti
						 int min1=0;
						 int i,d=0,dmin,pom2x,pom2y;
						 pom2x=(Start.x-path_witkowski_segment[0].x);
						 pom2y=(Start.y-path_witkowski_segment[0].y);
						 dmin=pom2x*pom2x+pom2y*pom2y;
						 for(i=1;i<PathLengthWitkowskiSegment-1;i++){
							 pom2x=(Start.x-path_witkowski_segment[i].x);
							 pom2y=(Start.y-path_witkowski_segment[i].y);
							 d=pom2x*pom2x+pom2y*pom2y;
							 if ((dmin>=d)&& ((fabs(path_witkowski_segment[i].x-double(path_witkowski_segment[i-1].x))<=1.) && (fabs(path_witkowski_segment[i].y-double(path_witkowski_segment[i-1].y))<=1.))){
								 min1=i;
								 dmin=d;
							 }else{
								 break;
							 }
						 }
						 if (min1>0){
							 for(int i=min1;i<PathLengthWitkowskiSegment;i++){
								 path_witkowski_segment[i-min1]=path_witkowski_segment[i];
							 }
							 PathLengthWitkowskiSegment-=min1;
							 printf("duljina DD* puta %d",PathLengthWitkowskiSegment);
						 }
					 }
					 

		 }
		 }
	 }else{
	 printf("radi skupine\n");
	 PathLengthWitkowski=0;
#if (DSTAR_REVERSE==0)
	 int sum_min_start=map[Start.x][Start.y].cost_sum;
	 int sum_min_goal=map[Goal.x][Goal.y].cost_sum;
	 int sum_min;
	 sum_min=std::max(sum_min_start,sum_min_goal);
	 printf("sum_min_start=%d, sum_min_goal=%d\n",sum_min_start, sum_min_goal);
	 //nepotrebno prelazenje preko cijelog polja, popraviti
	for (int i=0;i<MapSizeX;i++){
		 for(int j=0;j<MapSizeY;j++){
			 if (map[i][j].cost_sum<=sum_min){
				 path_witkowski[PathLengthWitkowski].x=i;
				 path_witkowski[PathLengthWitkowski].y=j;
				 PathLengthWitkowski++;
				 map[i][j].cvorovi=1;
/*				 if  (map[i][j].cost_sum<sum_min) {
					 sum_min=map[i][j].cost_sum;
				 }*/
			 }
		 }
	 }
	 path_witkowski_segment[0].x=Start.x;//Racunac.x;
	 path_witkowski_segment[0].y=Start.y;//Racunac.y;
#else
	 Goal=GoalRacunac;//treba za dalje
	 int minf=map[GoalRacunac.x][GoalRacunac.y].total_cost_int;
	 int drugistart=1; //1-ako uzimamo racunanje od trenutnog starta koji moze bit van opt. podrucja, 0-od startRacunca
	 int Wpomak=0;//pomak po DStar putu
	 w0.x=-1; w0.y=-1; //inicijalizacija pocetne tocke W puta
	 Numminfpts=0;
	 if (racunaoupromjeni){
	 //da gusak nema tolko posla
	 for (int i=0;((i<MapSizeX));i++){
		for(int j=0;((j<MapSizeY));j++){
			if ((map[i][j].total_cost_int==minf)){
				map[i][j].skupine=0;
				minfpoints[Numminfpts].x=i;
				minfpoints[Numminfpts].y=j;
				Numminfpts++;				 
			}
		}				 
	 }
#if 0
	 int okolina=std::max(abs(Start.x-GoalRacunac.x),abs(Start.y-GoalRacunac.y));
  for (int i=std::max((Start.x-okolina),0);((i<MapSizeX)&&(i<=Start.x+okolina));i++){
		 for(int j=std::max((Start.y-okolina),0);((j<MapSizeY)&&(j<=Start.y+okolina));j++){
			 map[i][j].skupine=0;
		 }
	 }
#endif
	 }
	 if (drugistart){
	 I_point temp;
	 for(int i=goal_index;i>=0;i--)
	 {
		 if (map[path[i].x][path[i].y].total_cost_int==minf){
			 temp=path[i];
			 Wpomak=i;
		 }else{
			 printf("Tek od %d elementa D* puta polja su na optimalnom podrucju -> (%d,%d)\n",Wpomak,temp.x,temp.y);
			 printf("goal_index=%d, i=%d, minf=%d, GoalRacunac (%d,%d), Goal (%d,%d)\n", goal_index, i, minf, GoalRacunac.x, GoalRacunac.y, Goal.x, Goal.y);
			 if (Wpomak==goal_index){
				 printf("kako to\n");
			 }
			 break;
		 }
	 }
/*	 temp.x=Start.x;
	 temp.y=Start.y;
	 while (map[temp.x][temp.y].total_cost_int>minf){
		 printf("(%d,%d) ima veci total_cost_int\n",temp.x,temp.y);
		 temp=map[temp.x][temp.y]._next;
	 }*/
	 path_witkowski_segment[0].x=temp.x;//Racunac.x;
	 path_witkowski_segment[0].y=temp.y;//Racunac.y;
	 w0=temp;
	 }else{
	 path_witkowski_segment[0].x=StartRacunac.x;
	 path_witkowski_segment[0].y=StartRacunac.y;
	 }
#endif
	 int brojskupine=0,nova_skupina=1;
	 PathLengthWitkowskiSegment=1;//koordinate tocaka segmentnog puta
	 printf("Goal=(%d,%d)\n",Goal.x,Goal.y);
	 while (1){
		 if (((path_witkowski_segment[PathLengthWitkowskiSegment-1].x==Goal.x)&&(path_witkowski_segment[PathLengthWitkowskiSegment-1].y==Goal.y)) || (nova_skupina==0))
			 break;
		 nova_skupina=RadiSkupinuCvorova(path_witkowski_segment[PathLengthWitkowskiSegment-1].x,path_witkowski_segment[PathLengthWitkowskiSegment-1].y,&brojskupine);
// 		 printf("brojskupine=%d\n",brojskupine);
// 		 if (gettimeofday(&timeNow, NULL) == 0)
//	 {
//		 mySecNow = timeNow.tv_sec;
//		 myMSecNow = timeNow.tv_usec / 1000;
//	 }
//	 vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
//	 printf("vrijeme nakon ove skupine = %d\n",vremenska_razlika);
		 if (nova_skupina==0){
			 printf("dambo je ljut\n");
		 }
	 }
	 if (gettimeofday(&timeNow, NULL) == 0)
	 {
		 mySecNow = timeNow.tv_sec;
		 myMSecNow = timeNow.tv_usec / 1000;
	 }
	 vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
	 printf("vrijeme za skupine = %d\n",vremenska_razlika);
	 //provjera vizibilitija
	 if (PathLengthWitkowskiSegment>=3){//potrebne su bar 3 tocke da provjera ima smisla
	 I_point cs1, cs2, pomocni_pws[PathLengthWitkowskiSegment], point;
	 R_point rpoint;
	 int pom_PLWS=0, cs1_index, cs2_index, brpom, brojvecih, broj_cvorova;
	 double orientation;
	 cs1.x=-1; cs1.y=-1;
	 cs2.x=-1; cs2.y=-1;
	 broj_konkavnih_tocaka=PathLengthWitkowskiSegment;
	 for (int i=0; i<PathLengthWitkowskiSegment; i++){
		 sve_konkavne_tocke[i]=path_witkowski_segment[i];
// 		 printf("path_witkowski_segment[%d]=(%d,%d)\n",i,path_witkowski_segment[i].x,path_witkowski_segment[i].y);
		 if (map[(path_witkowski_segment[i].x)][(path_witkowski_segment[i].y)].time_stamp_forward==-1){
			 if ((cs1.x==-1)&&(i>0)){
				 cs1.x=path_witkowski_segment[i-1].x;
				 cs1.y=path_witkowski_segment[i-1].y;//to je prva CS tocka, trazimo drugu
				 cs1_index=i-1;
// 				 printf("S1=(%d,%d), cs1_index=%d\n",cs1.x,cs1.y,cs1_index);
			 }
			 if (i==0){
				 pomocni_pws[pom_PLWS]=path_witkowski_segment[i];
// 				 printf("pomocni_pws[%d]=(%d,%d)\n",pom_PLWS,pomocni_pws[pom_PLWS].x,pomocni_pws[pom_PLWS].y);
				 pom_PLWS++;
			 }
		 }else if ((cs1.x!=-1)&&(cs2.x==-1)){//nasli smo drugu CS tocku
			 cs2_index=i;
			 cs2.x=path_witkowski_segment[i].x;
			 cs2.y=path_witkowski_segment[i].y;
// 			 printf("S2=(%d,%d), cs2_index=%d\nTrazenje najkraceg puta\n",cs2.x,cs2.y,cs2_index);
			 //prvo trazimo vidljivosti izmedju svih od CS1 do CS2
			 broj_cvorova=cs2_index-cs1_index+1;
			 int v[broj_cvorova][broj_cvorova];//, d[broj_cvorova][broj_cvorova];
			 int p[broj_cvorova], open[2*broj_cvorova], broj_open, el, put[broj_cvorova];
			 double d[broj_cvorova][broj_cvorova], g[broj_cvorova], gnew;
			 //inicijalizacija polja
			 for (int j=0; j<broj_cvorova; j++){
				 g[j]=OBSTACLE;
				 p[j]=-1;
				 for (int k=0; k<broj_cvorova; k++){
					 v[j][k]=0;
				 }
			 }
			 for (int j=0; j<broj_cvorova; j++){
				 for (int k=0; k<broj_cvorova; k++){
/*					 if (abs(j-k)==1){
						 v[j][k]=1;
						 v[k][j]=1;
					 }*/
					 if (j<k){
					 d[j][k]=sqrt((path_witkowski_segment[cs1_index+j].x-path_witkowski_segment[cs1_index+k].x)*(path_witkowski_segment[cs1_index+j].x-path_witkowski_segment[cs1_index+k].x)+(path_witkowski_segment[cs1_index+j].y-path_witkowski_segment[cs1_index+k].y)*(path_witkowski_segment[cs1_index+j].y-path_witkowski_segment[cs1_index+k].y));
					 d[k][j]=d[j][k];
					 }
					 if ((j<k)&&(abs(j-k)>=1)&&(j!=k)&&(v[j][k]!=1)){
						 orientation=atan2((path_witkowski_segment[cs1_index+k].y-path_witkowski_segment[cs1_index+j].y),(path_witkowski_segment[cs1_index+k].x-path_witkowski_segment[cs1_index+j].x));
						 point.x=path_witkowski_segment[cs1_index+j].x;
						 point.y=path_witkowski_segment[cs1_index+j].y;
						 rpoint.x=(point.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2.)+0.5*GM->Map_Cell_Size*cos(orientation);
						 rpoint.y=(point.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2.)+0.5*GM->Map_Cell_Size*sin(orientation);
						 GM->check_point(rpoint);
// 						 point.x=path_witkowski_segment[cs1_index+j].x+int(0.5*cos(orientation));
// 						 point.y=path_witkowski_segment[cs1_index+j].y+int(0.5*sin(orientation));
						 brpom=0;
						 brojvecih=0;
						 point=GM->cell_point_temp;
						 while ((abs(point.x-path_witkowski_segment[cs1_index+k].x)>0) || (abs(point.y-path_witkowski_segment[cs1_index+k].y)>0)){
							 brpom++;
							 point.x=path_witkowski_segment[cs1_index+j].x;
							 point.y=path_witkowski_segment[cs1_index+j].y;
							 rpoint.x=(point.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2.)+brpom*0.5*GM->Map_Cell_Size*cos(orientation);
							 rpoint.y=(point.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2.)+brpom*0.5*GM->Map_Cell_Size*sin(orientation);
							 GM->check_point(rpoint);
							 point=GM->cell_point_temp;
#if DSTAR_REVERSE
							 if (map[point.x][point.y].total_cost_int!=minf)
#else
							if (map[point.x][point.y].cvorovi!=1)
#endif
							 {
								 brojvecih++;
								 break;
							 }

							 if ((j==0)&&(k==2)){
// 								 printf("%d %d ",point.x,point.y);
							 }
						 }
						 if (!brojvecih){
							 v[j][k]=1;
							 v[k][j]=1;
// 							 printf("Vidljive su tocke %d i %d, a duljina je %f\n",j,k,d[j][k]);
						 }
					 }
				 }
			 }
			 	 if (gettimeofday(&timeNow, NULL) == 0)
	 {
		 mySecNow = timeNow.tv_sec;
		 myMSecNow = timeNow.tv_usec / 1000;
	 }
	 vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
	 printf("vrijeme nakon provjere vidljivosti = %d\n",vremenska_razlika);

			 //pretrazivanje, jednostavno, graf je takav da nema problema s dfs-om
			 open[0]=0;//nulta tocka je startna
			 broj_open=1;
			 g[0]=0.;
			 while (broj_open>0){
				 //skidanje cvora
				el=open[broj_open-1];
				broj_open--;
				//trazenje susjeda preko vizibilitija
				for (int j=0; j<broj_cvorova; j++){
					if (v[el][j]==1){
						gnew=d[el][j]+g[el];
						if (gnew<g[j]){
							open[broj_open]=j;
							broj_open++;
							p[j]=el;//pokazivac na prethodni
// 							printf("pokazivac na prethodni p[%d]=%d, broj_open=%d, d[%d][%d]=%f, g[%d]=%f, g[%d]=%f\n",j,el,broj_open,el,j,d[el][j],el,g[el],j,g[j]);
							g[j]=gnew;
						}
					}
				}
			 }
			 //odredjivanje puta
			 el=broj_cvorova-1;//krecemo od ciljnog cvora
			 brpom=0;
			 while (el){
				 put[brpom]=el;
				 el=p[el];//nulti se ne zapisuje
				 printf("el=%d ",el);
				 brpom++;
			 }
			 printf("\n");
			 	 if (gettimeofday(&timeNow, NULL) == 0)
	 {
		 mySecNow = timeNow.tv_sec;
		 myMSecNow = timeNow.tv_usec / 1000;
	 }
	 vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
	 printf("vrijeme nakon pretrazivanja S i C = %d\n",vremenska_razlika);

			 //prepisivanje u pomocno polje puta
			 for (int j=brpom-1; j>=0; j--){
				 pomocni_pws[pom_PLWS]=path_witkowski_segment[cs1_index+put[j]];
// 				 printf("pomocni_pws[%d]=(%d,%d)\n",pom_PLWS,pomocni_pws[pom_PLWS].x,pomocni_pws[pom_PLWS].y);
				 pom_PLWS++;
			 }
			 cs1.x=-1; cs1.y=-1;
			 cs2.x=-1; cs2.y=-1;
		 }//ovo dolje dodajem ekstra gore uvjet jer pocetni ne mora bit S tocka
		 else if (cs1.x==-1){
			 pomocni_pws[pom_PLWS]=path_witkowski_segment[i];
// 			 printf("pomocni_pws[%d]=(%d,%d)\n",pom_PLWS,pomocni_pws[pom_PLWS].x,pomocni_pws[pom_PLWS].y);
			 pom_PLWS++;
		 }
	 }//kraj for petlje provjere vizibilitija
	 //prijepis u pravo polje
	 PathLengthWitkowskiSegment=pom_PLWS;
#if DSTAR_REVERSE
	 PathLengthWitkowskiSegment=pom_PLWS+Wpomak;
	 for (int i=0;i<Wpomak+pom_PLWS;i++){
		 if (i<Wpomak){
			 path_witkowski_segment[i]=path[i]; 
		 }else{
			 path_witkowski_segment[i]=pomocni_pws[i-Wpomak];
		 }
	 }
#else
	 for (int i=0; i<pom_PLWS; i++){
		 path_witkowski_segment[i]=pomocni_pws[i];
	 }
#endif
	 }//kraj uvjeta da moraju bit bar 3 tocke na W putu za provjeru vizibilitija
	 else{//ako ima manje od 3 tocke
#if DSTAR_REVERSE
		 if (Wpomak>0){
		 for (int i=PathLengthWitkowskiSegment+Wpomak-1;i>=0;i--){
			 if (i>=Wpomak){
				 path_witkowski_segment[i]=path_witkowski_segment[i-Wpomak];
			 }else{
				 path_witkowski_segment[i]=path[i];
			 }
		 }
		 PathLengthWitkowskiSegment+=Wpomak;
			 if (nova_skupina==0){
				 printf("dambo je ljut\n");
				 nova_skupina=0;
			 }
		 }
#endif
	 }
#if DSTAR_REVERSE
	 if (racunaoupromjeni){
		 for (int i=0; i<PathLengthWitkowskiSegmentOld; i++){
			 path_witkowski_segment[PathLengthWitkowskiSegment+i]=path_witkowski_segment_old[i];
		 }
		 PathLengthWitkowskiSegment+=PathLengthWitkowskiSegmentOld;
	 }
#endif
	 }
	 if (gettimeofday(&timeNow, NULL) == 0)
	 {
		 mySecNow = timeNow.tv_sec;
		 myMSecNow = timeNow.tv_usec / 1000;
	 }
	 vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
	 vremena_skupine[brojac_izracuna-1]=vremenska_razlika;
	 printf("vremena skupine = %d\n",vremenska_razlika);
	 if (radiponovo){
	   double duljina_wit_puta=0., dodatak=0.;
	   for (int i=1; i<PathLengthWitkowskiSegment; i++){
		   dodatak=sqrt((path_witkowski_segment[i].x-path_witkowski_segment[i-1].x)*(path_witkowski_segment[i].x-path_witkowski_segment[i-1].x)+(path_witkowski_segment[i].y-path_witkowski_segment[i-1].y)*(path_witkowski_segment[i].y-path_witkowski_segment[i-1].y))*(CELL_DIM)*0.001;
	     duljina_wit_puta+=dodatak;
	     if (dodatak<0.0001){
		     printf("ponovljena tocka %d\n",i);
	     }
	   }
	   if ( (logfile = fopen("komb","a")) == NULL )
	     printf("Error! komb file couldn't be opened.");
	   else{
	     fprintf(logfile,"DS::getPathWitkowski(), vremena skupine = %d, duljina putanje [m]=%f\n",vremenska_razlika, duljina_wit_puta);
	     fclose(logfile);
	   }
	   duljina_wit_puta_um[brojac_izracuna-1]=duljina_wit_puta;
	 }
	 return path_witkowski_segment;
 }

 int DStar::RadiSkupinuCvorova(int i, int j, int *brojskupine){
 	 //radi skupine od starta pa do cilja po tockama koje imaju vise od 5 vecih costova okolo
	 I_point *skupolje, suma, point;
	 int maks=500;//maksimalna pocetna velicina polja
	 int ispis=0;//1-ako hocu ispis skupolja i broja skupine i kad ide van, 0-necu
	 skupolje = (I_point *)malloc(maks*sizeof(I_point)) ;
	 int imaih;
	 int brpom, brpompom, pomx, pomy, indeksd, indeksd2, indeksd3;
	 int brojvecih, nemadalje, brojzauzetih, slican;
	 int wmax,wtemp,gusak;
// 	 double orientation;
	 (*brojskupine)++;
	 if (ispis)
		printf("\nbrojskupine=%d\n",(*brojskupine));
	 map[i][j].skupine=*brojskupine;//oznaka da je prodjeno polje
	 skupolje[0].x=i;//dodaj ga u skupinu povezanih
	 skupolje[0].y=j;
	 imaih=1;
//#if DSTAR_REVERSE
	 int minf=map[GoalRacunac.x][GoalRacunac.y].total_cost_int;
//#endif
	 //gleda susjedne
	 //prvo next
/*	 point=map[i][j]._next;
	 if ( IsValid( point.x, point.y )!=0 )
	 {
#if DSTAR_REVERSE
		 if ((map[point.x][point.y].total_cost_int==minf)&&(map[point.x][point.y].skupine==0))
#else
			 if ((map[point.x][point.y].cvorovi==1)&&(map[point.x][point.y].skupine==0))
#endif
		 {
			 skupolje[imaih].x=point.x;//dodaj ga u skupinu povezanih
			 skupolje[imaih].y=point.y;
			 imaih++;
			 map[point.x][point.y].skupine=*brojskupine;
		 }
	 }*/
	for ( int d = 0; d < 8; d++ )
	{
		 point.x=i+xofs[d];
		 point.y=j+yofs[d];
		  //odredjivanje pozicije novog susjeda
		 if ( IsValid( point.x, point.y )!=0 )
		 {
// 			 printf("%d. susjed od glavnog: cvorovi=%d, skupine=%d\n",d,map[point.x][point.y].cvorovi,map[point.x][point.y].skupine);
#if DSTAR_REVERSE
			 if ((map[point.x][point.y].total_cost_int==minf)&&(map[point.x][point.y].skupine==0)&&(map[point.x][point.y].h_cost_int<=map[i][j].h_cost_int))
#else
			 if ((map[point.x][point.y].cvorovi==1)&&(map[point.x][point.y].skupine==0))
#endif
			 {
				 skupolje[imaih].x=point.x;//dodaj ga u skupinu povezanih
				 skupolje[imaih].y=point.y;
				 imaih++;
				 map[point.x][point.y].skupine=*brojskupine;
			 }
		 }
	}
	brpompom=imaih;//to dodajem zbog donjeg ispisa
	if (imaih>1){
		brpom=1;//krecemo od drugog jer je prvi glavni
		brpompom=imaih;//pamtimo zadnji
		while (1){
			 for (int in=brpom; in<imaih; in++){
						//gleda svakom susjedne
				 pomx=skupolje[in].x;
				 pomy=skupolje[in].y;
				 brojvecih=0;
				 brojzauzetih=0;
				 slican=0;
				 nemadalje=1;
				 wmax=0;
				 //prvo brojanje koliko ima vecih
				 if (!((pomx==Goal.x)&&(pomy==Goal.y))){
					 for ( int d = 0; d < 8; d++ )
					 {
						 point.x=pomx+xofs[d];
						 point.y=pomy+yofs[d];
		  //odredjivanje pozicije novog susjeda
		  //dodajem novi uvjet slicnog po h-u
		  //postavljen cost c izmedju najboljeg stanja i susjeda
						 if ( IsValid( point.x, point.y )!=0 )
						 {
						   arc_cost( point.x, point.y, pomx, pomy);
#if DSTAR_REVERSE
						   wtemp=COSTSTRAIGHT*c;
						   if (wmax<wtemp)
							   wmax=wtemp;
						   if ((map[point.x][point.y].total_cost_int==minf)&&(abs(map[point.x][point.y].h_cost_int-map[pomx][pomy].h_cost_int)<COSTSTRAIGHT*c))
#else
if ((map[point.x][point.y].cvorovi==1)&&(abs(map[point.x][point.y].cost_backward-map[pomx][pomy].cost_backward)<COSTSTRAIGHT*c)&&(abs(map[point.x][point.y].cost_forward-map[pomx][pomy].cost_forward)<COSTSTRAIGHT*c))
#endif
						{
// 							printf("back susjed %d, back point %d, sum susjed %d, sum point %d, for susjed %d, for point %d\n",map[point.x][point.y].cost_backward,map[pomx][pomy].cost_backward,map[point.x][point.y].cost_sum,map[pomx][pomy].cost_sum,map[point.x][point.y].cost_forward,map[pomx][pomy].cost_forward);
						     slican=1;
						   }
						   
#if DSTAR_REVERSE
							 if (map[point.x][point.y].total_cost_int!=minf)
#else
							 if (map[point.x][point.y].cvorovi!=1)
#endif
							 {
								 brojvecih++;
								 if (brojvecih==1)
									 indeksd=d;//zapamtim indeks
								 if (brojvecih==2)
									 indeksd2=d;//zapamtim drugi indeks
								 if (brojvecih==3)
									 indeksd3=d;//zapamtim drugi indeks
							 } else if (map[point.x][point.y].skupine==0){
								 nemadalje=0;
							 }
							 if (map[point.x][point.y].prepreka_bool){
								 brojzauzetih++;
							 }
						 }
					 }
				 }
// 				 if (((brojvecih>=5)&&(nemadalje==0)&&(brojzauzetih>=2)) || ((pomx==Goal.x)&&(pomy==Goal.y)))
				 if (((slican==0)&&(nemadalje==0)) || ((pomx==Goal.x)&&(pomy==Goal.y)))
// if (((brojvecih>=5)&&(nemadalje==0)) || ((pomx==Goal.x)&&(pomy==Goal.y)))
				 {
					 //gusak dolazi
					 gusak=0;
//					 printf("ide provjera za guska, Numminfpts=%d\n", Numminfpts);
					 for (int iii=0; iii<Numminfpts; iii++){
					 	if ((abs(map[minfpoints[iii].x][minfpoints[iii].y].h_cost_int-map[pomx][pomy].h_cost_int)<wmax)&&(abs(minfpoints[iii].x-pomx)+abs(minfpoints[iii].y-pomy)>2)){
					 		gusak=1;
					 		break;
					 	}
					 }
#if 0
					 for (int iii=0;((iii<MapSizeX));iii++){
						 for(int jjj=0;((jjj<MapSizeY));jjj++){
							 if ((map[iii][jjj].total_cost_int==minf)&&(abs(map[iii][jjj].h_cost_int-map[pomx][pomy].h_cost_int)<wmax)&&(abs(iii-pomx)+abs(jjj-pomy)>2)){
								 gusak=1;
								 break;
							 }
						 }
						 if (gusak==1)
						 	break;
					 }
#endif
					 if (gusak==1){
//					 	printf("gusak je tu\n");
						 for ( int d = 0; d < 8; d++ )
						 {
							 point.x=pomx+xofs[d];
							 point.y=pomy+yofs[d];
							 //odredjivanje pozicije novog susjeda
							 if ( IsValid( point.x, point.y )!=0 )
							 {
								 if (brpompom>=maks){
									 maks=brpompom+500;
									 skupolje=(I_point *)realloc(skupolje,(maks)*sizeof(I_point));
								 }
#if DSTAR_REVERSE
								 if ((map[point.x][point.y].total_cost_int==minf)&&(map[point.x][point.y].skupine==0)&&(map[point.x][point.y].h_cost_int<=map[pomx][pomy].h_cost_int))
#else
									 if ((map[point.x][point.y].cvorovi==1)&&(map[point.x][point.y].skupine==0))
#endif
										 {
											 skupolje[brpompom].x=point.x;//dodaj ga u skupinu povezanih
											 skupolje[brpompom].y=point.y;
											 brpompom++;
											 map[point.x][point.y].skupine=*brojskupine;
										 }
							 }
						 }
					 }
					 //gusak odlazi
					 path_witkowski_segment[PathLengthWitkowskiSegment].x=pomx;
					 path_witkowski_segment[PathLengthWitkowskiSegment].y=pomy;
					 PathLengthWitkowskiSegment++;
					 map[pomx][pomy].time_stamp_forward=-1;//oznaka CS tocke iako je S, tako da simetricne situacije uracuna. dolje se zapisuje 0 jer izadje kod S tocke, ali mozda ne ove.
					 //ne uracunava sve slucajeve - fali slucaj kad je simetricno do tanjine jednog polja s obje strane. tad bi trebalo gledati da je S tocka ona koja jedina ima tu vrijednost h-a i nema slicnih u cijeloj skupini. nije implementirano
// 					 if (ispis)
// 						printf("dodaje S point (%d,%d)\n",pomx,pomy);
					 if ((pomx==Goal.x)&&(pomy==Goal.y)){
						 break;
					 }
// 					 break;//zakomentriravam izlaz da uzme cijelu skupinu patak
				 }else{//inace siri skupinu
					 //a prije gledaj ima li onaj koji ima samo jednog veceg, u njegovom susjedstvu vecih <=4
					if ((brojvecih==2)||(brojvecih==3)){//eh ipak mogu bit i 2 takva
						int zapamti_samo_jednog=0; //imam duple u W putu
						  if (((xofs[indeksd2])==0)||((yofs[indeksd2])==0)){
							suma.x=pomx+xofs[indeksd2];
							suma.y=pomy+yofs[indeksd2];
							brojvecih=0;
				 //brojanje koliko ima vecih
							for ( int d = 0; d < 8; d++ )
							{
								point.x=suma.x+xofs[d];
								point.y=suma.y+yofs[d];
		  //odredjivanje pozicije novog susjeda
								if ( IsValid( point.x, point.y )!=0 )
								{
#if DSTAR_REVERSE
									if (map[point.x][point.y].total_cost_int!=minf)
#else
									if (map[point.x][point.y].cvorovi!=1)
#endif
									{
										brojvecih++;
									}
								}
							}
							if (brojvecih<=4){
								path_witkowski_segment[PathLengthWitkowskiSegment].x=pomx;
								path_witkowski_segment[PathLengthWitkowskiSegment].y=pomy;
								PathLengthWitkowskiSegment++;
								zapamti_samo_jednog=1;
								map[pomx][pomy].time_stamp_forward=-1;//oznaka CS tocke
// 								printf("dodaje ga (%d,%d), brojskupine=%d\n",pomx,pomy,map[pomx][pomy].skupine);
// 							break;
							}
						  }
						
						//ak nije taj bio onda za onog drugog jos moramo provjeriti
						
						if ((zapamti_samo_jednog==0) && (((xofs[indeksd])==0)||((yofs[indeksd])==0))){
						suma.x=pomx+xofs[indeksd];
						suma.y=pomy+yofs[indeksd];
						brojvecih=0;
				 //brojanje koliko ima vecih
						for ( int d = 0; d < 8; d++ )
						{
							point.x=suma.x+xofs[d];
							point.y=suma.y+yofs[d];
		  //odredjivanje pozicije novog susjeda
							if ( IsValid( point.x, point.y )!=0 )
							{
#if DSTAR_REVERSE
								if (map[point.x][point.y].total_cost_int!=minf)
#else
								if (map[point.x][point.y].cvorovi!=1)
#endif
								{
									brojvecih++;
								}
							}
						}
						if (brojvecih<=4){
							path_witkowski_segment[PathLengthWitkowskiSegment].x=pomx;
							path_witkowski_segment[PathLengthWitkowskiSegment].y=pomy;
							PathLengthWitkowskiSegment++;
							zapamti_samo_jednog=1;
							map[pomx][pomy].time_stamp_forward=-1;//oznaka CS tocke
// 					 printf("dodaje ga (%d,%d), brojskupine=%d\n",pomx,pomy,map[pomx][pomy].skupine);
// 							break;
						}
						}
						if ((zapamti_samo_jednog==0) && (brojvecih==3) && (((xofs[indeksd3])==0)||((yofs[indeksd3])==0))){
						  suma.x=pomx+xofs[indeksd3];
						  suma.y=pomy+yofs[indeksd3];
						  brojvecih=0;
						  //brojanje koliko ima vecih
						  for ( int d = 0; d < 8; d++ )
						  {
						    point.x=suma.x+xofs[d];
						    point.y=suma.y+yofs[d];
						    //odredjivanje pozicije novog susjeda
						    if ( IsValid( point.x, point.y )!=0 )
						    {
#if DSTAR_REVERSE
						      if (map[point.x][point.y].total_cost_int!=minf)
#else
							if (map[point.x][point.y].cvorovi!=1)
#endif
							  {
							    brojvecih++;
							  }
						    }
						  }
						  if (brojvecih<=4){
						    path_witkowski_segment[PathLengthWitkowskiSegment].x=pomx;
						    path_witkowski_segment[PathLengthWitkowskiSegment].y=pomy;
						    PathLengthWitkowskiSegment++;
						    zapamti_samo_jednog=1;
						    map[pomx][pomy].time_stamp_forward=-1;//oznaka CS tocke
// 						    printf("dodaje ga (%d,%d), brojskupine=%d\n",pomx,pomy,map[pomx][pomy].skupine);
						    // 							break;
						  }
						}
					}// brojvecih==2
					for ( int d = 0; d < 8; d++ )
					{
					 point.x=pomx+xofs[d];
					 point.y=pomy+yofs[d];
		  //odredjivanje pozicije novog susjeda
					 if ( IsValid( point.x, point.y )!=0 )
					 {
						 if (brpompom>=maks){
							 maks=brpompom+500;
							 skupolje=(I_point *)realloc(skupolje,(maks)*sizeof(I_point));
						 }
#if DSTAR_REVERSE
						 if ((map[point.x][point.y].total_cost_int==minf)&&(map[point.x][point.y].skupine==0)&&(map[point.x][point.y].h_cost_int<=map[pomx][pomy].h_cost_int))
#else
						if ((map[point.x][point.y].cvorovi==1)&&(map[point.x][point.y].skupine==0))
#endif
						 {
							 skupolje[brpompom].x=point.x;//dodaj ga u skupinu povezanih
							 skupolje[brpompom].y=point.y;
							 brpompom++;
							 map[point.x][point.y].skupine=*brojskupine;
						 }
					 }
					}
				 }
			 }
			 brpom=imaih;//to je sad pocetak
			 imaih=brpompom;//a ovo je kraj
			 if ((imaih==brpom) || ((pomx==Goal.x)&&(pomy==Goal.y))){
// // 				 if (ispis)
// 					printf("svi su oznaceni, izlazi van\n");
				 break;
			 }
		 }
	
// 	 printf("brojskupine=%d, imaih=%d\n",*brojskupine, imaih);
				//sad imamo skupinex i y
		 //gledanje prelazi li segment dviju susjednih tocaka preko sum_min costova
#if 0
	if ((PathLengthWitkowskiSegment>1)&&(imaih>5)&&(0)){
		orientation=atan2((path_witkowski_segment[PathLengthWitkowskiSegment-1].y-path_witkowski_segment[PathLengthWitkowskiSegment-2].y), (path_witkowski_segment[PathLengthWitkowskiSegment-1].x-path_witkowski_segment[PathLengthWitkowskiSegment-2].x));
		point.x=path_witkowski_segment[PathLengthWitkowskiSegment-2].x+int(0.5*cos(orientation));
		point.y=path_witkowski_segment[PathLengthWitkowskiSegment-2].y+int(0.5*sin(orientation));
		brpom=0;
		brojvecih=0;
		while ((abs(point.x-path_witkowski_segment[PathLengthWitkowskiSegment-1].x)>1) || (abs(point.y-path_witkowski_segment[PathLengthWitkowskiSegment-1].y)>1)){
			brpom++;
			if ((map[point.x][point.y].cvorovi!=1) && (map[point.x][point.y].skupine!=99))
			{
				brojvecih++;
// 				map[point.x][point.y].skupine=99;
			}
			map[point.x][point.y].skupine=99;
			point.x=path_witkowski_segment[PathLengthWitkowskiSegment-2].x+int(brpom*0.5*cos(orientation));
			point.y=path_witkowski_segment[PathLengthWitkowskiSegment-2].y+int(brpom*0.5*sin(orientation));
		}
// 		printf("brojvecih na pravcu=%d\n",brojvecih);
		 //tu sam stala
		 suma.x=0; suma.y=0;//inicijalizacija
		 for (int in=0; in<imaih; in++){
			 suma.x+=skupolje[in].x;
			 suma.y+=skupolje[in].y;
		 }
		 suma.x=int(suma.x/imaih);
		 suma.y=int(suma.y/imaih);
		 map[suma.x][suma.y].skupine=100;
		 if (map[suma.x][suma.y].cvorovi!=1){
			 for ( int d = 0; d < 8; d++ )//uzmi najblizeg
			 {
				 point.x=suma.x+xofs[d];
				 point.y=suma.y+yofs[d];
		  //odredjivanje pozicije novog susjeda
				 if ( IsValid( point.x, point.y )!=0 )
				 {
					 if (map[point.x][point.y].cvorovi==1){
						 suma=point;
						 map[point.x][point.y].skupine=101;
						 break;
					 }
				 }
			 }
		 }
		 if (brojvecih){
			 path_witkowski_segment[PathLengthWitkowskiSegment]=path_witkowski_segment[PathLengthWitkowskiSegment-1];
			 path_witkowski_segment[PathLengthWitkowskiSegment-1]=suma;
			 PathLengthWitkowskiSegment++;
		 }
	}
#endif
	}

	pomx=path_witkowski_segment[PathLengthWitkowskiSegment-1].x;
	pomy=path_witkowski_segment[PathLengthWitkowskiSegment-1].y;
	map[pomx][pomy].time_stamp_forward=0;
	if (ispis)
	for (int i=0;i<brpompom;i++){
		printf("skupolje(%d)=(%d,%d)\t",i,skupolje[i].x,skupolje[i].y);
	}
	 free(skupolje);
	 if (imaih>1){
		 return 1;
	 }
	 else{
		 return 0;
	 }
 }

I_point* DStar::getPathOri(){

	 PathLength=0;
	 if(!PathExists)
	 {
		 printf("DStar getPathOri> Path does not exist!!!");
		 return NULL;
	 }

  //ako treba inicijalizirati path
 I_point temp1,temp2;
 I_point trazipetlju[10];//tu ce se pamtiti zadnjih 10 polja puta iz susjeda. ako se neki ponavlja onda javi da je petlja i izadji van iz tog susjeda. cirkularno polje
 int problemcek=0;
 PathLength=1;
	  //prvo provjeravamo dal je put ok i kolko je dugacak
 temp1=Start; 
 while((temp1.x!=Goal.x)||(temp1.y!=Goal.y)||(temp1.th!=Goal.th))
 {
// 	printf("%d, %d, %d\n", temp1.x,temp1.y,temp1.th);
		 
	 if ( IsValidOri( temp1 )!=1 )
	 {
		 printf("DStar> Odriftali smo iz mape il je mozda prepreka!!! temp1.x=%d, temp1.y=%d, temp1.th=%d, Start.x=%d, Start.y=%d, Start.th=%d\n", temp1.x,temp1.y,temp1.th,Start.x,Start.y,Start.th);
		 problemcek=2;
		 break;
			 //return NULL;
	 }
	 else
	 {
		 temp2=map[temp1.x][temp1.y]._nextOri[temp1.th];
		 if ((temp2.x==-1)||(temp2.y==-1)) {
			 printf("nema next\n");
			 problemcek=3;
			 break;
			 //return NULL;
		 }
		 arc_costOri( temp2, temp1);
		 if (c>=OBSTACLE) {
			 printf("OBSTACLE u next-u.\n");
		         printf("DStar getPathOri> temp2= (%d, %d, %d)\n", temp2.x,temp2.y,temp2.th);
			 problemcek=2;
			 break;
			 //return NULL;
		 }
		 if (PathLength<=10){
			 trazipetlju[PathLength-1]=temp1;
			 for (int i=0;i<PathLength;i++){
				 if ((temp2.x==trazipetlju[i].x)&&(temp2.y==trazipetlju[i].y)&&(temp2.th==trazipetlju[i].th)){
					 printf("jao petlja na putu detektirana pomocu polja trazipetlju. duljina puta je %d\n",PathLength);
	printf("temp1 %d, %d, %d\n", temp1.x,temp1.y,temp1.th);
 	printf("temp2 %d, %d, %d\n", temp2.x,temp2.y,temp2.th);
					 problemcek=1;
					 break;
			 //					 return NULL;
				 }
			 }
			 if (problemcek)
				 break;
		 }else{
			 trazipetlju[(PathLength % 10) -1]=temp1;
			 for (int i=0;i<10;i++){
				 if ((temp2.x==trazipetlju[i].x)&&(temp2.y==trazipetlju[i].y)&&(temp2.th==trazipetlju[i].th)){
					 printf("jao petlja na putu detektirana pomocu polja trazipetlju. duljina puta je %d\n",PathLength);
	printf("temp1 %d, %d, %d\n", temp1.x,temp1.y,temp1.th);
 	printf("temp2 %d, %d, %d\n", temp2.x,temp2.y,temp2.th);

					 problemcek=1;
					 break;
			 //					 return NULL;
				 }
			 }
			 if (problemcek)
				 break;
		 }
		 temp1=temp2;
		 PathLength++;
		 if (PathLength>maxOri*MapSizeX*MapSizeY)
		 {
			 printf("jao petlja!time_stamp_counter=%d\n",time_stamp_counter);
			 problemcek=1;
			 break;
			 //return NULL;
		 }
	 }
 }
 if (problemcek){
	 printf("DS::getPathOri()>petlja=1, prepreka=2, nema next=3: (%d)\n",problemcek);
	 PathLength=0;
		 		 return NULL;
 }else{

 temp1=Start;
 for(int i=0;i<PathLength;i++)
 {
	 path[i]=temp1;
	 temp1=map[path[i].x][path[i].y]._nextOri[path[i].th];
 }
 cijena_puta[brojac_izracuna-1]=map[Start.x][Start.y].h_cost_intOri[Start.th];
 duljina_puta[brojac_izracuna-1]=PathLength;
 printf("cijena 3D puta je %d\n",map[Start.x][Start.y].h_cost_intOri[Start.th]);
if ((radiponovo) || (prviput)){
  double duljina_ds_puta=0.;
  int traversalcostpath;

	  if ( (logfile = fopen("logger//traversalcostpath","wt")) == NULL ){
	 	  printf("Error! file couldn't be opened.");
	  }else{

  for (int i=0; i<PathLength-1; i++){
#if USE3DCOST
    IntPose pointX((path[i].x),(path[i].y),path[i].th);
    traversalcostpath = std::max(1., (MAXDISTANCE - floor(cspace->getDistanceInCells(pointX))));
#else
    traversalcostpath = map[path[i].x][path[i].y].traversal_cost;
    arc_costOri( path[i+1], path[i]);
    traversalcostpath=c;
    
    
#endif
    fprintf(logfile,"%d %d\n",traversalcostpath, map[path[i].x][path[i].y].h_cost_intOri[path[i].th]);

    duljina_ds_puta+=sqrt((path[i+1].x-path[i].x)*(path[i+1].x-path[i].x)+(path[i+1].y-path[i].y)*(path[i+1].y-path[i].y))*(CELL_DIM)*0.001;
  }
	 	  fclose(logfile);
	  }
  if ( (logfile = fopen("komb","a")) == NULL )
    printf("Error! komb file couldn't be opened.");
  else{
    fprintf(logfile,"DS::getPath() duljina putanje [m]=%f\n", duljina_ds_puta);
    fclose(logfile);
  }
  duljina_puta_um[(brojac_izracuna-1)]=duljina_ds_puta;
}
  if (brojac_izracuna%(MapSizeX*MapSizeY)==0){
	  if(((azurirani_polje = (int *)realloc( azurirani_polje, (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((izracuni = (int *)realloc( izracuni, (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&(( f_biased_putanje= (int *)realloc( f_biased_putanje, (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((f_putanje = (int *)realloc(f_putanje , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((h_putanje = (int *)realloc( h_putanje, (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&(( broj_iteracija= (int *)realloc(broj_iteracija , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((broj_cvorova_na_listi = (int *)realloc(broj_cvorova_na_listi , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((max_broj_cvorova_na_listi = (int *)realloc(max_broj_cvorova_na_listi , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((cijena_puta = (int *)realloc(cijena_puta , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((duljina_puta = (int *)realloc(duljina_puta , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((duljina_puta_um = (double *)realloc(duljina_puta_um , (brojac_izracuna+(MapSizeX*MapSizeY))*sizeof(double)))!=NULL)){
		  printf("DStar::getPath> REALLOC succeeded\n");
	  }else{
		  printf("DStar::getPath> Error allocating memory REALLOC!!!\n");
		  return NULL;
	  }
 }
 return path;
}
}

 I_point* DStar::getPath_backward(){

	 PathLength_backward=0;
	 if(!PathExistsWitkowski)
	 {
		 printf("DStar getPath_backward> Path does not exist!!!");
		 return NULL;
	 }

  //ako treba inicijalizirati path
 I_point temp1,temp2;
 I_point trazipetlju[10];//tu ce se pamtiti zadnjih 10 polja puta iz susjeda. ako se neki ponavlja onda javi da je petlja i izadji van iz tog susjeda. cirkularno polje
 int traversal_cost,total_cost;
 int problemcek=0;
 total_cost=0;
 PathLength_backward=1;
	  //prvo provjeravamo dal je put ok i kolko je dugacak
//  temp1.x=StartRacunac.x; temp1.y=StartRacunac.y;
 temp1.x=Start.x; temp1.y=Start.y;
 while((temp1.x!=Goal.x)||(temp1.y!=Goal.y))
 {
	 if ( IsValid( temp1.x, temp1.y )!=1 )
	 {
		 printf("DStar> Odriftali smo iz mape il je mozda prepreka!!! temp1.x=%d, temp1.y=%d, Start.x=%d, Start.y=%d\n", temp1.x,temp1.y,Start.x,Start.y);
		 problemcek=2;
		 break;
			 //return NULL;
	 }
	 else
	 {
		 temp2.x=map[temp1.x][temp1.y]._next_backward.x;
		 temp2.y=map[temp1.x][temp1.y]._next_backward.y;
		 if ((temp2.x==-1)||(temp2.y==-1)) {
			 printf("nema next\n");
			 problemcek=3;
			 break;
			 //return NULL;
		 }
		 if ((temp1.x==temp2.x)||(temp1.y==temp2.y)){
			 traversal_cost=COSTSTRAIGHT;
		 } else {
			 traversal_cost=COSTDIAGONAL;
		 }
		 arc_cost( temp2.x,temp2.y,temp1.x,temp1.y);
		 if (c>=OBSTACLE) {
			 printf("OBSTACLE u next-u.\n");
			 problemcek=2;
			 break;
			 //return NULL;
		 }
		 total_cost+=traversal_cost*c;//racunam ukupni cost na putu
		 if (PathLength_backward<=10){
			 trazipetlju[PathLength_backward-1].x=temp1.x;
			 trazipetlju[PathLength_backward-1].y=temp1.y;
			 for (int i=0;i<PathLength_backward;i++){
				 if ((temp2.x==trazipetlju[i].x)&&(temp2.y==trazipetlju[i].y)){
					 printf("jao petlja na putu detektirana pomocu polja trazipetlju. duljina puta je %d\n",PathLength_backward);
					 problemcek=1;
					 break;
			 //					 return NULL;
				 }
			 }
			 if (problemcek)
				 break;
		 }else{
			 trazipetlju[(PathLength_backward % 10) -1].x=temp1.x;
			 trazipetlju[(PathLength_backward % 10) -1].y=temp1.y;
			 for (int i=0;i<10;i++){
				 if ((temp2.x==trazipetlju[i].x)&&(temp2.y==trazipetlju[i].y)){
					 printf("jao petlja na putu detektirana pomocu polja trazipetlju. duljina puta je %d\n",PathLength_backward);
					 problemcek=1;
					 break;
			 //					 return NULL;
				 }
			 }
			 if (problemcek)
				 break;
		 }
		 temp1.x=temp2.x;
		 temp1.y=temp2.y;
		 PathLength_backward++;
		 if (PathLength_backward>MapSizeX*MapSizeY)
		 {
			 printf("jao petlja!time_stamp_counter=%d\n",time_stamp_counter);
			 problemcek=1;
			 break;
			 //return NULL;
		 }
	 }
 }
 if (problemcek){
	 printf("DS::getPath_backward()>petlja=1, prepreka=2, nema next=3: (%d)\n",problemcek);
	 PathLength_backward=0;
		 		 return NULL;
 }else{

//  temp1.x=StartRacunac.x; temp1.y=StartRacunac.y;
 temp1.x=Start.x; temp1.y=Start.y;
 for(int i=0;i<PathLength_backward;i++)
 {
	 path_backward[i].x=temp1.x;
	 path_backward[i].y=temp1.y;
	 temp1.x=map[path_backward[i].x][path_backward[i].y]._next_backward.x;
	 temp1.y=map[path_backward[i].x][path_backward[i].y]._next_backward.y;
 }
 cijena_puta_backward[brojac_izracunaWitkowski-1]=total_cost;
 duljina_puta_backward[brojac_izracunaWitkowski-1]=PathLength_backward;
 printf("cijena backward puta je %d\n",total_cost);
 }
 if ( (logfile = fopen("komb","a")) == NULL )
	 printf("Error! komb file couldn't be opened.");
 else{
	 fprintf(logfile,"cijena backward puta je %d, a duljina puta u cvorovima %d\n",total_cost,PathLength_backward);
 }
 fclose(logfile);
 return path_backward;
}


 I_point* DStar::getPath_forward(){//ovaj se mora pozivati zadnji zbog provjere logera

	 PathLength_forward=0;
	 if(!PathExistsWitkowski)
	 {
		 printf("DStar getPath_forward> Path does not exist!!!");
		 return NULL;
	 }

  //ako treba inicijalizirati path
	 I_point temp1,temp2;
	 I_point trazipetlju[10];//tu ce se pamtiti zadnjih 10 polja puta iz susjeda. ako se neki ponavlja onda javi da je petlja i izadji van iz tog susjeda. cirkularno polje
	 int traversal_cost,total_cost,problemcek=0;
	 total_cost=0;
	 PathLength_forward=1;
	  //prvo provjeravamo dal je put ok i kolko je dugacak
	 temp1.x=Goal.x; temp1.y=Goal.y;
	 while((temp1.x!=Start.x)||(temp1.y!=Start.y))
// 	 while((abs(temp1.x-StartRacunac.x)>10)||(abs(temp1.y-StartRacunac.y)>10))
	 {
		 if ( IsValid( temp1.x, temp1.y )!=1 )
		 {
			 printf("DStar> Odriftali smo iz mape il je mozda prepreka!!! temp1.x=%d, temp1.y=%d, Start.x=%d, Start.y=%d\n", temp1.x,temp1.y,Start.x,Start.y);
			 problemcek=2;
			 break;
			 //return NULL;
		 }
		 else
		 {
			 temp2.x=map[temp1.x][temp1.y]._next_forward.x;
			 temp2.y=map[temp1.x][temp1.y]._next_forward.y;
			 if ((temp2.x==-1)||(temp2.y==-1)) {
				 printf("nema next (%d,%d)\n",temp1.x,temp1.y);
				 //nemanext=temp1;
				 problemcek=3;
				 break;
			 //return NULL;
			 }
			 if ((temp1.x==temp2.x)||(temp1.y==temp2.y)){
				 traversal_cost=COSTSTRAIGHT;
			 } else {
				 traversal_cost=COSTDIAGONAL;
			 }
			 arc_cost( temp2.x,temp2.y,temp1.x,temp1.y);
			 if (c>=OBSTACLE) {
				 printf("OBSTACLE u next-u.\n");
				 problemcek=2;
				 break;
			 //return NULL;
			 }
			 total_cost+=traversal_cost*c;//racunam ukupni cost na putu
			 if (PathLength_forward<=10){
				 trazipetlju[PathLength_forward-1].x=temp1.x;
				 trazipetlju[PathLength_forward-1].y=temp1.y;
				 for (int i=0;i<PathLength_forward;i++){
					 if ((temp2.x==trazipetlju[i].x)&&(temp2.y==trazipetlju[i].y)){
						 printf("jao petlja na putu detektirana pomocu polja trazipetlju. duljina puta je %d\n",PathLength_forward);
						 problemcek=1;
						 break;
			 //return NULL;
					 }
				 }
				 if (problemcek)
					 break;
			 }else{
				 trazipetlju[(PathLength_forward % 10) -1].x=temp1.x;
				 trazipetlju[(PathLength_forward % 10) -1].y=temp1.y;
				 for (int i=0;i<10;i++){
					 if ((temp2.x==trazipetlju[i].x)&&(temp2.y==trazipetlju[i].y)){
						 printf("jao petlja na putu detektirana pomocu polja trazipetlju. duljina puta je %d\n",PathLength_forward);
						 problemcek=1;
						 break;
			 //return NULL;
					 }
				 }
				 if (problemcek)
					 break;
			 }
			 temp1.x=temp2.x;
			 temp1.y=temp2.y;
			 PathLength_forward++;
			 if (PathLength_forward>MapSizeX*MapSizeY)
			 {
				 printf("jao petlja!time_stamp_counter=%d\n",time_stamp_counter);
				 problemcek=1;
				 break;
			 //return NULL;
			 }
		 }
	 }
	 if (problemcek){
		 printf("DS::getPath_forward()>petlja=1, prepreka=2, nema next=3: (%d)\n",problemcek);
		 PathLength_forward=0;
		 		 //return NULL;
	 }else{
	 printf("temp1=(%d,%d), Start=(%d,%d), pathCostEstimate=%d,broj celija %d\n",temp1.x,temp1.y,Start.x,Start.y,pathCostEstimateDiagonalStraight(temp1,Start),std::max( abs( temp1.x-Start.x ), abs( temp1.y-Start.y ) ));
	 total_cost+=pathCostEstimateDiagonalStraight(temp1,Start);//Racunac);
	 PathLength_forward+=std::max( abs( temp1.x-Start.x ), abs( temp1.y-Start.y ) );
	 cijena_puta_forward[brojac_izracunaWitkowski-1]=total_cost;
	 duljina_puta_forward[brojac_izracunaWitkowski-1]=PathLength_forward;
	 printf("cijena forward puta je %d\n",total_cost);

	 temp1.x=Goal.x; temp1.y=Goal.y;
	 for(int i=0;i<PathLength_forward;i++)
	 {
		 path_forward[i].x=temp1.x;
		 path_forward[i].y=temp1.y;
		 temp1.x=map[path_forward[i].x][path_forward[i].y]._next_forward.x;
		 temp1.y=map[path_forward[i].x][path_forward[i].y]._next_forward.y;
		 if (temp1.x==-1){
			 PathLength_forward=i+1;
			 break;
		 }
	 }
 	}
	 if ( (logfile = fopen("komb","a")) == NULL )
		 printf("Error! komb file couldn't be opened.");
	 else{
		 fprintf(logfile,"cijena forward puta je %d, a duljina puta u cvorovima %d\t",total_cost,PathLength_forward);
	 }
	 fclose(logfile);
	if (brojac_izracunaWitkowski%(MapSizeX*MapSizeY)==0){
		if(((azurirani_backward_polje = (int *)realloc( azurirani_backward_polje, (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((azurirani_forward_polje = (int *)realloc(azurirani_forward_polje , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((izracuniWitkowski = (int *)realloc(izracuniWitkowski , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((vremena_skupine = (int *)realloc(vremena_skupine , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&(( broj_iteracijaWitkowski= (int *)realloc(broj_iteracijaWitkowski , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((broj_cvorova_na_listi_forward = (int *)realloc(broj_cvorova_na_listi_forward , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((max_broj_cvorova_na_listi_forward = (int *)realloc(max_broj_cvorova_na_listi_forward , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((cijena_puta_forward = (int *)realloc( cijena_puta_forward, (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((cijena_puta_backward = (int *)realloc(cijena_puta_backward , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((duljina_puta_forward = (int *)realloc(duljina_puta_forward , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((broj_cvorova_na_listi_backward = (int *)realloc( broj_cvorova_na_listi_backward, (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((max_broj_cvorova_na_listi_backward = (int *)realloc(max_broj_cvorova_na_listi_backward , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)&&((duljina_puta_backward = (int *)realloc(duljina_puta_backward , (brojac_izracunaWitkowski+(MapSizeX*MapSizeY))*sizeof(int)))!=NULL)){
			printf("DStar::getPath_forward> REALLOC uspio\n");
		}else{
			printf("DStar::getPath_forward> Error allocating memory REALLOC!!!\n");
			return NULL;
		}
	 }
	 return path_forward;
 }
 
