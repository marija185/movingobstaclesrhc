#include "DynamicWindow.h"
#include "GridMap.h"    //
#include "DStar.h"
#include <iostream>

#if RECTANGULAR
#include <cspacevoronoi.h>
#endif

extern WorkHorse *WH;
extern Planner *PL;
extern GridMap *GM;
extern DStar *DS;
#if RECTANGULAR
extern CSpaceVoronoi *cspace;
#endif

DynamicWindow::DynamicWindow(){
   broj_DStar_celija=(int)ceil(DUZINA_DSTAR_PUTA/CELL_DIM);
   //reducirana putanja - efektivna putanja
   path_r_reduced = (R_point *)malloc(N_PATH_REDUCED*sizeof(R_point)) ;
   //oversempliranja geometrijska putanja
//    path_r_sampled=(R_point*)malloc((int)(floor(GM->Map_Dim_X_A/CELL_DIM)*floor(GM->Map_Dim_Y_A/CELL_DIM)*(REAL_PATH_SAMPLE+1))*sizeof(R_point));
   naStartu=true;
   poceo=false;
   obstacle_index=N_KL;
   min_obstacle_index=N_KL;
   naCilju=false;
   rotateongoal=false;
   blizu=false;
   flag_kl_old=false;
   best_old=false;
   T_old=0;//N_KL-4;//bilo je N_KL-3, ovako ima i plus trajektoriju na pocetku
   blizu_cilja=false;
   kut_E=0.;
   ukrivustranu=0;
   oznaka=0;
   obrnuto=0;
   path_length=0.;
   tocka_infleksije.x=0.;
   tocka_infleksije.y=0.;
   vs=0;ws=0;
ROBOT_X = 0;
ROBOT_Y= 0;
ROBOT_W= 0;
 ROBOT_V= 0;
 ROBOT_TH= 0;
}

//main function of DW
void DynamicWindow::Sekvenca_izvodjenja(){
	poceo=true;//the start of the DW calculation
	path_length=0.;
	if (flag_kl_old==false) T_old=0;//N_KL-4;//
	TimeLog(0);  //timing the start of the algorithm
  //current state of the robot
	Azuriraj_stanje_robota();
	SP.setpoint_v=0.;
	SP.setpoint_vy=0.;
	SP.setpoint_w=0.;
	SP.setpoint_x=RB.x;
	SP.setpoint_y=RB.y;
	SP.setpoint_th=RB.th;
	
//	if (flag_kl_old==false) desiredOrientation=WH->global_goal_workhorse.th;
//----------------------------------------------
//	desiredOrientation=checkDesiredOrientation(WH->global_goal_workhorse.th);
//------------------------------------------------
  //ovdje se pronalaze tocke effektivne putanje-to je set tocaka path_r_reduced sa kojim se usporedjuje pojedina trajektorija
#if (STARASUMA==1)
	if(!FindEffectivePath()){
		broj_DStar_celija=0;
		printf("DynamicWindow> Couldn't load the path=NULL");
		return;
	}
#endif
	Lambda=LAMBDA;
  //ubacivanje novih mjerenja lasera
	Ubaci_laser();
  //odredjivanje set-a brzina
  //1. s obzirom na DV_MAX, DW_MAX odredjujemo trenutni dinamicki prozor
  //2. s obzirom na V_MAX, W_MAX
	Odredi_dinamicki_prozor(); //svi parovi dobivaju zastavicu INITIAL, a oni koji premasuju kinematicka ogranicenja dobivaju zastavicu KINEMATIC_CONSTRAINTS
  //TimeLog(1);
	Pokretna_prepreka();//ako je detektirana pokretna prepreka u GM onda se puni polje PPP estimiranim polozajima prepreke u buducim trenucima
#if (STARASUMA==0)
	if (flag_kl_old){
		cost_old=Old_traj();
	}
	cost_breakage=Breakage_traj();
#endif
  		//za sve parove brzina ako su dohvatljivi,
		//odredjujemo kruzni luk i minimalni zaustavni put
#if OMNIDRIVE
	for(nk=0;nk<VY_DIM+1;nk++)
#endif
	{
	for(ni=0;ni<V_DIM+1;ni++){
		for(nj=0;nj<W_DIM+1;nj++){
				// Nastavak proracuna se vrsi samo ako je par dohvatljiv
        			//unutar dinamickog prozora
#if OMNIDRIVE
			if(TB.flag[ni][nj][nk]==INITIAL)
#else
			if(TB.flag[ni][nj]==INITIAL)
#endif
			{
				MinimalniZaustavniPut();    //izracunava se ss
#if (KRUZNI_LUK==1)
				Kruzni_luk();
#else
				Kruzni_luk_sa_zaustavljanjem();//postavi i za plus i minus KL, KL_plus i KL_minus
#endif
		     // TimeLog(1);
           //prohodnost odredjene trajektorije & zabrana ako bi doslo do kolizije
#if (KRUZNI_LUK==1)
				Prohodnost();
#endif
	      //TimeLog(1);
	     //ako je putanja omogucena s obzirom na prohodnost onda se racuna poklapanje s efektivnom putanjom
#if (STARASUMA==1)
#if OMNIDRIVE
				if((TB.flag[ni][nj][nk]==CLEAR)||(TB.flag[ni][nj][nk]==HAS_OBSTACLE)){
					if(broj_DStar_celija>0){ 
						TB.ocjena_path[ni][nj][nk]=PathAlignment();
               //TimeLog(1);
					}
				}
#else
				if((TB.flag[ni][nj]==CLEAR)||(TB.flag[ni][nj]==HAS_OBSTACLE)){
					if(broj_DStar_celija>0){ 
						TB.ocjena_path[ni][nj]=PathAlignment();
               //TimeLog(1);
					}
				}
#endif
#else
// 				if((TB.flag[ni][nj]==CLEAR)){
#if OMNIDRIVE
						TB.ocjena_path[ni][nj][nk]=PathAlignment();//ova postavi i za plus i minus
						TB.ocjena[ni][nj][nk]=TB.ocjena_path[ni][nj][nk];
						TB_plus.ocjena[ni][nj][nk]=TB_plus.ocjena_path[ni][nj][nk];
#if ROT
						TB_plus1.ocjena[ni][nj][nk]=TB_plus1.ocjena_path[ni][nj][nk];
						TB_plus2.ocjena[ni][nj][nk]=TB_plus2.ocjena_path[ni][nj][nk];
#endif
						TB_minus.ocjena[ni][nj][nk]=TB_minus.ocjena_path[ni][nj][nk];
						TB_minus2.ocjena[ni][nj][nk]=TB_minus2.ocjena_path[ni][nj][nk];
#else
						TB.ocjena_path[ni][nj]=PathAlignment();//ova postavi i za plus i minus
						TB.ocjena[ni][nj]=TB.ocjena_path[ni][nj];
						TB_plus.ocjena[ni][nj]=TB_plus.ocjena_path[ni][nj];
#if ROT
						TB_plus1.ocjena[ni][nj]=TB_plus1.ocjena_path[ni][nj];
						TB_plus2.ocjena[ni][nj]=TB_plus2.ocjena_path[ni][nj];
#endif
						TB_minus.ocjena[ni][nj]=TB_minus.ocjena_path[ni][nj];
						TB_minus2.ocjena[ni][nj]=TB_minus2.ocjena_path[ni][nj];
#endif
// 				}
#endif
//ovdje se azurira kruzni luk
				LogMisc(0);
         //ako zbog kinematickih ogranicenja (max i min brzine) odredjena trajektorija nije omogucena
			}else{   //za  KINEMATIC_CONSTRAINTS
#if (STARASUMA==0)
#if OMNIDRIVE
				TB_plus.flag[ni][nj][nk]=TB.flag[ni][nj][nk];
				TB_minus.flag[ni][nj][nk]=TB.flag[ni][nj][nk];
				TB_minus2.flag[ni][nj][nk]=TB.flag[ni][nj][nk];
#if ROT
				TB_plus1.flag[ni][nj][nk]=TB.flag[ni][nj][nk];
				TB_plus2.flag[ni][nj][nk]=TB.flag[ni][nj][nk];
#endif
#else
				TB_plus.flag[ni][nj]=TB.flag[ni][nj];
#if ROT
				TB_plus1.flag[ni][nj]=TB.flag[ni][nj];
				TB_plus2.flag[ni][nj]=TB.flag[ni][nj];
#endif
				TB_minus.flag[ni][nj]=TB.flag[ni][nj];
				TB_minus2.flag[ni][nj]=TB.flag[ni][nj];
#endif
#endif
				
				LogMisc(0);      //radi crtanja
			}
		} // po svim w
	}   //po svim v
	} // for all vy
#if (STARASUMA==1)
	DoprinosProhodnost();
  //TimeLog(1);
	if(broj_DStar_celija>0){
		Doprinos_path();               //doprinos za path (path_alignment)
	}
#endif
	ni=-1; nj=-1;
   //PRORACUN OPTIMALNOG PARA  uz kut okretanja u odnosu na efektivnu putanju
	double kut;
	kut=RB.th-path_orientation;
	//ako se radi o okretanju na cilju
	if (naCilju)
		kut=RB.th-WH->global_goal_workhorse.th;
// 	bool krivi=false;
	while (kut<-1*M_PI) kut+=2*M_PI;
	while (kut>M_PI) kut-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
	if (((kut<M_PI)&&(kut>0)&&(RB.w>0))||((kut>M_PI)&&(kut<2*M_PI)&&(RB.w<0))) {
// 		printf("DW:Sekvenca_izvodjenja> krivi smjer okretanja prije trazenja optimalnog para, kut=%f, RB.th=%f, ef_th=%f\n",kut*RuS, RB.th*RuS, path_orientation*RuS);
// 		krivi=true;
	}
#if (STARASUMA==1)
	Optimalni_par(kut);
#else
	Path_minimum();//ovdje pregleda sve CLEAR
#endif
/*	int vc,wc;
	vc=int(round(WH->v_refdin_current));
	wc=int(round(RuS*WH->w_refdin_current));*/
	double rbv,rbvy,rbw;
#if (OKO_REFERENCE==1)
	rbv=WH->v_refdin_current;
	rbvy=WH->vy_refdin_current;
	rbw=WH->w_refdin_current;
#else
	rbv=RB.v;
	rbvy=RB.vy;
	rbw=RB.w;
#endif
 //Postavljanje setpointa vektora brzine i azuriranje loggera
   //ako smo uspijeli nesto odabrati!
	if((ni>=0)&&(nj>=0)){
			LogMisc(2);//tu se upisuje KL optimalan
			SP.setpoint_v=KL.v[1];
			SP.setpoint_vy=KL.vy[1];
			SP.setpoint_w=KL.w[1];
			SP.setpoint_x=KL.x[1];
			SP.setpoint_y=KL.y[1];
			SP.setpoint_th=KL.th[1];
//			best_old=false;
#if (STARASUMA==1)
	//ograniciti okretanje (optimalni par zadaje 1/2 maksimalne, tu cemo ju korigirati)
			if ((naStartu)&&(fabs(SP.setpoint_w)>W_MAX/4)) {
					if (SP.setpoint_w>0) SP.setpoint_w=W_MAX/4;
					if (SP.setpoint_w<0) SP.setpoint_w=-W_MAX/4;
			}
#endif
			printf("DynamicWindow> setpoint from trajectory (%d,%d,%d) KL.(v,w,vy)=(%f,%f,%f)\n",ni,nj,nk,SP.setpoint_v,SP.setpoint_w*RuS,SP.setpoint_vy);
#if (NO_LASER==0) && 0
			if ((WH->LB.laser_pointer_new==0)&&(WH->LB2.laser_pointer_new==0)){
				//bira se stara referenca tako da bude jednaka staroj akceleraciji, ali ne smije bit van kinematickih ogranicenja
				printf("DW> no laser readings!\n");
		      //moramo postaviti brzine i set - point u nulu
				SP.setpoint_v=0.;
				SP.setpoint_vy=0.;
				SP.setpoint_w=0.;
			}
#endif

	}else{
      //nismo uspijeli nista odabrati
      //moramo postaviti brzine i set - point u nulu
		SP.setpoint_v=0.;
		SP.setpoint_w=0.;
		SP.setpoint_vy=0.;
        // ne zadajemo brzinu 0 nego ju smanjujemo u skladu sa kinematickim ogranicenjima (da se ne umaraju motori)
		best_old=true;//zakomentiravam ovdje jer ju postavljam samo ako nema nijedno drugo rj
		LogMisc(3); //polje optimalne trajektorije se puni trenutnom pozicijom robota
		TimeLog(1);
		printf("DynamicWindow> Feasible trajectory is not chosen from the search set!\n");
#if (STARASUMA==0)
		if (best_breakage){
			SP.setpoint_v=KL_breakage.v[1];
			SP.setpoint_vy=KL_breakage.vy[1];
			SP.setpoint_w=KL_breakage.w[1];
			SP.setpoint_x=KL_breakage.x[1];
			SP.setpoint_y=KL_breakage.y[1];
			SP.setpoint_th=KL_breakage.th[1];
			printf("DynamicWindow> setpoint from the breakage trajectory KL_breakage.(v,w,vy)=(%f,%f,%f)\n",SP.setpoint_v,SP.setpoint_w*RuS,SP.setpoint_vy);
			KL_old_temp=KL_breakage;
			flag_kl_old=true;
#if (IDEAL_MODEL==1)
			double tempS;
#if 1			
			printf("provjera spremanja pomocne stare trajektorije\n");
				for(int i=0;i<N_KL;i++){
				  tempS=computeInterpolatedCost(KL_old_temp.x[i],KL_old_temp.y[i],KL_old_temp.th[i]);
			if (KL_old_temp.S[i]!=tempS){
			  printf("provjera prijepisa pomocne stare trajektorije: cijene nisu jednake! KL_old_temp.S[i]=%.15f vs compute=%.15f, i=%d rotateongoal=%d,KL_old_temp.x[i]=%.15f, KL_old_temp.y[i]=%.15f, KL_old_temp.th[i]=%.15f\n",KL_old_temp.S[i],tempS,i,rotateongoal,KL_old_temp.x[i], KL_old_temp.y[i], KL_old_temp.th[i]);
			  }
			  }
#endif
#endif
		}else  {
		if ((flag_kl_old)  && ((ljapunov<MAXCOST))) //||(obstacle_index!=N_KL-1)))//||(min_obstacle_index>4)))
		{
		//biramo po staroj trajektoriji
			SP.setpoint_v=KL_old.v[1];
			SP.setpoint_vy=KL_old.vy[1];
			SP.setpoint_w=KL_old.w[1];
			SP.setpoint_x=KL_old.x[1];
			SP.setpoint_y=KL_old.y[1];
			SP.setpoint_th=KL_old.th[1];

			KL_old_temp=KL_old;//spremanje stare optimalne trajektorije
			ljapunov=cost_old;
//			for (int i=0; i<N_KL; i++){
//			printf("KL_old[%d]=(%f,%f,%f,%f,%f,%f)\n",i,KL_old.x[i],KL_old.y[i],KL_old.th[i],KL_old.v[i],KL_old.w[i]*RuS,KL_old.vy[i]);
//			}
			printf("DynamicWindow> setpoint from the old trajectory KL_old.(v,w,vy)=(%f,%f,%f)\n",SP.setpoint_v,SP.setpoint_w*RuS,SP.setpoint_vy);
		}else{
			if (flag_kl_old){
				T_old=0;//N_KL-4;
				printf("DW> old trajectory occupied using breaks!\n");
//				locmincnt=N_KL+1;
//        KL_old_temp=KL_breakage;
			}else{
				T_old=0;//N_KL-4;
				printf("DW> no old trajectory using breaks\n");
//				locmincnt=N_KL+1;
//        KL_old_temp=KL_breakage;
			}
#if (NO_PATH_RAMP>=1)
      best_breakage=true;
      flag_kl_old=true;
      KL_old_temp=KL_breakage;
			SP.setpoint_v=KL_breakage.v[1];
			SP.setpoint_vy=KL_breakage.vy[1];
			SP.setpoint_w=KL_breakage.w[1];
			SP.setpoint_x=KL_breakage.x[1];
			SP.setpoint_y=KL_breakage.y[1];
			SP.setpoint_th=KL_breakage.th[1];
#endif			
		}
		}
#endif

	}
#if (IDEAL_MODEL==0) 
//rhc should stop in the goal cell without this condition (ideally yes)
			double goal_tolerance=((WH->RB.x-WH->global_goal_workhorse.x)*(WH->RB.x-WH->global_goal_workhorse.x)+(WH->RB.y-WH->global_goal_workhorse.y)*(WH->RB.y-WH->global_goal_workhorse.y));
//#if RECTANGULAR
      if(naCilju)
//#else
//			if((goal_tolerance<=GOAL_POSITION_TOLERANCE*GOAL_POSITION_TOLERANCE))//&&(!naCilju))  //pola radijusa robota (radijus, *4)
//#endif
			{
//				SP.setpoint_v=0.;
				//SP.setpoint_w=0.;
//				SP.setpoint_vy=0.;
	printf("DW: near goal ramp: reference from the previous step (%f,%f,%f)\n",rbv,rbw*RuS,rbvy);
	double theta_tolerance=fabs(WH->RB.th-WH->global_goal_workhorse.th);
	    while (theta_tolerance>=M_PI)
		    theta_tolerance=fabs(theta_tolerance-2*M_PI);
	printf("DW: goal_tolerance=%f, theta_tolerance=%f deg\n",sqrt(goal_tolerance), fabs(theta_tolerance)*RuS);

#if THETAGOAL
//#if RECTANGULAR	
//			if ((fabs(theta_tolerance)<=cspace->angularResolution )&&(fabs(theta_tolerance)<=THETA_TOLERANCE))
//#else
      double theta_tolerance_ref=THETA_TOLERANCE;//(DW_MAX*STEP)/(W_DIM);

			if ((fabs(theta_tolerance)<=theta_tolerance_ref))
//#endif
#endif
			{
				SP.setpoint_w=0.;
				SP.setpoint_v=0.;
				SP.setpoint_vy=0.;
			}
				
			}
#endif

//rhc has no special cases (actually it has but it is currently implemented differently)
//if the velocities need to be modified that means that the robot has no solution so here it will break	
#if IDEAL_MODEL || 1//with ideal model this is only for checking the errors
if (flag_kl_old){ //do the ramp in the case of the goal tolerance
	printf("reference from the previous step (%f,%f,%f)\n",rbv,rbw*RuS,rbvy);
	if (fabs(rbv-SP.setpoint_v)>(DVX_MAX*STEP)+V_TOLERANCE) {//v_refdin_current
		SP.setpoint_v=rbv-(rbv-SP.setpoint_v)/fabs(rbv-SP.setpoint_v)*DVX_MAX*STEP;//v_refdin_current
	}
	if (fabs(rbvy-SP.setpoint_vy)>(DVY_MAX*STEP)+V_TOLERANCE) {//v_refdin_current
		SP.setpoint_vy=rbvy-(rbvy-SP.setpoint_vy)/fabs(rbvy-SP.setpoint_vy)*DVY_MAX*STEP;//v_refdin_current
	}
	if (fabs(rbw-SP.setpoint_w)>(DW_MAX*STEP)+W_TOLERANCE) {//w_refdin_current
		SP.setpoint_w=rbw-(rbw-SP.setpoint_w)/fabs(rbw-SP.setpoint_w)*DW_MAX*STEP;//w_refdin_current
	}
#if DIAGCONSTR //&& IDEAL_MODEL
	if ((fabs(SP.setpoint_v)+fabs(SP.setpoint_vy)>V_MAX+V_TOLERANCE)){
		//zadrzavam v brzinu, vy ostavljam na staroj
		SP.setpoint_vy=rbvy;
	}
	if ((fabs(SP.setpoint_v-rbv)+fabs(SP.setpoint_vy-rbvy)>DV_MAX*STEP+V_TOLERANCE)){
		//smanjujem ih po pola
		SP.setpoint_v=rbv-(rbv-SP.setpoint_v)/fabs(rbv-SP.setpoint_v)*0.5*DVX_MAX*STEP;
		SP.setpoint_vy=rbvy-(rbvy-SP.setpoint_vy)/fabs(rbvy-SP.setpoint_vy)*0.5*DVY_MAX*STEP;
	}
#endif
#if ACC_CONSTRAINT
//	if (WH->Tv>0.00001){
		if (((WH->delta_v_refdin>V_TOLERANCE)&&(SP.setpoint_v-rbv<0))||((WH->delta_v_refdin<-V_TOLERANCE)&&(SP.setpoint_v-rbv>0))){
			SP.setpoint_v=rbv;//neda odmah
		}
		if (((WH->delta_vy_refdin>V_TOLERANCE)&&(SP.setpoint_vy-rbvy<0))||((WH->delta_vy_refdin<-V_TOLERANCE)&&(SP.setpoint_vy-rbvy>0))){
			SP.setpoint_vy=rbvy;//neda odmah
		}
#if (I_ZA_ALFU==1)
		if ((((WH->delta_w_refdin)>W_TOLERANCE)&&(SP.setpoint_w-rbw<0))||(((WH->delta_w_refdin)<-W_TOLERANCE)&&(SP.setpoint_w-rbw>0))){ //pisalo fabs onda je uvijek pozitivno kad je rampa
			SP.setpoint_w=rbw;//neda odmah
		}
#endif
//	}
#endif
	if (WH->Tv>0.00001){ //samo za dinamicki model
#if (PO_PREDIKCIJI==0)
	SP.setpoint_v=RB.v*exp(-STEP/WH->Tv)+(1-exp(-STEP/WH->Tv))*SP.setpoint_v;
	SP.setpoint_w=RB.w*exp(-STEP/WH->Tw)+(1-exp(-STEP/WH->Tw))*SP.setpoint_w;
#else
	SP.setpoint_v=WH->v_des_current*exp(-STEP/WH->Tv)+(1-exp(-STEP/WH->Tv))*SP.setpoint_v;
	SP.setpoint_w=WH->w_des_current*exp(-STEP/WH->Tw)+(1-exp(-STEP/WH->Tw))*SP.setpoint_w;
#endif
	}
}
#endif
	printf("DW setpoint> v=%f, w=%f deg, vy=%f\n",SP.setpoint_v,SP.setpoint_w*RuS,SP.setpoint_vy);
	if (0&&(fabs(SP.setpoint_v)+fabs(SP.setpoint_vy)>V_TOLERANCE)&&(fabs(SP.setpoint_w)>W_TOLERANCE)){
	printf("hm: SP.setpoint=%f,%f,%f\n",SP.setpoint_v,SP.setpoint_vy,SP.setpoint_w);
	}
// 	vs=vc;//pamti za sljedeci ciklus
// 	ws=wc;
	if (ljapunov>=MAXCOST)
		printf("big ljapunov=%f\n",ljapunov);
	TimeLog(1);
}

void DynamicWindow::TimeLog(int state)
{
  //debug timer podaci
//	static long vremenska_razlika;
  //total time
	static struct timeval timeStart;
	static struct timeval timeNow;
	static long mySecStart, myMSecStart,mySecNow, myMSecNow;
	switch(state){
		case 0:
			if (gettimeofday(&timeStart, NULL) == 0)
			{
				mySecStart = timeStart.tv_sec;
				myMSecStart = timeStart.tv_usec / 1000;
			}
			break;
		case 1:
			if (gettimeofday(&timeNow, NULL) == 0)
			{
				mySecNow = timeNow.tv_sec;
				myMSecNow = timeNow.tv_usec / 1000;
			}
			vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
			printf("DynamicWindow::TimeLog> time diff:%ld\n", vremenska_razlika);
			break;
	}
}

void DynamicWindow::Azuriraj_stanje_robota(){
  //pozicija robota
	this->RB.x=WH->RB.x;
	this->RB.y=WH->RB.y;
	this->RB.th=WH->RB.th; //unutar intervala [0, 2*M_PI>
	//radimo predikciju za sljedeci korak buduci da ce referenca kasniti jedan korak i tada cemo se pomaknuti
  //azuriranje brzina
	this->RB.v=WH->RB.v;
	this->RB.vy=WH->RB.vy;
	this->RB.w=WH->RB.w;
	//radimo predikciju za sljedeci korak buduci da ce referenca kasniti jedan korak i tada cemo se pomaknuti
// 	double rbv=RB.v;
// 	double rbw=RB.w;
// 	double deltaxy,deltath;
// /*	if (rbv<V_TOLERANCE)
// 		rbv=0.;
// 	if (fabs(rbw)<W_TOLERANCE)
// 		rbw=0.;*/
// 	RB.v=rbv*exp(-STEP/WH->Tv)+(1-exp(-STEP/WH->Tv))*WH->v_refdin_current;//referenca iz proslog ciklusa koju sam na pocetku ciklusa zadala
// 	RB.w=rbw*exp(-STEP/WH->Tw)+(1-exp(-STEP/WH->Tw))*WH->w_refdin_current;
// // 	printf("v(k)=(%f,%f), v(k+1)=(%f,%f)\n",WH->RB.v,WH->RB.w,RB.v,RB.w);
// 	deltaxy=WH->v_refdin_current*STEP+(WH->v_refdin_current-WH->RB.v)*WH->Tv*(exp(-STEP/WH->Tv)-1);
// 	deltath=WH->w_refdin_current*STEP+(WH->w_refdin_current-WH->RB.w)*WH->Tw*(exp(-STEP/WH->Tw)-1);
// 	RB.th=deltath+WH->RB.th;
// 	if (RB.th>2*M_PI) RB.th-=2*M_PI;
// 	if (RB.th<0) RB.th+=2*M_PI;
// 	RB.x=RB.x+deltaxy*cos(RB.th);
// 	RB.y=RB.y+deltaxy*sin(RB.th);
}

//****************************************************************************************************
//u ovoj funkciji se trazi efektivna putanja robota na temelju globalnog geometrijskog puta koji je sastavljen od ravnih duzina koje zatvaraju s osi x kut koji je visekratnik kuta 45 (zbog nacina pretrazivanja kvatraticnih polja (celija))
//efektivnu putanju ce predstavljati polje  path_r_reduced koje ima N_PATH_REDUCED tocaka koje opisuju ravni segment ciji je pocetak u poziciji robota (RB.x,RB.y) a prolazi kroz tocku infleksije na globalnom geometrijskom putu, zavrsetak (produljetak) mu se racuna tako sto se udaljenost tocke infleksije od pozicije robota pomnozi s 3/2 (dokaz zasto 3/2 je naveden u clanku, a tu dolje je pokusano objasniti)
//tocka infleksije je promjena smjera tih ravnih duzina na globalnom geometrijskom putu i to konkretnije trazi se 2.tocka infleksije jer nam ona znaci "drasticnu" promjenu smjera glob. puta
//s efektivnom putanjom radimo usporedbu s DW trajektorijama, njezina duljina direktno utjece na odabir translacijske brzine robota, a smjer na odabir rotacijske brzine
//efektivna duljina je ogranicena odozgo i odozdo: maksimalna duljina je odredjena s maksimalnom mogucom translacijskom brzinom robota u sljedecem ciklusu kojom bi se gibao vrijeme Tmax (odredjeno s dometom lasera i V_MAX, TMAX=LASER_RANGE/V_MAX); minimalna duljina je odredjena minimalnom brzinom robota kod koje se robot moze sigurno zaustaviti (1/2*DV_MAX*TbMAX^2, gdje je TbMAX vrijeme kocenja pri maksimalnoj deceleraciji i maksimalnoj brzini, =V_MAX/DV_MAX, sredjeno iznosi 1/2*(V_MAX^2)/DV_MAX, uvrsteno 360),  naravno te duljine treba pomnoziti s 3/2
//3/2* je zbog oblika kriterija path_alignment kojeg koristimo (teziste one otezane sume upada na 2/3 sto znaci da je najbolje poklapanje onih trajektorija koje su dugacke 2/3 duljine efektivne putanje, zato efektivnu duljinu jos produljujemo 3/2 puta da bi dobili cijelo, a cijelo pazi na osiguravanje max brzine, bez mnozenja s 3/2 bismo dobili maksimalnu brzinu iznosa 2/3*V_MAX)
//****************************************************************************************************
int DynamicWindow::FindEffectivePath()
{
	if((path_r=PL->GetPath())==NULL){
		return 0;
	}
	double max_path=0.0;  //gornja granica efektivne duljine (prije mnozenja s 3/2), zadana je dolje
// 	double min_path=1/2.*(V_MAX)*(V_MAX)/DV_MAX+V_MAX*WH->Tv;  //donja granica efektivne duljine (prije mnozenja s 3/2), zadajemo je ovdje (666mm za Tv=0.11)
// 	double min_path=1/2.*(DV_MAX)*(V_MAX/DV_MAX+WH->Tv)*(V_MAX/DV_MAX+WH->Tv)-DV_MAX*(WH->Tv)*(WH->Tv)*(1-exp(-V_MAX/DV_MAX/WH->Tv-1));  //donja granica efektivne duljine (prije mnozenja s 3/2), zadajemo je ovdje (664mm za Tv=0.11)
	double min_path=1/2.*(V_MAX)*(V_MAX)/DV_MAX;//bilo je 1/2 medo 400/5=80 fiksan, da bude isti za sve slucajeve (600, v=600/5=120)
// 	double min_path=V_MAX/2*TMAX;//fiksan, da bude isti za sve slucajeve
#if (MAX_PATH==0)
	double vmin,limit_d,min_d;
	limit_d=(RR+SC1+(SC2-SC1)*RB.v/V_MAX); //uracunat security distance
	min_d=fabs(WH->rmin-limit_d);//najmanja udaljenost do prepreke s uracunatim scv
	vmin=DV_MAX*(sqrt(2*(min_d)/DV_MAX+2*WH->Tv*WH->Tv)-WH->Tv);
	if (vmin>V_MAX)
		vmin=V_MAX;
	max_path=(vmin)*TMAX;
#endif
// 	printf("rmin=%f, min_d=%f\n",WH->rmin,fabs(WH->rmin-(RR+SC1+(SC2-SC1)*RB.v/V_MAX)));
	int path_length_int=PL->GetPathLength(); //ukupan broj tocaka globalnog puta iz Plannera
//prvo uzimamo maksimalni broj celija koji odgovara gornjoj granici efektivne duljine: ako je put ravan to ce biti ujedno i efektivna putanja, a ako nije, onda ce se tocka infleksije naci negdje do tog maksimalnog broja celija, a nakon toga ce se udaljenost robota do tocke infleksije jos izmnoziti 3/2 puta
#if (MAX_PATH==1)
		double temp,tempr,tempry;
#if (OKO_REFERENCE==1)
		tempr=(WH->v_refdin_current);//brisem fabs
		tempry=WH->vy_refdin_current;
#else
		tempr=(RB.v);//brisem fabs
		tempry=RB.vy;
#endif
#if (PO_PREDIKCIJI==1)
		temp=(WH->v_des_current);//brisem fabs
#else
		temp=(RB.v);//brisem fabs
#endif
#if OMNIDRIVE
		tempr=sqrt(tempr*tempr+tempry*tempry);
		max_path=tempr+sqrt(DV_MAX*DV_MAX+DVY_MAX*DVY_MAX)*STEP;
		if (max_path<sqrt(V_MAX*V_MAX+VY_MAX*VY_MAX)){
			max_path=max_path*TMAX;
		}else{
			max_path=sqrt(V_MAX*V_MAX+VY_MAX*VY_MAX)*TMAX;
		}
#else
		if (tempr<0)
			tempr=0.;
		if (temp<0)
			temp=0.;

		max_path=(exp(-STEP/WH->Tv)*temp+(1-exp(-STEP/WH->Tv))*(tempr+DV_MAX*STEP));//ovo je samo brzina u koraku iza
// 			max_path=(RB.v+DV_MAX*STEP);  //max put kod maximalne moguce brzine u sljedecem koraku i vremena Tmax
// 		printf("DW ohrabrujuci max v %f (temp=%f, tempr=%f)\n",max_path,temp,tempr);
		if((max_path)<V_MAX){
// 			max_path=(RB.v+DV_MAX*STEP)*TMAX;  //max put kod maximalne moguce brzine u sljedecem koraku i vremena Tmax
			max_path=max_path*TMAX;
// 			printf("RB.v=%f, WH->RB.v=%f, max_path=%f, broj_DStar_celija=%d\n",RB.v, WH->RB.v,max_path,broj_DStar_celija);
		}else{
			max_path=V_MAX*TMAX;
		}
#endif
#endif
// 		max_path=V_MAX*TMAX;//pokusaj konstantnog max_patha, da ne oscilira referenca
		//max_path=MAX(min_path,max_path);
		min_path=std::min(min_path,max_path);//medo
		printf("min_path=%f, max_path=%f\n",min_path,max_path);
		broj_DStar_celija=(int) (ceil(max_path/(floor)(CELL_DIM)*1.4))+5;//dulje od max_patha
		broj_DStar_celija=std::min(path_length_int,broj_DStar_celija); //ako je kraci put
// 		min_path=max_path;//da stalno bude najveci odabir ref tocke
	//trazenje druge tocke infleksije
	int path_flag=0; //koliko promjena smjera ravnih segmenata detektiramo od pocetka puta do broj_DStar_celija
	indeks_infleksije=0; //indeks gdje se nalazi zadnja tocka infleksije
	double delta_x, delta_y;//razlika po x i y koordinatama izmedju dviju susjednih tocaka na putu
	double kut, kut_prvi, kut_drugi,kut_treci;//kut-pomocna varijabla za usporedbu, kut_prvi-kut prvog ravnog segmenta, kut_drugi-drugog, kut_treci-treceg
	//prva tocka infleksije je na promjeni izmedju prvog i drugog kuta, a druga nakon promjene drugog kuta
	//ako kut_drugi pripada prekratkom segmentu (samo 1 celija) i nakon njega dolazi segment kojem je kut jednak kut_prvi, takva promjena se zanemaruje i sva ta tri segmenta se smatraju jednim segmentom kojem je kut jednak kut_prvi i trazi se ispocetka prva tocka infleksije (za drugu tocku infleksije se tako nesto ne radi, predaleko je i mogla bi efektivna putanja prolaziti preblizu prepreke)
	//(ukljuci I_ZA_DRUGU)ajmo probat i za drugu tocku infleksije radit ko za prvu: ako nakon druge tocke infleksije segment s kutem kut_treci je prekratak (samo 1 celija), a nakon njega dolazi segment kom je kut jednak kut_drugi, onda se takva promjena zanemaruje i ponovo se trazi druga tocka infleksije
	int samoprvi=0;
	delta_x=path_r[1].x-path_r[0].x;
	delta_y=path_r[1].y-path_r[0].y;
	kut=atan2(delta_y,delta_x);
// 	delta_x=path_r[1].x-RB.x;
// 	delta_y=path_r[1].y-RB.y;
	//ako ima prepreka blizu robota unutar 1m onda gledaj do prve tocke infleksije (ovaj ce uvijek bit razlicit od sljedeceg zbog realnih brojeva, vjerojatnost je jako mala da nece)
//ako je blizu prepreka i robot je dalje od jedne celije od puta ili 3. polje puta nije empty, znaci da je blizu zida
// 	blizu=false;//gledam sto ce biti
// 	if (((MAX(fabs(delta_x),fabs(delta_y))>100.)&&(blizu))||(GM->nonempty>2)){//
// // 		printf("nonempty=%d\n",GM->nonempty);
// 		kut=atan2(delta_y,delta_x);
// 		samoprvi=1;
// 	}
/*	if (blizu)

		printf("blizu je za brzinu %f\n",WH->RB.v);*/
	if (kut<0)
		kut+=2*M_PI;
	kut_prvi=kut;
/*	kut_prvi=RB.th;
	kut_razlika=fabs(kut-kut_prvi);
	if (kut_razlika>2*M_PI){
		kut_razlika-=2*M_PI;

	}
	if (kut_razlika<M_PI/8)
		kut_prvi=kut;*/
	int prva_oznaka=0,druga_oznaka=0;
	double razlika_kuta;
	for(int i=1; i<broj_DStar_celija; i++) //od druge tocke puta
	{
		delta_x=path_r[i].x-path_r[i-1].x;
		delta_y=path_r[i].y-path_r[i-1].y;// kut izmedju i-te i (i-1)-ve tocke puta
		kut=atan2(delta_y,delta_x);
		if (kut<0.0)
			kut+=2*M_PI;
		if (path_flag==0){
				razlika_kuta=fabs(kut-kut_prvi);
			}else{
				razlika_kuta=fabs(kut-kut_drugi);
			}
			if (razlika_kuta>M_PI)
				razlika_kuta-=2*M_PI;
			razlika_kuta=fabs(razlika_kuta);	
		if ((path_flag==0) && (razlika_kuta*RuS>10.))  //prva promjena
		{
// 			printf("prva promjena, kut=%f, kut_prvi=%f\n",kut,kut_prvi);
			path_flag++;
			indeks_infleksije=i-1;//infleksija je tocka ispred ove i-te tocke kod koje je zabiljezena promjena smjera
			kut_drugi=kut;//zabiljezi novi kut
			continue;
		}
		if ((path_flag==1) && (razlika_kuta*RuS>10.)) //druga promjena
		{//sad tu gledamo da li je taj kut mozda jednak prvom kutu
// 			printf("druga promjena, kut_treci=%f, kut_drugi=%f\n",kut,kut_drugi);
			path_flag++;
			indeks_infleksije=i-1;//infleksija je tocka ispred ove i-te tocke kod koje je zabiljezena promjena smjera
			kut_treci=kut;
			break; //nasli smo drugu tocku infleksije
		}
	}
	//ako smo nasli drugu tocku infleksije gledamo prvo da li je udaljenost pozicije robota od tocke infleksije veca od donje granice efektivne duljine (bez mnozenja s 3/2), onda da li je manja od gornje granice
	double path_length_a1,path_length_a2,kuta1,kutb1,kutc,kutb,komadx,komadx2,da12;//
	if (path_flag==2)
	{
		printf("path_flag==2, indeks_infleksije=%d\n",indeks_infleksije);
		delta_x=path_r[indeks_infleksije].x-RB.x;
		delta_y=path_r[indeks_infleksije].y-RB.y;
// 		delta_x=path_r[indeks_infleksije].x-path_r[0].x;//oprez sto radim, zbog razlike pozicije robota od pozicije pocetka puta
// 		delta_y=path_r[indeks_infleksije].y-path_r[0].y;
		path_length_a1=(delta_x*delta_x+delta_y*delta_y);
		//donja granica
// 		printf("path_length_a1=%f\n",sqrt(path_length_a1));
	if ((path_length_a1<min_path*min_path)) {//patak nije glupan
	printf("manji od min_path-a\n");
	for(int i=indeks_infleksije+1; i<broj_DStar_celija; i++) {//od druge tocke jer bi delta_x i y bili 0
		delta_x=(path_r[i].x-RB.x);
		delta_y=(path_r[i].y-RB.y);
		path_length_a2=delta_x*delta_x+delta_y*delta_y;//i tu izbacujem sqrt
		if ((path_length_a2>=min_path*min_path)||(i==broj_DStar_celija-1)){//dodajem na kvadrat
			if ((path_length_a2<=max_path*max_path) && (i==broj_DStar_celija-1)){
				//specijalan slucaj na cilju
				tocka_infleksije.x=WH->global_goal_workhorse.x;
				tocka_infleksije.y=WH->global_goal_workhorse.y;
				delta_x=tocka_infleksije.x-RB.x;
				delta_y=tocka_infleksije.y-RB.y;
				path_length=min_path*MNOZI_SINKO_MNOZI;
				path_orientation=atan2((delta_y),(delta_x));
			}else{						//trazenje tocke na segmentu path_r[i-1],path_r[i] tocno udaljene min_path od robota
				//racunanje kuta preko skalarnog produkta kut(path_r[i-1],robot,path_r[i])
				path_length_a1=sqrt(path_length_a1);//od i-1
				path_length_a2=sqrt(path_length_a2);
				// 							kutb=acos(((path_r[i-1].x-RB.x)*(path_r[i].x-RB.x)+(path_r[i-1].y-RB.y)*(path_r[i].y-RB.y))/(path_length_a1*path_length_a2));//acos return [0,M_PI]
	da12=(path_r[i].x-path_r[i-1].x)*(path_r[i].x-path_r[i-1].x)+(path_r[i].y-path_r[i-1].y)*(path_r[i].y-path_r[i-1].y);//kvadratna udaljenost izmedju dvije tocke
	da12=sqrt(da12);
	kuta1=acos(((RB.x-path_r[i].x)*(path_r[i-1].x-path_r[i].x)+(RB.y-path_r[i].y)*(path_r[i-1].y-path_r[i].y))/(path_length_a2*da12));
	kutc=asin((path_length_a2/min_path)*sin(kuta1));//paznja kod asin, mogu bit dva rjesenja kutc i M_PI-kutc
	//zato gledamo oba rjesenja i odabiremo ono ciji je komadx manji
	kutb1=M_PI-kuta1-kutc;
	komadx2=(min_path)*sin(kutb1)/sin(kuta1);
	kutb1=-kuta1+kutc;
	komadx=(min_path)*sin(kutb1)/sin(kuta1);
	if (komadx>komadx2)
		komadx=komadx2;
	// 						printf("komadx=%f\n",komadx);
	tocka_infleksije.x=komadx/da12*path_r[i-1].x+(da12-komadx)/da12*path_r[i].x;
	tocka_infleksije.y=komadx/da12*path_r[i-1].y+(da12-komadx)/da12*path_r[i].y;
	// 						printf("path_r[%d]=(%f,%f), tocka_infleksije=(%f,%f), path_r[%d]=(%f,%f)\n",i-1,path_r[i-1].x,path_r[i-1].y,tocka_infleksije.x,tocka_infleksije.y,i,path_r[i].x,path_r[i].y);
	delta_x=(tocka_infleksije.x-RB.x);//opet moram racunati pravi delta
	delta_y=(tocka_infleksije.y-RB.y);
	// 						path_length=sqrt(delta_x*delta_x+delta_y*delta_y)*MNOZI_SINKO_MNOZI;
	path_length=(min_path)*MNOZI_SINKO_MNOZI;//globalna varijabla, duljina efektivne putanje (uracunat 3/2*) i dodatni korijen
	path_orientation=atan2(delta_y,delta_x); //globalna varijabla
	// 						printf("manji od min_patha (%f) bio, nadjen veci: path_length=%f\n",min_path,path_length);
			}
			break;
		}
		path_length_a1=path_length_a2;//da odgovara path_r[i-1] u sljedecoj iteraciji
	}
	if((delta_x==0.0)&&(delta_y==0.0)){
		printf("Dynamic Window::FindEffectivePath()> Ne mozemo pretvoriti atan2! Tocke patha su na istoj poziciji!");
		exit(1);
	}
}else if (path_length_a1>max_path*max_path){ //gornja granica, opet po sampliranom pathu trazimo koja tocka ce biti udaljena za <=max_path
	printf("veci od max_path-a\n");
	for(int i=indeks_infleksije-1; i>=0; i--) {//od infleksije prema robotu, brze cemo doc
		delta_x=(path_r[i].x-RB.x);
		delta_y=(path_r[i].y-RB.y);
		path_length_a2=delta_x*delta_x+delta_y*delta_y;
		// 					printf("path_length_a2=%f\n",sqrt(path_length_a2));
		if ((path_length_a2<=max_path*max_path) || (i==0)){//dodajem na kvadrat
			//trazenje tocke na segmentu path_r[i-1],path_r[i] tocno udaljene min_path od robota
			//racunanje kuta preko skalarnog produkta kut(path_r[i-1],robot,path_r[i])
			path_length_a1=sqrt(path_length_a1);
			path_length_a2=sqrt(path_length_a2);
			// 						kutb=acos(((path_r[i+1].x-RB.x)*(path_r[i].x-RB.x)+(path_r[i+1].y-RB.y)*(path_r[i].y-RB.y))/(path_length_a1*path_length_a2));//acos return [0,M_PI]
		da12=(path_r[i].x-path_r[i+1].x)*(path_r[i].x-path_r[i+1].x)+(path_r[i].y-path_r[i+1].y)*(path_r[i].y-path_r[i+1].y);//kvadratna udaljenost izmedju dvije tocke
		da12=sqrt(da12);
		kuta1=acos(((RB.x-path_r[i+1].x)*(path_r[i].x-path_r[i+1].x)+(RB.y-path_r[i+1].y)*(path_r[i].y-path_r[i+1].y))/(path_length_a1*da12));
		kutc=asin((path_length_a1/max_path)*sin(kuta1));//paznja kod asin, mogu bit dva rjesenja kutc i M_PI-kutc
		//zato gledamo oba rjesenja i odabiremo ono ciji je komadx manji
		kutb1=M_PI-kuta1-kutc;
		komadx2=(max_path)*sin(kutb1)/sin(kuta1);
		kutb1=-kuta1+kutc;
		komadx=(max_path)*sin(kutb1)/sin(kuta1);
		if (komadx>komadx2)
			komadx=komadx2;
		tocka_infleksije.x=komadx/da12*path_r[i].x+(da12-komadx)/da12*path_r[i+1].x;
		tocka_infleksije.y=komadx/da12*path_r[i].y+(da12-komadx)/da12*path_r[i+1].y;
		if (i==0){
			tocka_infleksije.x=path_r[i+1].x;
			tocka_infleksije.y=path_r[i+1].y;
		}
		delta_x=(tocka_infleksije.x-RB.x);//opet moram racunati pravi delta
		delta_y=(tocka_infleksije.y-RB.y);
		// 					path_length=sqrt(delta_x*delta_x+delta_y*delta_y)*MNOZI_SINKO_MNOZI;
		path_length=(max_path)*MNOZI_SINKO_MNOZI;//globalna varijabla, duljina efektivne putanje (uracunat 3/2*) i dodatni korijen
		path_orientation=atan2(delta_y,delta_x); //globalna varijabla
		// 						printf("veci od max_patha (%f) bio, sad je: path_length=%f\n",max_path,path_length);
		break;
		}
		path_length_a1=path_length_a2;//da odgovara path_r[i-1] u sljedecoj iteraciji
	}
	if((delta_x==0.0)&&(delta_y==0.0)){
		printf("Dynamic Window::FindEffectivePath()> Ne mozemo pretvoriti atan2! Tocke patha su na istoj poziciji!");
		exit(1);
	}
}else{
	//default: unutar donje i gornje granice je (da bude unutar gornje granice nije ograniceno na pocetku, broj_DStar_celija je uzet prevelik)
	tocka_infleksije.x=path_r[indeks_infleksije].x;
	tocka_infleksije.y=path_r[indeks_infleksije].y;
	path_length=sqrt(path_length_a1)*MNOZI_SINKO_MNOZI;//globalna varijabla, duljina efektivne putanje (uracunat 3/2*)
	path_orientation=atan2(delta_y,delta_x); //globalna varijabla
	// 			printf("unutar granica: path_length=%f ili je preblizu prepreci\n",path_length);
}
	}else{//ako nismo nasli tocku infleksije znaci da trebamo uzeti maksimalnu duljinu puta koju dozvoljava gornja granica, ali paziti na zadnju tocku puta (cilj)
	printf("path_flag!=2\n");
	delta_x=path_r[broj_DStar_celija-1].x-RB.x;
	delta_y=path_r[broj_DStar_celija-1].y-RB.y;
	path_length_a1=(delta_x*delta_x+delta_y*delta_y);
	if (path_length_a1>max_path*max_path){
		printf("veci od max_path-a\n");
		for(int i=broj_DStar_celija-2; i>=0; i--) {//od predzadnje tocke, brze cemo doc
			delta_x=(path_r[i].x-RB.x);
			delta_y=(path_r[i].y-RB.y);
			path_length_a2=delta_x*delta_x+delta_y*delta_y;
			if ((path_length_a2<=max_path*max_path) || (i==0)){//dodajem na kvadrat
				//trazenje tocke na segmentu path_r[i-1],path_r[i] tocno udaljene min_path od robota
				//racunanje kuta preko skalarnog produkta kut(path_r[i-1],robot,path_r[i])
				path_length_a1=sqrt(path_length_a1);
				path_length_a2=sqrt(path_length_a2);
				// 						kutb=acos(((path_r[i+1].x-RB.x)*(path_r[i].x-RB.x)+(path_r[i+1].y-RB.y)*(path_r[i].y-RB.y))/(path_length_a1*path_length_a2));//acos return [0,M_PI]
		da12=(path_r[i].x-path_r[i+1].x)*(path_r[i].x-path_r[i+1].x)+(path_r[i].y-path_r[i+1].y)*(path_r[i].y-path_r[i+1].y);//kvadratna udaljenost izmedju dvije tocke
		da12=sqrt(da12);
		kuta1=acos(((RB.x-path_r[i+1].x)*(path_r[i].x-path_r[i+1].x)+(RB.y-path_r[i+1].y)*(path_r[i].y-path_r[i+1].y))/(path_length_a1*da12));
		kutc=asin((path_length_a1/max_path)*sin(kuta1));//paznja kod asin, mogu bit dva rjesenja kutc i M_PI-kutc
		//zato gledamo oba rjesenja i odabiremo ono ciji je komadx manji
		kutb1=M_PI-kuta1-kutc;
		komadx2=(max_path)*sin(kutb1)/sin(kuta1);
		kutb1=-kuta1+kutc;
		komadx=(max_path)*sin(kutb1)/sin(kuta1);
		if (komadx>komadx2)
			komadx=komadx2;
		tocka_infleksije.x=komadx/da12*path_r[i].x+(da12-komadx)/da12*path_r[i+1].x;
		tocka_infleksije.y=komadx/da12*path_r[i].y+(da12-komadx)/da12*path_r[i+1].y;
		if (i==0){
			tocka_infleksije.x=path_r[i+1].x;
			tocka_infleksije.y=path_r[i+1].y;
		}
		delta_x=(tocka_infleksije.x-RB.x);//opet moram racunati pravi delta
		delta_y=(tocka_infleksije.y-RB.y);
		// 				path_length=sqrt(delta_x*delta_x+delta_y*delta_y)*MNOZI_SINKO_MNOZI;
		path_length=(max_path)*MNOZI_SINKO_MNOZI;//globalna varijabla, duljina efektivne putanje (uracunat 3/2*) i dodatni korijen
		path_orientation=atan2(delta_y,delta_x); //globalna varijabla
		// 						printf("ko max_path (%f): path_length=%f\n",max_path,path_length);
		break;
			}
			path_length_a1=path_length_a2;//da odgovara path_r[i-1] u sljedecoj iteraciji
		}//od for
	}else if (path_length_a1<min_path*min_path)
	{
		//specijalan slucaj na cilju
		printf("manji od min_path-a\n");
		if (((WH->global_goal_workhorse.x-path_r[broj_DStar_celija-1].x)<100.) && ((WH->global_goal_workhorse.y-path_r[broj_DStar_celija-1].y)<100.)){
			tocka_infleksije.x=WH->global_goal_workhorse.x;
			tocka_infleksije.y=WH->global_goal_workhorse.y;
		}else {
			tocka_infleksije.x=path_r[broj_DStar_celija-1].x;
			tocka_infleksije.y=path_r[broj_DStar_celija-1].y;
		}
		delta_x=tocka_infleksije.x-RB.x;
		delta_y=tocka_infleksije.y-RB.y;
		path_length=min_path*MNOZI_SINKO_MNOZI;
		path_orientation=atan2((delta_y),(delta_x));
	}else{
		//default: unutar donje i gornje granice je, ali radi se o cilju
		tocka_infleksije.x=path_r[broj_DStar_celija-1].x;
		tocka_infleksije.y=path_r[broj_DStar_celija-1].y;
		path_length=sqrt(path_length_a1)*MNOZI_SINKO_MNOZI;
		path_orientation=atan2(delta_y,delta_x); //globalna varijabla
		printf("unutar granica: path_length=%f\n",path_length);
	}
	}
	//ovdje se racuna zapravo straight-line segment sa kojim se usporedjuje svaka DW trajektorija
	double reduced_path_increment=path_length/(double)(N_PATH_REDUCED-1),kut_razlika;//falila mi je zadnja tocka, dijelis na N dijelova i onda trebas N+1 tocku
	for(int i=0; i<N_PATH_REDUCED;i++){
		path_r_reduced[i].x=RB.x+i*reduced_path_increment*cos(path_orientation);
		path_r_reduced[i].y=RB.y+i*reduced_path_increment*sin(path_orientation);
	}
	effective_path_length=N_PATH_REDUCED;
	//tu di smo izracunali orijentaciju efektivnog puta, treba provjeriti da li je orijentacija robota razlicita za vise od M_PI/2 tak da se prvo robot okrene, kao na startu
	kut=path_orientation;
		//ako se radi o okretanju na cilju
	if (naCilju)
		kut=WH->global_goal_workhorse.th;

	if (kut<0)
		kut+=2*M_PI;//svodjenje na [0,2*M_PI]
	kut_razlika=fabs(kut-RB.th);
	if (kut_razlika>M_PI)
		kut_razlika-=2*M_PI;
	if (kut_razlika<-1*M_PI)
		kut_razlika+=2*M_PI;
	kut_razlika=fabs(kut_razlika);
//	if ((kut_razlika>2*M_PI/4)||((kut_razlika>M_PI/4)&&(path_length_int<7))){//pisalo 3*M_PI/4
//		naStartu=true;//postavljanje zastavice, sad se radi kao na startu ako je razlika orijentacije takva
//	}
	if ((kut_razlika>M_PI/6)&&(temp<50.)){//V_TOLERANCE)){//(min_path<330.)){ //min_path je manji od 200 kad je brzina robota manja od 33 (200/TMAX), TMAX==6
		naStartu=true; //u frkafuljicama
	}
	return 1;
}

void DynamicWindow::Ubaci_laser(){
	int i;
// 	int index=0;
// 	double x_temp,y_temp;
// 	blizu=false;
	double limit_distance_path;
// 	limit_distance_path=RR+6*SC1+(5*SC2-6*SC1)*RB.v/V_MAX;//za brzinu 0 blizu je ako je najbliza prepreka unutar 500mm, a za max brzinu unutar 1010mm
// 	limit_distance_path=RR+SC1+(4.27*SC2-SC1)*RB.v/V_MAX;//za brzinu 200 blizu je ako je najbliza prepreka unutar 500mm, a za max brzinu unutar 900mm
// 	limit_distance_path=RR+(-0.375)*SC1+(5*SC2+0.375*SC1)*RB.v/V_MAX;//za brzinu 200 blizu je ako je najbliza prepreka unutar 500mm, a za max brzinu unutar 1010mm
	limit_distance_path=RR+4*SC1+(1.5*SC2-4*SC1)*RB.v/V_MAX;//min brzina 200mm, max brzina 300mm
/*	for(i=0;i<WH->LB.laser_pointer_new;i++){
		x_temp=(WH->LB.LB_pose[i]).x;
		y_temp=(WH->LB.LB_pose[i]).y;
		int pokretna=0;  //ne upisuju se pokretne u polje
		//izbacuju se pokretne prepreke iz laserskih ocitanja, one se tretiraju zasebno u funkciji prohodnost	
		for (int k=0;k<GM->numnewmovings;k++){
			if((x_temp==GM->moving[k].x)&&(y_temp==GM->moving[k].y)){
                 //printf("tu je\n");
				pokretna=1;
				break;
			}
		}
		if (pokretna==0){
			this->OPF[index].x=x_temp;
			this->OPF[index].y=y_temp;
			index++;
		}
	}*/
	for (i=0;i<GM->numnewstatic;i++){//tu se samo staticki hitovi zapisuju
		this->OPF[i]=GM->statickiHitovi[i];
/*		if (!blizu){
// 			printf("fabs(GM->statickiHitovi[i].x-WH->RB.x)=%f,fabs(GM->statickiHitovi[i].y-WH->RB.y)=%f,MAX=%f,limit_distance_path=%f\n",fabs(GM->statickiHitovi[i].x-WH->RB.x),fabs(GM->statickiHitovi[i].y-WH->RB.y),MAX(fabs(GM->statickiHitovi[i].x-WH->RB.x),fabs(GM->statickiHitovi[i].y-WH->RB.y)),limit_distance_path);
			if (MAX(fabs(GM->statickiHitovi[i].x-WH->RB.x),fabs(GM->statickiHitovi[i].y-WH->RB.y))<limit_distance_path) {
// 				printf("zakaj? hit (%f,%f), robot (%f,%f)\n",GM->statickiHitovi[i].x,GM->statickiHitovi[i].y,RB.x,RB.y);
				blizu=true;
			}
		}*/
	}
// 	this->laser_pointer_end=(index);
	this->laser_pointer_end=(i);
// 	printf("DW:laser_pointer_end=%d\n",laser_pointer_end);
	return;
}

/******************************************************
 * Funkcija koja odredjuje GRANICE DINAMICKOG PROZORA *
 *    na temelju trenutnih ocitanih brzina robota     *
 ******************************************************/
void DynamicWindow::Odredi_dinamicki_prozor(){   //tu se uzima trajanje ciklusa STEP (0.1 s)

	int i,j,k;
	double delta_v_minus,delta_v_plus,delta_w_plus,delta_w_minus,delta_vy_minus,delta_vy_plus;
	double rbv,rbw,rbvy;
#if (OKO_REFERENCE==1)
	delta_v_minus=WH->v_refdin_current-VX_MIN;//
	delta_vy_minus=WH->vy_refdin_current-VY_MIN;
	rbv=WH->v_refdin_current;
	rbw=WH->w_refdin_current;
	rbvy=WH->vy_refdin_current;
	delta_v_plus=(VX_MAX-rbv); //ne smije biti fabs jer ode brzina u nebo
//	printf("delta_v_minus=%f v_refdin_current=%f, V_MIN=%f\n",delta_v_minus,WH->v_refdin_current,V_MIN);
	delta_w_plus=(W_MAX-fabs(rbw));//w_refdin_current)); //tu je pretpostavka da je W_MAX=-W_MIN, korektno je izvesti kao za v i vy
	delta_vy_plus=(VY_MAX-rbvy);
#else
	delta_v_minus=RB.v-VX_MIN;//WH->v_refdin_current;//
	delta_vy_minus=RB.vy-VY_MIN;
	rbv=RB.v;//WH->v_refdin_current;
	rbw=RB.w;//w_refdin_current;
	rbvy=RB.vy;
	delta_v_plus=(VX_MAX-rbv);//v_refdin_current); //ne smije biti fabs jer ode brzina u nebo
	delta_vy_plus=(VY_MAX-rbvy);
	delta_w_plus=(W_MAX-fabs(rbw));//w_refdin_current)); //tu je pretpostavka da je W_MAX=-W_MIN
#endif
	if (fabs(rbv)<V_TOLERANCE)
		rbv=0.;
	if (fabs(rbvy)<V_TOLERANCE)
		rbvy=0.;
	if (fabs(rbw)<W_TOLERANCE)
		rbw=0.;
	if (delta_v_minus>DVX_MAX*STEP) {
		delta_v_minus=DVX_MAX*STEP; //maksimalni delta koji smije bit
	}
	if (delta_v_plus>DVX_MAX*STEP) {
		delta_v_plus=DVX_MAX*STEP;
	}
	if (delta_vy_minus>DVY_MAX*STEP) {
		delta_vy_minus=DVY_MAX*STEP; //maksimalni delta koji smije bit
	}
	if (delta_vy_plus>DVY_MAX*STEP) {
		delta_vy_plus=DVY_MAX*STEP;
	}
	if (delta_w_plus>DW_MAX*STEP) {
		delta_w_plus=DW_MAX*STEP; //ova granica se primijenjuje na onu stranu na kojoj je trenutna brzina
	}
	if (rbw<0.0) {//w_refdin_current<0.0) {  //ako se radi o negativnoj granici treba to promijeniti
		delta_w_minus=delta_w_plus;
		delta_w_plus=DW_MAX*STEP;
	}
	if (rbw>=0.0) {//w_refdin_current>=0.0) {    //ako se radi o pozitivnoj granici, vec smo ju odredili pa postavljamo negativnu
		delta_w_minus=DW_MAX*STEP;
	}

	//ne dozvoljavanje mijenjanja akceleracije svaki ciklus zbog zracnosti
	//samo za dinamicki model
#if ACC_CONSTRAINT
//	if (WH->Tv>0.00001){
		if (rbv>V_TOLERANCE){
			delta_v_minus=0.;
		}else if (rbv<-V_TOLERANCE){
			delta_v_plus=0.;
		}//inace moze i + i - akceleracija
		if (rbvy>V_TOLERANCE){
			delta_vy_minus=0.;
		}else if (rbvy<-V_TOLERANCE){
			delta_vy_plus=0.;
		}
#if (I_ZA_ALFU==1)
		if (rbw>W_TOLERANCE){
			delta_w_minus=0.;
		}else if (rbw<-W_TOLERANCE){
			delta_w_plus=0.;
		}
#endif
//	}
#endif
#if (IDEAL_MODEL==0)
  if (naCilju){//ovo ce sad radit cisto rotiranje na cilju da ne plese okolo zbog lose lokalizacije
    delta_v_plus=0;
    delta_v_minus=0;
    delta_vy_minus=0;
    delta_vy_plus=0;
    rbv=0;
    rbvy=0;
  }
#endif

#if ROT_EXCL_TRANS_CONSTR
    if (fabs(rbv)+fabs(rbvy)>V_TOLERANCE){
			delta_w_minus=0; delta_w_plus=0;
		}
		if (fabs(rbw)>W_TOLERANCE){
		  delta_v_minus=0; delta_v_plus=0; delta_vy_minus=0; delta_vy_plus=0;
		}
#endif
	//delta_v_plus moze biti < 0 ako brzina robota ode iznad gornje granice
	//a i delta_v_minus moze biti < 0 ako brzina robota ode ispod donje granice
	if ((delta_v_minus>=V_TOLERANCE)&&(delta_v_plus>=V_TOLERANCE)){
	TB.v[V_DIM/2]=rbv; //na sredinu prozora stavljam trenutnu brzinu
	for(i=0;i<V_DIM/2;i++){
	//ovo zakomentiraj jer bi isto trebalo raditi i kod VMAX
//		if (delta_v_minus<2*V_TOLERANCE){//nema smisla dijeliti donju polovicu prostora brzina ako je mala brzina, dobijemo manje od V_TOLERANCE, ondosno 0, stavila sam 2
//			TB.v[0]=V_MIN;
//			TB.v[1]=rbv;
//			for(i=2;i<V_DIM+1;i++){
//				TB.v[i]=rbv+(double)(i-1)/(V_DIM-1)*delta_v_plus;
//			}
//			break;
//		}else
		{
		TB.v[i]=rbv-(double)(V_DIM/2-i)/(V_DIM/2)*delta_v_minus;
		TB.v[V_DIM-i]=rbv+(double)(V_DIM/2-i)/(V_DIM/2)*delta_v_plus;
		}
	}
	}
	if ((delta_v_minus<V_TOLERANCE)&&(delta_v_plus<V_TOLERANCE)){//ovo je slucaj samo u dinamickom modelu kad je zabranjena akceleracija suprotnog predznaka a trenutna brzina se nalazi na rubu kinematickih ogranicenja
		for(i=0;i<V_DIM+1;i++){
			TB.v[i]=rbv;
			if (rbv>=VX_MAX-V_TOLERANCE)
				TB.v[i]=VX_MAX;
			if (rbv<=VX_MIN+V_TOLERANCE)
				TB.v[i]=VX_MIN;
		}
		
	}else {//inace je jedan ili drugi (> ili >)
		if (delta_v_minus<V_TOLERANCE) {   //specijalan slucaj da je trenutna brzina 0 ili se ne smije primijeniti negativna akc.
			TB.v[0]=rbv; //onda stavljam na pocetak prozora trenutnu brzinu
			if (rbv<=VX_MIN+V_TOLERANCE)
				TB.v[0]=VX_MIN; //onda stavljam na pocetak prozora trenutnu brzinu
			for(i=1;i<V_DIM+1;i++){
				TB.v[i]=rbv+(double)i/V_DIM*delta_v_plus;
			}
		}
		if (delta_v_plus<V_TOLERANCE) {   //specijalan slucaj da je trenutna brzina maksimalna ili se ne smije primijeniti poz akc.
			TB.v[V_DIM]=rbv; //onda stavljam na kraj prozora trenutnu brzinu
			if (rbv>=VX_MAX-V_TOLERANCE)
				TB.v[V_DIM]=VX_MAX; //onda stavljam na kraj prozora trenutnu brzinu
			for(i=0;i<V_DIM;i++){
				TB.v[i]=rbv-(double)(V_DIM-i)/(V_DIM)*delta_v_minus;
			}
		}
	}
//sve isto za vy
	if ((delta_vy_minus>=V_TOLERANCE)&&(delta_vy_plus>=V_TOLERANCE)){
	TB.vy[VY_DIM/2]=rbvy; //na sredinu prozora stavljam trenutnu brzinu
	for(i=0;i<VY_DIM/2;i++){
		{
		TB.vy[i]=rbvy-(double)(VY_DIM/2-i)/(VY_DIM/2)*delta_vy_minus;
		TB.vy[VY_DIM-i]=rbvy+(double)(VY_DIM/2-i)/(VY_DIM/2)*delta_vy_plus;
		}
	}
	}
	if ((delta_vy_minus<V_TOLERANCE)&&(delta_vy_plus<V_TOLERANCE)){//ovo je slucaj samo u dinamickom modelu kad je zabranjena akceleracija suprotnog predznaka a trenutna brzina se nalazi na rubu kinematickih ogranicenja ili naCilju
		for(i=0;i<VY_DIM+1;i++){
			TB.vy[i]=rbvy;
			if (rbvy>=VY_MAX-V_TOLERANCE)
				TB.vy[i]=VY_MAX;
			if (rbvy<=VY_MIN+V_TOLERANCE)
				TB.vy[i]=VY_MIN;
		}
		
	}else {//inace je jedan ili drugi (> ili >)
		if (delta_vy_minus<V_TOLERANCE) {   //specijalan slucaj da je trenutna brzina 0 ili se ne smije primijeniti negativna akc.
			TB.vy[0]=rbvy; //onda stavljam na pocetak prozora trenutnu brzinu
			if (rbvy<=VY_MIN+V_TOLERANCE)
				TB.vy[0]=VY_MIN;
			for(i=1;i<VY_DIM+1;i++){
				TB.vy[i]=rbvy+(double)i/VY_DIM*delta_vy_plus;
			}
		}
		if (delta_vy_plus<V_TOLERANCE) {   //specijalan slucaj da je trenutna brzina maksimalna
			TB.vy[VY_DIM]=rbvy; //onda stavljam na kraj prozora trenutnu brzinu
			if (rbvy>=VY_MAX-V_TOLERANCE)
				TB.vy[VY_DIM]=VY_MAX;
			for(i=0;i<VY_DIM;i++){
				TB.vy[i]=rbvy-(double)(VY_DIM-i)/(VY_DIM)*delta_vy_minus;
			}
		}
	}
#if UGURAVANJE
	for(i=1;i<V_DIM+1;i++){
		if ((TB.v[i-1]*TB.v[i]<0)){
			if (fabs(TB.v[i-1])>fabs(TB.v[i])) {
				TB.v[i]=0.0;
			}else{
				TB.v[i-1]=0.0;
			}
		}
	}
	for(i=1;i<VY_DIM+1;i++){
		if ((TB.vy[i-1]*TB.vy[i]<0)){
			if (fabs(TB.vy[i-1])>fabs(TB.vy[i])) {
				TB.vy[i]=0.0;
			}else{
				TB.vy[i-1]=0.0;
			}
		}
	}
#endif
	
	if ((delta_w_minus>=W_TOLERANCE)&&(delta_w_plus>=W_TOLERANCE)) {
	TB.w[W_DIM/2]=rbw; //na sredinu prozora stavljam trenutnu brzinu
	for(i=0;i<W_DIM/2;i++){
		TB.w[i]=rbw-(double)(W_DIM/2-i)/(W_DIM/2)*delta_w_minus;
		TB.w[W_DIM-i]=rbw+(double)(W_DIM/2-i)/(W_DIM/2)*delta_w_plus;
#if (UGURAVANJE)
			if (i>0) {  //uguravanje w=0 brzine u prostor, bitno za slucaj ravne putanje
// 			printf("TB.w[%d-1]*TB.w[%d]=%f*%f\n",i,i,TB.w[i-1],TB.w[i]);
				if ((TB.w[i-1]*TB.w[i]<0))//&&(fabs(TB.w[i])>W_TOLERANCE)&&(fabs(TB.w[i-1])>W_TOLERANCE))
				{
					if (fabs(TB.w[i-1])>fabs(TB.w[i])) {
						TB.w[i]=0.0;
// 					printf("ugurao na %d\n",i);
					}else{
						TB.w[i-1]=0.0;
// 					printf("ugurao na %d-1\n",i);
					}
				}
				if (i==W_DIM/2-1){
					if ((TB.w[i+1]*TB.w[i]<0))
					{
						TB.w[i]=0.0;
// 						printf("ugurao na %d\n",i);
					}
					if (((TB.w[i+1]*TB.w[i+2]<0)))
					{
						TB.w[i+2]=0.0;
// 						printf("ugurao na %d+2\n",i);
					}
				}
// 			printf("TB.w[W_DIM-%d+1]*TB.w[W_DIM-%d]=%f*%f\n",i,i,TB.w[W_DIM-i+1],TB.w[W_DIM-i]);
				if ((TB.w[W_DIM-i+1]*TB.w[W_DIM-i]<0))//&&(fabs(TB.w[i])>W_TOLERANCE)&&(fabs(TB.w[i-1])>W_TOLERANCE))
				{
					if ((fabs(TB.w[W_DIM-i+1])>fabs(TB.w[W_DIM-i]))) {
						TB.w[W_DIM-i]=0.0;
// 					printf("ugurao na W_DIM-%d\n",i);
					}else{
						TB.w[W_DIM-i+1]=0.0;
// 					printf("ugurao na W_DIM-%d+1\n",i);
					}
				}
			}
			//specijalni slucaj je W_DIM==2, samo jedan prolaz kroz petlju i==0
/*			if (W_DIM==2 && TB.w[0]*TB.w[2]<0){
				TB.w[1]=0.;
			}*/
#endif
	}
	}
	if ((delta_w_minus<W_TOLERANCE)&&(delta_w_plus<W_TOLERANCE)){//ovo je slucaj samo u dinamickom modelu kad je zabranjena akceleracija suprotnog predznaka a trenutna brzina se nalazi na rubu kinematickih ogranicenja
		for(i=0;i<W_DIM+1;i++){
			TB.w[i]=rbw;
			if (rbw>W_MAX)
				TB.w[i]=W_MAX;
			if (rbw<W_MIN)
				TB.w[i]=W_MIN;
		}
		
	}else {//inace je jedan ili drugi
		if (delta_w_minus<W_TOLERANCE) {   //specijalan slucaj da je trenutna brzina minimalna dozvoljena
			TB.w[0]=rbw; //onda stavljam na pocetak prozora trenutnu brzinu
			if (rbw<W_MIN)
				TB.w[0]=W_MIN; //onda stavljam na pocetak prozora trenutnu brzinu
			for(i=1;i<W_DIM+1;i++){
				TB.w[i]=rbw+(double)(i)/(W_DIM)*delta_w_plus;
			 //uguravanje w=0 brzine u prostor, bitno za slucaj ravne putanje
#if (UGURAVANJE)
				if ((TB.w[i-1]*TB.w[i]<0))//&&(fabs(TB.w[i])>W_TOLERANCE)&&(fabs(TB.w[i-1])>W_TOLERANCE))
				{
					if (fabs(TB.w[i-1])>fabs(TB.w[i])||(i==1)) {
						TB.w[i]=0.0;
					}else{
						TB.w[i-1]=0.0;
					}
				}
#endif
			}
		}
		if (delta_w_plus<W_TOLERANCE) {   //specijalan slucaj da je trenutna brzina maksimalna
			TB.w[W_DIM]=rbw; //onda stavljam na kraj prozora trenutnu brzinu
			if (rbw>W_MAX)
				TB.w[W_DIM]=W_MAX; //onda stavljam na kraj prozora trenutnu brzinu
			for(i=0;i<W_DIM;i++){//tu treba doci TB.w[W_DIM] umjesto rbw
				TB.w[i]=rbw-(double)(W_DIM-i)/(W_DIM)*delta_w_minus;
#if (UGURAVANJE)
				if (i>0) {  //uguravanje w=0 brzine u prostor, bitno za slucaj ravne putanje
					if ((TB.w[i-1]*TB.w[i]<0))//&&(fabs(TB.w[i])>W_TOLERANCE)&&(fabs(TB.w[i-1])>W_TOLERANCE))
					{
						if ((fabs(TB.w[i-1])>fabs(TB.w[i]))) {
							TB.w[i]=0.0;
						}else{
							TB.w[i-1]=0.0;
						}
					}
				}
#endif
			}
		}
	}


#if OMNIDRIVE
//	printf("delta_vy_plus=%f, delta_vy_minus=%f, delta_v_plus=%f, delta_v_minus=%f, DW->TB.vy=[",delta_vy_plus,delta_vy_minus,delta_v_plus,delta_v_minus);
	for (k=0;k<VY_DIM+1;k++){
//		printf(" %f",TB.vy[k]);
#endif  
	//zabrana nedozvoljenih brzina
	for(i=0;i<V_DIM+1;i++){
		for(j=0;j<W_DIM+1;j++){
			// Obrisi trenutni par
#if OMNIDRIVE
//	if ((k==0)&&(j==0)){
//		printf("DW->TB.v[%d]=%f\n",i,TB.v[i]);
//	}
			TB.flag[i][j][k]=INITIAL;
			if((TB.v[i]>VX_MAX+V_TOLERANCE) || (TB.v[i]<VX_MIN-V_TOLERANCE) || (TB.vy[k]>VY_MAX+V_TOLERANCE) || (TB.vy[k]<VY_MIN-V_TOLERANCE) || (TB.w[j]>W_MAX+W_TOLERANCE) || (TB.w[j]<W_MIN-W_TOLERANCE)){
				TB.flag[i][j][k]=KINEMATIC_CONSTRAINTS;
 				printf("KINEMATIC_CONSTRAINTS: i=%d, j=%d, k=%d\n",i,j,k);
			}
			//adding extra constraints for |vx|+|vy|<V_MAX for vels and for delta also
#if DIAGCONSTR
			if ((fabs(TB.v[i]-rbv)+fabs(TB.vy[k]-rbvy)>DV_MAX*STEP+V_TOLERANCE) || (fabs(TB.v[i])+fabs(TB.vy[k])>V_MAX+V_TOLERANCE)){
				TB.flag[i][j][k]=KINEMATIC_CONSTRAINTS;
// 				printf("KINEMATIC_CONSTRAINTS: i=%d, j=%d, k=%d, TB.v[i]=%f, rbv=%f, TB.vy[k]=%f, rbvy=%f\n",i,j,k, TB.v[i], rbv, TB.vy[k], rbvy);
			}
#endif
			TB.ocjena[i][j][k]=0.0;
			TB.ocjena_prohodnost[i][j][k]=0.0;
			TB.ocjena_path[i][j][k]=0.0;
#else
			TB.flag[i][j]=INITIAL;
			if((TB.v[i]>VX_MAX) || (TB.v[i]<VX_MIN) || (TB.w[j]>W_MAX) || (TB.w[j]<W_MIN)){
				TB.flag[i][j]=KINEMATIC_CONSTRAINTS;
 				printf("KINEMATIC_CONSTRAINTS: i=%d, j=%d, TB.v[i]=%f, TB.w[j]=%f, VX_MAX=%f, VX_MIN=%f, W_MAX=%f, W_MIN=%f\n",i,j,TB.v[i],TB.w[j],VX_MAX,VX_MIN,W_MAX,W_MIN);
			}
#if ROT_EXCL_TRANS_CONSTR
      if((fabs(TB.v[i])>V_TOLERANCE) && (fabs(TB.w[j])>W_TOLERANCE)){
				TB.flag[i][j]=KINEMATIC_CONSTRAINTS;
			}
#endif		
			TB.ocjena[i][j]=0.0;
			TB.ocjena_prohodnost[i][j]=0.0;
			TB.ocjena_path[i][j]=0.0;
#endif
// 			printf("prije w %f, wh=%f\n",TB.w[j],WH->RB.w);
			if(i==0){
				if (WH->Tv>0.00001){
#if (PO_PREDIKCIJI==0)
				TB.w[j]=RB.w*exp(-STEP/WH->Tw)+(1-exp(-STEP/WH->Tw))*TB.w[j];//predikcija
#else
				TB.w[j]=WH->w_des_current*exp(-STEP/WH->Tw)+(1-exp(-STEP/WH->Tw))*TB.w[j];//predikcija
#endif
				}
// 				printf("poslije %d\n",int(round(TB.w[j]*RuS)));
			}
		
		}
// 		printf("vR %f, rbv=%f\n",TB.v[i],rbv);
//		TB.v[i]=round(TB.v[i]);//zaokruzivanje na cijeli broj
// 		printf("i=%d referenca v prije %d\n",i,int(TB.v[i]));
		if (WH->Tv>0.00001){
#if (PO_PREDIKCIJI==0)
		TB.v[i]=RB.v*exp(-STEP/WH->Tv)+(1-exp(-STEP/WH->Tv))*TB.v[i];//predikcija
#else
		TB.v[i]=WH->v_des_current*exp(-STEP/WH->Tv)+(1-exp(-STEP/WH->Tv))*TB.v[i];//predikcija
// 		printf("poslije v %f (sad je to desired)\n",TB.v[i]);
#endif			
		}
	}
#if OMNIDRIVE
	}
//	printf("]\n");
#endif  

	return;
}

void DynamicWindow::ChangeVariables(double x,double y,double th,double w,double v){
 ROBOT_X = x;
 ROBOT_Y =y;
 ROBOT_W=w;
 ROBOT_V=v;
 ROBOT_TH=th;
}

bool DynamicWindow::Fourpointscollide(double x, double y, double th, double vx, double vy, int index){
	double ellipse_a, ellipse_b, p, xa, ya, xb, yb;
	bool checkcoll=false;
	int signvx=1, signvy=1;
	double backvx=0.01, backvy=0.01; //security distance is less in the opposite direction of motion
	double forvx=1., forvy=1.; //security distance is in the direction of motion
	double sc_vx=(SC1+(SC2-SC1)*fabs(vx)/VX_MAX)/CELL_DIM;
	double sc_vy=(SC1+(SC2-SC1)*fabs(vy)/VY_MAX)/CELL_DIM;
	ellipse_a= sc_vx;//+0.5*sc_vy;
	ellipse_b= sc_vy;//+0.5*sc_vx;
	p=0.1;//50 % of enlarging for the last point on trajectory - uncertainty in time
	
	if (vx<0) signvx=-1;
	if (vy<0) signvy=-1;
	if (fabs(vx)<V_TOLERANCE){
		backvx=1.;//sideway default
		forvx=1.;
	}
	if (fabs(vy)<V_TOLERANCE){
		backvy=1.;
		forvy=1.;
	}
	if ((fabs(vx)<V_TOLERANCE) && (fabs(vy)<V_TOLERANCE)){
		forvx=0.4;
		forvy=0.4;
		backvx=0.4;
		backvy=0.4;
	} 
//	if (fabs(vx)<V_TOLERANCE) forvx=0.4;
//	if (fabs(vy)<V_TOLERANCE) forvy=0.4;
//	if (fabs(vx)<V_TOLERANCE) backvx=0.4;
//	if (fabs(vy)<V_TOLERANCE) backvy=0.4;
//	if (backvx*ellipse_a<SC1) backvx=1.;
//	if (backvy*ellipse_b<SC1) backvy=1.;
	xa=signvx*(1.+p*index/N_KL)*ellipse_a*cos(th);
	ya=signvx*(1.+p*index/N_KL)*ellipse_a*sin(th);
	xb=signvy*(1.+p*index/N_KL)*ellipse_b*(-1.)*sin(th);
	yb=signvy*(1.+p*index/N_KL)*ellipse_b*cos(th);
//	printf("ellipse checking for a point : xa=%f, ya=%f, xb=%f, yb=%f, vx=%f, vy=%f, x=%f, y=%f, th=%f deg, ellipse_a=%f, ellipse_b=%f, index=%d, coefficient=%f\n",xa, ya, xb, yb, vx, vy, x, y, th*RuS, ellipse_a, ellipse_b, index, (1.+p*index/N_KL));
#if RECTANGULAR
	Pose checkme(x+forvx*xa,y+forvx*ya,th);
	if (cspace->checkCollision(checkme)) {
		checkcoll=true;
//		printf("ellipse checking: point x+xa, y+ya collide\n");
	}
	if (checkcoll==false) {
		Pose checkme(x+forvy*xb,y+forvy*yb,th);
		if (cspace->checkCollision(checkme)) {
			checkcoll=true;
//			printf("ellipse checking: point x+xb, y+yb collide\n");
		}
	}
	if (checkcoll==false) {
		Pose checkme(x-backvy*xb,y-backvy*yb,th);
		if (cspace->checkCollision(checkme)) {
			checkcoll=true;
//			printf("ellipse checking: point x-0.5*xb, y-0.5*yb collide\n");
		}
	}
	if (checkcoll==false) {
		Pose checkme(x-backvx*xa,y-backvx*ya,th);
		if (cspace->checkCollision(checkme)) {
			checkcoll=true;
//			printf("ellipse checking: point x-0.5*xa, y-0.5*ya collide\n");
		}
	}
#endif	
	return checkcoll;
}

double DynamicWindow::checkDesiredOrientation(double th){

//input th is goal orientation
	I_point p;
//	double desth=th;
	double desth=RB.th;
	double desthgoal=th;
	double start_orientation, goal_orientation;
//	if ((DS->PathLength<2*ROBOT_MASKY+1)) desth=th; //rotate as goal
//	int pathHorizon=std::min(DS->PathLength,(int)ceil(N_KL*STEP*V_MAX/CELL_DIM));
	int orientationS, orientationG;
	bool takegoal=true;
	int horizon=ROBOT_MASKY-ROBOT_MASK;
#if RECTANGULAR
	OrientationIntervals ints;
	OrientationIntervals intsintersect, intsnext;
        OrientationInterval interval;
//	if (floor(fabs(RB.v)+fabs(RB.vy))<CELL_DIM){
//		horizon=1;
//		takegoal=false;
//	}
	while (desth>=2*M_PI) desth-=2*M_PI;
	while (desth<0) desth+=2*M_PI;
	start_orientation=desth;
	while (desthgoal>=2*M_PI) desthgoal-=2*M_PI;
	while (desthgoal<0) desthgoal+=2*M_PI;
	goal_orientation=desthgoal;
	double deltath=std::min(fabs(desth-desthgoal),std::min(fabs(desth+2*M_PI-desthgoal),fabs(desth-desthgoal-2*M_PI)));
	printf("deltath=%f deg. \n",deltath*RuS);
//	if (deltath>=0.5*M_PI){
//		desth=RB.th;
//	}
       			orientationS=cspace->worldToMapTheta(desth);
       			orientationG=cspace->worldToMapTheta(desthgoal);
#if 0	
	for (int j=0; j<DS->PathLength; j++){
		p=DS->path[j];
    		ints= cspace->getAdmissibleOrientationStructure(p.x,p.y);
    		if (j>0){
			std::cerr<<"Intersection with ["<<interval.lower<<","<<interval.upper<<"]: "<<OrientationIntervals::printOrientationIntervals(ints.getIntersectionWithInterval(interval))<<std::endl;
    			
    		}
    		std::cerr<<ints.print()<<std::endl;
		std::vector<OrientationInterval> intervals = cspace->getAdmissibleOrientations(p.x,p.y);
		interval.lower=0;
		interval.upper=0;
		for(unsigned int i=0; i<intervals.size(); i++){
		        std::cerr<<"["<<intervals[i].lower<<","<<intervals[i].upper<<"]"<<std::endl;
			if ((intervals[i].lower <= orientationS && intervals[i].upper >= orientationS && intervals[i].upper > intervals[i].lower)||((intervals[i].upper >= orientationS || intervals[i].lower <= orientationS) && intervals[i].upper < intervals[i].lower)){
			    interval.lower = intervals[i].lower;
			    interval.upper = intervals[i].upper;
			    std::cout<<"at "<<p.x<<","<<p.y<<"start orientation "<<orientationS<<" is contained between: "<<interval.lower<<","<<interval.upper<<std::endl;
			}
		}
	
	}
#endif	
	
	if (flag_kl_old){
		GM->mapper_point_temp.x=KL_old_temp.x[N_KL-1];
		GM->mapper_point_temp.y=KL_old_temp.y[N_KL-1];
		desth=KL_old_temp.th[N_KL-1]; //around the last orientation of the last optimal trajectory
		while (desth>=2*M_PI) desth-=2*M_PI;
		while (desth<0) desth+=2*M_PI;
		start_orientation=desth;
		deltath=std::min(fabs(desth-desthgoal),std::min(fabs(desth+2*M_PI-desthgoal),fabs(desth-desthgoal-2*M_PI)));
		printf("from the old optimal traj: deltath=%f deg. \n",deltath*RuS);
		if(GM->check_point(GM->mapper_point_temp))
		{
       			orientationS=cspace->worldToMapTheta(desth);
       			orientationG=cspace->worldToMapTheta(desthgoal);
			p=GM->cell_point_temp;
//			p=DS->Start;
    			ints= cspace->getAdmissibleOrientationStructure(p.x,p.y);
    			std::cerr<<ints.print()<<std::endl;
      std::vector<OrientationInterval> intervals = cspace->getAdmissibleOrientations(p.x,p.y);
      for(unsigned int i=0; i<intervals.size(); i++){
      std::cerr<<"["<<intervals[i].lower<<","<<intervals[i].upper<<"]"<<std::endl;
	if ((intervals[i].lower <= orientationS && intervals[i].upper >= orientationS && intervals[i].upper > intervals[i].lower)||((intervals[i].upper >= orientationS || intervals[i].lower <= orientationS) && intervals[i].upper < intervals[i].lower)){
		    interval.lower = intervals[i].lower;
		    interval.upper = intervals[i].upper;
	std::cout<<"at "<<p.x<<","<<p.y<<"start orientation "<<orientationS<<" is contained between: "<<interval.lower<<","<<interval.upper<<std::endl;
			}
	}
	std::cout<<"at "<<p.x<<","<<p.y<<"start orientation "<<orientationS<<" is contained: "<<(ints.containsOrientation(orientationS)?"yes":"no")<<std::endl;
			for (int i=0; i<horizon; i++){
				int costk=DS->map[p.x][p.y].k_cost_int;
				p=DS->map[p.x][p.y]._next;
				if ((p.x!=-1)&&(p.y!=-1)){
					intsnext= cspace->getAdmissibleOrientationStructure(p.x,p.y);
//					intsintersect=ints.getIntersectionWithInterval(intsnext);
    std::cerr<<"Intersection with ["<<interval.lower<<","<<interval.upper<<"]: "<<OrientationIntervals::printOrientationIntervals(ints.getIntersectionWithInterval(interval))<<std::endl;
//    std::cerr<<"Intersection with next: "<<intsintersect.print()<<std::endl;
		    			ints= cspace->getAdmissibleOrientationStructure(p.x,p.y);
		    			std::cerr<<ints.print()<<std::endl;
//	            			orientation=cspace->worldToMapTheta(desth);
	std::cout<<"at "<<p.x<<","<<p.y<<"start orientation "<<orientationS<<" is contained: "<<(ints.containsOrientation(orientationS)?"yes":"no")<<std::endl;
	std::cout<<"at "<<p.x<<","<<p.y<<"goal orientation "<<orientationG<<" is contained: "<<(ints.containsOrientation(orientationG)?"yes":"no")<<std::endl;
	
		    			if (!ints.containsOrientation(orientationS)){
//		    				if (deltath>=0.5*M_PI)	takegoal=false;
//						takegoal=false;
		    				int tempOrientation=orientationS;
		    				int minD=ints.getMaxOrientation()+1;
		    				for (int j=ints.getMinOrientation(); j<=ints.getMaxOrientation(); j++){
		    					if (ints.containsOrientation(j)){
		    						if ((abs(j-orientationS)<minD)||(abs(j+ints.getMaxOrientation()+1-orientationS)<minD)||(abs(j-ints.getMaxOrientation()-1-orientationS)<minD)){
//		    							minD=abs(j-orientation);
		    							minD=std::min(abs(j-orientationS),std::min((abs(j+ints.getMaxOrientation()+1-orientationS)),(abs(j-ints.getMaxOrientation()-1-orientationS))));
//		    							if (minD>ints.getMaxOrientation()) minD-=ints.getMaxOrientation()+1;
		    							tempOrientation=j;
		    						}
		    					}
		    				}
#if 1		    				
		    				//check for neighbors with nonhigher k cost
		    				I_point temp2;
	  			for (int d=0;d<8;d++){
					temp2.x=p.x+xofs[d];
					temp2.y=p.y+yofs[d];
					if ((DS->IsValid(temp2.x, temp2.y)==1)&&(DS->map[temp2.x][temp2.y].k_cost_int<=costk)){
						ints= cspace->getAdmissibleOrientationStructure(temp2.x,temp2.y);
		    				std::cerr<<ints.print()<<std::endl;
		    				std::cout<<"at neighbor "<<temp2.x<<","<<temp2.y<<"start orientation "<<orientationS<<" is contained: "<<(ints.containsOrientation(orientationS)?"yes":"no")<<std::endl;
						if (!ints.containsOrientation(orientationS)){
							for (int j=ints.getMinOrientation(); j<=ints.getMaxOrientation(); j++){
		    					if (ints.containsOrientation(j)){
		    						if ((abs(j-orientationS)<minD)||(abs(j+ints.getMaxOrientation()+1-orientationS)<minD)||(abs(j-ints.getMaxOrientation()-1-orientationS)<minD)){
//		    							minD=abs(j-orientation);
		    							minD=std::min(abs(j-orientationS),std::min((abs(j+ints.getMaxOrientation()+1-orientationS)),(abs(j-ints.getMaxOrientation()-1-orientationS))));
//		    							if (minD>ints.getMaxOrientation()) minD-=ints.getMaxOrientation()+1;
		    							tempOrientation=j;
		    						}
		    					}
		    					}
						}else{
							tempOrientation=orientationS;
							minD=0;
							break;
						}
					}
				}
#endif
//						if (minD!=0) takegoal=false;
//		    				desth=cspace->mapToWorldTheta(tempOrientation);
						orientationS=tempOrientation;
		    				printf("new orientation is %d (%f deg), max is %d\n",tempOrientation,cspace->mapToWorldTheta(tempOrientation)*RuS,ints.getMaxOrientation());
//			return desth;
		    			}//imam gresku jer se sad ciljna gleda za zadnjeg susjeda
		    			if (takegoal){
//		    				orientation=cspace->worldToMapTheta(desthgoal);
		    				if (!ints.containsOrientation(orientationG)){
			    				int tempOrientation=orientationG;
			    				int minD=ints.getMaxOrientation()+1;
			    				for (int j=ints.getMinOrientation(); j<=ints.getMaxOrientation(); j++){
			    					if (ints.containsOrientation(j)){
			    						if ((abs(j-orientationG)<minD)||(abs(j+ints.getMaxOrientation()+1-orientationG)<minD)||(abs(j-ints.getMaxOrientation()-1-orientationG)<minD)){
			    							minD=std::min(abs(j-orientationG),std::min((abs(j+ints.getMaxOrientation()+1-orientationG)),(abs(j-ints.getMaxOrientation()-1-orientationG))));
			    							tempOrientation=j;
			    						}
			    					}
			    				}
//			    				desthgoal=cspace->mapToWorldTheta(tempOrientation);
							orientationG=tempOrientation;
		    					printf("new orientation acc. to goal is %d (%f deg), max is %d\n",tempOrientation,cspace->mapToWorldTheta(tempOrientation)*RuS,ints.getMaxOrientation());
//			return desth;
		    				}
		    			}
		    		}else{
		    			desth=th;//goal orientation at the goal
		    			break;
		    		}
		    	}
		}
	//checking start and goal once again
		desth=cspace->mapToWorldTheta(orientationS);
		desthgoal=cspace->mapToWorldTheta(orientationG);
		double deltath1,deltath2,deltath3,deltath4;
		deltath1=std::min(fabs(desthgoal-goal_orientation),std::min(fabs(desthgoal+2*M_PI-goal_orientation),fabs(desthgoal-goal_orientation-2*M_PI)));
		printf("deltath around goal =%f deg. \n",deltath1*RuS);
		deltath2=std::min(fabs(desth-start_orientation),std::min(fabs(desth+2*M_PI-start_orientation),fabs(desth-start_orientation-2*M_PI)));
		printf("deltath around start =%f deg. \n",deltath2*RuS);
		deltath3=std::min(fabs(desthgoal-desth),std::min(fabs(desthgoal+2*M_PI-desth),fabs(desthgoal-desth-2*M_PI)));
		printf("deltath between new goal and new start orientations =%f deg. \n",deltath3*RuS);
		deltath4=std::min(fabs(start_orientation-goal_orientation),std::min(fabs(start_orientation+2*M_PI-goal_orientation),fabs(start_orientation-goal_orientation-2*M_PI)));
		printf("deltath between old start and goal orientations =%f deg. \n",deltath4*RuS);
		if (deltath1>=0.5*M_PI){
			takegoal=false;
		}else{
			if ((deltath2>=M_PI/12.) && (deltath3>=0.75*M_PI))
			{
				takegoal=false;
			}
			if (1&&(deltath3>= deltath4) && (deltath3>=0.75*M_PI) && (deltath4>=0.75*M_PI || deltath2>0))
			{
				takegoal=false;
			}
		}
		if (takegoal){
			desth=desthgoal;
			printf("taking the goal orientation as desired %f deg.\n",desth*RuS);
		}

	}else{
		desth=desthgoal;
	}
#endif
	return desth;
}


double DynamicWindow::computeInterpolatedCost(double x, double y, double th){

	I_point temp,best,succ,A1cell, A2cell;
	R_point C, A, A1, A2, M1, M2, M3, M4;
	double VDStar,t,t1,t2, distance_temp, distance_tempUpOri, VDStarUpOri;
   int drugi_E=0;//1-novo racunanje E-a, 0-staro, bez toga
   int susjedni_A=0;//1-novo racunanje costa prema najboljem A-u, 0-staro, bez toga
   int simplex=1; //1-novo racunanje S funkcije, 0-staro
   double min_distance, temp_min_distance, VDStar_A, min_traversalcost, min_traversalcost2, min_traversalcost3, min_traversalcost4, traversalcostVDStar, traversalcostVDStarUpOri, traversalcostbest;
   double min_distanceUpOri1, min_distanceUpOri2, temp_min_distanceUpOri, min_traversalcostUpOri1, min_traversalcostUpOri2;
   double VM1,VM2,VM3,VM4,min_distance2, min_distance3,min_distance4,a1=0,a2=0,a3=0,c1=0,c2=0, KLiE, EC;
  double closeness=COSTSTRAIGHT*0; 
  double costrot=COSTROTATION; 
  int kaozacircular=1; //test interpolacije kao za circular robot - 0 stara verzija
  int kaozacdw=1;  //podijela na 8 trokuta unutar cella
  R_point P;
  double VP;
  distance_temp=OBSTACLE;

	GM->mapper_point_temp.x=x;
	GM->mapper_point_temp.y=y;

	if(GM->check_point(GM->mapper_point_temp)) {
		temp=GM->cell_point_temp;
		VDStar=DS->map[temp.x][temp.y].h_cost_int;
		C.x=temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
		C.y=temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
		best=DS->map[temp.x][temp.y]._next;
		traversalcostVDStar=DS->map[temp.x][temp.y].traversal_cost;
		t=th;
		while (t>=2*M_PI) t-=2*M_PI;
		while (t<0) t+=2*M_PI;
#if DSTAR3D
   DStarSearchNode* currentNode;
   DStarSearchNode* nextNode;
   int o, maxOri, midth, distanceOri, distanceOri2;
	double tg=WH->global_goal_workhorse.th;
	while (tg>=2*M_PI) tg-=2*M_PI;
	while (tg<0) tg+=2*M_PI;
	int goalth=cspace->worldToMapTheta(tg);
		maxOri=OrientationIntervals::getMaxOrientation();
		temp.th=cspace->worldToMapTheta(t);
		t1=cspace->mapToWorldTheta(temp.th);
		t2=cspace->mapToWorldTheta(temp.th+1);
#if (DSTAR3DINT==0)
		c2=(t-t1)/cspace->angularResolution;// isto(t2-t1);
#endif
    if (costrot==0)
		costrot=(double)(COSTDIAGONAL-COSTSTRAIGHT)/((double)maxOri);
  		if (kaozacdw==1){
  		  costrot=(double)(COSTSTRAIGHT)/((double)maxOri); //pola straight za pi/2
  		}
		best=temp;//not used for simplex
		best.th=temp.th+1;
		if (t2>=2*M_PI) best.th=0;
		traversalcostVDStarUpOri=traversalcostVDStar;
#if USE3DCOST && (DSTAR3DORI==0)
		IntPose pointX((temp.x),(temp.y),temp.th);
		traversalcostVDStar = std::max(1., (MAXDISTANCE - floor(cspace->getDistanceInCells(pointX))));
		pointX=IntPose((temp.x),(temp.y),best.th);
		traversalcostVDStarUpOri = std::max(1., (MAXDISTANCE - floor(cspace->getDistanceInCells(pointX))));
#endif					   
#if DSTAR3DORI
    currentNode = cspace->getDStarSearchNode(temp.x,temp.y,temp.th);
    if (currentNode==NULL){
      VDStar=OBSTACLE;
    }else{
      nextNode=currentNode->getNext();
      if (nextNode!=NULL){
        best.x= ((OIDStarSearchNode*) nextNode)->x;
        best.y= ((OIDStarSearchNode*) nextNode)->y; 
      }    
      midth=currentNode->getDesiredOrientation(); 
      if (midth<0) costrot=0;
      if (naCilju){
        best.x=temp.x; best.y=temp.y;
        midth=goalth;
        c2=std::min(fabs(t-tg),std::min(fabs(t-tg+2*M_PI),fabs(t-tg-2*M_PI)))/cspace->angularResolution; //isto (t2-t1);
      }
//      VDStar = currentNode->h + COSTROTATION*std::min(abs(temp.th-midth),std::min((abs(temp.th+maxOri+1-midth)),(abs(temp.th-maxOri-1-midth))));
      OrientationIntervals::computeOrientationDistanceInInterval(currentNode->getOrientationInterval(),temp.th,midth,distanceOri);
      OrientationIntervals::computeOrientationDistanceInInterval(currentNode->getOrientationInterval(),best.th,midth,distanceOri2);
      if (naCilju){
        VDStar = currentNode->h + costrot*c2;
      }else{
        if (distanceOri2<distanceOri) c2=fabs(t-t2)/cspace->angularResolution; //isto (t2-t1);
        VDStar = currentNode->h + costrot*(std::min(distanceOri,distanceOri2)+c2);
      }
      VDStar_A = currentNode->h;
//      if (currentNode->h==0) VDStar=0;
    }
    
    nextNode = cspace->getDStarSearchNode(temp.x,temp.y,best.th);//maybe not included
    if (nextNode==NULL){
      VDStarUpOri = OBSTACLE;
    }else{
      midth=nextNode->getDesiredOrientation(); 
//      VDStarUpOri = nextNode->h + COSTROTATION*std::min(abs(best.th-midth),std::min((abs(best.th+maxOri+1-midth)),(abs(best.th-maxOri-1-midth))));
      OrientationIntervals::computeOrientationDistanceInInterval(nextNode->getOrientationInterval(),best.th,midth,distanceOri);
      VDStarUpOri = nextNode->h + costrot*distanceOri;
    }

#else
		VDStar=DS->map[temp.x][temp.y].h_cost_intOri[temp.th];
		VDStarUpOri=DS->map[temp.x][temp.y].h_cost_intOri[best.th];
#endif
#else //DSTAR3D==0
      double alfa, delta;
  		if ((naCilju)||(best.x==-1))
  		{
  		  alfa=WH->global_goal_workhorse.th;
  		  traversalcostbest=traversalcostVDStar;
  		  best=temp;
  		}else{     
        alfa=atan2((best.y-temp.y),(best.x-temp.x));
        traversalcostbest=DS->map[best.x][best.y].traversal_cost;
      }
//  		costrot=(double)(COSTSTRAIGHT)/(M_PI);
  		costrot=(double)(COSTDIAGONAL-COSTSTRAIGHT)/(2.*M_PI);
  		if (kaozacdw==1){
  		  costrot=(double)(COSTSTRAIGHT)/(2.*M_PI); //pola straight za pi/2
  		}
//  		costrot=(double)(3.*COSTDIAGONAL-4.*COSTSTRAIGHT)/(2.*M_PI);
      if (alfa<0){
      	alfa=alfa+2*M_PI;
      }
      delta=fabs(t-alfa);
      while (delta>M_PI){
        delta=fabs(delta-2*M_PI);
      }
      VDStar_A=VDStar;
      VDStar=VDStar + costrot*delta*traversalcostVDStar; //*std::min(traversalcostVDStar,traversalcostbest);

#endif
		distance_temp=VDStar;
#if (DSTARINT==0) && SPECIAL_GOAL_F
			   if (VDStar_A==0){ 

			      c1=std::max(fabs(x-WH->global_goal_workhorse.x),fabs(y-WH->global_goal_workhorse.y))/CELL_DIM*2.;//from 0 to 1
			      c2=1.;
            alfa=atan2((WH->global_goal_workhorse.y-y),(WH->global_goal_workhorse.x-x));
            if (alfa<0){
            	alfa=alfa+2*M_PI;
            }
            delta=fabs(t-alfa);
            while (delta>M_PI){
              delta=fabs(delta-2*M_PI);
            }
            
//			      if ((c1)<1.) c2=c1/1.;
			      distance_temp=c1*COSTSTRAIGHT/2.*std::min(traversalcostVDStar,traversalcostbest)+c2*costrot*delta*std::min(traversalcostVDStar,traversalcostbest)+(1-c2)*VDStar;
            if (rotateongoal){
              distance_temp=VDStar/(costrot*M_PI)*c1*COSTSTRAIGHT/2.;
            }
			   }
#endif
#if DSTARINT
		   C_plus.x=best.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
		   C_plus.y=best.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
		   E.x=(C.x+C_plus.x)/2.;
		   E.y=(C.y+C_plus.y)/2.;
		   if (drugi_E){
			   if ((abs(C_plus.x-C.x)+abs(C_plus.y-C.y))<2*GM->Map_Cell_Size){
				   E=C_plus;
			   }
		   }
		   if (susjedni_A){
			   min_distance=VDStar+0.1*sqrt((x-C.x)*(x-C.x)+(y-C.y)*(y-C.y));
			   for (int d=0; d<8; d++){
				   A.x=C.x+xofs[d]*GM->Map_Cell_Size;
				   A.y=C.y+yofs[d]*GM->Map_Cell_Size;
				   if ((DS->IsValid( temp.x+xofs[d], temp.y+yofs[d] )!=1)) continue;
				   VDStar_A=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].h_cost_int;
				   temp_min_distance=VDStar_A+0.1*sqrt((x-A.x)*(x-A.x)+(y-A.y)*(y-A.y));
				   if (temp_min_distance<min_distance){
					   min_distance=temp_min_distance;
					   E=A;
				   }
			   }
		   }
		   if (simplex==1){
#if (DSTAR3D==0) && SPECIAL_GOAL_F
			   if (VDStar_A==0){ 
			      c1=std::max(fabs(x-WH->global_goal_workhorse.x),fabs(y-WH->global_goal_workhorse.y))/CELL_DIM*2.;
			      c2=1.;
            alfa=atan2((WH->global_goal_workhorse.y-y),(WH->global_goal_workhorse.x-x));
            if (alfa<0){
            	alfa=alfa+2*M_PI;
            }
            delta=fabs(t-alfa);
            while (delta>M_PI){
              delta=fabs(delta-2*M_PI);
            }
            
//			      if ((c1)<1.) c2=c1/1.;
			      distance_temp=c1*COSTSTRAIGHT/2.*std::min(traversalcostVDStar,traversalcostbest)+c2*costrot*delta*std::min(traversalcostVDStar,traversalcostbest)+(1-c2)*VDStar;
            if (rotateongoal){
              distance_temp=VDStar/(costrot*M_PI)*c1*COSTSTRAIGHT/2.;
            }
			   } else
#endif
//			   if (VDStar==0){ 
//			      distance_temp=((x-WH->global_goal_workhorse.x)*(x-WH->global_goal_workhorse.x)+(y-WH->global_goal_workhorse.y)*(y-WH->global_goal_workhorse.y))/CELL_DIM/CELL_DIM/2. + (1-cos(t-WH->global_goal_workhorse.th))/8.;//0.5/2+2/8=0.5
//            distance_temp=costrot*fabs((t-WH->global_goal_workhorse.th)/cspace->angularResolution);
//			   }else
			   {
			   min_distance=3*GM->Map_Cell_Size;
			   min_distance2=3*GM->Map_Cell_Size;
			   for (int d=0; d<8; d++){
				   if (abs(xofs[d])+abs(yofs[d])==2){//samo dijagonalni
					   A.x=C.x+xofs[d]*GM->Map_Cell_Size;
					   A.y=C.y+yofs[d]*GM->Map_Cell_Size;
					   temp_min_distance=sqrt((x-A.x)*(x-A.x)+(y-A.y)*(y-A.y));
					   if (temp_min_distance<min_distance){
						   if (min_distance<min_distance2){
							   min_distance2=min_distance;
							   A2=A1;
							   A2cell=A1cell;
						   }
						   min_distance=temp_min_distance;
						   A1=A;
						   A1cell.x=temp.x+xofs[d];
						   A1cell.y=temp.y+yofs[d];
					   }else if (temp_min_distance<min_distance2){
						   min_distance2=temp_min_distance;
						   A2=A;
						   A2cell.x=temp.x+xofs[d];
						   A2cell.y=temp.y+yofs[d];
					   }
				   }
			   }
			   M1.x=(A1.x+C.x)/2.;
			   M1.y=(A1.y+C.y)/2.;
			   M2.x=(A2.x+C.x)/2.;
			   M2.y=(A2.y+C.y)/2.;
			   P.x=(M1.x+M2.x)/2.;
			   P.y=(M1.y+M2.y)/2.;
			   if (A1cell.y==A2cell.y){
			    M3.x=M1.x;
			    M3.y=M1.y-2.*(M1.y-C.y);
			    M4.x=M2.x;
			    M4.y=M3.y;
			   }else{
 			    M3.x=M1.x-2.*(M1.x-C.x);
			    M3.y=M1.y;
			    M4.x=M3.x;
			    M4.y=M2.y;
			   }

			   min_distance=VDStar;
			   min_distance2=VDStar;
			   min_distance3=VDStar;
			   min_distance4=VDStar;
			   min_traversalcost=traversalcostVDStar;
			   min_traversalcost2=traversalcostVDStar;
			   min_traversalcost3=traversalcostVDStar;
			   min_traversalcost4=traversalcostVDStar;
			   min_distanceUpOri1=VDStarUpOri;
			   min_distanceUpOri2=VDStarUpOri;
			   min_traversalcostUpOri1=traversalcostVDStarUpOri; //stupid error
			   min_traversalcostUpOri2=traversalcostVDStarUpOri;
			   
			   //new thing
#if DSTAR3DORI
      double vertexcost;
      if (kaozacdw==1){
        vertexcost=COSTSTRAIGHT;
      }else{
        vertexcost=COSTDIAGONAL/2.;
      }

        if (currentNode!=NULL && VDStar<OBSTACLE){ //new condition

        
        bool obstacle_near_M1=false, obstacle_near_M2=false, upobstacle_near_M1=false, upobstacle_near_M2=false; 

//        if (VDStarUpOri>=OBSTACLE){ //ovo zakomentiras ako koristis samo informaciju o zauzetim poljima a ne konfiguracijama (nize isvalid)
//          upobstacle_near_M1=true;
//          upobstacle_near_M2=true;
//        }
        int alfa,distance;
        double alfa_r;
if (kaozacircular){
      min_distance=VDStar_A;//for vertices independend of rotation cost
      min_distance2=min_distance;
      min_distance3=min_distance;
      min_distance4=min_distance;
}
			   for (int d=0; d<8; d++){
				   A.x=C.x+xofs[d]*GM->Map_Cell_Size;
				   A.y=C.y+yofs[d]*GM->Map_Cell_Size;
          
			      temp_min_distanceUpOri=OBSTACLE;//VDStarUpOri;//ovo je falilo
			      temp_min_distance=OBSTACLE;//VDStar;
            
            succ.x=temp.x+xofs[d];
            succ.y=temp.y+yofs[d];

//            if ((abs(A.x-M1.x)+abs(A.y-M1.y)==GM->Map_Cell_Size) || (abs(A.x-M2.x)+abs(A.y-M2.y)==GM->Map_Cell_Size)){
            if ((abs(A.x-M1.x)+abs(A.y-M1.y)==GM->Map_Cell_Size) || (abs(A.x-M2.x)+abs(A.y-M2.y)==GM->Map_Cell_Size) || (abs(A.x-M3.x)+abs(A.y-M3.y)==GM->Map_Cell_Size) || (abs(A.x-M4.x)+abs(A.y-M4.y)==GM->Map_Cell_Size)){
 
              currentNode = cspace->getDStarSearchNode(succ.x,succ.y,temp.th);
              nextNode = cspace->getDStarSearchNode(succ.x,succ.y,best.th);

//constraint on the sideway interpolation
#if 1
if (kaozacircular==0){
      alfa_r=atan2((succ.y-temp.y),(succ.x-temp.x));
      if (alfa_r<0){
      	alfa_r=alfa_r+2*M_PI;
      }
      alfa=cspace->worldToMapTheta(alfa_r);

      if (currentNode!=NULL){
      if (!OrientationIntervals::containsOrientation((currentNode)->getOrientationInterval(),alfa)){
      	alfa=(alfa+(maxOri+1)/2)%(maxOri+1);
      	if (!OrientationIntervals::containsOrientation((currentNode)->getOrientationInterval(),alfa)){
      	  currentNode = NULL;
      	}
      }
      }

      alfa=cspace->worldToMapTheta(alfa_r);
      if (nextNode!=NULL){
      if (!OrientationIntervals::containsOrientation((nextNode)->getOrientationInterval(),alfa)){
      	alfa=(alfa+(maxOri+1)/2)%(maxOri+1);
      	if (!OrientationIntervals::containsOrientation((nextNode)->getOrientationInterval(),alfa)){
      	  nextNode = NULL;
      	}
      }
      }
}
#endif
              int tempth, minD, desth;
              if (currentNode!=NULL){
                desth=(currentNode)->getDesiredOrientation();
                OrientationIntervals::computeOrientationDistanceInInterval(currentNode->getOrientationInterval(),temp.th,desth,minD);
//                minD=std::min(abs(temp.th-desth),std::min((abs(temp.th+maxOri+1-desth)),(abs(temp.th-maxOri-1-desth))));
                temp_min_distance = (currentNode)->h + costrot*(minD+c2);
                if (kaozacircular) temp_min_distance = (currentNode)->h;
              }
              if (nextNode!=NULL){
                desth=(nextNode)->getDesiredOrientation();
                OrientationIntervals::computeOrientationDistanceInInterval(nextNode->getOrientationInterval(),best.th,desth,minD);
//                minD=std::min(abs(best.th-desth),std::min((abs(best.th+maxOri+1-desth)),(abs(best.th-maxOri-1-desth))));
                temp_min_distanceUpOri = (nextNode)->h + costrot*minD;
              }

            }

				    if (abs(A.x-M1.x)+abs(A.y-M1.y)==GM->Map_Cell_Size){//around vertex M1
					    if (temp_min_distance<min_distance){
						    min_distance=temp_min_distance;
						  }
					    if (temp_min_distanceUpOri<min_distanceUpOri1){
						    min_distanceUpOri1=temp_min_distanceUpOri;
						  }
//						  if (currentNode==NULL || temp_min_distance>=OBSTACLE) obstacle_near_M1=true;
//						  if (nextNode==NULL || temp_min_distanceUpOri>=OBSTACLE) upobstacle_near_M1=true;
						  if ((DS->IsValid( succ.x, succ.y )!=1)) obstacle_near_M1=true;
						  if ((DS->IsValid( succ.x, succ.y )!=1)) upobstacle_near_M1=true;
            }

				    if (abs(A.x-M2.x)+abs(A.y-M2.y)==GM->Map_Cell_Size){//around vertex M2
					    if (temp_min_distance<min_distance2){
						    min_distance2=temp_min_distance;
						  }
					    if (temp_min_distanceUpOri<min_distanceUpOri2){
						    min_distanceUpOri2=temp_min_distanceUpOri;
						  }
//						  if (currentNode==NULL || temp_min_distance>=OBSTACLE) obstacle_near_M2=true;
//						  if (nextNode==NULL || temp_min_distanceUpOri>=OBSTACLE) upobstacle_near_M2=true;
						  if ((DS->IsValid( succ.x, succ.y )!=1)) obstacle_near_M2=true;
						  if ((DS->IsValid( succ.x, succ.y )!=1)) upobstacle_near_M2=true;
            }

				   if (abs(A.x-M3.x)+abs(A.y-M3.y)==GM->Map_Cell_Size){//around vertex M3
					   if (temp_min_distance<min_distance3){
						   min_distance3=temp_min_distance;
					   }
				   }
				   if (abs(A.x-M4.x)+abs(A.y-M4.y)==GM->Map_Cell_Size){//around vertex M4
					   if (temp_min_distance<min_distance4){
						   min_distance4=temp_min_distance;
					   }
				   }

         }

//        if (min_distance>=OBSTACLE){
//          min_distance=VDStar;
//        }
//        if (min_distance2>=OBSTACLE){
//          min_distance2=VDStar;
//        }
//        if (VDStarUpOri<OBSTACLE){
//          if (min_distanceUpOri1>=OBSTACLE){
//            min_distanceUpOri1=VDStarUpOri;
//          }
//          if (min_distanceUpOri2>=OBSTACLE){
//            min_distanceUpOri2=VDStarUpOri;
//          }        
//        }
			   VM1=min_distance+vertexcost*min_traversalcost;
			   VM2=min_distance2+vertexcost*min_traversalcost2;

			   VM3=min_distance3+vertexcost*min_traversalcost3;
			   VM4=min_distance4+vertexcost*min_traversalcost4;
			   if (kaozacdw==1){
			    I_point Scell;
			    Scell.x=(A1cell.x+A2cell.x)/2;
			    Scell.y=(A1cell.y+A2cell.y)/2;
			    currentNode = cspace->getDStarSearchNode(Scell.x,Scell.y,temp.th);
			    if (currentNode!=NULL){
			    if (currentNode->h<VDStar_A){
			      VP = currentNode->h + COSTSTRAIGHT/2.;//*DS->map[Scell.x][Scell.y].traversal_cost;
			    }else{
			      VP=VDStar_A+COSTSTRAIGHT/2.;//*traversalcostVDStar;
			    }
			    }
			   }
// 			  VM1=min_distance+COSTDIAGONAL/2.;
//			  VM2=min_distance2+COSTDIAGONAL/2.; 

//			   VM3=min_distance3+COSTDIAGONAL/2.;
//			   VM4=min_distance4+COSTDIAGONAL/2.;
#if 0
if (kaozacircular){
          R_point C_plusplus;
          I_point next_best;
          DStarSearchNode* nextnextNode;
          double v1,v2,v3,v4;//v1,v2 front, v3,v4 back
          next_best.x=-1; next_best.y=-1;
          currentNode = cspace->getDStarSearchNode(temp.x,temp.y,temp.th);
          if (currentNode!=NULL){
            nextNode=currentNode->getNext();
            if (nextNode!=NULL){
              nextnextNode = nextNode->getNext();
              if (nextnextNode!=NULL){
                next_best.x= ((OIDStarSearchNode*) nextnextNode)->x;
                next_best.y= ((OIDStarSearchNode*) nextnextNode)->y;
              }
              VDStarUpOri=nextNode->h;
            }else{
              next_best.x= ((OIDStarSearchNode*) currentNode)->x;
              next_best.y= ((OIDStarSearchNode*) currentNode)->y;
              VDStarUpOri=currentNode->h;
            }
          }
//          next_best=DS->map[best.x][best.y]._next;
         	C_plusplus.x=next_best.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
		      C_plusplus.y=next_best.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;

//         VDStarUpOri=DS->map[best.x][best.y].h_cost_int;
         if (fabs(C_plus.x-M1.x)+fabs(C_plus.y-M1.y)==GM->Map_Cell_Size){
            v1=VM1;
            if (fabs(C_plus.x-M2.x)+fabs(C_plus.y-M2.y)==GM->Map_Cell_Size){
                v2=VM2;
//                v3=VM3;
//                v4=VM4;
//                if (v3<=v1) v3=VDStar_A+COSTDIAGONAL/2.*traversalcostVDStar;
//                if (v4<=v2) v4=VDStar_A+COSTDIAGONAL/2.*traversalcostVDStar;
//next node - check v1=VM1 and v2=VM2
                if ((next_best.x!=-1)&&((v1+v2)/2.<VDStarUpOri)) //(v1<VDStarUpOri)&&(v2<VDStarUpOri))
                {
                  if (fabs(C_plusplus.x-M1.x)+fabs(C_plusplus.y-M1.y)==2.*GM->Map_Cell_Size){
                    v1=VDStarUpOri+COSTDIAGONAL/2.;
                  }
                  if (fabs(C_plusplus.x-M2.x)+fabs(C_plusplus.y-M2.y)==2.*GM->Map_Cell_Size){
                    v2=VDStarUpOri+COSTDIAGONAL/2.;
                  }
                  VM1=v1;
                  VM2=v2;
                }
            }else{
                v3=VM2;
                v2=VM3;
                v4=VM4;
                if (v3<v1) v3=VDStar_A+COSTDIAGONAL/2.;
                if (v4<v2) v4=VDStar_A+COSTDIAGONAL/2.;//falilo
                if ((v3+v4)/2.<VDStar_A)//((v3<VDStar_A) && (v4<VDStar_A))
                {
                  if (v3==v1) v3=VDStar_A+COSTDIAGONAL/2.;
                  if (v4==v2) v4=VDStar_A+COSTDIAGONAL/2.;             
                }
//                if ((v3<VDStar_A) && (v4<VDStar_A)) v3=VDStar_A+COSTDIAGONAL/2.;
                if ((v3+v4)/2.<VDStar_A) v3=VDStar_A+COSTDIAGONAL/2.;
                VM2=v3;
//next node - check v1=VM1 and v2=VM3
                if ((next_best.x!=-1)&&((v1+v2)/2.<VDStarUpOri)) //(v1<VDStarUpOri)&&(v2<VDStarUpOri))
                {
                  if (fabs(C_plusplus.x-M1.x)+fabs(C_plusplus.y-M1.y)==2.*GM->Map_Cell_Size){
                    v1=VDStarUpOri+COSTDIAGONAL/2.;
                  }
                  VM1=v1;
                }
                
            }
         }else{
            v3=VM1;
            if (fabs(C_plus.x-M2.x)+fabs(C_plus.y-M2.y)==GM->Map_Cell_Size){
                v1=VM2;
                v2=VM4;
                v4=VM3;
                if (v4<v2) v4=VDStar_A+COSTDIAGONAL/2.;//falilo
                if (v3<v1) v3=VDStar_A+COSTDIAGONAL/2.;
                if ((v3+v4)/2.<VDStar_A)//((v3<VDStar_A) && (v4<VDStar_A))
                {
                  if (v3==v1) v3=VDStar_A+COSTDIAGONAL/2.;
                  if (v4==v2) v4=VDStar_A+COSTDIAGONAL/2.;             
                }
//                if ((v3<VDStar_A) && (v4<VDStar_A)) v3=VDStar_A+COSTDIAGONAL/2.;
                if ((v3+v4)/2.<VDStar_A) v3=VDStar_A+COSTDIAGONAL/2.;
                VM1=v3;
//next node - check v1=VM2 and v2=VM4
                if ((next_best.x!=-1)&&((v1+v2)/2.<VDStarUpOri)) //(v1<VDStarUpOri)&&(v2<VDStarUpOri))
                {
                  if (fabs(C_plusplus.x-M2.x)+fabs(C_plusplus.y-M2.y)==2.*GM->Map_Cell_Size){
                    v1=VDStarUpOri+COSTDIAGONAL/2.;
                  }
                  VM2=v1;
                }
            }else{
                v4=VM2;
                v2=VM4;
                v1=VM3;
                if (v3<v1) v3=VDStar_A+COSTDIAGONAL/2.;
                if (v4<v2) v4=VDStar_A+COSTDIAGONAL/2.;
                if ((v3+v4)/2.<VDStar_A)//((v3<VDStar_A) && (v4<VDStar_A))
                {
                  if (v3==v1) v3=VDStar_A+COSTDIAGONAL/2.;
                  if (v4==v2) v4=VDStar_A+COSTDIAGONAL/2.;             
                }
                if ((v3+v4)/2.<VDStar_A)//((v3<VDStar_A) && (v4<VDStar_A))
                {
                  v3=VDStar_A+COSTDIAGONAL/2.;
                  v4=v3;
                }
                VM1=v3;
                VM2=v4;
//no need of checking next node            
            }
         
         }
}         
#endif


//			  if (obstacle_near_M1 || upobstacle_near_M1) VM1+=OBSTACLE/10.;
//			  if (obstacle_near_M2 || upobstacle_near_M2) VM2+=OBSTACLE/10.;
			  if (obstacle_near_M1) VM1+=closeness;//OBSTACLE/10.;
			  if (obstacle_near_M2) VM2+=closeness;//OBSTACLE/10.;
if (kaozacdw==0){//C, M1, M2
			   a3=((y-C.y)*(M1.x-C.x)-(x-C.x)*(M1.y-C.y))/((M1.x-C.x)*(M2.y-C.y)-(M2.x-C.x)*(M1.y-C.y));
			   a2=(x-C.x-(M2.x-C.x)*a3)/(M1.x-C.x);
			   }
else{//C, M1 i P
			    if (fabs(C.x-P.x)>0.1){
			      a2=(y-C.y)/(M1.y-C.y);
			      a3=(x-C.x-a2*(M1.x-C.x))/(P.x-C.x);
			    }else if (fabs(C.y-P.y)>0.1){
			      a2=(x-C.x)/(M1.x-C.x);
			      a3=(y-C.y-a2*(M1.y-C.y))/(P.y-C.y);
			    }
}
			   a1=1-a2-a3;
			   if (a1<=DEADZONE){
			      a1=DEADZONE; 
			      if (1-a3-a1<0){
			        a3=1-a2-a1;
			      }else{
			        a2=1-a3-a1;
			      }
			   }
if (kaozacdw==0){
			   distance_temp=a1*VDStar+a2*VM1+a3*VM2;
}else{
			   distance_temp=a1*VDStar+a2*VM1+a3*VP;
}

#if DSTAR3DINT

          if (VDStarUpOri>=OBSTACLE && VDStar<OBSTACLE){
            //printf("up orientation has obstacle cost\n");
            VDStarUpOri = VDStar + COSTDIAGONAL/2.;//COSTROTATION/2.; //travCostOri[0]/2.*traversalcostVDStarUpOri;//std::min(min_traversalcostUpOri2,min_traversalcostUpOri1);
//            VDStarUpOri += OBSTACLE/10.;
            if (VDStarUpOri>=OBSTACLE && VDStar<OBSTACLE){
              printf("obstacle!");
            }
          }
//          if (obstacle_near_M1==false && upobstacle_near_M1==false){
          if (min_distanceUpOri1<OBSTACLE){
    			   VM1=min_distanceUpOri1+COSTDIAGONAL/2.; //travCost[0]/2.*min_traversalcostUpOri1;
 	   			   if (upobstacle_near_M1) VM1+=closeness;//OBSTACLE/10.;

          }else{
//             VM1=VDStarUpOri;
          }
//          }//else it will reuse lower layer
//          if (obstacle_near_M2==false && upobstacle_near_M2==false){
          if (min_distanceUpOri2<OBSTACLE){
    			   VM2=min_distanceUpOri2+COSTDIAGONAL/2.; //travCost[0]/2.*min_traversalcostUpOri2;
    			   if (upobstacle_near_M2) VM2+=closeness;//OBSTACLE/10.;
          }else{
//             VM2=VDStarUpOri;
          }
//          }//else it will reuse lower layer

			   if (VM1>=OBSTACLE || VM2>=OBSTACLE){
			      printf("obstacle!");
			   }

			   distance_tempUpOri=a1*VDStarUpOri+a2*VM1+a3*VM2;
			   c2=(t-t1)/cspace->angularResolution; //isto (t2-t1);
			   c1=1-c2;
			   distance_temp=c1*distance_temp+c2*distance_tempUpOri;			   
//          distance_temp=distance_tempUpOri;
//          distance_temp=min_distance;//upobstacle_near_M1;//VM1;//min_traversalcostUpOri1;//VDStarUpOri;//std::max(1., (MAXDISTANCE - floor(cspace->getDistanceInCells(pointX))));//pum2.th;//min_distanceUpOri2;//VDStarUpOri;
//          if (temp.x==95 && temp.y==61 && temp.th==1 && 0){
//            printf("obstacle");
//          }
#endif			  
}else{
  distance_temp=VDStar;
}        
        
#else

#if USE3DCOST
      IntPose PM1, PM2, PUM1, PUM2;
      PM1=IntPose(temp.x,temp.y,temp.th);
      PM2=IntPose(temp.x,temp.y,temp.th);
      PUM1=IntPose(best.x,best.y,best.th);
      PUM2=IntPose(best.x,best.y,best.th);
#endif
      I_point pm1, pm2, pum1, pum2;
//      I_point best2;
      double temptraversalcost;
      double vertexcost;
      if (kaozacdw==1){
        vertexcost=COSTSTRAIGHT;
      }else{
        vertexcost=COSTDIAGONAL/2.;
      }
      pm1=temp; pm2=pm1;
      pum1=best; pum2=pum1;
      
#if (DSTAR3D==0)
//        if (min_distance!=VDStar || min_distance2!=VDStar){
//          printf("min_distance=%f,%f, VDStar=%f\n",min_distance,min_distance2,VDStar);
//        }
      min_distance=VDStar_A;//for vertices independend of rotation cost
      min_distance2=min_distance;
      min_distance3=min_distance;
      min_distance4=min_distance;
#endif				   
      if (VDStar<OBSTACLE){
			   for (int d=0; d<8; d++){
				   A.x=C.x+xofs[d]*GM->Map_Cell_Size;
				   A.y=C.y+yofs[d]*GM->Map_Cell_Size;
				   if ((DS->IsValid( temp.x+xofs[d], temp.y+yofs[d] )!=1)) continue;

//      best2=DS->map[temp.x+xofs[d]][temp.y+yofs[d]]._next;
//      temptraversalcost=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].traversal_cost;
//  		if (best2.x==-1){
//  		  alfa=WH->global_goal_workhorse.th;
//  		}else{     
//        alfa=atan2((best2.y-temp.y-yofs[d]),(best2.x-temp.x-xofs[d]));
//      }
//      if (alfa<0){
//      	alfa=alfa+2*M_PI;
//      }
//      delta=fabs(t-alfa);
//      while (delta>M_PI){
//        delta=fabs(delta-2*M_PI);
//      }

				   
				   if (abs(A.x-M1.x)+abs(A.y-M1.y)==GM->Map_Cell_Size){//around vertex M1
					   temp_min_distance=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].h_cost_int;//+ costrot*delta*temptraversalcost;// + costrot*delta*traversalcostVDStar;// 
#if DSTAR3D
					   temp_min_distance=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].h_cost_intOri[temp.th];
					   temp_min_distanceUpOri=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].h_cost_intOri[best.th];

					   if (temp_min_distanceUpOri<min_distanceUpOri1){
						   min_distanceUpOri1=temp_min_distanceUpOri;
#if USE3DCOST
						   PUM1=IntPose((temp.x+xofs[d]),(temp.y+yofs[d]),best.th);
						   pum1.x=temp.x+xofs[d];
						   pum1.y=temp.y+yofs[d];
						   min_traversalcostUpOri1 = std::max(1., (MAXDISTANCE - floor(cspace->getDistanceInCells(PUM1))));
#else
						   min_traversalcostUpOri1=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].traversal_cost;
#endif				   
					   }
#endif
#if USE3DCOST
						   PM1=IntPose((temp.x+xofs[d]),(temp.y+yofs[d]),temp.th);
						   pm1.x=temp.x+xofs[d];
						   pm1.y=temp.y+yofs[d];
						   temptraversalcost = std::max(1., (MAXDISTANCE - floor(cspace->getDistanceInCells(PM1))));
#else
						   temptraversalcost=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].traversal_cost;
#endif					   
					   if (temp_min_distance+temptraversalcost*vertexcost<min_distance+min_traversalcost*vertexcost){
						   min_distance=temp_min_distance;
						   min_traversalcost=temptraversalcost;
					   }
				   }
				   if (abs(A.x-M2.x)+abs(A.y-M2.y)==GM->Map_Cell_Size){//around vertex M2
					   temp_min_distance=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].h_cost_int;//+ costrot*delta*temptraversalcost;// + costrot*delta*traversalcostVDStar;// 
#if DSTAR3D

					   temp_min_distance=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].h_cost_intOri[temp.th];
					   temp_min_distanceUpOri=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].h_cost_intOri[best.th];

					   if (temp_min_distanceUpOri<min_distanceUpOri2){
						   min_distanceUpOri2=temp_min_distanceUpOri;
#if USE3DCOST
						   PUM2=IntPose((temp.x+xofs[d]),(temp.y+yofs[d]),best.th);
						   pum2.x=temp.x+xofs[d];
						   pum2.y=temp.y+yofs[d];
						   min_traversalcostUpOri2 = std::max(1., (MAXDISTANCE - floor(cspace->getDistanceInCells(PUM2))));
#else
						   min_traversalcostUpOri2=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].traversal_cost;					   
#endif					   
					   }
#endif
#if USE3DCOST
						   PM2=IntPose((temp.x+xofs[d]),(temp.y+yofs[d]),temp.th);
						   pm2.x=temp.x+xofs[d];
						   pm2.y=temp.y+yofs[d];
						   temptraversalcost = std::max(1., (MAXDISTANCE - floor(cspace->getDistanceInCells(PM2))));
#else
						   temptraversalcost=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].traversal_cost;
#endif					   
					   if (temp_min_distance+temptraversalcost*vertexcost<min_distance2+min_traversalcost2*vertexcost){
						   min_distance2=temp_min_distance;
						   min_traversalcost2=temptraversalcost;
					   }
				   }
				   if (abs(A.x-M3.x)+abs(A.y-M3.y)==GM->Map_Cell_Size){//around vertex M3
					   temp_min_distance=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].h_cost_int;//+ costrot*delta*temptraversalcost;// + costrot*delta*traversalcostVDStar;// 
						 temptraversalcost=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].traversal_cost;
					   if (temp_min_distance+temptraversalcost*vertexcost<min_distance3+min_traversalcost3*vertexcost){
						   min_distance3=temp_min_distance;
						   min_traversalcost3=temptraversalcost;
					   }
				   }
				   if (abs(A.x-M4.x)+abs(A.y-M4.y)==GM->Map_Cell_Size){//around vertex M4
					   temp_min_distance=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].h_cost_int;//+ costrot*delta*temptraversalcost;// + costrot*delta*traversalcostVDStar;// 
						 temptraversalcost=DS->map[temp.x+xofs[d]][temp.y+yofs[d]].traversal_cost;
					   if (temp_min_distance+temptraversalcost*vertexcost<min_distance4+min_traversalcost4*vertexcost){
						   min_distance4=temp_min_distance;
						   min_traversalcost4=temptraversalcost;
					   }
				   }
				   
				   
			   }

//test 
//          min_traversalcost=1;
//          min_traversalcost2=1;
//          min_traversalcost3=1;
//          min_traversalcost4=1; //must not use occupancy values in interpolation - produces loc min
//          traversalcostVDStarUpOri=1;
//          min_traversalcostUpOri1=1;
//          min_traversalcostUpOri2=1;
//          traversalcostVDStar=1;
//          traversalcostbest=1;
			   VM1=min_distance+vertexcost*min_traversalcost;
			   VM2=min_distance2+vertexcost*min_traversalcost2;

			   VM3=min_distance3+vertexcost*min_traversalcost3;
			   VM4=min_distance4+vertexcost*min_traversalcost4;
			   if (kaozacdw==1){
			    I_point Scell;
			    Scell.x=(A1cell.x+A2cell.x)/2;
			    Scell.y=(A1cell.y+A2cell.y)/2;
			    if (DS->map[Scell.x][Scell.y].h_cost_int<VDStar_A){
			      VP=DS->map[Scell.x][Scell.y].h_cost_int+COSTSTRAIGHT/2.*DS->map[Scell.x][Scell.y].traversal_cost;
			    }else{
			      VP=VDStar_A+COSTSTRAIGHT/2.*traversalcostVDStar;
			    }
			   }

#if DSTAR3D==0 && 0
          R_point C_plusplus;
          I_point next_best;
          double v1,v2,v3,v4;//v1,v2 front, v3,v4 back
          next_best=DS->map[best.x][best.y]._next;
         	C_plusplus.x=next_best.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
		      C_plusplus.y=next_best.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;

         VDStarUpOri=DS->map[best.x][best.y].h_cost_int;
         if (fabs(C_plus.x-M1.x)+fabs(C_plus.y-M1.y)==GM->Map_Cell_Size){
            v1=VM1;
            if (fabs(C_plus.x-M2.x)+fabs(C_plus.y-M2.y)==GM->Map_Cell_Size){
                v2=VM2;
//                v3=VM3;
//                v4=VM4;
//                if (v3<=v1) v3=VDStar_A+COSTDIAGONAL/2.*traversalcostVDStar;
//                if (v4<=v2) v4=VDStar_A+COSTDIAGONAL/2.*traversalcostVDStar;
//next node - check v1=VM1 and v2=VM2
                if ((next_best.x!=-1)&&(v1<VDStarUpOri)&&(v2<VDStarUpOri)){
                  if (fabs(C_plusplus.x-M1.x)+fabs(C_plusplus.y-M1.y)==2.*GM->Map_Cell_Size){
                    v1=VDStarUpOri+vertexcost*traversalcostbest;
                  }
                  if (fabs(C_plusplus.x-M2.x)+fabs(C_plusplus.y-M2.y)==2.*GM->Map_Cell_Size){
                    v2=VDStarUpOri+vertexcost*traversalcostbest;
                  }
                  VM1=v1;
                  VM2=v2;
                  if (VP<VDStarUpOri) VP=VDStarUpOri+COSTSTRAIGHT/2.*traversalcostbest;
                }
            }else{
                v3=VM2;
                v2=VM3;
                v4=VM4;
                if (v3<v1) v3=VDStar_A+vertexcost*traversalcostVDStar;
                if (v4<v2) v4=VDStar_A+vertexcost*traversalcostVDStar;//falilo
                if ((v3<VDStar_A) && (v4<VDStar_A)){
                  if (v3==v1) v3=VDStar_A+vertexcost*traversalcostVDStar;
                  if (v4==v2) v4=VDStar_A+vertexcost*traversalcostVDStar;             
                }
                if ((v3<VDStar_A) && (v4<VDStar_A)) v3=VDStar_A+vertexcost*traversalcostVDStar;
                VM2=v3;
//next node - check v1=VM1 and v2=VM3
                if ((next_best.x!=-1)&&(v1<VDStarUpOri)&&(v2<VDStarUpOri)){
                  if (fabs(C_plusplus.x-M1.x)+fabs(C_plusplus.y-M1.y)==2.*GM->Map_Cell_Size){
                    v1=VDStarUpOri+vertexcost*traversalcostbest;
                  }
                  VM1=v1;
                }
                if (VP<v1) VP=(v1+v3)/2.;
                
            }
         }else{
            v3=VM1;
            if (fabs(C_plus.x-M2.x)+fabs(C_plus.y-M2.y)==GM->Map_Cell_Size){
                v1=VM2;
                v2=VM4;
                v4=VM3;
                if (v4<v2) v4=VDStar_A+vertexcost*traversalcostVDStar;//falilo
                if (v3<v1) v3=VDStar_A+vertexcost*traversalcostVDStar;
                if ((v3<VDStar_A) && (v4<VDStar_A)){
                  if (v3==v1) v3=VDStar_A+vertexcost*traversalcostVDStar;
                  if (v4==v2) v4=VDStar_A+vertexcost*traversalcostVDStar;             
                }
                if ((v3<VDStar_A) && (v4<VDStar_A)) v3=VDStar_A+vertexcost*traversalcostVDStar;
                VM1=v3;
//next node - check v1=VM2 and v2=VM4
                if ((next_best.x!=-1)&&(v1<VDStarUpOri)&&(v2<VDStarUpOri)){
                  if (fabs(C_plusplus.x-M2.x)+fabs(C_plusplus.y-M2.y)==2.*GM->Map_Cell_Size){
                    v1=VDStarUpOri+vertexcost*traversalcostbest;
                  }
                  VM2=v1;
                }
                if (VP<v1) VP=(v1+v3)/2.;

            }else{
                v4=VM2;
                v2=VM4;
                v1=VM3;
                if (v3<v1) v3=VDStar_A+vertexcost*traversalcostVDStar;
                if (v4<v2) v4=VDStar_A+vertexcost*traversalcostVDStar;
                if ((v3<VDStar_A) && (v4<VDStar_A)){
                  if (v3==v1) v3=VDStar_A+vertexcost*traversalcostVDStar;
                  if (v4==v2) v4=VDStar_A+vertexcost*traversalcostVDStar;             
                }
                if ((v3<VDStar_A) && (v4<VDStar_A)){
                  v3=VDStar_A+vertexcost*traversalcostVDStar;
                  v4=v3;
                }
                VM1=v3;
                VM2=v4;
                if (VP<VDStar_A) VP=VDStar_A+COSTSTRAIGHT/2.*traversalcostVDStar;
//no need of checking next node            
            }
         
         }
#endif         


if (kaozacdw==0){//C, M1, M2
			   a3=((y-C.y)*(M1.x-C.x)-(x-C.x)*(M1.y-C.y))/((M1.x-C.x)*(M2.y-C.y)-(M2.x-C.x)*(M1.y-C.y));
			   a2=(x-C.x-(M2.x-C.x)*a3)/(M1.x-C.x);
			   }
else{//C, M1 i P
			    if (fabs(C.x-P.x)>0.1){
			      a2=(y-C.y)/(M1.y-C.y);
			      a3=(x-C.x-a2*(M1.x-C.x))/(P.x-C.x);
			    }else if (fabs(C.y-P.y)>0.1){
			      a2=(x-C.x)/(M1.x-C.x);
			      a3=(y-C.y-a2*(M1.y-C.y))/(P.y-C.y);
			    }
}
			   a1=1-a2-a3;
			   if (a1<=DEADZONE){
			      a1=DEADZONE; 
			      if (1-a3-a1<0){
			        a3=1-a2-a1;
			      }else{
			        a2=1-a3-a1;
			      }
			   }
if (kaozacdw==0){
			   distance_temp=a1*VDStar+a2*VM1+a3*VM2;
}else{
			   distance_temp=a1*VDStar+a2*VM1+a3*VP;
}
			   
//			   distance_temp=DS->map[temp.x][temp.y].h_cost_int;//VDStar;//min_distance;

#if DSTAR3DINT
          if (VDStarUpOri>=OBSTACLE){
            //printf("up orientation has obstacle cost\n");
            VDStarUpOri = VDStar + COSTDIAGONAL/2.*traversalcostVDStarUpOri;//COSTROTATION/2.*traversalcostVDStarUpOri;//std::min(min_traversalcostUpOri2,min_traversalcostUpOri1);
//            if (VDStarUpOri>=OBSTACLE && VDStar<OBSTACLE){
//              printf("obstacle!");
//            }
          }
          if (min_distanceUpOri1<OBSTACLE){
    			   VM1=min_distanceUpOri1+COSTDIAGONAL/2.*min_traversalcostUpOri1;
    			}else{
//    			   VM1=VDStarUpOri;//mora ovako jer ga s druge strane mozda ima
    			}

          if (min_distanceUpOri2<OBSTACLE){
    			   VM2=min_distanceUpOri2+COSTDIAGONAL/2.*min_traversalcostUpOri2;
    			}else{
//    			   VM2=VDStarUpOri;
    			}

//         if (min_distanceUpOri1>=OBSTACLE) VM1=VDStarUpOri;
//         if (min_distanceUpOri2>=OBSTACLE) VM2=VDStarUpOri;
// 			   VM1=min_distanceUpOri1+travCost[0]/2.*min_traversalcostUpOri1;//ovo radi skokove
// 			   VM2=min_distanceUpOri2+travCost[0]/2.*min_traversalcostUpOri2;

//			   if (VM1>=OBSTACLE || VM2>=OBSTACLE){
//			      printf("obstacle!");
//			   }
			   distance_tempUpOri=a1*VDStarUpOri+a2*VM1+a3*VM2;
			   c2=(t-t1)/(t2-t1);
			   c1=1-c2;
			   distance_temp=c1*distance_temp+c2*distance_tempUpOri;			   
//          distance_temp=distance_tempUpOri;
//          distance_temp=min_traversalcostUpOri2;//std::max(1., (MAXDISTANCE - floor(cspace->getDistanceInCells(pointX))));//pum2.th;//min_distanceUpOri2;//VDStarUpOri;
//          if (pum2.y==26 && pum2.x==35 && temp.x==36 && temp.y==26 && pum2.th==-1 && 1){
//            printf("obstacle");
//          }
#endif
}else{
  distance_temp=VDStar;
}        
#endif //if not dstar3dori
#if DSTAR3DORI && 0
          distance_temp+=(1-cos(t-t2))/2.;
#endif
			   if ((a1<-0.0000001)||(a2<-0.0000001)||(a3<-0.0000001)||(c1<-0.0000001)||(c2<-0.0000001)){
				   printf("c1=%f, c2=%f, t=%f, t1=%f, t2=%f, temp.th=%d, best.th=%d\n",c1,c2,t,t1,t2,temp.th, best.th);
      std::cerr<<"best ["<<best.x<<","<<best.y<<","<<best.th<<"]"<<std::endl;
      std::cerr<<"temp ["<<temp.x<<","<<temp.y<<","<<temp.th<<"]"<<std::endl;
      std::cerr<<"temp.th (t)="<<temp.th<<"("<<t<<"), t1="<<t1<<" t2="<<t2<<std::endl;
				   printf("a1,a2,a3=%f,%f,%f, sum=%f\n",a1,a2,a3,a1+a2+a3);
	      std::cerr<<"P ["<<P.x<<","<<P.y<<"]"<<std::endl;
	      std::cerr<<"M1 ["<<M1.x<<","<<M1.y<<"]"<<std::endl;
	      std::cerr<<"C ["<<C.x<<","<<C.y<<"]"<<std::endl;
	      std::cerr<<"M2 ["<<M2.x<<","<<M2.y<<"]"<<std::endl;
	      std::cerr<<"x,y ["<<x<<","<<y<<"]"<<std::endl;
			   
			   }
        }//VDStar not zero (at the goal)
			   
		   }else{ //if not simplex
			   KLiE=sqrt((x-E.x)*(x-E.x)+(y-E.y)*(y-E.y));
			   EC=sqrt((C.x-E.x)*(C.x-E.x)+(C.y-E.y)*(C.y-E.y));
			   distance_temp = KLiE*0.1 + VDStar - EC*0.1;
		   }
	   
#endif
		   					   					   //odredjivanje E tocke
		   if (simplex==1){
			   if ((VM1<VM2) && (VM1<VDStar)){
				   E=M1;
			   }
			   if ((VM2<VM1) && (VM2<VDStar)){
				   E=M2;
			   }
			   if ((VDStar<VM1) && (VDStar<VM2)){
				   E=C;
			   }
		   }
	}
	   
   return distance_temp;

}



void DynamicWindow::Pokretna_prepreka(){
	int i,j;
// 	double minudalj,udalj,brzina,kut;
	double x_c, y_c, th_c, r, v, w,x,y,th;	
	int smjer_kretanja;
	double current_rotation_angle, angle_increment;
	broj_pokretnih_prepreka=0;
	bool robot1=true,robot2=true;
	double udalj=RR+CELL_DIM/2.;

	//dodajem da se svaka celija giba kao teziste i na taj nacin povecavam broj pokretnih prepreka
	for(i=0;i<GM->numnewmovingcells;i++){
		PPP[broj_pokretnih_prepreka].x=GM->moving_cell[i].x;
		PPP[broj_pokretnih_prepreka].y=GM->moving_cell[i].y;
// 		PPP[broj_pokretnih_prepreka].brzina=300.;//umjetno sam stavila da se giba brzinom 100 mm/s
// 		PPP[broj_pokretnih_prepreka].omega=0.1;
		PPP[broj_pokretnih_prepreka].omega=0.;
		PPP[broj_pokretnih_prepreka].brzina=0.;
		PPP[broj_pokretnih_prepreka].kut=0.;

		 //gledamo udaljenost pozicija pokretnih prepreka od drugih pametnih robota: ako je udaljenost<RR+CELL_DIM/2 (pola radijusa robota + pogreska od pol polja)
		if (((PPP[broj_pokretnih_prepreka].x-ROBOT_X)*(PPP[broj_pokretnih_prepreka].x-ROBOT_X)+(PPP[broj_pokretnih_prepreka].y-ROBOT_Y)*(PPP[broj_pokretnih_prepreka].y-ROBOT_Y)<udalj*udalj)) {
		  PPP[broj_pokretnih_prepreka].omega=ROBOT_W;
		  PPP[broj_pokretnih_prepreka].brzina=ROBOT_V;
		  PPP[broj_pokretnih_prepreka].kut=ROBOT_TH;
		}
//		if ((robot2)&&((PPP[broj_pokretnih_prepreka].x-x2)*(PPP[broj_pokretnih_prepreka].x-x2)+(PPP[broj_pokretnih_prepreka].y-y2)*(PPP[broj_pokretnih_prepreka].y-y2)<udalj*udalj)) {
//		  PPP[broj_pokretnih_prepreka].omega=w2;
//		  PPP[broj_pokretnih_prepreka].brzina=v2;
//		  PPP[broj_pokretnih_prepreka].kut=th2;
//		}

		for (int m=0;m<GM->numnewcg;m++){
			if (i<GM->moving_cell_index[m]) {
				PPP[broj_pokretnih_prepreka].indeks=m;
				break;
			}
		}
		
//		PPP[broj_pokretnih_prepreka].indeks=-1;//o kojoj pokretnoj prepreci se radi (o kojem tezistu po redu)  - ne radi se o tezistu
		broj_pokretnih_prepreka++;
	}
  
	for (i=0;i<broj_pokretnih_prepreka;i++){
		//kruzni luk
		v=PPP[i].brzina;
		w=PPP[i].omega;
		x=PPP[i].x;
		y=PPP[i].y;
		th=PPP[i].kut;
		MO[i].x[0]=x;
		MO[i].y[0]=y;
		MO[i].th[0]=th;
		for(j=1;j<N_KL;j++){
			if(fabs(w)>W_TOLERANCE){
				MO[i].th[j]=MO[i].th[j-1]+w*STEP;
				MO[i].x[j]=MO[i].x[j-1]+v/w*(sin(MO[i].th[j])-sin(MO[i].th[j-1]));
				MO[i].y[j]=MO[i].y[j-1]-v/w*(cos(MO[i].th[j])-cos(MO[i].th[j-1]));
			}else{
				MO[i].x[j]=MO[i].x[j-1]+v*STEP*cos(MO[i].th[j-1]);
				MO[i].y[j]=MO[i].y[j-1]+v*STEP*sin(MO[i].th[j-1]);
				MO[i].th[j]=MO[i].th[j-1];
			}
		}
	}
	return;
}



//ovo se ne koristi
void DynamicWindow::MinimalniZaustavniPut()
{
// Minimalni zaustavni put na toj trajektoriji (po v ili w)
	double v;
#if OMNIDRIVE
	v=sqrt(TB.v[ni]*TB.v[ni]+TB.vy[nk]*TB.vy[nk]);
#else
	v=fabs(TB.v[ni]); //gornji izraz isto daje ali se izbjegne racunanje korijena
#endif
	if(v/DV_MAX>fabs(TB.w[nj]/DW_MAX))
	{
             //zaustavljanje po linearnoj brzini je duze od zaustavljanja po kutnoj brzini
             //onda se koci sa maksimalnom linearnom deceleracijom
		ts=(v)/DV_MAX;
		as=DV_MAX;
	}else{
             //zaustavljanje po linearnoj brzini je krace od zaustavljanja po kutnoj brzini
		ts=fabs(TB.w[nj]/DW_MAX);
		as=(v)/ts; //linearna deceleracija je manja od maksimalne
		if ((v)<V_TOLERANCE)
			as=DW_MAX;//da se ne bi djelilo s nulom dolje pri odredjivanju ss
	}
	   //ovdje je sa Tsecurity opisan broj ciklusa koji mozemo 
	   //"promasiti" - 4 ciklusa
// 	double Tsecurity=WH->Tv;//ovo je pravo kasnjenje
// ss=TB.v[ni]*(ts+Tsecurity)-0.5*as*ts*ts;
//	v=TB.v[ni];
	if ((v)<V_TOLERANCE)
		v=fabs(TB.w[nj]);
	ss=1/2.*(as)*(v/as+WH->Tv)*(v/as+WH->Tv)-as*(WH->Tv)*(WH->Tv)*(1-exp(-v/as/WH->Tv-1));
	ss+=v*STEP; //tek nakon jednog koraka bi islo kocenje inace se zabija u prepreku
	if (((v)<V_TOLERANCE)&&(fabs(ss)<THETA_TOLERANCE))
		ss=0.;
	if ((v>V_TOLERANCE)&&(fabs(ss)<V_TOLERANCE*STEP))
		ss=0.;
	//if ((naCilju)&&(fabs(TB.v[ni])<V_TOLERANCE))
	//	printf("minimalni zaustavni put: (ni,nj)=(%d,%d), (v,w)=(%f,%f), as=%f, ss=%f, v=%f\n",ni,nj,TB.v[ni],TB.w[nj],as,ss,v);
	// 	ss=TB.v[ni]*(ts)-0.5*as*ts*ts;
}

/*****************************************************************
 * Funkcija koja nalazi zadani broj tocaka KRUZNOG LUKA / PRAVCA *
 *****************************************************************/
void DynamicWindow::Kruzni_luk(){

	static int i;
	double v,vy,w,r,si;
	// Ako je w<>0, kruzni luk
	v=TB.v[ni]; w=TB.w[nj]; vy=0.;
#if OMNIDRIVE
	vy=TB.vy[nk];
#endif
	KL.x[0]=RB.x;
	KL.y[0]=RB.y;
	KL.th[0]=RB.th;
	KL.v[0]=v;
	KL.vy[0]=vy;
	KL.w[0]=w;
			//inicijalizacija tocaka zaustavljanja na kruznom luku
		
#if OMNIDRIVE
//	TB.breakage_point_x[ni][nj][nk]=RB.x;
//	TB.breakage_point_y[ni][nj][nk]=RB.y;
//	TB.breakage_point_th[ni][nj][nk]=RB.th;
//	r=fabs(sqrt(v*v+vy*vy)/w);
//	TB.radius[ni][nj][nk]=r;  //mjerimo zakrivljenost krivulje pomocu radiusa
//	if(fabs(w)<W_TOLERANCE)
//		TB.radius[ni][nj]=-1.0;  //beskonacni radius
#else
	TB.breakage_point_x[ni][nj]=RB.x;
	TB.breakage_point_y[ni][nj]=RB.y;
	TB.breakage_point_th[ni][nj]=RB.th;
	r=fabs(v/w);
	TB.radius[ni][nj]=r;  //mjerimo zakrivljenost krivulje pomocu radiusa
	if(fabs(w)<W_TOLERANCE)
		TB.radius[ni][nj]=-1.0;  //beskonacni radius
#endif
		
	for(i=1;i<N_KL;i++){
		KL.v[i]=v;
		KL.vy[i]=vy;
		KL.w[i]=w;
		kin_model_evol(&KL,i);
		w=KL.w[i-1];
		v=KL.v[i-1];
		si=v*((double)i)*STEP;
		if (fabs(v)<V_TOLERANCE)
			si=fabs(KL.th[i]);
			 //ako je breakage
		if(si<=ss){ //ss je izracunat u MinimalniZaustavniPut()
			TB.breakage_point_x[ni][nj]=KL.x[i];
			TB.breakage_point_y[ni][nj]=KL.y[i];
			TB.breakage_point_th[ni][nj]=KL.th[i];
		}
	}
}


void DynamicWindow::kin_model_evol(MB_KL *KLa, int i)
{
    //delay of 3 time steps on p3dx
    double v, vy, w;
    int delaydt;
    delaydt=DELAY3DT;
    if (i<=delaydt){
//      v=KLa->v[0];
//      vy=KLa->vy[0];
//      w=KLa->w[0];
      v=oldref.v[delaydt-i];
      vy=oldref.vy[delaydt-i];
      w=oldref.w[delaydt-i];
    }else{
      v=KLa->v[i-delaydt];
      vy=KLa->vy[i-delaydt];
      w=KLa->w[i-delaydt];
    }
#if OMNIDRIVE
		KLa->x[i]=KLa->x[i-1]+ (v*cos(KLa->th[i-1]) - vy*sin(KLa->th[i-1]))*STEP;
		KLa->y[i]=KLa->y[i-1]+ (v*sin(KLa->th[i-1]) + vy*cos(KLa->th[i-1]))*STEP;
		KLa->th[i]=KLa->th[i-1]+w*STEP;
#else
			if(fabs(w)>W_TOLERANCE){
				KLa->th[i]=KLa->th[i-1]+w*STEP;
				KLa->x[i]=KLa->x[i-1]+v/w*(sin(KLa->th[i])-sin(KLa->th[i-1]));
				KLa->y[i]=KLa->y[i-1]-v/w*(cos(KLa->th[i])-cos(KLa->th[i-1]));
			}else{
				KLa->x[i]=KLa->x[i-1]+v*STEP*cos(KLa->th[i-1]);
				KLa->y[i]=KLa->y[i-1]+v*STEP*sin(KLa->th[i-1]);
				KLa->th[i]=KLa->th[i-1];
			}
#endif
}

void DynamicWindow::Kruzni_luk_sa_zaustavljanjem(){

	static int i,j;
 	int signw0,signv0,signvy0;
	double v,vy,w,v0,vy0,w0,wM, wM2, vM=0., vyM=0., vM2=0., vyM2=0., s1=0.,s2=0.,sg,dg,dgoal=-1.,dtemp=0.,dtemp_plus=0.,dtemp_minus=0.,dtemp_minus2=0.;
#if ROT
	double wrot, wrot2, vrot, vrot2, vyrot, vyrot2;
#endif
	double S1;
	bool prvi=false,drugi=false;//,blizu_cilja=false;
	int Tv,Tvy,Tw,N,M,Md, Mw, Nw, N0, Nend, Nv0, Mv, Nv, upsideVforvy, upsideVforv, upsideVforw;
	int i_temp=1,i_end=0,i_poc, T_kraj,i_temp_plus=1,i_temp_minus=1,i_temp_minus2=1,
	i_temp_plus1=1, i_temp_plus2=1;
	int bez_kracenja;
	int provjeraluka;
//new part
	int usecostmask;
	int ellipse;
	double vlimit=30.; //low velocity for real collision calculation
	int occval=int(floor(COST_MASK*((V_MAX-fabs(TB.vy[nk]))/(V_MAX-vlimit))+1+1));
//end of new part
	I_point polje, A11, A22;
	R_point A1,A2;
	double px,py,pa,pb;
	// Ako je w<>0, kruzni luk
	v0=TB.v[ni]; w0=TB.w[nj];
	vy0=0.;
#if OMNIDRIVE
	vy0=TB.vy[nk];
#endif
  signv0=1; signw0=1; signvy0=1;
  if (v0<0) signv0=-1;
  if (w0<0) signw0=-1;
  if (vy0<0) signvy0=-1;
//according to the omnidrive constraints - zbog plus1 i plus2 moram stavit pola kao tamo
#if DIAGCONSTR
	Tv=(int)ceil((fabs(v0))/(0.5*DVX_MAX*STEP));//+fabs(vy0))/(DVX_MAX*STEP));//vrijeme zaustavljanja
	Tvy=(int)ceil(fabs(vy0)/(0.5*DVY_MAX*STEP));
#else
	Tv=(int)ceil((fabs(v0))/(DVX_MAX*STEP));//vrijeme zaustavljanja
	Tvy=(int)ceil(fabs(vy0)/(DVY_MAX*STEP));
#endif
	Tw=(int)ceil(fabs(w0)/(DW_MAX*STEP));
	N=std::max(Tv,Tw);
	N=std::max(N,Tvy);
#if (OKO_REFERENCE==1)
	v=WH->v_refdin_current;
	vy=WH->vy_refdin_current;
	w=WH->w_refdin_current;
#else
	v=RB.v;
	w=RB.w;//prvi ciklus je jos stara brzina na snazi
	vy=RB.vy;
#endif
	KL.x[0]=RB.x;
	KL.y[0]=RB.y;
	KL.th[0]=RB.th;
	KL.v[0]=v;
	KL.vy[0]=vy;
	KL.w[0]=w;
	//initialization: plus
	KL_plus.x[0]=RB.x;
	KL_plus.y[0]=RB.y;
	KL_plus.th[0]=RB.th;
	KL_plus.v[0]=v;
	KL_plus.vy[0]=vy;
	KL_plus.w[0]=w;
#if ROT
	//plus1
	KL_plus1.x[0]=RB.x;
	KL_plus1.y[0]=RB.y;
	KL_plus1.th[0]=RB.th;
	KL_plus1.v[0]=v;
	KL_plus1.vy[0]=vy;
	KL_plus1.w[0]=w;
	//plus2
	KL_plus2.x[0]=RB.x;
	KL_plus2.y[0]=RB.y;
	KL_plus2.th[0]=RB.th;
	KL_plus2.v[0]=v;
	KL_plus2.vy[0]=vy;
	KL_plus2.w[0]=w;
#endif
	//minus
	KL_minus.x[0]=RB.x;
	KL_minus.y[0]=RB.y;
	KL_minus.th[0]=RB.th;
	KL_minus.v[0]=v;
	KL_minus.vy[0]=vy;
	KL_minus.w[0]=w;
	//minus2
	KL_minus2.x[0]=RB.x;
	KL_minus2.y[0]=RB.y;
	KL_minus2.th[0]=RB.th;
	KL_minus2.v[0]=v;
	KL_minus2.vy[0]=vy;
	KL_minus2.w[0]=w;
#if OMNIDRIVE
	TB.flag[ni][nj][nk]=CLEAR;//inicijalizacija
	TB_plus.flag[ni][nj][nk]=CLEAR;//inicijalizacija
#if ROT
	TB_plus1.flag[ni][nj][nk]=CLEAR;//inicijalizacija
	TB_plus2.flag[ni][nj][nk]=CLEAR;//inicijalizacija
#endif
	TB_minus.flag[ni][nj][nk]=CLEAR;//inicijalizacija
	TB_minus2.flag[ni][nj][nk]=CLEAR;//inicijalizacija
#else
	TB.flag[ni][nj]=CLEAR;//inicijalizacija
	TB_plus.flag[ni][nj]=CLEAR;//inicijalizacija
	TB_minus.flag[ni][nj]=CLEAR;//inicijalizacija
	TB_minus2.flag[ni][nj]=CLEAR;//inicijalizacija
#endif
	//izracun s2 (put zaustavljanja)
	if (N>1)
		s2=(N-1)/2.;
	s1=v0*STEP;
#if OMNIDRIVE
	s1=sqrt(v0*v0+vy0*vy0)*STEP;
#endif
	M=(int)ceil(s2);
	Md=M;//donja granica pomaka po trajektoriji
	if (M>1){
		Md=M-1;
	}
/*	else if (M>1){
		Md=M-1;
	}*/
// 	sg=(N_KL-1-N)*s1+s2*s1;//duljina trajektorije
// 	sg=v*STEP+(N_KL-2-N)*s1+s2*s1;
// 	dg=((RB.x-WH->global_goal_workhorse.x)*(RB.x-WH->global_goal_workhorse.x)+(RB.y-WH->global_goal_workhorse.y)*(RB.y-WH->global_goal_workhorse.y));//ovo sluzi samo kao gornja granica
// 	sg=dg;
// 	dg=S1*10.;//da bude u mm, ovo je zapravo S0 u ovom koraku, ali S1 iz proslog koraka
/*	if ((dg<sg)&&(flag_kl_old)){//ako je udaljenost od cilja manja od duljine trajektorije onda je kratim
		blizu_cilja=true;
	}*/
	if ((flag_kl_old)&&(!blizu_cilja)){
		S1=KL_old.S[1];
		dg=S1*CELL_DIM/COSTSTRAIGHT;//da bude u mm to bas ne radi sa costmaskom!!!!
		sg=(N_KL-2-N+s2)*s1;
// 			printf("do cilja dg=%f, trajektorija sg=%f\n",dg,sg);
		if ((dg<=sg)){//ako je udaljenost od cilja manja od duljine trajektorije onda je kratim
//			blizu_cilja=true;
 			blizu_cilja=false;//kad ne zelim kracenje eliminiranje trajektorija zbog cilja
		}
	}
	bez_kracenja=1;//1 ako ne kratimo trajektorije da ne udaraju u prepreku, 0-kratimo
	provjeraluka=0;//1 ako provjeravamo sijece li luk prepreku, 0 ako ne (staro)
	usecostmask=0; //1 ako smatramo sudarom vece vrijednosti costmaske da bi usporili robota
	ellipse=ELLIPSE;//ako zamucujemo svaku tocku za provjeru sudara
	int checkdiag=0; //provjeru ogranicenja za debug
	T_kraj=T_old+1;//jer je T_old u ovom koraku pomaknut za 1 unaprijed (smanjen)
//	T_kraj=T_old+2;//sad je obicni ono sto je prije bio plus, a minus2 stara, a plus koristim za kocenje
	if (best_old) T_kraj=T_old;//sad je obicni stara, a plus malo dulja T_old-1;//tako da plus bude stara
//	T_kraj=N;//only ramps up and down all four trajectories N+3, only plus N (comment this, uncomment two lines below)
    int delaydt;
    delaydt=DELAY3DT;

	T_kraj=std::min(T_kraj,(N_KL-3-delaydt+1));//omogucava plus uvijek, znaci da plus postaje mozda cak i minus2
	T_kraj=std::max(T_kraj,(N+3));//ovo znaci da minus2 postaje mozda cak i plus, a ovi dolje ionako ne bi postojali
//	T_kraj=N_KL-3;//tu pazi! zakomentiraj kada zelis setajuci horizont
	if ((fabs(v0)<V_TOLERANCE)&& (fabs(vy0)<V_TOLERANCE) && (fabs(w0)<W_TOLERANCE)) printf("T_end=%d, best_old=%d, T_old=%d\n", T_kraj,best_old, T_old);
//this is for different orientational velocity profile
	upsideVforvy=0;
	upsideVforv=0;
	upsideVforw=1;
	//plus
	if (1 || ((fabs(v0)<V_TOLERANCE)&& (fabs(vy0)<V_TOLERANCE) && (fabs(w0)<W_TOLERANCE))){
	//first rotation then translation
		Nw=T_kraj/4;//T_kraj/4;//T_kraj/8;//T_kraj/4;//T_kraj/2;//Nw=N;
		N0=-1;//std::min(2*Nw,(T_kraj-Tw));//6*Nw;//2*Nw;
		Nend=std::min(T_kraj/4,(T_kraj));//tolko prije kraja ce stat// 0; vracas na staro	
	}else{
	Nw=T_kraj/4;//T_kraj/4;//T_kraj/8;//T_kraj/4;//T_kraj/2;//Nw=N;
	N0=std::min(1,(T_kraj-Tw));//std::min(2*Nw,(T_kraj-Tw));//6*Nw;//2*Nw;
	Nend=std::min(T_kraj/4,(T_kraj-Tw-N0));//tolko prije kraja ce stat// 0; vracas na staro
	}
	Mw=T_kraj+1-Nw-N0-Nend; //oduzela jos 2Nw mirovanja naprijed
	Nv=T_kraj/8;//T_kraj/8;//T_kraj/4;//T_kraj/2;//Nv=N;
	Nv0=std::min(6*Nv,(T_kraj-std::max(Tv,Tvy)));//6*Nv;//2*Nv;//
	Mv=T_kraj+1-Nv-Nv0; //oduzela jos 2Nw mirovanja naprijed
	if (w0>W_TOLERANCE){
		wM=std::min((w0+(Mw-1)*DW_MAX*STEP),Nw*DW_MAX*STEP);
		wM=std::min(wM,W_MAX);
//		wM=std::min(wM,(w0+5*DW_MAX*STEP));
		wM2=std::max((w0-(Mw-1)*DW_MAX*STEP),-Nw*DW_MAX*STEP);
		wM2=std::max(wM2,W_MIN);
//		wM2=MAX(wM2,(w0-5*DW_MAX*STEP));
	}else{
		wM=std::max((w0-(Mw-1)*DW_MAX*STEP),-Nw*DW_MAX*STEP);
		wM=std::max(wM,W_MIN);
//		wM=std::max(wM,(w0-5*DW_MAX*STEP));
		wM2=std::min((w0+(Mw-1)*DW_MAX*STEP),Nw*DW_MAX*STEP);
		wM2=std::min(wM2,W_MAX);
//		wM2=std::min(wM2,(w0+5*DW_MAX*STEP));
		if (fabs(w0)<W_TOLERANCE){
//			wM=0.;//still allowing straight line trajectories
		}
	}
#if (DIAGCONSTR==0)
	if ((v0)>V_TOLERANCE){
		vM=std::min((v0+(Mv-1)*DVX_MAX*STEP),Nv*DVX_MAX*STEP);
		vM=std::min(vM,VX_MAX);
		vM2=std::max((v0-(Mv-1)*DVX_MAX*STEP),-Nv*DVX_MAX*STEP);
		vM2=std::max(vM2,VX_MIN);
	}else{
		vM=std::max((v0-(Mv-1)*DVX_MAX*STEP),-Nv*DVX_MAX*STEP);
		vM=std::max(vM,VX_MIN);
		vM2=std::min((v0+(Mv-1)*DVX_MAX*STEP),Nv*DVX_MAX*STEP);
		vM2=std::min(vM2,VX_MAX);
		if (fabs(v0)<V_TOLERANCE){
//			vM=0.;//still allowing only rotating trajectories
//			vM2=0.;
		}
	}	
	if ((vy0)>V_TOLERANCE){
		vyM=std::min((vy0+(Mv-1)*DVY_MAX*STEP),Nv*DVY_MAX*STEP);
		vyM=std::min(vyM,VY_MAX);
		vyM2=std::max((vy0-(Mv-1)*DVY_MAX*STEP),-Nv*DVY_MAX*STEP);
		vyM2=std::max(vyM2,VY_MIN);
	}else{
		vyM=std::max((vy0-(Mv-1)*DVY_MAX*STEP),-Nv*DVY_MAX*STEP);
		vyM=std::max(vyM,VY_MIN);
		vyM2=std::min((vy0+(Mv-1)*DVY_MAX*STEP),Nv*DVY_MAX*STEP);
		vyM2=std::min(vyM2,VY_MAX);
		if (fabs(vy0)<V_TOLERANCE){
//			vyM=0.;//still allowing only rotating trajectories
//			vyM2=0.;
		}
	}
#else	
	if ((v0)>V_TOLERANCE){
		vM=std::min((v0+(Mv-1)*0.5*DVX_MAX*STEP),Nv*0.5*DVX_MAX*STEP);
		vM=std::min(vM,VX_MAX/2.);
		vM2=std::max((v0-(Mv-1)*0.5*DVX_MAX*STEP),-Nv*0.5*DVX_MAX*STEP);
		vM2=std::max(vM2,VX_MIN/2.);
	}else{
		vM=std::max((v0-(Mv-1)*0.5*DVX_MAX*STEP),-Nv*0.5*DVX_MAX*STEP);
		vM=std::max(vM,VX_MIN/2.);
		vM2=std::min((v0+(Mv-1)*0.5*DVX_MAX*STEP),Nv*0.5*DVX_MAX*STEP);
		vM2=std::min(vM2,VX_MAX/2.);
		if (fabs(v0)<V_TOLERANCE){
//			vM=0.;//still allowing only rotating trajectories
//			vM2=0.;
		}
	}	
	if ((vy0)>V_TOLERANCE){
		vyM=std::min((vy0+(Mv-1)*0.5*DVY_MAX*STEP),Nv*0.5*DVY_MAX*STEP);
		vyM=std::min(vyM,VY_MAX/2.);
		vyM2=std::max((vy0-(Mv-1)*0.5*DVY_MAX*STEP),-Nv*0.5*DVY_MAX*STEP);
		vyM2=std::max(vyM2,VY_MIN/2.);
	}else{
		vyM=std::max((vy0-(Mv-1)*0.5*DVY_MAX*STEP),-Nv*0.5*DVY_MAX*STEP);
		vyM=std::max(vyM,VY_MIN/2.);
		vyM2=std::min((vy0+(Mv-1)*0.5*DVY_MAX*STEP),Nv*0.5*DVY_MAX*STEP);
		vyM2=std::min(vyM2,VY_MAX/2.);
		if (fabs(vy0)<V_TOLERANCE){
//			vyM=0.;//still allowing only rotating trajectories
//			vyM2=0.;
		}
	}
#endif	
	//gledamo T_kraj-1, T_kraj i T_kraj+1
	//ako se ne moze zaustaviti do tocke T_kraj koristim flag DYNAMIC_CONSTRAINTS
#if OMNIDRIVE
	if (T_kraj<N+1){
		TB.flag[ni][nj][nk]=DYNAMIC_CONSTRAINTS;
	}else if (T_kraj>N_KL-2){
		TB.flag[ni][nj][nk]=DYNAMIC_CONSTRAINTS;
	}
	if (T_kraj-2<N+1){
		TB_minus2.flag[ni][nj][nk]=DYNAMIC_CONSTRAINTS;
	}else if (T_kraj-2>N_KL-2){
		TB_minus2.flag[ni][nj][nk]=DYNAMIC_CONSTRAINTS;
	}
	if (T_kraj-1<N+1){
		TB_minus.flag[ni][nj][nk]=DYNAMIC_CONSTRAINTS;
	}else if (T_kraj-1>N_KL-2){
		TB_minus.flag[ni][nj][nk]=DYNAMIC_CONSTRAINTS;
	}
	if (T_kraj+1<N+1){
		TB_plus.flag[ni][nj][nk]=DYNAMIC_CONSTRAINTS;
	}else if (T_kraj+1>N_KL-2){
		TB_plus.flag[ni][nj][nk]=DYNAMIC_CONSTRAINTS;
	}
#if ROT
	TB_plus1.flag[ni][nj][nk]=TB_plus.flag[ni][nj][nk];//inicijalizacija
	TB_plus2.flag[ni][nj][nk]=TB_plus.flag[ni][nj][nk];//inicijalizacija
#endif

//provjera bar jedna mora bit clear
	if ((TB_plus.flag[ni][nj][nk]==CLEAR)||(TB_minus.flag[ni][nj][nk]==CLEAR)||(TB_minus2.flag[ni][nj][nk]==CLEAR)||(TB.flag[ni][nj][nk]==CLEAR))
#else
	if (T_kraj<N+1){
		TB.flag[ni][nj]=DYNAMIC_CONSTRAINTS;
	}else if (T_kraj>N_KL-2){
		TB.flag[ni][nj]=DYNAMIC_CONSTRAINTS;
	}
	if (T_kraj-2<N+1){
		TB_minus2.flag[ni][nj]=DYNAMIC_CONSTRAINTS;
	}else if (T_kraj-2>N_KL-2){
		TB_minus2.flag[ni][nj]=DYNAMIC_CONSTRAINTS;
	}
	if (T_kraj-1<N+1){
		TB_minus.flag[ni][nj]=DYNAMIC_CONSTRAINTS;
	}else if (T_kraj-1>N_KL-2){
		TB_minus.flag[ni][nj]=DYNAMIC_CONSTRAINTS;
	}
	if (T_kraj+1<N+1){
		TB_plus.flag[ni][nj]=DYNAMIC_CONSTRAINTS;
	}else if (T_kraj+1>N_KL-2){
		TB_plus.flag[ni][nj]=DYNAMIC_CONSTRAINTS;
	}
#if ROT
	TB_plus1.flag[ni][nj]=TB_plus.flag[ni][nj];//inicijalizacija
	TB_plus2.flag[ni][nj]=TB_plus.flag[ni][nj];//inicijalizacija
#endif
//			TB_plus.flag[ni][nj]=DYNAMIC_CONSTRAINTS;//provjera kak radi bez minus2 trajektorije
//provjera bar jedna mora bit clear
	if ((TB_plus.flag[ni][nj]==CLEAR)||(TB_minus.flag[ni][nj]==CLEAR)||(TB_minus2.flag[ni][nj]==CLEAR)||(TB.flag[ni][nj]==CLEAR))
#endif
	{
	for(i=1;i<N_KL;i++){

		//minus2
#if OMNIDRIVE
		if ((i<=T_kraj-2-N)||(TB_minus2.flag[ni][nj][nk]==CLEAR))//ovaj se upisuje, a poslije dolje se prebrisuje
#else
		if ((i<=T_kraj-2-N)||(TB_minus2.flag[ni][nj]==CLEAR))//ovaj se upisuje, a poslije dolje se prebrisuje
#endif
		{
			if (i>T_kraj-2-N){
				v=v0*(T_kraj-2-i)/N;
				w=w0*(T_kraj-2-i)/N;
				vy=vy0*(T_kraj-2-i)/N;
			}else{
				v=v0; w=w0;
				vy=vy0;
			}
#if ROT
			if (i<=T_kraj-2-N){
				if (upsideVforw){
				if (i>Mw+N0){
					wrot=wM*(T_kraj+1-Nend-i)/Nw;
					wrot2=wM2*(T_kraj+1-Nend-i)/Nw;
					if (i>T_kraj+1-Nend){
						wrot=0;
						wrot2=0;
					}
				}else{
					wrot=w0;
					wrot2=w0;
					if ((i>N0)&&((fabs(w0)>W_TOLERANCE)||1) && (Mw>1)){
						wrot=w0+(wM-w0)*(i-1-N0)/(Mw-1);
						wrot2=w0+(wM2-w0)*(i-1-N0)/(Mw-1);				
					}
				}
				}else{
					wrot=w;	wrot2=w;
				}
				if (upsideVforv){
				if (i>Mv+Nv0){
					vrot=vM*(T_kraj+1-i)/Nv;
					vrot2=vM2*(T_kraj+1-i)/Nv;
				}else{
					vrot=v0;
					vrot2=v0;
					if ((i>Nv0)&&((fabs(v0)>V_TOLERANCE)||1) && (Mv>1)){
						vrot=v0+(vM-v0)*(i-1-Nv0)/(Mv-1);
						vrot2=v0+(vM2-v0)*(i-1-Nv0)/(Mv-1);				
					}
				}
				}else{
				vrot=v; vrot2=v;
				}
				if (upsideVforvy){
				if (i>Mv+Nv0){
					vyrot=vyM*(T_kraj+1-i)/Nv;
					vyrot2=vyM2*(T_kraj+1-i)/Nv;
				}else{
					vyrot=vy0;
					vyrot2=vy0;
					if ((i>Nv0)&&((fabs(vy0)>V_TOLERANCE)||1) && (Mv>1)){
						vyrot=vy0+(vyM-vy0)*(i-1-Nv0)/(Mv-1);
						vyrot2=vy0+(vyM2-vy0)*(i-1-Nv0)/(Mv-1);				
					}
				}
				}else{
				vyrot=vy; vyrot2=vy;
				}
			}
#endif
			if (i>T_kraj-2){
				v=0.;
				w=0.;
				vy=0.;
			}
			KL_minus2.v[i]=v;
			KL_minus2.vy[i]=vy;
			KL_minus2.w[i]=w;
      kin_model_evol(&KL_minus2,i);
			if (i<=T_kraj-2-N){//do ovog trenutka su sve trajektorije jednake
#if OMNIDRIVE
				if ((TB.flag[ni][nj][nk]==CLEAR))
#else
				if ((TB.flag[ni][nj]==CLEAR))
#endif
				{
					KL.v[i]=v;
					KL.vy[i]=vy;
					KL.w[i]=w;
					KL.x[i]=KL_minus2.x[i];
					KL.y[i]=KL_minus2.y[i];
					KL.th[i]=KL_minus2.th[i];
				}
#if OMNIDRIVE
				if ((TB_minus.flag[ni][nj][nk]==CLEAR))
#else
				if ((TB_minus.flag[ni][nj]==CLEAR))
#endif
				{
					KL_minus.v[i]=v;
					KL_minus.vy[i]=vy;
					KL_minus.w[i]=w;
					KL_minus.x[i]=KL_minus2.x[i];
					KL_minus.y[i]=KL_minus2.y[i];
					KL_minus.th[i]=KL_minus2.th[i];
				}
#if OMNIDRIVE
				if ((TB_plus.flag[ni][nj][nk]==CLEAR))
#else
				if ((TB_plus.flag[ni][nj]==CLEAR))
#endif
				{
					KL_plus.v[i]=v;
					KL_plus.vy[i]=vy;
					KL_plus.w[i]=w;
					KL_plus.x[i]=KL_minus2.x[i];
					KL_plus.y[i]=KL_minus2.y[i];
					KL_plus.th[i]=KL_minus2.th[i];
				}
#if ROT
#if OMNIDRIVE
				if ((TB_plus1.flag[ni][nj][nk]==CLEAR))
#else
				if ((TB_plus1.flag[ni][nj]==CLEAR))
#endif
				{
					KL_plus1.v[i]=vrot;
					KL_plus1.vy[i]=vyrot;
					KL_plus1.w[i]=wrot;
		      kin_model_evol(&KL_plus1,i);
				}
#if OMNIDRIVE
				if ((TB_plus2.flag[ni][nj][nk]==CLEAR))
#else
				if ((TB_plus2.flag[ni][nj]==CLEAR))
#endif
				{
					KL_plus2.v[i]=vrot2;
					KL_plus2.vy[i]=vyrot2;
					KL_plus2.w[i]=wrot2;
		      kin_model_evol(&KL_plus2,i);
				}

#endif
			}

		}
		if (i>T_kraj-2-N){//ovi se razlikuju
#if OMNIDRIVE
			if ((TB_minus.flag[ni][nj][nk]==CLEAR))
#else
			if ((TB_minus.flag[ni][nj]==CLEAR))
#endif
			{
				if (i>T_kraj-1-N){
					v=v0*(T_kraj-1-i)/N;
					w=w0*(T_kraj-1-i)/N;
					vy=vy0*(T_kraj-1-i)/N;
				}else{
					v=v0; w=w0;
					vy=vy0;
				}
				if (i>T_kraj-1){
					v=0.;
					w=0.;
					vy=0.;
				}
				KL_minus.v[i]=v;
				KL_minus.vy[i]=vy;
				KL_minus.w[i]=w;
				kin_model_evol(&KL_minus,i);
			}
#if OMNIDRIVE
		if ((TB.flag[ni][nj][nk]==CLEAR))
#else
		if ((TB.flag[ni][nj]==CLEAR))
#endif
		{
			if (i>T_kraj-N){
				v=v0*(T_kraj-i)/N;
				w=w0*(T_kraj-i)/N;
				vy=vy0*(T_kraj-i)/N;;
			}else{
				v=v0; w=w0;
				vy=vy0;
			}
			if (i>T_kraj){
				v=0.;
				w=0.;
				vy=0.;
			}
			KL.v[i]=v;
			KL.vy[i]=vy;
			KL.w[i]=w;
			kin_model_evol(&KL,i);
		}
//		}//zatvoren uvjet za plus, pazi, zakomentiraj to kad koristis normalne plus trajektorije
//		{//otvoren uvjet za plus, pazi, zakomentiraj to kad koristis normalne plus trajektorije
		//plus
#if OMNIDRIVE
		if ((TB_plus.flag[ni][nj][nk]==CLEAR))
#else
		if ((TB_plus.flag[ni][nj]==CLEAR))
#endif
		{
//			if (i>1){
			if (i>T_kraj+1-N){
			  v=v0*(T_kraj+1-i)/N;
			  w=w0*(T_kraj+1-i)/N;
			  vy=vy0*(T_kraj+1-i)/N;
//			  v=v0*(N+1-i)/N;
//			  w=w0*(N+1-i)/N;
//			  vy=vy0*(N+1-i)/N;
//			  v=v0-signv0*(i-1)*DVX_MAX*STEP;
//			  if (((v<0)&&(signv0>0))||((v>0)&&(signv0<0))) v=0; 
//				w=w0-signw0*(i-1)*DW_MAX*STEP;
//			  if (((w<0)&&(signw0>0))||((w>0)&&(signw0<0))) w=0; 
//				vy=vy0-signvy0*(i-1)*DVY_MAX*STEP;
//			  if (((vy<0)&&(signvy0>0))||((vy>0)&&(signvy0<0))) vy=0; 
			}else{
				v=v0; w=w0;
				vy=vy0;
			}
//			if (i>=N+1){
			if (i>=T_kraj+1){
				v=0.;
				w=0.;
				vy=0.;
			}
			KL_plus.v[i]=v;
			KL_plus.vy[i]=vy;
			KL_plus.w[i]=w;
			kin_model_evol(&KL_plus,i);
		}
#if ROT
#if OMNIDRIVE
				if ((TB_plus1.flag[ni][nj][nk]==CLEAR))
#else
				if ((TB_plus1.flag[ni][nj]==CLEAR))
#endif
		{
			if (i>T_kraj+1-N){
				v=v0*(T_kraj+1-i)/N;
				w=w0*(T_kraj+1-i)/N;
				vy=vy0*(T_kraj+1-i)/N;
			}else{
				v=v0; w=w0;
				vy=vy0;
			}
			if (upsideVforw){
			if (i>Mw+N0){
				wrot=wM*(T_kraj+1-Nend-i)/Nw;
				if (i>T_kraj+1-Nend){
					wrot=0;
				}
			}else{
				wrot=w0;
				if ((i>N0)&&((fabs(w0)>W_TOLERANCE)||1) && (Mw>1)){
					wrot=w0+(wM-w0)*(i-1-N0)/(Mw-1);
				}
			}
			}else{
				wrot=w;
			}
			if (upsideVforv){
				if (i>Mv+Nv0){
					vrot=vM*(T_kraj+1-i)/Nv;
				}else{
					vrot=v0;
					if ((i>Nv0)&&((fabs(v0)>V_TOLERANCE)||1) && (Mv>1)){
						vrot=v0+(vM-v0)*(i-1-Nv0)/(Mv-1);
					}
				}
			}else{
				vrot=v; 
			}
			if (upsideVforvy){
				if (i>Mv+Nv0){
					vyrot=vyM*(T_kraj+1-i)/Nv;
				}else{
					vyrot=vy0;
					if ((i>Nv0)&&((fabs(vy0)>V_TOLERANCE)||1) && (Mv>1)){
						vyrot=vy0+(vyM-vy0)*(i-1-Nv0)/(Mv-1);
					}
				}
			}else{
				vyrot=vy;
			}
			if (i>T_kraj+1){
				v=0.;
				vy=0.;
				wrot=0.;
				vrot=0.;
				vyrot=0.;
			}
			KL_plus1.v[i]=vrot;
			KL_plus1.vy[i]=vyrot;
			KL_plus1.w[i]=wrot;
			kin_model_evol(&KL_plus1,i);
		}
//plus2
#if OMNIDRIVE
				if ((TB_plus2.flag[ni][nj][nk]==CLEAR))
#else
				if ((TB_plus2.flag[ni][nj]==CLEAR))
#endif
		{
			if (i>T_kraj+1-N){
				v=v0*(T_kraj+1-i)/N;
				w=w0*(T_kraj+1-i)/N;
				vy=vy0*(T_kraj+1-i)/N;
			}else{
				v=v0;w=w0;
				vy=vy0;
			}
			if (upsideVforw){
			if (i>Mw+N0){
				wrot2=wM2*(T_kraj+1-Nend-i)/Nw;
				if (i>T_kraj+1-Nend){
					wrot2=0;
				}
			}else{
				wrot2=w0;
				if ((i>N0)&&((fabs(w0)>W_TOLERANCE)||1) && (Mw>1)){
					wrot2=w0+(wM2-w0)*(i-1-N0)/(Mw-1);				
				}
			}
			}else{
				wrot2=w;
			}
			if (upsideVforv){
				if (i>Mv+Nv0){
					vrot2=vM2*(T_kraj+1-i)/Nv;
				}else{
					vrot2=v0;
					if ((i>Nv0)&&((fabs(v0)>V_TOLERANCE)||1) && (Mv>1)){
						vrot2=v0+(vM2-v0)*(i-1-Nv0)/(Mv-1);				
					}
				}
			}else{
				vrot2=v;
			}
			if (upsideVforvy){
				if (i>Mv+Nv0){
					vyrot2=vyM2*(T_kraj+1-i)/Nv;
				}else{
					vyrot2=vy0;
					if ((i>Nv0)&&((fabs(vy0)>V_TOLERANCE)||1) && (Mv>1)){
						vyrot2=vy0+(vyM2-vy0)*(i-1-Nv0)/(Mv-1);				
					}
				}
			}else{
				vyrot2=vy;
			}
			if (i>T_kraj+1){
				v=0.;
				vy=0.;
				wrot2=0.;
				vrot2=0.;
				vyrot2=0.;
			}
			KL_plus2.v[i]=vrot2;
			KL_plus2.vy[i]=vyrot2;
			KL_plus2.w[i]=wrot2;
			kin_model_evol(&KL_plus2,i);
		}
#endif
		}//razlikuju
#if ROT
//provjera ogranicenja
  bool flagplus1=false, flagplus2=false;
#if OMNIDRIVE
				if ((TB_plus1.flag[ni][nj][nk]==CLEAR)) flagplus1=true;
				if ((TB_plus2.flag[ni][nj][nk]==CLEAR)) flagplus2=true;
#else
				if ((TB_plus1.flag[ni][nj]==CLEAR)) flagplus1=true;
				if ((TB_plus2.flag[ni][nj]==CLEAR)) flagplus2=true;
#endif

		if ((flagplus1)&& ((fabs(KL_plus1.v[i-1]-KL_plus1.v[i])>(DVX_MAX*STEP)+V_TOLERANCE) || (fabs(KL_plus1.vy[i-1]-KL_plus1.vy[i])>(DVY_MAX*STEP)+V_TOLERANCE) || (fabs(KL_plus1.w[i-1]-KL_plus1.w[i])>(DW_MAX*STEP)+W_TOLERANCE))) {
			printf("plus1: dynamic constraints at index i=%d, dvx=%f, dvy=%f, dw=%f\n", i,(KL_plus1.v[i-1]-KL_plus1.v[i]), (KL_plus1.vy[i-1]-KL_plus1.vy[i]), (KL_plus1.w[i-1]-KL_plus1.w[i])*RuS);
		}
if (checkdiag){
		if ((flagplus1==CLEAR)&& ((fabs(KL_plus1.v[i-1]-KL_plus1.v[i])+fabs(KL_plus1.vy[i-1]-KL_plus1.vy[i])>DV_MAX*STEP+V_TOLERANCE) || ((fabs(KL_plus1.v[i])+fabs(KL_plus1.vy[i])>V_MAX+V_TOLERANCE))))
		{
			printf("plus1: KINEMATIC_CONSTRAINTS at index i=%d, dvx=%f, dvy=%f, v=%f, vy=%f\n", i,(KL_plus1.v[i-1]-KL_plus1.v[i]), (KL_plus1.vy[i-1]-KL_plus1.vy[i]), KL_plus1.v[i],KL_plus1.vy[i]);
		}
}
		if ((flagplus2) && ((fabs(KL_plus2.v[i-1]-KL_plus2.v[i])>(DVX_MAX*STEP)+V_TOLERANCE) || (fabs(KL_plus2.vy[i-1]-KL_plus2.vy[i])>(DVY_MAX*STEP)+V_TOLERANCE) || (fabs(KL_plus2.w[i-1]-KL_plus2.w[i])>(DW_MAX*STEP)+W_TOLERANCE))) {
			printf("plus2: dynamic constraints at index i=%d, dvx=%f, dvy=%f, dw=%f\n", i,(KL_plus2.v[i-1]-KL_plus2.v[i]), (KL_plus2.vy[i-1]-KL_plus2.vy[i]), (KL_plus2.w[i-1]-KL_plus2.w[i])*RuS);
		}
if (checkdiag){
		if ((flagplus2==CLEAR)&& ((fabs(KL_plus2.v[i-1]-KL_plus2.v[i])+fabs(KL_plus2.vy[i-1]-KL_plus2.vy[i])>DV_MAX*STEP+V_TOLERANCE) || ((fabs(KL_plus2.v[i])+fabs(KL_plus2.vy[i])>V_MAX+V_TOLERANCE))))
		{
			printf("plus2: KINEMATIC_CONSTRAINTS at index i=%d, dvx=%f, dvy=%f, v=%f, vy=%f\n", i,(KL_plus2.v[i-1]-KL_plus2.v[i]), (KL_plus2.vy[i-1]-KL_plus2.vy[i]), KL_plus2.v[i],KL_plus2.vy[i]);
		}
}
#endif

#if OMNIDRIVE
		if ((blizu_cilja)&&(i>1)){//ne radi za costmasku (nece kratit)
			if (i==2){//ova je upisana bez obzira
				dgoal=sqrt((KL_minus2.x[1]-WH->global_goal_workhorse.x)*(KL_minus2.x[1]-WH->global_goal_workhorse.x)+(KL_minus2.y[1]-WH->global_goal_workhorse.y)*(KL_minus2.y[1]-WH->global_goal_workhorse.y));//ovo sluzi samo kao gornja granica
			}
			if ((TB_plus.flag[ni][nj][nk]==CLEAR)){

				dtemp_plus+=(sqrt(KL_plus.v[i-1]*KL_plus.v[i-1]+KL_plus.vy[i-1]*KL_plus.vy[i-1]))*STEP;
				if (dtemp_plus<=dgoal){
					i_temp_plus=i;
				}else{
					TB_plus.flag[ni][nj][nk]=GOAL_CONSTRAINTS;
				}
			}
			if ((TB_minus.flag[ni][nj][nk]==CLEAR)){

				dtemp_minus+=(sqrt(KL_minus.v[i-1]*KL_minus.v[i-1]+KL_minus.vy[i-1]*KL_minus.vy[i-1]))*STEP;
				if (dtemp_minus<=dgoal){
					i_temp_minus=i;
				}else{
					TB_minus.flag[ni][nj][nk]=GOAL_CONSTRAINTS;
				}
			}
			if ((TB_minus2.flag[ni][nj][nk]==CLEAR)){

				dtemp_minus2+=(sqrt(KL_minus2.v[i-1]*KL_minus2.v[i-1]+KL_minus2.vy[i-1]*KL_minus2.vy[i-1]))*STEP;
				if (dtemp_minus2<=dgoal){
					i_temp_minus2=i;
				}else{
					TB_minus2.flag[ni][nj][nk]=GOAL_CONSTRAINTS;
				}
			}
			if ((TB.flag[ni][nj][nk]==CLEAR)){

				dtemp+=(sqrt(KL.v[i-1]*KL.v[i-1]+KL.vy[i-1]*KL.vy[i-1]))*STEP;
				if (dtemp<=dgoal){//sg){
// 				sg=dtemp;
				i_temp=i;
				}else{//zaustavi do i_temp
				if ((i_temp<Md+1)){//|| 1){//s 1 nece kratit
						TB.flag[ni][nj][nk]=GOAL_CONSTRAINTS;
						break;//samo dalje, ide skroz van i tu vise nikad ne ulazi
				}else{
					i_poc=i_temp;
					for (int n=Md; n<=std::max(M+6,N); n++){//gledam prvo za najmanji moguci pomak
						if (i_poc<=n+1){
							TB.flag[ni][nj][nk]=GOAL_CONSTRAINTS;
							break;
						}
						i_end=i_poc-n+N;
						prvi=false;
						drugi=false;
						dtemp=((i_poc-n-1-1)*KL.v[1])*STEP;
						for (i=i_poc-n; i<N_KL; i++){
							dtemp+=(sqrt(KL.v[i-1]*KL.v[i-1]+KL.vy[i-1]*KL.vy[i-1]))*STEP;
							if (i<i_end){
								v=v0*(i_end-1-i)/N;
								w=w0*(i_end-1-i)/N;
								vy=vy0*(i_end-1-i)/N;
							}else{
								v=0.;
								w=0.;
								vy=0.;
							}
							KL.v[i]=v;
							KL.w[i]=w;
							KL.vy[i]=vy;
							kin_model_evol(&KL,i);
							if ((dtemp<=dgoal)){//sg)){//||(hd==0)){
								i_temp=i;
							}else{
								if((n==std::max(M+6,N))){
									TB.flag[ni][nj][nk]=GOAL_CONSTRAINTS;
									break;//brejka petlju po tockama traj.
								}
								break;//trazi sa sljedecim pomakom
							}
							
						}//petlja po tockama trajektorije
						if ((i==N_KL)){
							break;//brejka petlju s pomacima
						}
						
					}//petlja s pomacima
					break;//brejka vanjsku for petlju po i-evima
				}
			}//zaustavi do i_temp
			}
		}
#else
		if ((blizu_cilja)&&(i>1)){//ne radi za costmasku (nece kratit)
// 			j=0;//inicijalizacija
			//za plus i minus odmah zabranjujemo, a samo ova srednja se krati
			//indekse pamtimo i nastavljamo vani
			if (i==2){//ova je upisana bez obzira
				dgoal=sqrt((KL_minus2.x[1]-WH->global_goal_workhorse.x)*(KL_minus2.x[1]-WH->global_goal_workhorse.x)+(KL_minus2.y[1]-WH->global_goal_workhorse.y)*(KL_minus2.y[1]-WH->global_goal_workhorse.y));//ovo sluzi samo kao gornja granica
			}
			if ((TB_plus.flag[ni][nj]==CLEAR)){

				dtemp_plus+=KL_plus.v[i-1]*STEP;//nece bit dobro za omnidrive
				if (dtemp_plus<=dgoal){
					i_temp_plus=i;
				}else{
					TB_plus.flag[ni][nj]=GOAL_CONSTRAINTS;
				}
			}
			if ((TB_minus.flag[ni][nj]==CLEAR)){

				dtemp_minus+=KL_minus.v[i-1]*STEP;
				if (dtemp_minus<=dgoal){
					i_temp_minus=i;
				}else{
					TB_minus.flag[ni][nj]=GOAL_CONSTRAINTS;
				}
			}
			if ((TB_minus2.flag[ni][nj]==CLEAR)){

				dtemp_minus2+=KL_minus2.v[i-1]*STEP;
				if (dtemp_minus2<=dgoal){
					i_temp_minus2=i;
				}else{
					TB_minus2.flag[ni][nj]=GOAL_CONSTRAINTS;
				}
			}
// 			dtemp=((KL.x[i]-WH->global_goal_workhorse.x)*(KL.x[i]-WH->global_goal_workhorse.x)+(KL.y[i]-WH->global_goal_workhorse.y)*(KL.y[i]-WH->global_goal_workhorse.y));
// 			printf("dtemp=%f, sg=%f\n",sqrt(dtemp), sqrt(sg));
			if ((TB.flag[ni][nj]==CLEAR)){

				dtemp+=KL.v[i-1]*STEP;
				if (dtemp<=dgoal){//sg){
// 				sg=dtemp;
				i_temp=i;
				}else{//zaustavi do i_temp
// 				printf("N=%d, s1=%f, s2=%f, Md=%d, M=%d, i_temp=%d\n",N,s1,s2,Md,M,i_temp);
				if ((i_temp<Md+1)){//|| 1){//s 1 nece kratit
// 					if(TB.v[ni]>V_TOLERANCE){//onemoguci ju
// 						printf("onemogucena zbog cilja i_temp=%d\n",i_temp);
						TB.flag[ni][nj]=GOAL_CONSTRAINTS;
// 						TB.obstacle_point_x[ni][nj]=KL.x[i];
// 						TB.obstacle_point_y[ni][nj]=KL.y[i];
// 						TB.ocjena_prohodnost[ni][nj]=0.0;
						break;//samo dalje, ide skroz van i tu vise nikad ne ulazi
// 					}
				}else{
					i_poc=i_temp;
					for (int n=Md; n<=std::max(M+6,N); n++){//gledam prvo za najmanji moguci pomak
						if (i_poc<=n+1){
// 							if(TB.v[ni]>V_TOLERANCE){//onemoguci ju
// 								printf("onemogucena za cilj i_poc=%d i_temp=%d, pomak unazad n=%d\n",i_poc,i_temp,n);
							TB.flag[ni][nj]=GOAL_CONSTRAINTS;
							break;
// 							}
						}
						i_end=i_poc-n+N;
						prvi=false;
						drugi=false;
						dtemp=((i_poc-n-1-1)*KL.v[1])*STEP;
// 						sg=dg;//resetiranje
						for (i=i_poc-n; i<N_KL; i++){
							dtemp+=KL.v[i-1]*STEP;
							if (i<i_end){
								v=v0*(i_end-1-i)/N;
								w=w0*(i_end-1-i)/N;
								vy=0.;
// 								printf("v,w=%f,%f\n",v,w);
							}else{
								v=0.;
								w=0.;
								vy=0.;
							}
							KL.v[i]=v;
							KL.w[i]=w;
							KL.vy[i]=vy;
							kin_model_evol(&KL,i);
// 							dtemp=((KL.x[i]-WH->global_goal_workhorse.x)*(KL.x[i]-WH->global_goal_workhorse.x)+(KL.y[i]-WH->global_goal_workhorse.y)*(KL.y[i]-WH->global_goal_workhorse.y));
// 							GM->mapper_point_temp.x=KL.x[i];
// 							GM->mapper_point_temp.y=KL.y[i];
// 							if(GM->check_point(GM->mapper_point_temp)) {
// 								hd=DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].h_cost_int;
// 							}
// 							printf("dtemp=%f, sg=%f\n",sqrt(dtemp), sqrt(sg));

							if ((dtemp<=dgoal)){//sg)){//||(hd==0)){
// 								sg=dtemp;
								i_temp=i;
							}else{
// 								printf("potreban je veci pomak unazad, n=%d\n",n);
// 								if((TB.v[ni]>V_TOLERANCE)&&(n==MAX(M+6,N))){//onemoguci ju
								if((n==std::max(M+6,N))){
// 									printf("onemogucena za cilj i_temp=%d\n",i_temp);
									TB.flag[ni][nj]=GOAL_CONSTRAINTS;
// 									TB.obstacle_point_x[ni][nj]=KL.x[j];
// 									TB.obstacle_point_y[ni][nj]=KL.y[j];
// 									TB.ocjena_prohodnost[ni][nj]=0.0;
									break;//brejka petlju po tockama traj.
								}
								break;//trazi sa sljedecim pomakom
							}
							
						}//petlja po tockama trajektorije
						if ((i==N_KL)){
// 							printf("dobra je s pomakom n=%d, ni=%d, nj=%d\n",n,ni,nj);
							break;//brejka petlju s pomacima
						}
						
					}//petlja s pomacima
// 					if ((j==N_KL))
					break;//brejka vanjsku for petlju po i-evima
				}
			}//zaustavi do i_temp
			}
		}
#endif
//collision checking - forbid plus and minus, only middle is shortened - if flag for this is set
#if OMNIDRIVE
		if ((i<=T_kraj-2-N)||(TB_minus2.flag[ni][nj][nk]==CLEAR))
#else
		if ((i<=T_kraj-2-N)||(TB_minus2.flag[ni][nj]==CLEAR))
#endif
		{
			GM->mapper_point_temp.x=KL_minus2.x[i];
			GM->mapper_point_temp.y=KL_minus2.y[i];
      //ako je tocka unutar mape
      //stavljam od 2-tocke tek provjeru jer se tek na nju moze utjecati
			if((i>1)&&(GM->check_point(GM->mapper_point_temp))) {
			
#if RECTANGULAR
//------------------------------------------------------------------------------			
   R_point point_ri;
   if(!PL->RealToReal(GM->mapper_point_temp, point_ri, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> point on trajectory is beyond the bounds!\n");
//      return 0;
   }
//real coordinates within a cell
	double t=KL_minus2.th[i];
	while (t>=2*M_PI) t-=2*M_PI;
	while (t<0) t+=2*M_PI;
	Pose checkme((point_ri.x+0.5),(point_ri.y+0.5),t);
//	IntPose checkme(GM->cell_point_temp.x, GM->cell_point_temp.y,cspace->worldToMapTheta(t));
	if ((cspace->checkCollision(checkme))|| ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval))||((ellipse)&&(Fourpointscollide((point_ri.x+0.5),(point_ri.y+0.5),t,KL_minus2.v[i],KL_minus2.vy[i],i))))
//------------------------------------------------------------------------------------		
#else	
				if((DS->IsValid( GM->cell_point_temp.x, GM->cell_point_temp.y )!=1)|| ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval)))
#endif
				{
#if OMNIDRIVE
					TB_minus2.flag[ni][nj][nk]=HAS_OBSTACLE;
					if (i<=T_kraj-2-N){
//#if (ROT==0)
						if ((TB_plus.flag[ni][nj][nk]==CLEAR)){
							TB_plus.flag[ni][nj][nk]=HAS_OBSTACLE;
						}
//#endif
						if ((TB_minus.flag[ni][nj][nk]==CLEAR)){
							TB_minus.flag[ni][nj][nk]=HAS_OBSTACLE;
						}
						if ((TB.flag[ni][nj][nk]==CLEAR)){
							prvi=true;
						}
					}
#else
					TB_minus2.flag[ni][nj]=HAS_OBSTACLE;
					if (i<=T_kraj-2-N){
						if ((TB_plus.flag[ni][nj]==CLEAR)){
							TB_plus.flag[ni][nj]=HAS_OBSTACLE;
						}
						if ((TB_minus.flag[ni][nj]==CLEAR)){
							TB_minus.flag[ni][nj]=HAS_OBSTACLE;
						}
						if ((TB.flag[ni][nj]==CLEAR)){
							prvi=true;
						}
					}
#endif
				}else{

					if (provjeraluka==1){
						polje=GM->cell_point_temp;
						GM->mapper_point_temp.x=KL_minus2.x[i-1];
						GM->mapper_point_temp.y=KL_minus2.y[i-1];
						GM->check_point(GM->mapper_point_temp);//polje predhodne tocke
						if (abs(GM->cell_point_temp.x-polje.x)+abs(GM->cell_point_temp.y-polje.y)==2){
							A11.x=polje.x; A11.y=GM->cell_point_temp.y;
							A22.y=polje.y; A22.x=GM->cell_point_temp.x;
							A1.x=polje.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
							A1.y=GM->cell_point_temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
							A2.y=polje.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
							A2.x=GM->cell_point_temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
						//odrediti male x i y, a i b
							px=fabs(KL_minus2.y[i]-A1.y)-GM->Map_Cell_Size/2;
							pa=fabs(KL_minus2.x[i]-A2.x)-GM->Map_Cell_Size/2;
							py=fabs(KL_minus2.y[i-1]-A2.y)-GM->Map_Cell_Size/2;
							pb=fabs(KL_minus2.x[i-1]-A1.x)-GM->Map_Cell_Size/2;
							
							if (pb/pa*px>py){ //znaci da je kod A2
								A11=A22;
							}
//							if(DS->map[A11.x][A11.y].prepreka_bool){
							if((DS->IsValid( A11.x, A11.y )!=1)|| ((usecostmask)&&(DS->map[A11.x][A11.y].traversal_cost>=occval))){
#if OMNIDRIVE
								TB_minus2.flag[ni][nj][nk]=HAS_OBSTACLE;
								if (i<=T_kraj-2-N){
									if ((TB_plus.flag[ni][nj][nk]==CLEAR)){
										TB_plus.flag[ni][nj][nk]=HAS_OBSTACLE;
									}
									if ((TB.flag[ni][nj][nk]==CLEAR)){
										prvi=true;
									}
									if ((TB_minus.flag[ni][nj][nk]==CLEAR)){
										TB_minus.flag[ni][nj][nk]=HAS_OBSTACLE;
									}
								}
#else
								TB_minus2.flag[ni][nj]=HAS_OBSTACLE;
								if (i<=T_kraj-2-N){
									if ((TB_plus.flag[ni][nj]==CLEAR)){
										TB_plus.flag[ni][nj]=HAS_OBSTACLE;
									}
									if ((TB.flag[ni][nj]==CLEAR)){
										prvi=true;
									}
									if ((TB_minus.flag[ni][nj]==CLEAR)){
										TB_minus.flag[ni][nj]=HAS_OBSTACLE;
									}
								}
#endif
							}else{
								if (i<=T_kraj-2-N){
#if OMNIDRIVE
									if ((TB_plus.flag[ni][nj][nk]==CLEAR)){
										i_temp_plus=i;
									}
									if ((TB.flag[ni][nj][nk]==CLEAR)){
										i_temp=i;
									}
									if ((TB_minus.flag[ni][nj][nk]==CLEAR)){
										i_temp_minus=i;
									}
#else
									if ((TB_plus.flag[ni][nj]==CLEAR)){
										i_temp_plus=i;
									}
									if ((TB.flag[ni][nj]==CLEAR)){
										i_temp=i;
									}
									if ((TB_minus.flag[ni][nj]==CLEAR)){
										i_temp_minus=i;
									}
#endif
								}
								i_temp_minus2=i;
							}
						}else{
							if (i<=T_kraj-2-N){
#if OMNIDRIVE
								if ((TB_plus.flag[ni][nj][nk]==CLEAR)){
									i_temp_plus=i;
								}
								if ((TB.flag[ni][nj][nk]==CLEAR)){
									i_temp=i;
								}
								if ((TB_minus.flag[ni][nj][nk]==CLEAR)){
									i_temp_minus=i;
								}
#else	
								if ((TB_plus.flag[ni][nj]==CLEAR)){
									i_temp_plus=i;
								}
								if ((TB.flag[ni][nj]==CLEAR)){
									i_temp=i;
								}
								if ((TB_minus.flag[ni][nj]==CLEAR)){
									i_temp_minus=i;
								}
#endif
							}
							i_temp_minus2=i;//zapamti slobodnu
						}
					}else{
						if (i<=T_kraj-2-N){
#if OMNIDRIVE
							if ((TB_plus.flag[ni][nj][nk]==CLEAR)){
								i_temp_plus=i;
							}
							if ((TB.flag[ni][nj][nk]==CLEAR)){
								i_temp=i;
							}
							if ((TB_minus.flag[ni][nj][nk]==CLEAR)){
								i_temp_minus=i;
							}
#else
							if ((TB_plus.flag[ni][nj]==CLEAR)){
								i_temp_plus=i;
							}
							if ((TB.flag[ni][nj]==CLEAR)){
								i_temp=i;
							}
							if ((TB_minus.flag[ni][nj]==CLEAR)){
								i_temp_minus=i;
							}
#endif
						}
						i_temp_minus2=i;
					}

				}
			}
		}//minus2 ako je clear bit ce do kraja provjeren i zapisan
		//dalje se razlikuju minus, obicni i plus
#if OMNIDRIVE
		if ((i>T_kraj-2-N)&&(TB_minus.flag[ni][nj][nk]==CLEAR))
#else
		if ((i>T_kraj-2-N)&&(TB_minus.flag[ni][nj]==CLEAR))
#endif
		{
			GM->mapper_point_temp.x=KL_minus.x[i];
			GM->mapper_point_temp.y=KL_minus.y[i];
      //ako je tocka unutar mape
			if((i>1)&&(GM->check_point(GM->mapper_point_temp))) {
#if RECTANGULAR
//------------------------------------------------------------------------------			
   R_point point_ri;
   if(!PL->RealToReal(GM->mapper_point_temp, point_ri, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> point on trajectory is beyond the bounds!\n");
//      return 0;
   }
//real coordinates within a cell
	double t=KL_minus.th[i];
	while (t>=2*M_PI) t-=2*M_PI;
	while (t<0) t+=2*M_PI;
	Pose checkme((point_ri.x+0.5),(point_ri.y+0.5),t);
//	IntPose checkme(GM->cell_point_temp.x, GM->cell_point_temp.y,cspace->worldToMapTheta(t));
	if ((cspace->checkCollision(checkme))|| ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval))||((ellipse)&&(Fourpointscollide((point_ri.x+0.5),(point_ri.y+0.5),t,KL_minus.v[i],KL_minus.vy[i],i))))
//------------------------------------------------------------------------------------		
#else	
				if((DS->IsValid( GM->cell_point_temp.x, GM->cell_point_temp.y )!=1)|| ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval)))
#endif
				{
#if OMNIDRIVE
					TB_minus.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
					TB_minus.flag[ni][nj]=HAS_OBSTACLE;
#endif
				}else{

					if (provjeraluka==1){
						polje=GM->cell_point_temp;
						GM->mapper_point_temp.x=KL_minus.x[i-1];
						GM->mapper_point_temp.y=KL_minus.y[i-1];
						GM->check_point(GM->mapper_point_temp);//polje predhodne tocke
						if (abs(GM->cell_point_temp.x-polje.x)+abs(GM->cell_point_temp.y-polje.y)==2){
							A11.x=polje.x; A11.y=GM->cell_point_temp.y;
							A22.y=polje.y; A22.x=GM->cell_point_temp.x;
							A1.x=polje.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
							A1.y=GM->cell_point_temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
							A2.y=polje.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
							A2.x=GM->cell_point_temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
						//odrediti male x i y, a i b
							px=fabs(KL_minus.y[i]-A1.y)-GM->Map_Cell_Size/2;
							pa=fabs(KL_minus.x[i]-A2.x)-GM->Map_Cell_Size/2;
							py=fabs(KL_minus.y[i-1]-A2.y)-GM->Map_Cell_Size/2;
							pb=fabs(KL_minus.x[i-1]-A1.x)-GM->Map_Cell_Size/2;
							
							if (pb/pa*px>py){ //znaci da je kod A2
								A11=A22;
							}
//							if(DS->map[A11.x][A11.y].prepreka_bool){
							if((DS->IsValid( A11.x, A11.y )!=1)|| ((usecostmask)&&(DS->map[A11.x][A11.y].traversal_cost>=occval))){
#if OMNIDRIVE
								TB_minus.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
								TB_minus.flag[ni][nj]=HAS_OBSTACLE;
#endif
							}else{

								i_temp_minus=i;
							}
						}else{

							i_temp_minus=i;//zapamti slobodnu
						}
					}else{

						i_temp_minus=i;
					}

				}
			}
		}//kraj za minus
#if OMNIDRIVE
		if ((i>T_kraj-2-N)&&(TB_plus.flag[ni][nj][nk]==CLEAR))
#else	
		if ((i>T_kraj-2-N)&&(TB_plus.flag[ni][nj]==CLEAR))
#endif
		{
		//indekse pamtimo i nastavljamo vani
			GM->mapper_point_temp.x=KL_plus.x[i];
			GM->mapper_point_temp.y=KL_plus.y[i];
      //ako je tocka unutar mape
			if((i>1)&&(GM->check_point(GM->mapper_point_temp))) {
#if RECTANGULAR
//------------------------------------------------------------------------------			
   R_point point_ri;
   if(!PL->RealToReal(GM->mapper_point_temp, point_ri, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> point on trajectory is beyond the bounds!\n");
//      return 0;
   }
//real coordinates within a cell
	double t=KL_plus.th[i];
	while (t>=2*M_PI) t-=2*M_PI;
	while (t<0) t+=2*M_PI;
	Pose checkme((point_ri.x+0.5),(point_ri.y+0.5),t);
//	IntPose checkme(GM->cell_point_temp.x, GM->cell_point_temp.y,cspace->worldToMapTheta(t));
	if ((cspace->checkCollision(checkme)) || ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval))||((ellipse)&&(Fourpointscollide((point_ri.x+0.5),(point_ri.y+0.5),t,KL_plus.v[i],KL_plus.vy[i],i))))
//------------------------------------------------------------------------------------		
#else	
				if((DS->IsValid( GM->cell_point_temp.x, GM->cell_point_temp.y )!=1)|| ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval)))
#endif
				{
#if OMNIDRIVE
					TB_plus.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
					TB_plus.flag[ni][nj]=HAS_OBSTACLE;
#endif
				}else{
					//provjera sudara luka s preprekom, samo za tocke u dijagonalnim poljima
					if (provjeraluka==1){
						polje=GM->cell_point_temp;
						GM->mapper_point_temp.x=KL_plus.x[i-1];
						GM->mapper_point_temp.y=KL_plus.y[i-1];
						GM->check_point(GM->mapper_point_temp);//polje predhodne tocke
						if (abs(GM->cell_point_temp.x-polje.x)+abs(GM->cell_point_temp.y-polje.y)==2){
							A11.x=polje.x; A11.y=GM->cell_point_temp.y;
							A22.y=polje.y; A22.x=GM->cell_point_temp.x;
							A1.x=polje.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
							A1.y=GM->cell_point_temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
							A2.y=polje.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
							A2.x=GM->cell_point_temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
						//odrediti male x i y, a i b 
							px=fabs(KL_plus.y[i]-A1.y)-GM->Map_Cell_Size/2;
							pa=fabs(KL_plus.x[i]-A2.x)-GM->Map_Cell_Size/2;
							py=fabs(KL_plus.y[i-1]-A2.y)-GM->Map_Cell_Size/2;
							pb=fabs(KL_plus.x[i-1]-A1.x)-GM->Map_Cell_Size/2;
							
							if (pb/pa*px>py){ //znaci da je kod A2
								A11=A22;
							}
//							if(DS->map[A11.x][A11.y].prepreka_bool){
							if((DS->IsValid( A11.x, A11.y )!=1)|| ((usecostmask)&&(DS->map[A11.x][A11.y].traversal_cost>=occval))){
#if OMNIDRIVE
								TB_plus.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
								TB_plus.flag[ni][nj]=HAS_OBSTACLE;
#endif
							}else{
								i_temp_plus=i;
							}
						}else{
							i_temp_plus=i;//zapamti slobodnu
						}
					}else{
						i_temp_plus=i;
					}
				}
			}
		}//kraj za plus

#if ROT
#if OMNIDRIVE
				if ((TB_plus1.flag[ni][nj][nk]==CLEAR))
#else
				if ((TB_plus1.flag[ni][nj]==CLEAR))
#endif
		{
		//indekse pamtimo i nastavljamo vani
			GM->mapper_point_temp.x=KL_plus1.x[i];
			GM->mapper_point_temp.y=KL_plus1.y[i];
      //ako je tocka unutar mape
			if((i>1)&&(GM->check_point(GM->mapper_point_temp))) {
#if RECTANGULAR
//------------------------------------------------------------------------------			
   R_point point_ri;
   if(!PL->RealToReal(GM->mapper_point_temp, point_ri, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> point on trajectory is beyond the bounds!\n");
//      return 0;
   }
//real coordinates within a cell
	double t=KL_plus1.th[i];
	while (t>=2*M_PI) t-=2*M_PI;
	while (t<0) t+=2*M_PI;
	Pose checkme((point_ri.x+0.5),(point_ri.y+0.5),t);
//	IntPose checkme(GM->cell_point_temp.x, GM->cell_point_temp.y,cspace->worldToMapTheta(t));
	if ((cspace->checkCollision(checkme)) || ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval))||((ellipse)&&(Fourpointscollide((point_ri.x+0.5),(point_ri.y+0.5),t,KL_plus1.v[i],KL_plus1.vy[i],i))))
#else	
				if((DS->IsValid( GM->cell_point_temp.x, GM->cell_point_temp.y )!=1)|| ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval)))
#endif
				{
#if OMNIDRIVE
								TB_plus1.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
								TB_plus1.flag[ni][nj]=HAS_OBSTACLE;
#endif
//					printf("plus1 obstacle ni=%d nj=%d nk=%d i_temp_plus1=%d \n",ni,nj,nk,i_temp_plus1);
				}else{
//provjera sudara luka s preprekom, samo za tocke u dijagonalnim poljima
					if (provjeraluka==1){
						polje=GM->cell_point_temp;
						GM->mapper_point_temp.x=KL_plus1.x[i-1];
						GM->mapper_point_temp.y=KL_plus1.y[i-1];
						GM->check_point(GM->mapper_point_temp);//polje predhodne tocke
						if (abs(GM->cell_point_temp.x-polje.x)+abs(GM->cell_point_temp.y-polje.y)==2){
							A11.x=polje.x; A11.y=GM->cell_point_temp.y;
							A22.y=polje.y; A22.x=GM->cell_point_temp.x;
							A1.x=polje.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
							A1.y=GM->cell_point_temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
							A2.y=polje.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
							A2.x=GM->cell_point_temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
						//odrediti male x i y, a i b
							px=fabs(KL_plus1.y[i]-A1.y)-GM->Map_Cell_Size/2;
							pa=fabs(KL_plus1.x[i]-A2.x)-GM->Map_Cell_Size/2;
							py=fabs(KL_plus1.y[i-1]-A2.y)-GM->Map_Cell_Size/2;
							pb=fabs(KL_plus1.x[i-1]-A1.x)-GM->Map_Cell_Size/2;
							
							if (pb/pa*px>py){ //znaci da je kod A2
								A11=A22;
							}
//							if(DS->map[A11.x][A11.y].prepreka_bool){
							if((DS->IsValid( A11.x, A11.y )!=1)|| ((usecostmask)&&(DS->map[A11.x][A11.y].traversal_cost>=occval))){
#if OMNIDRIVE
								TB_plus1.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
								TB_plus1.flag[ni][nj]=HAS_OBSTACLE;
#endif
							}else{
								i_temp_plus1=i;
							}
						}else{
							i_temp_plus1=i;//zapamti slobodnu
						}
					}else{
						i_temp_plus1=i;
					}
				}
			}
		}//kraj za plus1
#if OMNIDRIVE
				if ((TB_plus2.flag[ni][nj][nk]==CLEAR))
#else
				if ((TB_plus2.flag[ni][nj]==CLEAR))
#endif
		{
		//indekse pamtimo i nastavljamo vani
			GM->mapper_point_temp.x=KL_plus2.x[i];
			GM->mapper_point_temp.y=KL_plus2.y[i];
      //ako je tocka unutar mape
			if((i>1)&&(GM->check_point(GM->mapper_point_temp))) {
#if RECTANGULAR
//------------------------------------------------------------------------------			
   R_point point_ri;
   if(!PL->RealToReal(GM->mapper_point_temp, point_ri, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> point on trajectory is beyond the bounds!\n");
//      return 0;
   }
//real coordinates within a cell
	double t=KL_plus2.th[i];
	while (t>=2*M_PI) t-=2*M_PI;
	while (t<0) t+=2*M_PI;
	Pose checkme((point_ri.x+0.5),(point_ri.y+0.5),t);
	if ((cspace->checkCollision(checkme)) || ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval))||((ellipse)&&(Fourpointscollide((point_ri.x+0.5),(point_ri.y+0.5),t,KL_plus2.v[i],KL_plus2.vy[i],i))))
//------------------------------------------------------------------------------------		
#else	
				if((DS->IsValid( GM->cell_point_temp.x, GM->cell_point_temp.y )!=1)|| ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval)))
#endif
				{
#if OMNIDRIVE
								TB_plus2.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
								TB_plus2.flag[ni][nj]=HAS_OBSTACLE;
#endif
				}else{
//provjera sudara luka s preprekom, samo za tocke u dijagonalnim poljima
					if (provjeraluka==1){
						polje=GM->cell_point_temp;
						GM->mapper_point_temp.x=KL_plus2.x[i-1];
						GM->mapper_point_temp.y=KL_plus2.y[i-1];
						GM->check_point(GM->mapper_point_temp);//polje predhodne tocke
						if (abs(GM->cell_point_temp.x-polje.x)+abs(GM->cell_point_temp.y-polje.y)==2){
							A11.x=polje.x; A11.y=GM->cell_point_temp.y;
							A22.y=polje.y; A22.x=GM->cell_point_temp.x;
							A1.x=polje.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
							A1.y=GM->cell_point_temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
							A2.y=polje.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
							A2.x=GM->cell_point_temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
						//odrediti male x i y, a i b
							px=fabs(KL_plus2.y[i]-A1.y)-GM->Map_Cell_Size/2;
							pa=fabs(KL_plus2.x[i]-A2.x)-GM->Map_Cell_Size/2;
							py=fabs(KL_plus2.y[i-1]-A2.y)-GM->Map_Cell_Size/2;
							pb=fabs(KL_plus2.x[i-1]-A1.x)-GM->Map_Cell_Size/2;
							
							if (pb/pa*px>py){ //znaci da je kod A2
								A11=A22;
							}
//							if(DS->map[A11.x][A11.y].prepreka_bool){
							if((DS->IsValid( A11.x, A11.y )!=1)|| ((usecostmask)&&(DS->map[A11.x][A11.y].traversal_cost>=occval))){
#if OMNIDRIVE
								TB_plus2.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
								TB_plus2.flag[ni][nj]=HAS_OBSTACLE;
#endif
							}else{
								i_temp_plus2=i;
							}
						}else{
							i_temp_plus2=i;//zapamti slobodnu
						}
					}else{
						i_temp_plus2=i;
					}
				}
			}
		}//kraj za plus2
#endif
#if OMNIDRIVE
		if ((TB.flag[ni][nj][nk]==CLEAR))
#else		
		if ((TB.flag[ni][nj]==CLEAR))
#endif
		{
		GM->mapper_point_temp.x=KL.x[i];
		GM->mapper_point_temp.y=KL.y[i];
      //ako je tocka unutar mape
		if((i>1)&&(GM->check_point(GM->mapper_point_temp))) {
			if (i>T_kraj-2-N){
#if RECTANGULAR
//------------------------------------------------------------------------------			
   R_point point_ri;
   if(!PL->RealToReal(GM->mapper_point_temp, point_ri, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> point on trajectory is beyond the bounds!\n");
//      return 0;
   }
//real coordinates within a cell
	double t=KL.th[i];
	while (t>=2*M_PI) t-=2*M_PI;
	while (t<0) t+=2*M_PI;
	Pose checkme((point_ri.x+0.5),(point_ri.y+0.5),t);
	if ((cspace->checkCollision(checkme)) || ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval))||((ellipse)&&(Fourpointscollide((point_ri.x+0.5),(point_ri.y+0.5),t,KL.v[i],KL.vy[i],i))))
//------------------------------------------------------------------------------------		
#else	
				if((DS->IsValid( GM->cell_point_temp.x, GM->cell_point_temp.y )!=1)|| ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval)))
#endif
				{
				if (prvi==false){
					prvi=true;
				}else {
					drugi=true;
				}
			}else{
				if (prvi){
					printf("samo jedna tocka zauzeta na trajektoriji\n");
				}
				prvi=false;//ako je odmah sljedeca slobodna resetirat ce prvi
				drugi=false;
				if (provjeraluka==1){
					polje=GM->cell_point_temp;
					GM->mapper_point_temp.x=KL.x[i-1];
					GM->mapper_point_temp.y=KL.y[i-1];
					GM->check_point(GM->mapper_point_temp);//polje predhodne tocke
					if (abs(GM->cell_point_temp.x-polje.x)+abs(GM->cell_point_temp.y-polje.y)==2){
						A11.x=polje.x; A11.y=GM->cell_point_temp.y;
						A22.y=polje.y; A22.x=GM->cell_point_temp.x;
						A1.x=polje.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
						A1.y=GM->cell_point_temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
						A2.y=polje.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
						A2.x=GM->cell_point_temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
						//odrediti male x i y, a i b
						px=fabs(KL.y[i]-A1.y)-GM->Map_Cell_Size/2;
						pa=fabs(KL.x[i]-A2.x)-GM->Map_Cell_Size/2;
						py=fabs(KL.y[i-1]-A2.y)-GM->Map_Cell_Size/2;
						pb=fabs(KL.x[i-1]-A1.x)-GM->Map_Cell_Size/2;
							
						if (pb/pa*px>py){ //znaci da je kod A2
							A11=A22;
						}
//						if(DS->map[A11.x][A11.y].prepreka_bool){
						if((DS->IsValid( A11.x, A11.y )!=1)|| ((usecostmask)&&(DS->map[A11.x][A11.y].traversal_cost>=occval))){
							prvi=true;
						}else{
							i_temp=i;//zapamti slobodnu
						}
					}else{
						i_temp=i;//zapamti slobodnu
					}
				}else{
					i_temp=i;//zapamti slobodnu
				}

			}
			}
			if (bez_kracenja==1){
				if(prvi){
#if OMNIDRIVE
					TB.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
					TB.flag[ni][nj]=HAS_OBSTACLE;
#endif
				}else{
					i_temp=i;
				}
			}
			if (prvi && bez_kracenja==0){ //(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].prepreka_bool){
// 				j=0;//inicijalizacija
// 				printf("N=%d, s1=%f, s2=%f, Md=%d, M=%d, i_temp=%d\n",N,s1,s2,Md,M,i_temp);
				if (i_temp<Md+1){//indeks zadnje slobodne tocke i ne stignemo kociti ni s najmanjim pomakom ranije
// 					if(TB.v[ni]>V_TOLERANCE){//onemoguci ju
// 						printf("onemogucena i_temp=%d\n",i_temp);
#if OMNIDRIVE
						TB.flag[ni][nj][nk]=HAS_OBSTACLE;
						TB.obstacle_point_x[ni][nj][nk]=KL.x[i];
						TB.obstacle_point_y[ni][nj][nk]=KL.y[i];
#else
						TB.flag[ni][nj]=HAS_OBSTACLE;
						TB.obstacle_point_x[ni][nj]=KL.x[i];
						TB.obstacle_point_y[ni][nj]=KL.y[i];
#endif
// 						TB.ocjena_prohodnost[ni][nj]=0.0;
						break;//samo dalje, ide skroz van i tu vise nikad ne ulazi
// 					}
				}else{
					i_poc=i_temp;
					for (int n=Md; n<=std::max(M+6,N); n++){//gledam prvo za najmanji moguci pomak
						if (i_poc<=n+1){//indeks zadnje slobodne tocke i ne stignemo kociti ni s najmanjim pomakom ranije
// 							if(TB.v[ni]>V_TOLERANCE){//onemoguci ju
// 								printf("onemogucena i_temp=%d, pomak unazad n=%d\n",i_temp,n);
#if OMNIDRIVE
							TB.flag[ni][nj][nk]=HAS_OBSTACLE;
							TB.obstacle_point_x[ni][nj][nk]=KL.x[i];
							TB.obstacle_point_y[ni][nj][nk]=KL.y[i];
							TB.ocjena_prohodnost[ni][nj][nk]=0.0;
#else
							TB.flag[ni][nj]=HAS_OBSTACLE;
							TB.obstacle_point_x[ni][nj]=KL.x[i];
							TB.obstacle_point_y[ni][nj]=KL.y[i];
							TB.ocjena_prohodnost[ni][nj]=0.0;
#endif
							break;
// 							}
						}
						i_end=i_poc-n+N;
						prvi=false;
						drugi=false;
						for (i=i_poc-n; i<N_KL; i++){
							if (i<i_end){
								v=v0*(i_end-1-i)/N;
								w=w0*(i_end-1-i)/N;
// 								printf("v,w=%f,%f\n",v,w);
								vy=vy0*(i_end-1-i)/N;
							}else{
								v=0.;
								w=0.;
								vy=0.;
							}
							KL.v[i]=v;
							KL.vy[i]=vy;
							KL.w[i]=w;
							if (i==1 && ni==0 && nj==0){//mislim da treba bit i_poc<=n+1
								printf("KL.v[1]=%f\n",KL.v[1]);
							}
							kin_model_evol(&KL,i);
							GM->mapper_point_temp.x=KL.x[i];
							GM->mapper_point_temp.y=KL.y[i];
      //ako je tocka unutar mape
							if((GM->check_point(GM->mapper_point_temp))) {
//								if(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].prepreka_bool){
								if((DS->IsValid( GM->cell_point_temp.x, GM->cell_point_temp.y )!=1)|| ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval))){
									if (prvi==false){
										prvi=true;
									}else {
										drugi=true;
									}
								}else{
									if (prvi){
										printf("samo jedna tocka zauzeta na trajektoriji\n");
									}
									prvi=false;//ako je odmah sljedeca slobodna resetirat ce prvi
									drugi=false;
									if (provjeraluka==1){
										polje=GM->cell_point_temp;
										GM->mapper_point_temp.x=KL.x[i-1];
										GM->mapper_point_temp.y=KL.y[i-1];
										GM->check_point(GM->mapper_point_temp);//polje predhodne tocke
										if (abs(GM->cell_point_temp.x-polje.x)+abs(GM->cell_point_temp.y-polje.y)==2){
										A11.x=polje.x; A11.y=GM->cell_point_temp.y;
										A22.y=polje.y; A22.x=GM->cell_point_temp.x;
										A1.x=polje.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
										A1.y=GM->cell_point_temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
										A2.y=polje.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
										A2.x=GM->cell_point_temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
						//odrediti male x i y, a i b
										px=fabs(KL.y[i]-A1.y)-GM->Map_Cell_Size/2;
										pa=fabs(KL.x[i]-A2.x)-GM->Map_Cell_Size/2;
										py=fabs(KL.y[i-1]-A2.y)-GM->Map_Cell_Size/2;
										pb=fabs(KL.x[i-1]-A1.x)-GM->Map_Cell_Size/2;
							
										if (pb/pa*px>py){ //znaci da je kod A2
										A11=A22;
										}
//										if(DS->map[A11.x][A11.y].prepreka_bool){
										if((DS->IsValid( A11.x, A11.y )!=1)|| ((usecostmask)&&(DS->map[A11.x][A11.y].traversal_cost>=occval))){
										prvi=true;
										}else{
										i_temp=i;//zapamti slobodnu
										}
										}else{
										i_temp=i;//zapamti slobodnu
										}
									}else{
										i_temp=i;//zapamti slobodnu
									}
								}
								if(prvi){
// 									printf("potreban je veci pomak unazad, n=%d\n",n);
// 									if((TB.v[ni]>V_TOLERANCE)&&(n==MAX(M+6,N))){//onemoguci ju
									if((n==std::max(M+6,N))){
// 									printf("onemogucena i_temp=%d\n",i_temp);
#if OMNIDRIVE
										TB.flag[ni][nj][nk]=HAS_OBSTACLE;
										TB.obstacle_point_x[ni][nj][nk]=KL.x[i];
										TB.obstacle_point_y[ni][nj][nk]=KL.y[i];
#else
										TB.flag[ni][nj]=HAS_OBSTACLE;
										TB.obstacle_point_x[ni][nj]=KL.x[i];
										TB.obstacle_point_y[ni][nj]=KL.y[i];
#endif
// 										TB.ocjena_prohodnost[ni][nj]=0.0;
										break;//tu se upisuje ostatak tocaka
									}
									break;
								}
							}
						}//po tockama trajektorije
						if ((i==N_KL)){
// 							printf("dobra je s pomakom n=%d\n",n);
							break;
						}
						
					}//po pomacima
// 					if ((j==N_KL))
					break;//brejka vanjsku for petlju po i-evima
				}
			}//prepreka
		}//checkpoint
		}//if CLEAR
	}
	}//ako nijedna nije CLEAR
// 	if (1)
#if OMNIDRIVE
	if (TB.flag[ni][nj][nk]!=CLEAR)
#else
	if (TB.flag[ni][nj]!=CLEAR)
#endif
	{
		for(i=i_temp+1+1;i<N_KL;i++){
			KL.x[i]=KL.x[i-1];
			KL.y[i]=KL.y[i-1];
			KL.th[i]=KL.th[i-1];
			KL.v[i]=0.;
			KL.vy[i]=0.;
			KL.w[i]=0.;
		}
	}
#if ROT
#if OMNIDRIVE
				if ((TB_plus1.flag[ni][nj][nk]!=CLEAR))
#else
				if ((TB_plus1.flag[ni][nj]!=CLEAR))
#endif
	{
		for(i=i_temp_plus1+1+1;i<N_KL;i++){
			KL_plus1.x[i]=KL_plus1.x[i-1];
			KL_plus1.y[i]=KL_plus1.y[i-1];
			KL_plus1.th[i]=KL_plus1.th[i-1];
			KL_plus1.v[i]=0.;
			KL_plus1.vy[i]=0.;
			KL_plus1.w[i]=0.;
		}
	}
#if OMNIDRIVE
				if ((TB_plus2.flag[ni][nj][nk]!=CLEAR))
#else
				if ((TB_plus2.flag[ni][nj]!=CLEAR))
#endif
	{
		for(i=i_temp_plus2+1+1;i<N_KL;i++){
			KL_plus2.x[i]=KL_plus2.x[i-1];
			KL_plus2.y[i]=KL_plus2.y[i-1];
			KL_plus2.th[i]=KL_plus2.th[i-1];
			KL_plus2.v[i]=0.;
			KL_plus2.vy[i]=0.;
			KL_plus2.w[i]=0.;
		}
	}
#endif
#if OMNIDRIVE
	if (TB_plus.flag[ni][nj][nk]!=CLEAR)
#else
	if (TB_plus.flag[ni][nj]!=CLEAR)
#endif
	{
		for(i=i_temp_plus+1+1;i<N_KL;i++){
			KL_plus.x[i]=KL_plus.x[i-1];
			KL_plus.y[i]=KL_plus.y[i-1];
			KL_plus.th[i]=KL_plus.th[i-1];
			KL_plus.v[i]=0.;
			KL_plus.vy[i]=0.;
			KL_plus.w[i]=0.;
		}
	}else if (i_temp_plus<N_KL-1){//zato sto sam budala koristila za kracenje isti indeks i i breakove, ne znam zas
		dtemp_plus=0.;
		for(i=i_temp_plus+1;i<N_KL;i++){
#if OMNIDRIVE
			if ((TB_plus.flag[ni][nj][nk]==CLEAR))
#else
			if ((TB_plus.flag[ni][nj]==CLEAR))
#endif
			{
				if (i>T_kraj+1-N){
					v=v0*(T_kraj+1-i)/N;
					w=w0*(T_kraj+1-i)/N;
					vy=vy0*(T_kraj+1-i)/N;
				}else{
					v=v0; w=w0;
					vy=vy0;
				}
				if (i>T_kraj+1){
					v=0.;
					w=0.;
					vy=0.;
				}
				KL_plus.v[i]=v;
				KL_plus.vy[i]=v;
				KL_plus.w[i]=w;
				kin_model_evol(&KL_plus,i);
#if OMNIDRIVE
				dtemp_plus+=sqrt(KL_plus.v[i-1]*KL_plus.v[i-1]+KL_plus.vy[i-1]*KL_plus.vy[i-1])*STEP;
				if ((dtemp_plus>dgoal)&&(blizu_cilja)){

					TB_plus.flag[ni][nj][nk]=GOAL_CONSTRAINTS;
				}
#else
				dtemp_plus+=KL_plus.v[i-1]*STEP;
				if ((dtemp_plus>dgoal)&&(blizu_cilja)){

					TB_plus.flag[ni][nj]=GOAL_CONSTRAINTS;
				}
#endif
				GM->mapper_point_temp.x=KL_plus.x[i];
				GM->mapper_point_temp.y=KL_plus.y[i];
      //ako je tocka unutar mape
				if((GM->check_point(GM->mapper_point_temp))) {
#if RECTANGULAR
//------------------------------------------------------------------------------			
   R_point point_ri;
   if(!PL->RealToReal(GM->mapper_point_temp, point_ri, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> point on trajectory is beyond the bounds!\n");
//      return 0;
   }
//real coordinates within a cell
	double t=KL_plus.th[i];
	while (t>=2*M_PI) t-=2*M_PI;
	while (t<0) t+=2*M_PI;
	Pose checkme((point_ri.x+0.5),(point_ri.y+0.5),t);
	if ((cspace->checkCollision(checkme)) || ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval))||((ellipse)&&(Fourpointscollide((point_ri.x+0.5),(point_ri.y+0.5),t,KL_plus.v[i],KL_plus.vy[i],i))))
//------------------------------------------------------------------------------------		
#else	
				if((DS->IsValid( GM->cell_point_temp.x, GM->cell_point_temp.y )!=1)|| ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval)))
#endif
				{
#if OMNIDRIVE
						TB_plus.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
						TB_plus.flag[ni][nj]=HAS_OBSTACLE;
#endif
					}else{
						if (provjeraluka==1){
							polje=GM->cell_point_temp;
							GM->mapper_point_temp.x=KL_plus.x[i-1];
							GM->mapper_point_temp.y=KL_plus.y[i-1];
							GM->check_point(GM->mapper_point_temp);//polje predhodne tocke
							if (abs(GM->cell_point_temp.x-polje.x)+abs(GM->cell_point_temp.y-polje.y)==2){
								A11.x=polje.x; A11.y=GM->cell_point_temp.y;
								A22.y=polje.y; A22.x=GM->cell_point_temp.x;
								A1.x=polje.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
								A1.y=GM->cell_point_temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
								A2.y=polje.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
								A2.x=GM->cell_point_temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
						//odrediti male x i y, a i b
								px=fabs(KL_plus.y[i]-A1.y)-GM->Map_Cell_Size/2;
								pa=fabs(KL_plus.x[i]-A2.x)-GM->Map_Cell_Size/2;
								py=fabs(KL_plus.y[i-1]-A2.y)-GM->Map_Cell_Size/2;
								pb=fabs(KL_plus.x[i-1]-A1.x)-GM->Map_Cell_Size/2;
							
								if (pb/pa*px>py){ //znaci da je kod A2
									A11=A22;
								}
//								if(DS->map[A11.x][A11.y].prepreka_bool){
								if((DS->IsValid( A11.x, A11.y )!=1)|| ((usecostmask)&&(DS->map[A11.x][A11.y].traversal_cost>=occval))){
#if OMNIDRIVE
									TB_plus.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
									TB_plus.flag[ni][nj]=HAS_OBSTACLE;
#endif
								}
							}
						}
					}

				}
			}else{
				KL_plus.x[i]=KL_plus.x[i-1];
				KL_plus.y[i]=KL_plus.y[i-1];
				KL_plus.th[i]=KL_plus.th[i-1];
				KL_plus.v[i]=0.;
				KL_plus.vy[i]=0.;
				KL_plus.w[i]=0.;
			}

		}
	}
#if OMNIDRIVE
	if (TB_minus.flag[ni][nj][nk]!=CLEAR){
#else
	if (TB_minus.flag[ni][nj]!=CLEAR){
#endif
		for(i=i_temp_minus+1+1;i<N_KL;i++){
			KL_minus.x[i]=KL_minus.x[i-1];
			KL_minus.y[i]=KL_minus.y[i-1];
			KL_minus.th[i]=KL_minus.th[i-1];
			KL_minus.v[i]=0.;
			KL_minus.vy[i]=0.;
			KL_minus.w[i]=0.;
		}
	}else if (i_temp_minus<N_KL-1){
		dtemp_minus=0.;
		for(i=i_temp_minus+1;i<N_KL;i++){
#if OMNIDRIVE
			if ((TB_minus.flag[ni][nj][nk]==CLEAR)){
#else
			if ((TB_minus.flag[ni][nj]==CLEAR)){
#endif
				if (i>T_kraj-1-N){
					v=v0*(T_kraj-1-i)/N;
					w=w0*(T_kraj-1-i)/N;
					vy=v0*(T_kraj-1-i)/N;
				}else{
					v=v0; w=w0;
					vy=vy0;
				}
				if (i>T_kraj-1){
					v=0.;
					w=0.;
					vy=0.;
				}
				KL_minus.v[i]=v;
				KL_minus.vy[i]=vy;
				KL_minus.w[i]=w;
				kin_model_evol(&KL_minus,i);
#if OMNIDRIVE
				dtemp_minus+=sqrt(KL_minus.v[i-1]*KL_minus.v[i-1]+KL_minus.vy[i-1]*KL_minus.vy[i-1])*STEP;
				if ((blizu_cilja)&&(dtemp_minus>dgoal)){
					TB_minus.flag[ni][nj][nk]=GOAL_CONSTRAINTS;
				}
#else
				dtemp_minus+=KL_minus.v[i-1]*STEP;
				if ((blizu_cilja)&&(dtemp_minus>dgoal)){
					TB_minus.flag[ni][nj]=GOAL_CONSTRAINTS;
				}
#endif
				GM->mapper_point_temp.x=KL_minus.x[i];
				GM->mapper_point_temp.y=KL_minus.y[i];
      //ako je tocka unutar mape
				if((GM->check_point(GM->mapper_point_temp))) {
#if RECTANGULAR
//------------------------------------------------------------------------------			
   R_point point_ri;
   if(!PL->RealToReal(GM->mapper_point_temp, point_ri, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> point on trajectory is beyond the bounds!\n");
//      return 0;
   }
//real coordinates within a cell
	double t=KL_minus.th[i];
	while (t>=2*M_PI) t-=2*M_PI;
	while (t<0) t+=2*M_PI;
	Pose checkme((point_ri.x+0.5),(point_ri.y+0.5),t);
	if ((cspace->checkCollision(checkme)) || ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval))||((ellipse)&&(Fourpointscollide((point_ri.x+0.5),(point_ri.y+0.5),t,KL_minus.v[i],KL_minus.vy[i],i))))
//------------------------------------------------------------------------------------		
#else	
				if((DS->IsValid( GM->cell_point_temp.x, GM->cell_point_temp.y )!=1)|| ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval)))
#endif
				{
#if OMNIDRIVE
						TB_minus.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
						TB_minus.flag[ni][nj]=HAS_OBSTACLE;
#endif
					}else{
						if (provjeraluka==1){
							polje=GM->cell_point_temp;
							GM->mapper_point_temp.x=KL_minus.x[i-1];
							GM->mapper_point_temp.y=KL_minus.y[i-1];
							GM->check_point(GM->mapper_point_temp);//polje predhodne tocke
							if (abs(GM->cell_point_temp.x-polje.x)+abs(GM->cell_point_temp.y-polje.y)==2){
								A11.x=polje.x; A11.y=GM->cell_point_temp.y;
								A22.y=polje.y; A22.x=GM->cell_point_temp.x;
								A1.x=polje.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
								A1.y=GM->cell_point_temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
								A2.y=polje.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
								A2.x=GM->cell_point_temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
						//odrediti male x i y, a i b
								px=fabs(KL_minus.y[i]-A1.y)-GM->Map_Cell_Size/2;
								pa=fabs(KL_minus.x[i]-A2.x)-GM->Map_Cell_Size/2;
								py=fabs(KL_minus.y[i-1]-A2.y)-GM->Map_Cell_Size/2;
								pb=fabs(KL_minus.x[i-1]-A1.x)-GM->Map_Cell_Size/2;
							
								if (pb/pa*px>py){ //znaci da je kod A2
									A11=A22;
								}
//								if(DS->map[A11.x][A11.y].prepreka_bool){
								if((DS->IsValid( A11.x, A11.y )!=1)|| ((usecostmask)&&(DS->map[A11.x][A11.y].traversal_cost>=occval))){
#if OMNIDRIVE
									TB_minus.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
									TB_minus.flag[ni][nj]=HAS_OBSTACLE;
#endif
								}
							}
						}
					}
				}
			}else{
				KL_minus.x[i]=KL_minus.x[i-1];
				KL_minus.y[i]=KL_minus.y[i-1];
				KL_minus.th[i]=KL_minus.th[i-1];
				KL_minus.v[i]=0.;
				KL_minus.vy[i]=0.;
				KL_minus.w[i]=0.;
			}

		}
	}
#if OMNIDRIVE
	if (TB_minus2.flag[ni][nj][nk]!=CLEAR){
#else
	if (TB_minus2.flag[ni][nj]!=CLEAR){
#endif
		for(i=i_temp_minus2+1+1;i<N_KL;i++){
			KL_minus2.x[i]=KL_minus2.x[i-1];
			KL_minus2.y[i]=KL_minus2.y[i-1];
			KL_minus2.th[i]=KL_minus2.th[i-1];
			KL_minus2.v[i]=0.;
			KL_minus2.vy[i]=0.;
			KL_minus2.w[i]=0.;
		}
	}else if (i_temp_minus2<N_KL-1){
		dtemp_minus2=0.;
		for(i=i_temp_minus2+1;i<N_KL;i++){
#if OMNIDRIVE
			if ((TB_minus2.flag[ni][nj][nk]==CLEAR)){
#else
			if ((TB_minus2.flag[ni][nj]==CLEAR)){
#endif
				if (i>T_kraj-2-N){
					v=v0*(T_kraj-2-i)/N;
					w=w0*(T_kraj-2-i)/N;
					vy=vy0*(T_kraj-2-i)/N;
				}else{
					v=v0; w=w0;
					vy=vy0;
				}
				if (i>T_kraj-2){
					v=0.;
					w=0.;
					vy=0.;
				}
				KL_minus2.v[i]=v;
				KL_minus2.vy[i]=vy;
				KL_minus2.w[i]=w;
				kin_model_evol(&KL_minus2,i);
#if OMNIDRIVE
				dtemp_minus2+=sqrt(KL_minus2.v[i-1]*KL_minus2.v[i-1]+KL_minus2.vy[i-1]*KL_minus2.vy[i-1])*STEP;
				if ((blizu_cilja)&&(dtemp_minus2>dgoal)){
					TB_minus2.flag[ni][nj][nk]=GOAL_CONSTRAINTS;
				}
#else
				dtemp_minus2+=KL_minus2.v[i-1]*STEP;
				if ((blizu_cilja)&&(dtemp_minus2>dgoal)){
					TB_minus2.flag[ni][nj]=GOAL_CONSTRAINTS;
				}
#endif
				GM->mapper_point_temp.x=KL_minus2.x[i];
				GM->mapper_point_temp.y=KL_minus2.y[i];
      //ako je tocka unutar mape
				if((GM->check_point(GM->mapper_point_temp))) {
#if RECTANGULAR
//------------------------------------------------------------------------------			
   R_point point_ri;
   if(!PL->RealToReal(GM->mapper_point_temp, point_ri, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> point on trajectory is beyond the bounds!\n");
//      return 0;
   }
//real coordinates within a cell
	double t=KL_minus2.th[i];
	while (t>=2*M_PI) t-=2*M_PI;
	while (t<0) t+=2*M_PI;
	Pose checkme((point_ri.x+0.5),(point_ri.y+0.5),t);
	if ((cspace->checkCollision(checkme)) || ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval))||((ellipse)&&(Fourpointscollide((point_ri.x+0.5),(point_ri.y+0.5),t,KL_minus2.v[i],KL_minus2.vy[i],i))))
//------------------------------------------------------------------------------------		
#else	
				if((DS->IsValid( GM->cell_point_temp.x, GM->cell_point_temp.y )!=1)|| ((usecostmask)&&(DS->map[GM->cell_point_temp.x][GM->cell_point_temp.y].traversal_cost>=occval)))
#endif
				{
#if OMNIDRIVE
						TB_minus2.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
						TB_minus2.flag[ni][nj]=HAS_OBSTACLE;
#endif
					}else{
						if (provjeraluka==1){
							polje=GM->cell_point_temp;
							GM->mapper_point_temp.x=KL_minus2.x[i-1];
							GM->mapper_point_temp.y=KL_minus2.y[i-1];
							GM->check_point(GM->mapper_point_temp);//polje predhodne tocke
							if (abs(GM->cell_point_temp.x-polje.x)+abs(GM->cell_point_temp.y-polje.y)==2){
								A11.x=polje.x; A11.y=GM->cell_point_temp.y;
								A22.y=polje.y; A22.x=GM->cell_point_temp.x;
								A1.x=polje.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
								A1.y=GM->cell_point_temp.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
								A2.y=polje.y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2;
								A2.x=GM->cell_point_temp.x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2;
						//odrediti male x i y, a i b
								px=fabs(KL_minus2.y[i]-A1.y)-GM->Map_Cell_Size/2;
								pa=fabs(KL_minus2.x[i]-A2.x)-GM->Map_Cell_Size/2;
								py=fabs(KL_minus2.y[i-1]-A2.y)-GM->Map_Cell_Size/2;
								pb=fabs(KL_minus2.x[i-1]-A1.x)-GM->Map_Cell_Size/2;
							
								if (pb/pa*px>py){ //znaci da je kod A2
									A11=A22;
								}
//								if(DS->map[A11.x][A11.y].prepreka_bool){
								if((DS->IsValid( A11.x, A11.y )!=1)|| ((usecostmask)&&(DS->map[A11.x][A11.y].traversal_cost>=occval))){
#if OMNIDRIVE
									TB_minus2.flag[ni][nj][nk]=HAS_OBSTACLE;
#else
									TB_minus2.flag[ni][nj]=HAS_OBSTACLE;
#endif
								}
							}
						}
					}
				}
			}else{
				KL_minus2.x[i]=KL_minus2.x[i-1];
				KL_minus2.y[i]=KL_minus2.y[i-1];
				KL_minus2.th[i]=KL_minus2.th[i-1];
				KL_minus2.v[i]=0.;
				KL_minus2.vy[i]=0.;
				KL_minus2.w[i]=0.;
			}

		}
	}

}


void DynamicWindow::LogMisc(int state)
{
	int i;
	switch(state){
		case 0:
            //AZURIRANJE kruznog luka s indeksima ni, nj i breakage puta, odredjen sa ss
			double si;
			for(i=0;i<N_KL;i++){
#if OMNIDRIVE
				Log.xkl[i][ni][nj][nk]=KL.x[i];
				Log.ykl[i][ni][nj][nk]=KL.y[i];
				Log.th[i][ni][nj][nk]=KL.th[i];
				Log.wkl[i][ni][nj][nk]=KL.w[i];
				Log.vkl[i][ni][nj][nk]=KL.v[i];
				Log.vykl[i][ni][nj][nk]=KL.vy[i];
				Log.Skl[i][ni][nj][nk]=KL.S[i];
				
				Log_plus.xkl[i][ni][nj][nk]=KL_plus.x[i];
				Log_plus.ykl[i][ni][nj][nk]=KL_plus.y[i];
				Log_plus.th[i][ni][nj][nk]=KL_plus.th[i];
				Log_plus.wkl[i][ni][nj][nk]=KL_plus.w[i];
				Log_plus.vkl[i][ni][nj][nk]=KL_plus.v[i];
				Log_plus.vykl[i][ni][nj][nk]=KL_plus.vy[i];
				Log_plus.Skl[i][ni][nj][nk]=KL_plus.S[i];

#if ROT
				Log_plus1.xkl[i][ni][nj][nk]=KL_plus1.x[i];
				Log_plus1.ykl[i][ni][nj][nk]=KL_plus1.y[i];
				Log_plus1.th[i][ni][nj][nk]=KL_plus1.th[i];
				Log_plus1.wkl[i][ni][nj][nk]=KL_plus1.w[i];
				Log_plus1.vkl[i][ni][nj][nk]=KL_plus1.v[i];
				Log_plus1.vykl[i][ni][nj][nk]=KL_plus1.vy[i];
				Log_plus1.Skl[i][ni][nj][nk]=KL_plus1.S[i];

				Log_plus2.xkl[i][ni][nj][nk]=KL_plus2.x[i];
				Log_plus2.ykl[i][ni][nj][nk]=KL_plus2.y[i];
				Log_plus2.th[i][ni][nj][nk]=KL_plus2.th[i];
				Log_plus2.wkl[i][ni][nj][nk]=KL_plus2.w[i];
				Log_plus2.vkl[i][ni][nj][nk]=KL_plus2.v[i];
				Log_plus2.vykl[i][ni][nj][nk]=KL_plus2.vy[i];
				Log_plus2.Skl[i][ni][nj][nk]=KL_plus2.S[i];
#endif
				
				Log_minus.xkl[i][ni][nj][nk]=KL_minus.x[i];
				Log_minus.ykl[i][ni][nj][nk]=KL_minus.y[i];
				Log_minus.th[i][ni][nj][nk]=KL_minus.th[i];
				Log_minus.wkl[i][ni][nj][nk]=KL_minus.w[i];
				Log_minus.vkl[i][ni][nj][nk]=KL_minus.v[i];
				Log_minus.vykl[i][ni][nj][nk]=KL_minus.vy[i];
				Log_minus.Skl[i][ni][nj][nk]=KL_minus.S[i];
				
				Log_minus2.xkl[i][ni][nj][nk]=KL_minus2.x[i];
				Log_minus2.ykl[i][ni][nj][nk]=KL_minus2.y[i];
				Log_minus2.th[i][ni][nj][nk]=KL_minus2.th[i];
				Log_minus2.wkl[i][ni][nj][nk]=KL_minus2.w[i];
				Log_minus2.vkl[i][ni][nj][nk]=KL_minus2.v[i];
				Log_minus2.vykl[i][ni][nj][nk]=KL_minus2.vy[i];
				Log_minus2.Skl[i][ni][nj][nk]=KL_minus2.S[i];
#else
				Log.xkl[i][ni][nj]=KL.x[i];
				Log.ykl[i][ni][nj]=KL.y[i];
				Log.th[i][ni][nj]=KL.th[i];
				Log.wkl[i][ni][nj]=KL.w[i];
				Log.vkl[i][ni][nj]=KL.v[i];
				Log.vykl[i][ni][nj]=KL.vy[i];
				Log.Skl[i][ni][nj]=KL.S[i];
				
				Log_plus.xkl[i][ni][nj]=KL_plus.x[i];
				Log_plus.ykl[i][ni][nj]=KL_plus.y[i];
				Log_plus.th[i][ni][nj]=KL_plus.th[i];
				Log_plus.wkl[i][ni][nj]=KL_plus.w[i];
				Log_plus.vkl[i][ni][nj]=KL_plus.v[i];
				Log_plus.vykl[i][ni][nj]=KL_plus.vy[i];
				Log_plus.Skl[i][ni][nj]=KL_plus.S[i];

#if ROT
				Log_plus1.xkl[i][ni][nj]=KL_plus1.x[i];
				Log_plus1.ykl[i][ni][nj]=KL_plus1.y[i];
				Log_plus1.th[i][ni][nj]=KL_plus1.th[i];
				Log_plus1.wkl[i][ni][nj]=KL_plus1.w[i];
				Log_plus1.vkl[i][ni][nj]=KL_plus1.v[i];
				Log_plus1.vykl[i][ni][nj]=KL_plus1.vy[i];
				Log_plus1.Skl[i][ni][nj]=KL_plus1.S[i];

				Log_plus2.xkl[i][ni][nj]=KL_plus2.x[i];
				Log_plus2.ykl[i][ni][nj]=KL_plus2.y[i];
				Log_plus2.th[i][ni][nj]=KL_plus2.th[i];
				Log_plus2.wkl[i][ni][nj]=KL_plus2.w[i];
				Log_plus2.vkl[i][ni][nj]=KL_plus2.v[i];
				Log_plus2.vykl[i][ni][nj]=KL_plus2.vy[i];
				Log_plus2.Skl[i][ni][nj]=KL_plus2.S[i];
#endif
				
				Log_minus.xkl[i][ni][nj]=KL_minus.x[i];
				Log_minus.ykl[i][ni][nj]=KL_minus.y[i];
				Log_minus.th[i][ni][nj]=KL_minus.th[i];
				Log_minus.wkl[i][ni][nj]=KL_minus.w[i];
				Log_minus.vkl[i][ni][nj]=KL_minus.v[i];
				Log_minus.vykl[i][ni][nj]=KL_minus.vy[i];
				Log_minus.Skl[i][ni][nj]=KL_minus.S[i];
				
				Log_minus2.xkl[i][ni][nj]=KL_minus2.x[i];
				Log_minus2.ykl[i][ni][nj]=KL_minus2.y[i];
				Log_minus2.th[i][ni][nj]=KL_minus2.th[i];
				Log_minus2.wkl[i][ni][nj]=KL_minus2.w[i];
				Log_minus2.vkl[i][ni][nj]=KL_minus2.v[i];
				Log_minus2.vykl[i][ni][nj]=KL_minus2.vy[i];
				Log_minus2.Skl[i][ni][nj]=KL_minus2.S[i];

          //prevaljeni put do i-te tocke koji je manji od ss
          //jer za prikaz breakaga puta koristimo vec gotovi kruzni luk za v,w 
				si=TB.v[ni]*((double)i)*STEP;///(N_KL-1)*TMAX;
		        //fii=TB.w[nj]*((double)i)/(N_KL-1)*TMAX;  //"prevaljeni kut do i-te tocke"
            //ako je breakage
				if(si<=ss){
					Log.xkl_breakage[i][ni][nj]=KL.x[i];
					Log.ykl_breakage[i][ni][nj]=KL.y[i];
				}else{
              //napunimo buffer do kraja, fizikalno nerelevantno
					if(i>0){
						Log.xkl_breakage[i][ni][nj]=Log.xkl_breakage[i-1][ni][nj];
						Log.ykl_breakage[i][ni][nj]=Log.ykl_breakage[i-1][ni][nj];
					}
				}
#endif
			}
			break;

  	//cisto resetiranje kruznog luka ako odredjena trajektorija nije dozvoljena
			case 1:  //to ne koristim jer zelim vidjeti koja to trajektorija nije bila dozvoljena
#if 0
			for(i=0;i<N_KL;i++){
				Log.xkl[i][ni][nj]=RB.x;
				Log.ykl[i][ni][nj]=RB.y;
				Log.xkl_breakage[i][ni][nj]=RB.x;
				Log.ykl_breakage[i][ni][nj]=RB.y;
			}
#endif
			break;

		case 2:
   //azuriranje optimalne trajektorije
#if OMNIDRIVE
			if (plus){
				for(i=0;i<N_KL;i++){
					KL.x[i]=Log_plus.xkl[i][ni][nj][nk];
					KL.y[i]=Log_plus.ykl[i][ni][nj][nk];
					KL.th[i]=Log_plus.th[i][ni][nj][nk];
					KL.v[i]=Log_plus.vkl[i][ni][nj][nk];
					KL.vy[i]=Log_plus.vykl[i][ni][nj][nk];
					KL.w[i]=Log_plus.wkl[i][ni][nj][nk];
					KL.S[i]=Log_plus.Skl[i][ni][nj][nk];
				}
			}
#if ROT
			else if (plus1){
				for(i=0;i<N_KL;i++){
					KL.x[i]=Log_plus1.xkl[i][ni][nj][nk];
					KL.y[i]=Log_plus1.ykl[i][ni][nj][nk];
					KL.th[i]=Log_plus1.th[i][ni][nj][nk];
					KL.v[i]=Log_plus1.vkl[i][ni][nj][nk];
					KL.vy[i]=Log_plus1.vykl[i][ni][nj][nk];
					KL.w[i]=Log_plus1.wkl[i][ni][nj][nk];
					KL.S[i]=Log_plus1.Skl[i][ni][nj][nk];
				}
			}else if (plus2){
				for(i=0;i<N_KL;i++){
					KL.x[i]=Log_plus2.xkl[i][ni][nj][nk];
					KL.y[i]=Log_plus2.ykl[i][ni][nj][nk];
					KL.th[i]=Log_plus2.th[i][ni][nj][nk];
					KL.v[i]=Log_plus2.vkl[i][ni][nj][nk];
					KL.vy[i]=Log_plus2.vykl[i][ni][nj][nk];
					KL.w[i]=Log_plus2.wkl[i][ni][nj][nk];
					KL.S[i]=Log_plus2.Skl[i][ni][nj][nk];
				}
			}
#endif
			else if (minus){
				for(i=0;i<N_KL;i++){
					KL.x[i]=Log_minus.xkl[i][ni][nj][nk];
					KL.y[i]=Log_minus.ykl[i][ni][nj][nk];
					KL.th[i]=Log_minus.th[i][ni][nj][nk];
					KL.v[i]=Log_minus.vkl[i][ni][nj][nk];
					KL.vy[i]=Log_minus.vykl[i][ni][nj][nk];
					KL.w[i]=Log_minus.wkl[i][ni][nj][nk];
					KL.S[i]=Log_minus.Skl[i][ni][nj][nk];
				}
			}else if (minus2){
				for(i=0;i<N_KL;i++){
					KL.x[i]=Log_minus2.xkl[i][ni][nj][nk];
					KL.y[i]=Log_minus2.ykl[i][ni][nj][nk];
					KL.th[i]=Log_minus2.th[i][ni][nj][nk];
					KL.v[i]=Log_minus2.vkl[i][ni][nj][nk];
					KL.vy[i]=Log_minus2.vykl[i][ni][nj][nk];
					KL.w[i]=Log_minus2.wkl[i][ni][nj][nk];
					KL.S[i]=Log_minus2.Skl[i][ni][nj][nk];
				}
			}else{
				for(i=0;i<N_KL;i++){
					KL.x[i]=Log.xkl[i][ni][nj][nk];
					KL.y[i]=Log.ykl[i][ni][nj][nk];
					KL.th[i]=Log.th[i][ni][nj][nk];
					KL.v[i]=Log.vkl[i][ni][nj][nk];
					KL.vy[i]=Log.vykl[i][ni][nj][nk];
					KL.w[i]=Log.wkl[i][ni][nj][nk];
					KL.S[i]=Log.Skl[i][ni][nj][nk];
				}
			}
#else
			if (plus){
				for(i=0;i<N_KL;i++){
					KL.x[i]=Log_plus.xkl[i][ni][nj];
					KL.y[i]=Log_plus.ykl[i][ni][nj];
					KL.th[i]=Log_plus.th[i][ni][nj];
					KL.v[i]=Log_plus.vkl[i][ni][nj];
					KL.vy[i]=Log_plus.vykl[i][ni][nj];
					KL.w[i]=Log_plus.wkl[i][ni][nj];
					KL.S[i]=Log_plus.Skl[i][ni][nj];
				}
			}
#if ROT
			else if (plus1){
				for(i=0;i<N_KL;i++){
					KL.x[i]=Log_plus1.xkl[i][ni][nj];
					KL.y[i]=Log_plus1.ykl[i][ni][nj];
					KL.th[i]=Log_plus1.th[i][ni][nj];
					KL.v[i]=Log_plus1.vkl[i][ni][nj];
					KL.vy[i]=Log_plus1.vykl[i][ni][nj];
					KL.w[i]=Log_plus1.wkl[i][ni][nj];
					KL.S[i]=Log_plus1.Skl[i][ni][nj];
				}
			}else if (plus2){
				for(i=0;i<N_KL;i++){
					KL.x[i]=Log_plus2.xkl[i][ni][nj];
					KL.y[i]=Log_plus2.ykl[i][ni][nj];
					KL.th[i]=Log_plus2.th[i][ni][nj];
					KL.v[i]=Log_plus2.vkl[i][ni][nj];
					KL.vy[i]=Log_plus2.vykl[i][ni][nj];
					KL.w[i]=Log_plus2.wkl[i][ni][nj];
					KL.S[i]=Log_plus2.Skl[i][ni][nj];
				}
			}
#endif
			else if (minus){
				for(i=0;i<N_KL;i++){
					KL.x[i]=Log_minus.xkl[i][ni][nj];
					KL.y[i]=Log_minus.ykl[i][ni][nj];
					KL.th[i]=Log_minus.th[i][ni][nj];
					KL.v[i]=Log_minus.vkl[i][ni][nj];
					KL.vy[i]=Log_minus.vykl[i][ni][nj];
					KL.w[i]=Log_minus.wkl[i][ni][nj];
					KL.S[i]=Log_minus.Skl[i][ni][nj];
				}
			}else if (minus2){
				for(i=0;i<N_KL;i++){
					KL.x[i]=Log_minus2.xkl[i][ni][nj];
					KL.y[i]=Log_minus2.ykl[i][ni][nj];
					KL.th[i]=Log_minus2.th[i][ni][nj];
					KL.v[i]=Log_minus2.vkl[i][ni][nj];
					KL.vy[i]=Log_minus2.vykl[i][ni][nj];
					KL.w[i]=Log_minus2.wkl[i][ni][nj];
					KL.S[i]=Log_minus2.Skl[i][ni][nj];
				}
			}else{
				for(i=0;i<N_KL;i++){
					KL.x[i]=Log.xkl[i][ni][nj];
					KL.y[i]=Log.ykl[i][ni][nj];
					KL.th[i]=Log.th[i][ni][nj];
					KL.v[i]=Log.vkl[i][ni][nj];
					KL.vy[i]=Log.vykl[i][ni][nj];
					KL.w[i]=Log.wkl[i][ni][nj];
					KL.S[i]=Log.Skl[i][ni][nj];
				}
			}
#endif
			for(i=0;i<N_KL/DEC_KL_DRAW;i++){
				Log.xkl_optimal[i]=KL.x[i*DEC_KL_DRAW];
				Log.ykl_optimal[i]=KL.y[i*DEC_KL_DRAW];
			}
			Log.xkl_optimal[N_KL/DEC_KL_DRAW]=KL.x[N_KL-1];
			Log.ykl_optimal[N_KL/DEC_KL_DRAW]=KL.y[N_KL-1];
			KL_old_temp=KL;//spremanje stare optimalne trajektorije
#if (IDEAL_MODEL==1)
			double tempS;
#if 1			
			printf("provjera spremanja pomocne stare trajektorije\n");
				for(i=0;i<N_KL;i++){
				  tempS=computeInterpolatedCost(KL_old_temp.x[i],KL_old_temp.y[i],KL_old_temp.th[i]);
			if (KL_old_temp.S[i]!=tempS){
			  printf("provjera prijepisa pomocne stare trajektorije: cijene nisu jednake! KL_old_temp.S[i]=%.15f vs compute=%.15f, i=%d rotateongoal=%d,KL_old_temp.x[i]=%.15f, KL_old_temp.y[i]=%.15f, KL_old_temp.th[i]=%.15f\n",KL_old_temp.S[i],tempS,i,rotateongoal,KL_old_temp.x[i], KL_old_temp.y[i], KL_old_temp.th[i]);
			  }
			  }
#endif
#endif
			flag_kl_old=true;
//provjera ogranicenja
			for (i=1;i<N_KL;i++){
				if ((fabs(KL.v[i-1]-KL.v[i])>(DVX_MAX*STEP)+V_TOLERANCE) || (fabs(KL.vy[i-1]-KL.vy[i])>(DVY_MAX*STEP)+V_TOLERANCE) || (fabs(KL.w[i-1]-KL.w[i])>(DW_MAX*STEP)+W_TOLERANCE) || (KL.v[i]>VX_MAX) || (KL.v[i]<VX_MIN) || (KL.w[i]>W_MAX) || (KL.w[i]<W_MIN)) {
					printf("dynamic constraints at index i=%d, dvx=%f, dvy=%f, dw=%f deg, v=%f, vy=%f, w=%f\n", i,(KL.v[i-1]-KL.v[i]), (KL.vy[i-1]-KL.vy[i]), (KL.w[i-1]-KL.w[i])*RuS, KL.v[i], KL.vy[i], KL.w[i]);
					break;
				}
#if DIAGCONSTR
				if ((fabs(KL.v[i-1]-KL.v[i])>(DVX_MAX*STEP)+V_TOLERANCE) || (fabs(KL.vy[i-1]-KL.vy[i])>(DVY_MAX*STEP)+V_TOLERANCE) || (fabs(KL.w[i-1]-KL.w[i])>(DW_MAX*STEP)+W_TOLERANCE) || (fabs(KL.v[i-1]-KL.v[i])+fabs(KL.vy[i-1]-KL.vy[i])>DV_MAX*STEP+V_TOLERANCE) || ((fabs(KL.v[i])+fabs(KL.vy[i])>V_MAX+V_TOLERANCE))) {
					printf("dynamic constraints at index i=%d, dvx=%f, dvy=%f, dw=%f, v=%f, vy=%f\n", i,(KL.v[i-1]-KL.v[i]), (KL.vy[i-1]-KL.vy[i]), (KL.w[i-1]-KL.w[i])*RuS, KL.v[i], KL.vy[i]);
					break;
				}
#endif
			}
			break;
	
		case 3:
		//azuriranje optimalne trajektorije u slucaju ne odabiranja optimalnog vektora
			for(i=0;i<=N_KL/DEC_KL_DRAW;i++){
				Log.xkl_optimal[i]=RB.x;
				Log.ykl_optimal[i]=RB.y;
			}
			break;
	
		case 4:
			Ispis_polja();
			break;
     
	}
}

void DynamicWindow::Prohodnost(){

	int i,j;
	double si, razmak2, movrazmak, minmovrazmak,pomak_x,pomak_y;
	double v;
// double fii;
	razmak2=0.0; //razmak do svih prepreka ili samo do nepokretnih (ovisno o varijabli UPIS_POKRETNE_U_DSTAR_MAPU)
	movrazmak=0.0; //razmak do pokretne prepreke
	minmovrazmak=(double)LASER_RANGE*LASER_RANGE;

	int broj_prepreka;
	int indeks_prepreke,k_ta_pokretna,indeks_tezista;
	int pokretna,mg;
	pokretna=0;  //zastavica da li je minimalna udaljenost do pokretne prepreke ili ne
	broj_prepreka=laser_pointer_end;
/*	if (broj_prepreka) //kod prazne karte nema laserskih ocitanja, sve su pokretne, osim prvog detektiranja pokretne prepreke
		printf("prvo detektiranje, broj tocaka=%d\n",broj_prepreka);*/
#if OMNIDRIVE
	v=sqrt(TB.v[ni]*TB.v[ni]+TB.vy[nk]*TB.vy[nk]);
#else
	v=TB.v[ni];
#endif
	limit_distance=(RR+SC1+(SC2-SC1)*v/V_MAX)+SC_W*TB.w[nj]/W_MAX; //uracunat security distance
        //limit_distance2=200.;
        //double kutkl, point d;
	for(i=0;i<N_KL;i++){
		minmovrazmak=(double)LASER_RANGE*LASER_RANGE;
	//	if (i>0){
			
//----------------------------------za pokretnu prepreku--------------------------------------------------------
  //ako postoji pokretna prepreka onda se usporedjuju udaljenosti izmedju i-tih tocaka na KL i MO. Vremena su ista na i-tim tockama na obje trajektorije
  //znaci i-tu i j-tu tocku za i!=j nema smisla usporedjivati jer i-ta prepreka postoji samo u i-tom trenutku na tom mjestu
      for (int k=0;k<broj_pokretnih_prepreka;k++){
      //printf("broj_pokretnih_prepreka=%d\n",broj_pokretnih_prepreka);
	movrazmak=((KL.x[i]-MO[k].x[i])*(KL.x[i]-MO[k].x[i]))+((KL.y[i]-MO[k].y[i])*(KL.y[i]-MO[k].y[i]));//tu je bio sqrt
             //kasnije dolazi provjera da li je buduci polozaj pokretne prepreke unutar security radiusa koji je definiran geometrijom robota i trenutnim v,w
		if (movrazmak<minmovrazmak){
			minmovrazmak=movrazmak;
			indeks_prepreke=i;//indeks na KL i na trajektoriji k-te pokretne prepreke MO
			indeks_tezista=PPP[k].indeks;  //o kojem tezistu je rijec, da dodjemo do svih moving tocaka koje cine to teziste
			k_ta_pokretna=k; //o kojoj pokretnoj prepreci se radi u polju pokretnih prepreka PPP
			pokretna=1; //zastavica
			si=v*((double)i)*STEP;///(N_KL-1)*TMAX;   //"prevaljeni put do i-te tocke"
			kut_prepreke=atan2((MO[k].y[i]-RB.y),(MO[k].x[i]-RB.x));//gledamo pod kojim kutem je najbliza prepreka
			udaljenost_prepreke=(MO[k].x[i]-RB.x)*(MO[k].x[i]-RB.x)+(MO[k].y[i]-RB.y)*(MO[k].y[i]-RB.y);
		}
	}
//------------------------------------------------------------------------------------------------------------------
		//za svaku tocku na trajektoriji provjeravamo udaljenosti do svih nepokretnih prepreka, tj. laserskih hitova
	for(j=0;j<broj_prepreka;j++){
		razmak2=((KL.x[i]-OPF[j].x)*(KL.x[i]-OPF[j].x))+((KL.y[i]-OPF[j].y)*(KL.y[i]-OPF[j].y));//tu je bio sqrt
		if (razmak2<minmovrazmak){
			minmovrazmak=razmak2;   //minimalni od pokretnih i statickih prepreka
			pokretna=0;
			si=v*((double)i)*STEP;///(N_KL-1)*TMAX;   //"prevaljeni put do i-te tocke"
			kut_prepreke=atan2((OPF[j].y-RB.y),(OPF[j].x-RB.x));//gledamo pod kojim kutem je najbliza prepreka
			udaljenost_prepreke=(OPF[j].x-RB.x)*(OPF[j].x-RB.x)+(OPF[j].y-RB.y)*(OPF[j].y-RB.y);
		}
	}//zatvorena for petlja po preprekama

		if (minmovrazmak<limit_distance*limit_distance){//sad sam tu stavila na kvadrat
 //hm zapravo je kod pokretnih prepreka ovo sumnjivo jer bi trebalo ukljuciti sudar s pokretnom uvijek - ovako je ako je pokretna prepreka blizu staticke, a staticka je bliza robotu, onda se sudar s pokretnom ne translatira a trebao bi TODO nekako rijesiti
 
			if(pokretna==1){  //znaci da je od pokretne prepreke
				if (indeks_tezista==0){
					mg=0;
				}else if (indeks_tezista>0){
					mg=GM->moving_cell_index[indeks_tezista-1];
				}
				//zelim upisati u dstar mapu ovu translatiranu prepreku
				pomak_x=MO[k_ta_pokretna].x[indeks_prepreke]-PPP[k_ta_pokretna].x;
				pomak_y=MO[k_ta_pokretna].y[indeks_prepreke]-PPP[k_ta_pokretna].y;
				//if (indeks_tezista>0){
// 				printf("DW> Prebacujem %d tocaka u poljima koje cine %d. teziste zbog %d. pokretnog polja po redu cija trajektorija sadrzi sudar u %d. tocki.\n",(GM->moving_cell_index[indeks_tezista]-mg),indeks_tezista,k_ta_pokretna,indeks_prepreke);
				if ((fabs(pomak_x)+fabs(pomak_y))>0.001){
				for (int m=mg;m<GM->moving_cell_index[indeks_tezista];m++){
					GM->mapper_point_temp.x=GM->moving_cell[m].x;
					GM->mapper_point_temp.y=GM->moving_cell[m].y;//zabiljeziti koje indekse ne puniti u dstar mapu, ovi koji se translatiraju, ostale da
					if(GM->check_point(GM->mapper_point_temp)) {
/*						int duplic=0;
						for (int z=0;z<GM->numindeksizanepunjenje;z++){//provjera da li smo vec zapisali taj indeks
							if ((GM->indeksizanepunjenje[z].x==GM->cell_point_temp.x)&&(GM->indeksizanepunjenje[z].y==GM->cell_point_temp.y)){
								duplic=1;
								break;
							}
						}*/
						if ((GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].time_stamp != -1)&&(GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].time_stamp != -2)){
							GM->indeksizanepunjenje[GM->numindeksizanepunjenje].x=GM->cell_point_temp.x;
							GM->indeksizanepunjenje[GM->numindeksizanepunjenje].y=GM->cell_point_temp.y;
							GM->numindeksizanepunjenje++;
							GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].time_stamp = -2;//postavljamo na praznjenje
							GM->indeksimapepraznjenje[GM->numindeksimapepraznjenje]=GM->cell_point_temp;//svi koji su -2 moraju ic u polje za praznjenje jer cemo ih obrisati iz D* mape
							GM->numindeksimapepraznjenje++;//i ta polja se ne smiju puniti ako je na njima prepreka (riskiramo jedan ciklus dok ne postanu -3)
						}
					}
					GM->mapper_point_temp.x=GM->moving_cell[m].x+pomak_x;
					GM->mapper_point_temp.y=GM->moving_cell[m].y+pomak_y;
      //ako je hit unutar mape
					if(GM->check_point(GM->mapper_point_temp)) {
						if (GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].time_stamp != -1) {    //oznaka da samo jednom zabiljezi tu celiju
							GM->col_moving[GM->num_col_points].x=GM->cell_point_temp.x;
							GM->col_moving[GM->num_col_points].y=GM->cell_point_temp.y;
							GM->num_col_points++;
							GM->Map[GM->cell_point_temp.x][GM->cell_point_temp.y].time_stamp = -1;
							for (int z=0;z<GM->numindeksimapepraznjenje;z++){//provjera da li je novi col point bio namjesten u planeru kao stari ili ovdje gore
								if ((GM->indeksimapepraznjenje[z].x==GM->cell_point_temp.x)&&(GM->indeksimapepraznjenje[z].y==GM->cell_point_temp.y)){
									GM->indeksimapepraznjenje[z]=GM->indeksimapepraznjenje[--GM->numindeksimapepraznjenje];//brise ga iz polja
									break;
								}
							}
							GM->indeksimapepunjenje[GM->numindeksimapepunjenje].x=GM->cell_point_temp.x;   //azuriram indekse za planera
							GM->indeksimapepunjenje[GM->numindeksimapepunjenje].y=GM->cell_point_temp.y;
							GM->numindeksimapepunjenje++;
//printf("DW> teziste (%d), pokretno polje (%d), tocka na trajektoriji (%d) od 30, shift za pomak (%.1f,%.1f) na mjesto sudara (%d,%d), num_col_points=%d\n",indeks_tezista,k_ta_pokretna,indeks_prepreke,pomak_x,pomak_y,GM->cell_point_temp.x,GM->cell_point_temp.y,GM->num_col_points);
						}
					}
				}
				}
				//}
			}
	//tu di je si je prepreka nadjena ako je si manji od mimalnog zaustavnog puta na toj trajektoriji, onda tu trajektoriju moramo onemoguciti
			ts=fabs(v)/DV_MAX; //smije rotirati u mjestu pa je uvijek zaustavljanje po trans brz

			//if(si<ss){
			if (ts>i*STEP){
				if(v>V_TOLERANCE){//ovo dodajem
#if OMNIDRIVE
					TB.flag[ni][nj][nk]=NON_ADMISSIBLE;
				TB.obstacle_point_x[ni][nj][nk]=KL.x[i];
				TB.obstacle_point_y[ni][nj][nk]=KL.y[i];
				TB.ocjena_prohodnost[ni][nj][nk]=0.0;
#else
					TB.flag[ni][nj]=NON_ADMISSIBLE;
				TB.obstacle_point_x[ni][nj]=KL.x[i];
				TB.obstacle_point_y[ni][nj]=KL.y[i];
				TB.ocjena_prohodnost[ni][nj]=0.0;
#endif
				return;
				}else{
	     	//dodajem SINGULARNi SLUCAJ kada je translatorna brzina 0
		//smije se vrtiti na mjestu, dajem mu flag has_obstacle
#if OMNIDRIVE
					TB.ocjena_prohodnost[ni][nj][nk]=0.0; 
 					TB.flag[ni][nj][nk]=HAS_OBSTACLE;   
#else
					TB.ocjena_prohodnost[ni][nj]=0.0; 
 					TB.flag[ni][nj]=HAS_OBSTACLE;   
#endif
//					TB.flag[ni][nj]=NON_ADMISSIBLE;
					return;
				}

	
			}else{
             //HAS_OBSTACLE             
             
	     //PAZNJA - ovdje ne koristimo udaljenost, vec vrijeme do kolizije
	     //kao mjerilo, sto je bolja varijanta, jer uzima direktno brzinu 
	     //robota u obzir
				if(v>V_TOLERANCE){
#if OMNIDRIVE
					TB.obstacle_point_x[ni][nj][nk]=KL.x[i];
					TB.obstacle_point_y[ni][nj][nk]=KL.y[i];
					TB.ocjena_prohodnost[ni][nj][nk]=i*STEP;
					TB.flag[ni][nj][nk]=HAS_OBSTACLE; //azuriranje flaga
#else
					TB.obstacle_point_x[ni][nj]=KL.x[i];
					TB.obstacle_point_y[ni][nj]=KL.y[i];
					TB.ocjena_prohodnost[ni][nj]=i*STEP;
					TB.flag[ni][nj]=HAS_OBSTACLE; //azuriranje flaga
#endif
					return; 
				}else{
	     	//SINGULARNO SLUCAJ kada je translatorna brzina prakticki 0
		//dakle kada se vrtimo na mjestu!!!
#if OMNIDRIVE
					TB.ocjena_prohodnost[ni][nj][nk]=0.0;//ovo vrati na 0 ak bu ne stimalo 
					TB.flag[ni][nj][nk]=HAS_OBSTACLE;   
#else
					TB.ocjena_prohodnost[ni][nj]=0.0;//ovo vrati na 0 ak bu ne stimalo 
					TB.flag[ni][nj]=HAS_OBSTACLE;   
#endif
					return; 
				}
			}//od else
		}   //limit distance
	} //prema svim tockama luka
    //AKO JE PUTANJA CLEAR
	if(v>V_TOLERANCE){
#if OMNIDRIVE
		TB.ocjena_prohodnost[ni][nj][nk]=1.0;      //ovo ispod je da vece brzine imaju veci doprinos za prohodnost
           //TB.ocjena_prohodnost[ni][nj][nk]=1.0+TB.v[ni]/V_MAX;//+minrazmak/LASER_RANGE; 
		TB.flag[ni][nj][nk]=CLEAR;
#else
		TB.ocjena_prohodnost[ni][nj]=1.0;      //ovo ispod je da vece brzine imaju veci doprinos za prohodnost
           //TB.ocjena_prohodnost[ni][nj]=1.0+TB.v[ni]/V_MAX;//+minrazmak/LASER_RANGE; 
		TB.flag[ni][nj]=CLEAR;
#endif
	}else{
    	//ovdje stavljamo ocjenu 0 na singularitet, jer zapravo
	//tu ne mozemo nista zakljuciti s obzirom na konfiguraciju prepreka
#if OMNIDRIVE
		TB.ocjena_prohodnost[ni][nj][nk]=0.0;//stavila iz 0.0 u 1.0 zbog okretanja u krivu stranu
		TB.flag[ni][nj][nk]=CLEAR;
#else
		TB.ocjena_prohodnost[ni][nj]=0.0;//stavila iz 0.0 u 1.0 zbog okretanja u krivu stranu
		TB.flag[ni][nj]=CLEAR;
#endif
	}
	return;
}


double DynamicWindow::PathAlignment()
{
	double path_alignment=0.0;
	double distance_temp;
#if (STARASUMA==1)
#if (PROMJENA_KRITERIJA==0)
   for(int i=0;i<N_KL;i++){
            distance_temp=0.0;
          //usporedba sa pravcem
           for(int j=0; j<N_PATH_REDUCED;j++){
               distance_temp+=(j+1)*sqrt((path_r_reduced[j].x-KL.x[i])*(path_r_reduced[j].x-KL.x[i])+(path_r_reduced[j].y-KL.y[i])*(path_r_reduced[j].y-KL.y[i]));
          }
        path_alignment+=distance_temp;
   }
#else
   for(int i=0;i<N_KL;i++){
          //usporedba sa pravcem
		   distance_temp=((path_r_reduced[i].x-KL.x[i])*(path_r_reduced[i].x-KL.x[i])+(path_r_reduced[i].y-KL.y[i])*(path_r_reduced[i].y-KL.y[i]));
	   path_alignment+=distance_temp;
   }
#endif
#else
   double path_alignment_minus=0., path_alignment_minus2=0., path_alignment_plus=0.;
#if ROT
   double path_alignment_plus1=0., path_alignment_plus2=0., Cplus_kut_plus1, Cplus_kut_plus2, E_kut_plus1, E_kut_plus2, TB_plus1flag, TB_plus2flag;
#endif
   int svi_uvjeti,kriterij,dodatni_kut,TBflag,TB_plusflag,TB_minusflag,TB_minus2flag;
   double sumvel=0.,sumvelplus=0., sumvelplus1=0., sumvelplus2=0., sumvelminus=0., sumvelminus2=0.;
   double Cplus_kut, E_kut, Cplus_kut_minus, E_kut_minus, Cplus_kut_minus2, E_kut_minus2, Cplus_kut_plus, E_kut_plus, epsilon, rho;
   double goal_angle, goal_angle_minus, goal_angle_minus2, goal_angle_plus, goal_angle_plus1, goal_angle_plus2;
#if OMNIDRIVE
     TBflag=TB.flag[ni][nj][nk];
     TB_plusflag=TB_plus.flag[ni][nj][nk];
#if ROT
     TB_plus1flag=TB_plus1.flag[ni][nj][nk];
     TB_plus2flag=TB_plus2.flag[ni][nj][nk];
#endif
     TB_minusflag=TB_minus.flag[ni][nj][nk];
     TB_minus2flag=TB_minus2.flag[ni][nj][nk];
#else
     TBflag=TB.flag[ni][nj];
     TB_plusflag=TB_plus.flag[ni][nj];
#if ROT
     TB_plus1flag=TB_plus1.flag[ni][nj];
     TB_plus2flag=TB_plus2.flag[ni][nj];
#endif
     TB_minusflag=TB_minus.flag[ni][nj];
     TB_minus2flag=TB_minus2.flag[ni][nj];
#endif
   for(int i=0;i<N_KL;i++){
	   if (TBflag==CLEAR)
	   {
	     if ((i>0) && (KL.x[i-1]-KL.x[i])==0 && (KL.y[i-1]-KL.y[i])==0 && (KL.th[i-1]-KL.th[i])==0){ //mora ovako zbog kasnjenja u kin modelu
	        KL.S[i]=KL.S[i-1];
	     }else{
		   KL.S[i]=computeInterpolatedCost(KL.x[i],KL.y[i],KL.th[i]);
//		   if (KL.S[i]>=OBSTACLE) TBflag=HAS_OBSTACLE;
		   }
//		    if (KL.S[i]!=computeInterpolatedCost(KL.x[i],KL.y[i],KL.th[i])){
//          printf("provjera prijepisa: nije jednako za KL trajektoriju\n");
//        }
		   //also determine global R_point E, C_plus
		   if (i==1){
			   kut_E=atan2((E.y-KL.y[1]),(E.x-KL.x[1]))-KL.th[1];
			   if (kut_E<-1*M_PI) kut_E+=2*M_PI;
			   if (kut_E>M_PI) kut_E-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
//					   printf("Odredio kut_E=%f deg, tocka E=(%f,%f) mm, distance_temp=%f\n",kut_E*RuS,E.x,E.y,distance_temp);
		   }
		   if (i==N_KL-1){
			   Cplus_kut=1-cos(KL.th[i]-atan2((C_plus.y-KL.y[i]),(C_plus.x-KL.x[i])));
			   E_kut=1-cos(KL.th[i]-atan2((E.y-KL.y[i]),(E.x-KL.x[i])));
			   goal_angle=1-cos(KL.th[i]-desiredOrientation);
		   }
		   if (i>0)
		   	path_alignment+=KL.S[i];
//			   	path_alignment+=(distance_temp);//+0.1*epsilon;
		   if (i>0){
		   	sumvel+=GAINV*fabs(KL.vy[i])+GAINV*fabs(KL.v[i])+fabs(KL.w[i]);
		   }
	   }
	   if (TB_plusflag==CLEAR)
	   {
	   //plus
	     if ((i>0) && (KL_plus.x[i-1]-KL_plus.x[i])==0 && (KL_plus.y[i-1]-KL_plus.y[i])==0 && (KL_plus.th[i-1]-KL_plus.th[i])==0){
	        KL_plus.S[i]=KL_plus.S[i-1];
	     }else{
		   KL_plus.S[i]=computeInterpolatedCost(KL_plus.x[i],KL_plus.y[i],KL_plus.th[i]);
//		   if (KL_plus.S[i]>=OBSTACLE) TB_plusflag=HAS_OBSTACLE;
        }
//        if (KL_plus.S[i]!=computeInterpolatedCost(KL_plus.x[i],KL_plus.y[i],KL_plus.th[i])){
//          printf("provjera prijepisa: nije jednako za plus trajektoriju\n");
//        }
		   if (i==1){
			   kut_E=atan2((E.y-KL_plus.y[1]),(E.x-KL_plus.x[1]))-KL_plus.th[1];
			   if (kut_E<-1*M_PI) kut_E+=2*M_PI;
			   if (kut_E>M_PI) kut_E-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
		   }
		   if (i==N_KL-1){
			   Cplus_kut_plus=1-cos(KL_plus.th[i]-atan2((C_plus.y-KL_plus.y[i]),(C_plus.x-KL_plus.x[i])));
			   E_kut_plus=1-cos(KL_plus.th[i]-atan2((E.y-KL_plus.y[i]),(E.x-KL_plus.x[i])));
			   goal_angle_plus=1-cos(KL_plus.th[i]-desiredOrientation);
		   }
		   if (i>0)
		   	path_alignment_plus+=KL_plus.S[i];//path_alignment_plus+=(distance_temp_plus);//+0.1*epsilon;
		   if (i>0){
		   	sumvelplus+=GAINV*fabs(KL_plus.vy[i])+GAINV*fabs(KL_plus.v[i])+fabs(KL_plus.w[i]);
		   }
	   }
#if ROT
	   if (TB_plus1flag==CLEAR)
	   {
	     if ((i>0) && (KL_plus1.x[i-1]-KL_plus1.x[i])==0 && (KL_plus1.y[i-1]-KL_plus1.y[i])==0 && (KL_plus1.th[i-1]-KL_plus1.th[i])==0){
	        KL_plus1.S[i]=KL_plus1.S[i-1];
	     }else{
		   KL_plus1.S[i]=computeInterpolatedCost(KL_plus1.x[i],KL_plus1.y[i],KL_plus1.th[i]);
//		   if (KL_plus1.S[i]>=OBSTACLE) TB_plus1flag=HAS_OBSTACLE;
        }
		   //also determine global R_point E, C_plus
		   if (i==1){
			   kut_E=atan2((E.y-KL_plus1.y[1]),(E.x-KL_plus1.x[1]))-KL_plus1.th[1];
			   if (kut_E<-1*M_PI) kut_E+=2*M_PI;
			   if (kut_E>M_PI) kut_E-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
		   }
		   if (i==N_KL-1){
				   					   //odredjivanje E tocke
			   Cplus_kut_plus1=1-cos(KL_plus1.th[i]-atan2((C_plus.y-KL_plus1.y[i]),(C_plus.x-KL_plus1.x[i])));
			   E_kut_plus1=1-cos(KL_plus1.th[i]-atan2((E.y-KL_plus1.y[i]),(E.x-KL_plus1.x[i])));
			   goal_angle_plus1=1-cos(KL_plus1.th[i]-desiredOrientation);
		   }
		   if (i>0)
		   	path_alignment_plus1+=KL_plus1.S[i];//path_alignment_plus+=(distance_temp_plus);//+0.1*epsilon;
		   if (i>0){
		   	sumvelplus1+=GAINV*fabs(KL_plus1.vy[i])+GAINV*fabs(KL_plus1.v[i])+fabs(KL_plus1.w[i]);
		   }
	   }
//plus2
	   if (TB_plus2flag==CLEAR)
	   {
	   //plus
	     if ((i>0) && (KL_plus2.x[i-1]-KL_plus2.x[i])==0 && (KL_plus2.y[i-1]-KL_plus2.y[i])==0 && (KL_plus2.th[i-1]-KL_plus2.th[i])==0){
	        KL_plus2.S[i]=KL_plus2.S[i-1];
	     }else{
		   KL_plus2.S[i]=computeInterpolatedCost(KL_plus2.x[i],KL_plus2.y[i],KL_plus2.th[i]);
//		   if (KL_plus2.S[i]>=OBSTACLE) TB_plus2flag=HAS_OBSTACLE;
        }
		   //also determine global R_point E, C_plus
		   if (i==1){
			   kut_E=atan2((E.y-KL_plus2.y[1]),(E.x-KL_plus2.x[1]))-KL_plus2.th[1];
			   if (kut_E<-1*M_PI) kut_E+=2*M_PI;
			   if (kut_E>M_PI) kut_E-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
		   }
		   if (i==N_KL-1){
				   					   //odredjivanje E tocke
			   Cplus_kut_plus2=1-cos(KL_plus2.th[i]-atan2((C_plus.y-KL_plus2.y[i]),(C_plus.x-KL_plus2.x[i])));
			   E_kut_plus2=1-cos(KL_plus2.th[i]-atan2((E.y-KL_plus2.y[i]),(E.x-KL_plus2.x[i])));
			   goal_angle_plus2=1-cos(KL_plus2.th[i]-desiredOrientation);

		   }
		   if (i>0)
		   	path_alignment_plus2+=KL_plus2.S[i];//path_alignment_plus+=(distance_temp_plus);//+0.1*epsilon;
		   if (i>0){
		   	sumvelplus2+=GAINV*fabs(KL_plus2.vy[i])+GAINV*fabs(KL_plus2.v[i])+fabs(KL_plus2.w[i]);
		   }
	   }
#endif
	   if (TB_minusflag==CLEAR){
	   //minus
	     if ((i>0) && (KL_minus.x[i-1]-KL_minus.x[i])==0 && (KL_minus.y[i-1]-KL_minus.y[i])==0 && (KL_minus.th[i-1]-KL_minus.th[i])==0){
	        KL_minus.S[i]=KL_minus.S[i-1];
	     }else{
		   KL_minus.S[i]=computeInterpolatedCost(KL_minus.x[i],KL_minus.y[i],KL_minus.th[i]);
//		   if (KL_minus.S[i]>=OBSTACLE) TB_minusflag=HAS_OBSTACLE;
      }
//        if (KL_minus.S[i]!=computeInterpolatedCost(KL_minus.x[i],KL_minus.y[i],KL_minus.th[i])){
//          printf("provjera prijepisa: nije jednako za minus trajektoriju\n");
//        }
		   //also determine global R_point E, C_plus
		   if (i==1){
			   kut_E=atan2((E.y-KL_minus.y[1]),(E.x-KL_minus.x[1]))-KL_minus.th[1];
			   if (kut_E<-1*M_PI) kut_E+=2*M_PI;
			   if (kut_E>M_PI) kut_E-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
//					   printf("Odredio kut_E=%f deg, tocka E=(%f,%f) mm, distance_temp=%f\n",kut_E*RuS,E.x,E.y,distance_temp);
		   }
		   if (i==N_KL-1){
				   					   //odredjivanje E tocke
			   Cplus_kut_minus=1-cos(KL_minus.th[i]-atan2((C_plus.y-KL_minus.y[i]),(C_plus.x-KL_minus.x[i])));
			   E_kut_minus=1-cos(KL_minus.th[i]-atan2((E.y-KL_minus.y[i]),(E.x-KL_minus.x[i])));
			   goal_angle_minus=1-cos(KL_minus.th[i]-desiredOrientation);
		   }
		   if (i>0)
		   	path_alignment_minus+=KL_minus.S[i];//path_alignment_minus+=(distance_temp_minus);//+0.1*epsilon;
		   if (i>0){
		   	sumvelminus+=GAINV*fabs(KL_minus.vy[i])+GAINV*fabs(KL_minus.v[i])+fabs(KL_minus.w[i]);
		   }
	   }
	   if (TB_minus2flag==CLEAR){
	   //minus2
	     if ((i>0) && (KL_minus2.x[i-1]-KL_minus2.x[i])==0 && (KL_minus2.y[i-1]-KL_minus2.y[i])==0 && (KL_minus2.th[i-1]-KL_minus2.th[i])==0){
	        KL_minus2.S[i]=KL_minus2.S[i-1];
	     }else{
		   KL_minus2.S[i]=computeInterpolatedCost(KL_minus2.x[i],KL_minus2.y[i],KL_minus2.th[i]);
//		   if (KL_minus2.S[i]>=OBSTACLE) TB_minus2flag=HAS_OBSTACLE;
        }
//        if (KL_minus2.S[i]!=computeInterpolatedCost(KL_minus2.x[i],KL_minus2.y[i],KL_minus2.th[i])){
//          printf("provjera prijepisa: nije jednako za minus2 trajektoriju\n");
//        }
		   //also determine global R_point E, C_plus
		   if (i==1){
			   kut_E=atan2((E.y-KL_minus2.y[1]),(E.x-KL_minus2.x[1]))-KL_minus2.th[1];
			   if (kut_E<-1*M_PI) kut_E+=2*M_PI;
			   if (kut_E>M_PI) kut_E-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
//					   printf("Odredio kut_E=%f deg, tocka E=(%f,%f) mm, distance_temp=%f\n",kut_E*RuS,E.x,E.y,distance_temp);
		   }
		   if (i==N_KL-1){
				   					   //odredjivanje E tocke
			   Cplus_kut_minus2=1-cos(KL_minus2.th[i]-atan2((C_plus.y-KL_minus2.y[i]),(C_plus.x-KL_minus2.x[i])));
			   E_kut_minus2=1-cos(KL_minus2.th[i]-atan2((E.y-KL_minus2.y[i]),(E.x-KL_minus2.x[i])));
			   goal_angle_minus2=1-cos(KL_minus2.th[i]-desiredOrientation);
		   }
		   if (i>0)
		   	path_alignment_minus2+=KL_minus2.S[i];//path_alignment_minus2+=(distance_temp_minus2);//+0.1*epsilon;
		   if (i>0){
		   	sumvelminus2+=GAINV*fabs(KL_minus2.vy[i])+GAINV*fabs(KL_minus2.v[i])+fabs(KL_minus2.w[i]);
		   }
	   }
   }
#if OMNIDRIVE
     TB.flag[ni][nj][nk]=TBflag;
     TB_plus.flag[ni][nj][nk]=TB_plusflag;
#if ROT
     TB_plus1.flag[ni][nj][nk]=TB_plus1flag;
     TB_plus2.flag[ni][nj][nk]=TB_plus2flag;
#endif
     TB_minus.flag[ni][nj][nk]=TB_minusflag;
     TB_minus2.flag[ni][nj][nk]=TB_minus2flag;
#else
     TB.flag[ni][nj]=TBflag;
     TB_plus.flag[ni][nj]=TB_plusflag;
#if ROT
     TB_plus1.flag[ni][nj]=TB_plus1flag;
     TB_plus2.flag[ni][nj]=TB_plus2flag;
#endif
     TB_minus.flag[ni][nj]=TB_minusflag;
     TB_minus2.flag[ni][nj]=TB_minus2flag;
#endif
 
//    printf("ni=%d, nj=%d, path_alignment=%f, S0=%f, ST=%f\n",ni,nj,path_alignment,S0,ST);
   svi_uvjeti=OBJECTIVE_CONDITIONS;//1-svi S(i)>=S(T); 0-nijedan uvjet, -1-samo S(0)>=S(T)
   kriterij=OBJECTIVE_SUM;//1-suma, 0-S(T)
   dodatni_kut=3;//0-nema dodatnog kuta, 1-prema E, 2-prema Cplus, 3-prema ciljnom kutu
   epsilon=EPSILON;//0.1;//za dodatni_kut 
   rho=RHO;//0.1;//za dodatnu sumu svih brzina stavi 0 u DynamicWindow.h kad hoces bez toga
	if (rotateongoal) rho=0;
   int decreasingslope=0; //za extra uvjet
#if OMNIDRIVE
   if (TB.flag[ni][nj][nk]==CLEAR){
	   if (KL.S[1]<KL.S[N_KL-1] && (svi_uvjeti==-1)){
	   TB.flag[ni][nj][nk]=NON_ADMISSIBLE;
	}
   }
   if (TB_plus.flag[ni][nj][nk]==CLEAR){
	   if (kriterij){
		   TB_plus.ocjena_path[ni][nj][nk]=path_alignment_plus+rho*sumvelplus;
	   }else{
		   TB_plus.ocjena_path[ni][nj][nk]=KL_plus.S[N_KL-1]+rho*sumvelplus;
	   }
	   if (dodatni_kut==1){
			   TB_plus.ocjena_path[ni][nj][nk]+=epsilon*E_kut_plus;
		   }else if (dodatni_kut==2){
			   TB_plus.ocjena_path[ni][nj][nk]+=epsilon*Cplus_kut_plus;
		   }else if (dodatni_kut==3){
			   TB_plus.ocjena_path[ni][nj][nk]+=epsilon*goal_angle_plus;
		   }
	   if (KL_plus.S[1]<KL_plus.S[N_KL-1] && (svi_uvjeti==-1)){
		   TB_plus.flag[ni][nj][nk]=NON_ADMISSIBLE;
	   }
   }
#if ROT
   if (TB_plus1.flag[ni][nj][nk]==CLEAR){
	   if (kriterij){
		   TB_plus1.ocjena_path[ni][nj][nk]=path_alignment_plus1+rho*sumvelplus1;
	   }else{
		   TB_plus1.ocjena_path[ni][nj][nk]=KL_plus1.S[N_KL-1]+rho*sumvelplus1;
	   }
	   if (dodatni_kut==1){
			   TB_plus1.ocjena_path[ni][nj][nk]+=epsilon*E_kut_plus1;
		   }else if (dodatni_kut==2){
			   TB_plus1.ocjena_path[ni][nj][nk]+=epsilon*Cplus_kut_plus1;
		   }else if (dodatni_kut==3){
			   TB_plus1.ocjena_path[ni][nj][nk]+=epsilon*goal_angle_plus1;
		   }
	   if (KL_plus1.S[1]<KL_plus1.S[N_KL-1] && (svi_uvjeti==-1)){
		   TB_plus1.flag[ni][nj][nk]=NON_ADMISSIBLE;
	   }
   }
   if (TB_plus2.flag[ni][nj][nk]==CLEAR){
	   if (kriterij){
		   TB_plus2.ocjena_path[ni][nj][nk]=path_alignment_plus2+rho*sumvelplus2;
	   }else{
		   TB_plus2.ocjena_path[ni][nj][nk]=KL_plus2.S[N_KL-1]+rho*sumvelplus2;
	   }
	   if (dodatni_kut==1){
			   TB_plus2.ocjena_path[ni][nj][nk]+=epsilon*E_kut_plus2;
		   }else if (dodatni_kut==2){
			   TB_plus2.ocjena_path[ni][nj][nk]+=epsilon*Cplus_kut_plus2;
		   }else if (dodatni_kut==3){
			   TB_plus2.ocjena_path[ni][nj][nk]+=epsilon*goal_angle_plus2;
		   }
	   if (KL_plus2.S[1]<KL_plus2.S[N_KL-1] && (svi_uvjeti==-1)){
		   TB_plus2.flag[ni][nj][nk]=NON_ADMISSIBLE;
	   }
   }
#endif
   if (TB_minus.flag[ni][nj][nk]==CLEAR){
	   if (kriterij){
		   TB_minus.ocjena_path[ni][nj][nk]=path_alignment_minus+rho*sumvelminus;
	   }else {
		   TB_minus.ocjena_path[ni][nj][nk]=KL_minus.S[N_KL-1]+rho*sumvelminus;
	   }
	   if (dodatni_kut==1){
			   TB_minus.ocjena_path[ni][nj][nk]+=epsilon*E_kut_minus;
		   }else if (dodatni_kut==2){
			   TB_minus.ocjena_path[ni][nj][nk]+=epsilon*Cplus_kut_minus;
		   }else if (dodatni_kut==3){
			   TB_minus.ocjena_path[ni][nj][nk]+=epsilon*goal_angle_minus;
		   }
	   if (KL_minus.S[1]<KL_minus.S[N_KL-1] && (svi_uvjeti==-1)){
		   TB_minus.flag[ni][nj][nk]=NON_ADMISSIBLE;
	   }
   }
   if (TB_minus2.flag[ni][nj][nk]==CLEAR){
	   if (kriterij){
		   TB_minus2.ocjena_path[ni][nj][nk]=path_alignment_minus2+rho*sumvelminus2;
	   }else {
		   TB_minus2.ocjena_path[ni][nj][nk]=KL_minus2.S[N_KL-1]+rho*sumvelminus2;
	   }
	   if (dodatni_kut==1){
			   TB_minus2.ocjena_path[ni][nj][nk]+=epsilon*E_kut_minus2;
		   }else if (dodatni_kut==2){
			   TB_minus2.ocjena_path[ni][nj][nk]+=epsilon*Cplus_kut_minus2;
		   }else if (dodatni_kut==3){
			   TB_minus2.ocjena_path[ni][nj][nk]+=epsilon*goal_angle_minus2;
		   }
	   if (KL_minus2.S[1]<KL_minus2.S[N_KL-1] && (svi_uvjeti==-1)){
		   TB_minus2.flag[ni][nj][nk]=NON_ADMISSIBLE;
	   }
   }
   if (svi_uvjeti==1){//za dodatni uvjet po svim tockama
	   if(TB.flag[ni][nj][nk]==CLEAR){
		   for(int i=1;i<N_KL-1;i++){//do predzadnjeg
			   if (KL.S[i]<KL.S[N_KL-1]){
				   TB.flag[ni][nj][nk]=NON_ADMISSIBLE;
				   break;
			   }
		   }
	   }
	   if(TB_plus.flag[ni][nj][nk]==CLEAR){
		   for(int i=1;i<N_KL-1;i++){//do predzadnjeg
			   if (KL_plus.S[i]<KL_plus.S[N_KL-1]){
				   TB_plus.flag[ni][nj][nk]=NON_ADMISSIBLE;
				   break;
			   }
		   }
	   }
#if ROT
	   if(TB_plus1.flag[ni][nj][nk]==CLEAR){
		   for(int i=1;i<N_KL-1;i++){//do predzadnjeg
			   if (KL_plus1.S[i]<KL_plus1.S[N_KL-1]){
				   TB_plus1.flag[ni][nj][nk]=NON_ADMISSIBLE;
				   break;
			   }
		   }
	   }
	   if(TB_plus2.flag[ni][nj][nk]==CLEAR){
		   for(int i=1;i<N_KL-1;i++){//do predzadnjeg
			   if (KL_plus2.S[i]<KL_plus2.S[N_KL-1]){
				   TB_plus2.flag[ni][nj][nk]=NON_ADMISSIBLE;
				   break;
			   }
		   }
	   }
#endif
	   if(TB_minus.flag[ni][nj][nk]==CLEAR){
		   for(int i=1;i<N_KL-1;i++){//do predzadnjeg
			   if (KL_minus.S[i]<KL_minus.S[N_KL-1]){
				   TB_minus.flag[ni][nj][nk]=NON_ADMISSIBLE;
				   break;
			   }
		   }
	   }
	   if(TB_minus2.flag[ni][nj][nk]==CLEAR){
		   for(int i=1;i<N_KL-1;i++){//do predzadnjeg
			   if (KL_minus2.S[i]<KL_minus2.S[N_KL-1]){
				   TB_minus2.flag[ni][nj][nk]=NON_ADMISSIBLE;
				   break;
			   }
		   }
	   }
   }
#else
   if (TB.flag[ni][nj]==CLEAR){
	   if (KL.S[1]<KL.S[N_KL-1] && (svi_uvjeti==-1)){
	   TB.flag[ni][nj]=NON_ADMISSIBLE;
	}
   }
   if (TB_plus.flag[ni][nj]==CLEAR){
	   if (kriterij){
		   TB_plus.ocjena_path[ni][nj]=path_alignment_plus+rho*sumvelplus;
	   }else{
		   TB_plus.ocjena_path[ni][nj]=KL_plus.S[N_KL-1]+rho*sumvelplus;
	   }
	   if (dodatni_kut==1){
			   TB_plus.ocjena_path[ni][nj]+=epsilon*E_kut_plus;
		   }else if (dodatni_kut==2){
			   TB_plus.ocjena_path[ni][nj]+=epsilon*Cplus_kut_plus;
		   }else if (dodatni_kut==3){
			   TB_plus.ocjena_path[ni][nj]+=epsilon*goal_angle_plus;
		   }
	   if (KL_plus.S[1]<KL_plus.S[N_KL-1] && (svi_uvjeti==-1)){
		   TB_plus.flag[ni][nj]=NON_ADMISSIBLE;
	   }
   }
#if ROT
   if (TB_plus1.flag[ni][nj]==CLEAR){
	   if (kriterij){
		   TB_plus1.ocjena_path[ni][nj]=path_alignment_plus1+rho*sumvelplus1;
	   }else{
		   TB_plus1.ocjena_path[ni][nj]=KL_plus1.S[N_KL-1]+rho*sumvelplus1;
	   }
	   if (dodatni_kut==1){
			   TB_plus1.ocjena_path[ni][nj]+=epsilon*E_kut_plus1;
		   }else if (dodatni_kut==2){
			   TB_plus1.ocjena_path[ni][nj]+=epsilon*Cplus_kut_plus1;
		   }else if (dodatni_kut==3){
			   TB_plus1.ocjena_path[ni][nj]+=epsilon*goal_angle_plus1;
		   }
	   if (KL_plus1.S[1]<KL_plus1.S[N_KL-1] && (svi_uvjeti==-1)){
		   TB_plus1.flag[ni][nj]=NON_ADMISSIBLE;
	   }
   }
   if (TB_plus2.flag[ni][nj]==CLEAR){
	   if (kriterij){
		   TB_plus2.ocjena_path[ni][nj]=path_alignment_plus2+rho*sumvelplus2;
	   }else{
		   TB_plus2.ocjena_path[ni][nj]=KL_plus2.S[N_KL-1]+rho*sumvelplus2;
	   }
	   if (dodatni_kut==1){
			   TB_plus2.ocjena_path[ni][nj]+=epsilon*E_kut_plus2;
		   }else if (dodatni_kut==2){
			   TB_plus2.ocjena_path[ni][nj]+=epsilon*Cplus_kut_plus2;
		   }else if (dodatni_kut==3){
			   TB_plus2.ocjena_path[ni][nj]+=epsilon*goal_angle_plus2;
		   }
	   if (KL_plus2.S[1]<KL_plus2.S[N_KL-1] && (svi_uvjeti==-1)){
		   TB_plus2.flag[ni][nj]=NON_ADMISSIBLE;
	   }
   }
#endif
   if (TB_minus.flag[ni][nj]==CLEAR){
	   if (kriterij){
		   TB_minus.ocjena_path[ni][nj]=path_alignment_minus+rho*sumvelminus;
	   }else {
		   TB_minus.ocjena_path[ni][nj]=KL_minus.S[N_KL-1]+rho*sumvelminus;
	   }
	   if (dodatni_kut==1){
			   TB_minus.ocjena_path[ni][nj]+=epsilon*E_kut_minus;
		   }else if (dodatni_kut==2){
			   TB_minus.ocjena_path[ni][nj]+=epsilon*Cplus_kut_minus;
		   }else if (dodatni_kut==3){
			   TB_minus.ocjena_path[ni][nj]+=epsilon*goal_angle_minus;
		   }
	   if (KL_minus.S[1]<KL_minus.S[N_KL-1] && (svi_uvjeti==-1)){
		   TB_minus.flag[ni][nj]=NON_ADMISSIBLE;
	   }
   }
   if (TB_minus2.flag[ni][nj]==CLEAR){
	   if (kriterij){
		   TB_minus2.ocjena_path[ni][nj]=path_alignment_minus2+rho*sumvelminus2;
	   }else {
		   TB_minus2.ocjena_path[ni][nj]=KL_minus2.S[N_KL-1]+rho*sumvelminus2;
	   }
	   if (dodatni_kut==1){
			   TB_minus2.ocjena_path[ni][nj]+=epsilon*E_kut_minus2;
		   }else if (dodatni_kut==2){
			   TB_minus2.ocjena_path[ni][nj]+=epsilon*Cplus_kut_minus2;
		   }else if (dodatni_kut==3){
			   TB_minus2.ocjena_path[ni][nj]+=epsilon*goal_angle_minus2;
		   }
	   if (KL_minus2.S[1]<KL_minus2.S[N_KL-1] && (svi_uvjeti==-1)){
		   TB_minus2.flag[ni][nj]=NON_ADMISSIBLE;
	   }
   }
   if (svi_uvjeti==1){//za dodatni uvjet po svim tockama
	   if(TB.flag[ni][nj]==CLEAR){
		   for(int i=1;i<N_KL-1;i++){//do predzadnjeg
			   if ((KL.S[i]<KL.S[N_KL-1]) || (KL.S[i]<decreasingslope*KL.S[i+1])){
				   TB.flag[ni][nj]=NON_ADMISSIBLE;
				   break;
			   }
		   }
	   }
	   if(TB_plus.flag[ni][nj]==CLEAR){
		   for(int i=1;i<N_KL-1;i++){//do predzadnjeg
			   if ((KL_plus.S[i]<KL_plus.S[N_KL-1]) || (KL_plus.S[i]<decreasingslope*KL_plus.S[i+1])){
//			   			      printf("i=%d S(i)=%.15f, S(end)=%.15f\t",i,KL_plus.S[i],KL_plus.S[N_KL-1]);
				   TB_plus.flag[ni][nj]=NON_ADMISSIBLE;
				   break;
			   }
		   }
	   }
#if ROT
	   if(TB_plus1.flag[ni][nj]==CLEAR){
		   for(int i=1;i<N_KL-1;i++){//do predzadnjeg
			   if ((KL_plus1.S[i]<KL_plus1.S[N_KL-1]) || (KL_plus1.S[i]<decreasingslope*KL_plus1.S[i+1])){
//			   			      printf("i=%d S(i)=%.15f, S(end)=%.15f\t",i,KL_plus.S[i],KL_plus.S[N_KL-1]);
				   TB_plus1.flag[ni][nj]=NON_ADMISSIBLE;
				   break;
			   }
		   }
	   }
	   if(TB_plus2.flag[ni][nj]==CLEAR){
		   for(int i=1;i<N_KL-1;i++){//do predzadnjeg
			   if ((KL_plus2.S[i]<KL_plus2.S[N_KL-1]) || (KL_plus2.S[i]<decreasingslope*KL_plus2.S[i+1])){
//			   			      printf("i=%d S(i)=%.15f, S(end)=%.15f\t",i,KL_plus.S[i],KL_plus.S[N_KL-1]);
				   TB_plus2.flag[ni][nj]=NON_ADMISSIBLE;
				   break;
			   }
		   }
	   }
#endif
	   if(TB_minus.flag[ni][nj]==CLEAR){
		   for(int i=1;i<N_KL-1;i++){//do predzadnjeg
			   if ((KL_minus.S[i]<KL_minus.S[N_KL-1]) || (KL_minus.S[i]<decreasingslope*KL_minus.S[i+1])){
				   TB_minus.flag[ni][nj]=NON_ADMISSIBLE;
				   break;
			   }
		   }
	   }
	   if(TB_minus2.flag[ni][nj]==CLEAR){
		   for(int i=1;i<N_KL-1;i++){//do predzadnjeg
			   if ((KL_minus2.S[i]<KL_minus2.S[N_KL-1]) || (KL_minus2.S[i]<decreasingslope*KL_minus2.S[i+1])){
				   TB_minus2.flag[ni][nj]=NON_ADMISSIBLE;
				   break;
			   }
		   }
	   }
   }
#endif
   if (kriterij==0){
	   path_alignment=KL.S[N_KL-1];
   }
   if (dodatni_kut==1){
		   path_alignment+=epsilon*E_kut;
	   }else if (dodatni_kut==2){
		   path_alignment+=epsilon*Cplus_kut;
	   }else if (dodatni_kut==3){
		   path_alignment+=epsilon*goal_angle;
	   }
//    path_alignment+=TB.v[ni]*0.1*STEP*10.;//pomak od brzine u jednom koraku u cm
#endif
   return (path_alignment+rho*sumvel);
}

double DynamicWindow::Old_traj(){
	double path_alignment=0.;
	bool zastavica=true;
	double rho=RHO;
	if (rotateongoal) rho=0;
	double goal_angle_old;
	double epsilon=EPSILON;//za dodatni_kut 
	double sumvel=0;
	int kriterij=OBJECTIVE_SUM;//1-suma, 0-S(T)
	KL_old.x[0]=RB.x;
	KL_old.y[0]=RB.y;
	KL_old.th[0]=RB.th;
	KL_old.v[0]=KL_old_temp.v[1];//ovo su zadnje zadane brzine
	KL_old.vy[0]=KL_old_temp.vy[1];
	KL_old.w[0]=KL_old_temp.w[1];
	KL_old.S[0]=KL_old_temp.S[1];
#if (IDEAL_MODEL==1)
//	printf("KL_old x,y,th,v,w,S = %f,%f,%f,%f,%f,%f\n",KL_old.x[0],KL_old.y[0],KL_old.th[0],KL_old.v[0],KL_old.w[0],KL_old.S[0]);
	for(int i=0;i<N_KL;i++){
		if (i<N_KL-1){
			KL_old.x[i]=KL_old_temp.x[i+1];
			KL_old.y[i]=KL_old_temp.y[i+1];
			KL_old.th[i]=KL_old_temp.th[i+1];
			KL_old.v[i]=KL_old_temp.v[i+1];
			KL_old.vy[i]=KL_old_temp.vy[i+1];
			KL_old.w[i]=KL_old_temp.w[i+1];
			KL_old.S[i]=KL_old_temp.S[i+1];
			if (KL_old.S[i]!=computeInterpolatedCost(KL_old.x[i],KL_old.y[i],KL_old.th[i])){
			  printf("provjera prijepisa stare trajektorije: cijene nisu jednake! KL_old.S[i]=%.15f vs compute=%.15f, i=%d rotateongoal=%d,KL_old.x[i]=%.15f, KL_old.y[i]=%.15f, KL_old.th[i]=%.15f\n",KL_old.S[i],computeInterpolatedCost(KL_old.x[i],KL_old.y[i],KL_old.th[i]),i,rotateongoal,KL_old.x[i], KL_old.y[i], KL_old.th[i]);
			  if (rotateongoal){
			  KL_old.S[i]=computeInterpolatedCost(KL_old.x[i],KL_old.y[i],KL_old.th[i]);
			    for(int ii=i+1;ii<N_KL;ii++){
			KL_old.x[ii]=KL_old.x[ii-1];
			KL_old.y[ii]=KL_old.y[ii-1];
			KL_old.th[ii]=KL_old.th[ii-1];
			KL_old.v[ii]=0.;
			KL_old.vy[ii]=0.;
			KL_old.w[ii]=0.;
			KL_old.S[ii]=KL_old.S[ii-1];
			      
			    }
			  break;
			  }
			}
//			if (i==0)
//				printf("provjera: KL_old x,y,th,v,w,S = %f,%f,%f,%f,%f,%f\n",KL_old.x[0],KL_old.y[0],KL_old.th[0],KL_old.v[0],KL_old.w[0],KL_old.S[0]);

		}else{
			KL_old.x[i]=KL_old.x[i-1];
			KL_old.y[i]=KL_old.y[i-1];
			KL_old.th[i]=KL_old.th[i-1];
			KL_old.v[i]=0.;
			KL_old.vy[i]=0.;
			KL_old.w[i]=0.;
			KL_old.S[i]=KL_old.S[i-1];
		}
#if RECTANGULAR
		GM->mapper_point_temp.x=KL_old.x[i];
		GM->mapper_point_temp.y=KL_old.y[i];
	if(GM->check_point(GM->mapper_point_temp)) {
   R_point point_ri;
   if(!PL->RealToReal(GM->mapper_point_temp, point_ri, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> point on trajectory is beyond the bounds!\n");
//      return 0;
   }
//real coordinates within a cell
	double t=KL_old.th[i];
	while (t>=2*M_PI) t-=2*M_PI;
	while (t<0) t+=2*M_PI;
	Pose checkme((point_ri.x+0.5),(point_ri.y+0.5),t);
	if ((cspace->checkCollision(checkme))){
	printf("obstacle! i=%d\t",i);
	}
	}
#endif	
		
	}

#else
   I_point temp,best;
   R_point C, E, A, A1, A2, M1, M2;
   double VDStar, distance_temp_old, min_distance, min_distance2, temp_min_distance;
   double VM1,VM2,a1,a2,a3;
#if (OKO_REFERENCE==1)
	KL_old.v[0]=WH->v_refdin_current;
	KL_old.vy[0]=WH->vy_refdin_current;
	KL_old.w[0]=WH->w_refdin_current;
#else
	KL_old.v[0]=RB.v;
	KL_old.vy[0]=RB.w;
	KL_old.w[0]=RB.vy;
#endif
	
	int racunajS=1; //0 ne racunaj, 1 racunaj (ne znam da li to sta usporava, ali prelaze preko beskonacnih u voznji)
	int ellipse=ELLIPSE;
	double a,b,mind;
	int index=1;
	bool obstacle_flag=true;
	mind=MAXCOST;
	for(int i=0;i<N_KL-1;i++){
		a=(RB.x-KL_old_temp.x[i])*(RB.x-KL_old_temp.x[i])+(RB.y-KL_old_temp.y[i])*(RB.y-KL_old_temp.y[i]);
		b=0.;//fali kut!!! ako stoji i rotira
//		b=(RB.x-KL_old_temp.x[i+1])*(RB.x-KL_old_temp.x[i+1])+(RB.y-KL_old_temp.y[i+1])*(RB.y-KL_old_temp.y[i+1]);
		if (mind>a+b){
			mind=a+b;
		}else{
			index=std::max(i-1,1);
			printf("DW old_traj index=%d, RB=(%f,%f), KL_old_temp[index]=(%f,%f), [index+1]=(%f,%f)\n",index,RB.x,RB.y,KL_old_temp.x[index],KL_old_temp.y[index],KL_old_temp.x[index+1],KL_old_temp.y[index+1]);
			break;
		}
	}
	for(int i=0;i<N_KL;i++){
		
		if (i<N_KL-index){
			if (i>0){
				KL_old.v[i]=KL_old_temp.v[i+index];
				KL_old.vy[i]=KL_old_temp.vy[i+index];
				KL_old.w[i]=KL_old_temp.w[i+index];
				KL_old.S[i]=KL_old_temp.S[i+index];
	//check the constraints
	if (fabs(KL_old.v[i-1]-KL_old.v[i])>(DVX_MAX*STEP)+V_TOLERANCE) {
		KL_old.v[i]=KL_old.v[i-1]-(KL_old.v[i-1]-KL_old.v[i])/fabs(KL_old.v[i-1]-KL_old.v[i])*DVX_MAX*STEP;
	}
	if (fabs(KL_old.vy[i-1]-KL_old.vy[i])>(DVY_MAX*STEP)+V_TOLERANCE) {
		KL_old.vy[i]=KL_old.vy[i-1]-(KL_old.vy[i-1]-KL_old.vy[i])/fabs(KL_old.vy[i-1]-KL_old.vy[i])*DVY_MAX*STEP;
	}
	if (fabs(KL_old.w[i-1]-KL_old.w[i])>(DW_MAX*STEP)+W_TOLERANCE) {
		KL_old.w[i]=KL_old.w[i-1]-(KL_old.w[i-1]-KL_old.w[i])/fabs(KL_old.w[i-1]-KL_old.w[i])*DW_MAX*STEP;
	}
#if DIAGCONSTR
	if ((fabs(KL_old.v[i])+fabs(KL_old.vy[i])>V_MAX+V_TOLERANCE)){
		//zadrzavam v brzinu, vy ostavljam na staroj
		KL_old.vy[i]=KL_old.vy[i-1];
	}
	if ((fabs(KL_old.v[i-1]-KL_old.v[i])+fabs(KL_old.vy[i-1]-KL_old.vy[i])>DV_MAX*STEP+V_TOLERANCE)){
		//smanjujem ih po pola
		KL_old.v[i]=KL_old.v[i-1]-(KL_old.v[i-1]-KL_old.v[i])/fabs(KL_old.v[i-1]-KL_old.v[i])*0.5*DVX_MAX*STEP;
		KL_old.vy[i]=KL_old.vy[i-1]-(KL_old.vy[i-1]-KL_old.vy[i])/fabs(KL_old.vy[i-1]-KL_old.vy[i])*0.5*DVY_MAX*STEP;
	}
#endif
  kin_model_evol(&KL_old,i);
				if ((racunajS) && ((i==1) || (fabs(KL_old.th[i-1]-KL_old.th[i])>0) || (fabs(KL_old.x[i-1]-KL_old.x[i])>0) || (fabs(KL_old.y[i-1]-KL_old.y[i])>0))){
					GM->mapper_point_temp.x=KL_old.x[i];
		   			GM->mapper_point_temp.y=KL_old.y[i];
		   			distance_temp_old=MAXCOST;
      //ako je tocka unutar mape
		   if(GM->check_point(GM->mapper_point_temp)) {
#if RECTANGULAR
   R_point point_ri;
   if(!PL->RealToReal(GM->mapper_point_temp, point_ri, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> point on trajectory is beyond the bounds!\n");
//      return 0;
   }
//real coordinates within a cell
	double t=KL_old.th[i];
	while (t>=2*M_PI) t-=2*M_PI;
	while (t<0) t+=2*M_PI;
	Pose checkme((point_ri.x+0.5),(point_ri.y+0.5),t);
	if ((cspace->checkCollision(checkme))||((ellipse)&&(Fourpointscollide((point_ri.x+0.5),(point_ri.y+0.5),t,KL_old.v[i],KL_old.vy[i],i)))){
	printf("obstacle! i=%d\t",i);
		if (obstacle_flag){
			min_obstacle_index=i;
			obstacle_flag=false;
		}
			obstacle_index=i;
//		}
	}else
#else
	if ((DS->IsValid( GM->cell_point_temp.x, GM->cell_point_temp.y )!=1)){
	printf("obstacle! i=%d\t",i);
	flag_kl_old=false;
	}else
#endif
	{	

		distance_temp_old=computeInterpolatedCost(KL_old.x[i],KL_old.y[i],KL_old.th[i]);
      
	}//za provjeru sudara s rectangle
	}//checkpoint
	KL_old.S[i]=distance_temp_old;
	} // if racunajS i nenul brzine
			else if (racunajS){
				KL_old.S[i]=KL_old.S[i-1];
				if ((obstacle_flag==false)&&(obstacle_index==i-1)){
					obstacle_index=i;
				}
			}
			} //if i>0
		}else{
			if ((obstacle_flag==false)&&(obstacle_index==i-1)){
				obstacle_index=i;
			}
			KL_old.x[i]=KL_old.x[i-1];
			KL_old.y[i]=KL_old.y[i-1];
			KL_old.th[i]=KL_old.th[i-1];
			KL_old.v[i]=0.;
			KL_old.vy[i]=0.;
			KL_old.w[i]=0.;
			KL_old.S[i]=KL_old.S[i-1];
		}
		
	}
	if (obstacle_flag) obstacle_index=N_KL;
//		for(int i=0;i<N_KL;i++){
//			if (i>1)
//				path_alignment+=KL_old.S[i];
//			if ((zastavica)&&(KL_old_temp.v[i]==0)&&(KL_old_temp.w[i]==0)&&(KL_old.vy[i]==0)){
//				zastavica=false;
//				T_old=i;
//				printf("T_old=%d\n",T_old);
//			}
//			if (0)
//			if ((KL_old.S[i]<KL_old.S[N_KL-1])&&(i<N_KL-1)){
//				printf("kakvo je to cudno prepisivanje\n");
//			}

//		}
//		if (T_old>0)
//			T_old=T_old-1;
//		if (T_old==0) T_old=N_KL-4;
#endif
		for(int i=0;i<N_KL;i++){
			if (i>0){
			   	sumvel+=GAINV*fabs(KL_old.vy[i])+GAINV*fabs(KL_old.v[i])+fabs(KL_old.w[i]);
			   }
			if (i>0)
				path_alignment+=KL_old.S[i];
//		  if (path_alignment>=OBSTACLE){
//		    path_alignment=MAXCOST;
//		  }

			if (i==N_KL-1){
				goal_angle_old=1-cos(KL_old.th[i]-desiredOrientation);
			}
			if ((zastavica)&&(KL_old.v[i]==0)&&(KL_old.w[i]==0)&&(KL_old.vy[i]==0)){
				zastavica=false;
				printf("T_old=%d, locmincnt=%d\n",i,locmincnt);
				T_old=i;
				if (T_old==0){
				  locmincnt++;
				  T_old=locmincnt;
				}else{
				  locmincnt=0;
				}
//				if ((i==0)&& (T_old>0)){
//					T_old=T_old-1;
//					locmincnt++;
//				}
//				if (T_old==0) {
//					T_old=N_KL-4;
//				}else {
//					if (i>0){
//						T_old=i;
//						locmincnt=0;
//					}
//				}
				
			}
			if (0)
			if ((KL_old.S[i]<KL_old.S[N_KL-1])&&(i<N_KL-1)){
				printf("kakvo je to cudno prepisivanje\n");
			}

		}
		if ((kriterij==0) && (path_alignment<MAXCOST)) path_alignment=KL_old.S[N_KL-1];
		printf("old traj: last point cost %.15f rho part %.15f rho(rot,trans)=(%.15f,%.15f), gainv=%.15f, path_alignment=%.15f DEADZONE=%.15f\n",path_alignment, rho*sumvel, rho, GAINV*rho, GAINV, path_alignment+rho*sumvel,DEADZONE);
	return (path_alignment+rho*sumvel+epsilon*goal_angle_old);
}

double DynamicWindow::Breakage_traj(){
	double path_alignment=0.;
	double rbvx, rbvy, rbw;
  int N,Tv,Tvy,Tw;
	bool zastavica=true;
	double rho=RHO;
	if (rotateongoal) rho=0;
	double epsilon=EPSILON;
	double goal_angle_breakage;
	double sumvel=0;
	int svi_uvjeti=OBJECTIVE_CONDITIONS;
	int kriterij=OBJECTIVE_SUM;//1-suma, 0-S(T)
  int decreasingslope=0; //za extra uvjet

#if (OKO_REFERENCE==1)
	rbvx=WH->v_refdin_current;
	rbvy=WH->vy_refdin_current;
	rbw=WH->w_refdin_current;
#else
	rbvx=RB.v;
	rbvy=RB.vy;
	rbw=RB.w;
#endif
  Tv=(int)ceil((fabs(rbvx))/(DVX_MAX*STEP));//vrijeme zaustavljanja
	Tvy=(int)ceil(fabs(rbvy)/(DVY_MAX*STEP));
	Tw=(int)ceil(fabs(rbw)/(DW_MAX*STEP));
	N=std::max(Tv,Tw);
	N=std::max(N,Tvy);

	KL_breakage.x[0]=RB.x;
	KL_breakage.y[0]=RB.y;
	KL_breakage.th[0]=RB.th;
	KL_breakage.v[0]=rbvx;
	KL_breakage.vy[0]=rbvy;
	KL_breakage.w[0]=rbw;
	KL_breakage.S[0]=computeInterpolatedCost(RB.x,RB.y,RB.th);
  double distance_temp_old;
	
	int racunajS=1; //0 ne racunaj, 1 racunaj (ne znam da li to sta usporava, ali prelaze preko beskonacnih u voznji)
	int ellipse=ELLIPSE;
	double a,b,mind;
	int index=1;
	bool obstacle_flag=true;
	for(int i=1;i<N_KL;i++){
		
	//check the constraints
#if DIAGCONSTR
	//first ramp for vy, the rest for vx because sideway motion is more risky
	if (fabs(KL_breakage.vy[i-1])>DVY_MAX*STEP+V_TOLERANCE) {//trebalo bi ici minus V_tol jer ovako dozvoljavas vecu rampu za vtol
		KL_breakage.vy[i]=KL_breakage.vy[i-1]-(KL_breakage.vy[i-1])/fabs(KL_breakage.vy[i-1])*DVY_MAX*STEP;
		if ((DV_MAX-DVY_MAX>V_TOLERANCE)){
			if (fabs(KL_breakage.v[i-1])>(DV_MAX-DVY_MAX)*STEP+V_TOLERANCE){
				KL_breakage.v[i]=KL_breakage.v[i-1]-(KL_breakage.v[i-1])/fabs(KL_breakage.v[i-1])*(DV_MAX-DVY_MAX)*STEP;
			}else{
				KL_breakage.v[i]=0;
			}
		}else{
			KL_breakage.v[i]=KL_breakage.v[i-1];
		}
	}else{
		KL_breakage.vy[i]=0;
		if (fabs(KL_breakage.v[i-1])>DVX_MAX*STEP+V_TOLERANCE) {
			KL_breakage.v[i]=KL_breakage.v[i-1]-(KL_breakage.v[i-1])/fabs(KL_breakage.v[i-1])*DVX_MAX*STEP;
		}else{
			KL_breakage.v[i]=0;
		}
	}
#else
	if (fabs(KL_breakage.v[i-1])>DVX_MAX*STEP+V_TOLERANCE) {
			KL_breakage.v[i]=KL_breakage.v[i-1]-(KL_breakage.v[i-1])/fabs(KL_breakage.v[i-1])*DVX_MAX*STEP;
	}else{
		KL_breakage.v[i]=0;
	}
	if (fabs(KL_breakage.vy[i-1])>(DVY_MAX*STEP)+V_TOLERANCE) {
		KL_breakage.vy[i]=KL_breakage.vy[i-1]-(KL_breakage.vy[i-1])/fabs(KL_breakage.vy[i-1])*DVY_MAX*STEP;
	}else{
		KL_breakage.vy[i]=0;
	}
#endif
	if (fabs(KL_breakage.w[i-1])>(DW_MAX*STEP)+W_TOLERANCE) {
		KL_breakage.w[i]=KL_breakage.w[i-1]-(KL_breakage.w[i-1])/fabs(KL_breakage.w[i-1])*DW_MAX*STEP;
	}else{
		KL_breakage.w[i]=0;
	}
#if (NO_PATH_RAMP==2) //circular arc
//				KL_breakage.v[i]=std::max(0.,rbvx*(N-i)/N);
//				KL_breakage.w[i]=std::max(0.,rbw*(N-i)/N);//tu ode u nebo za negativne brzine i indekse vece od N
//				KL_breakage.vy[i]=std::max(0.,rbvy*(N-i)/N);
        if (i<N){
				  KL_breakage.v[i]=rbvx*(N-i)/N;
				  KL_breakage.w[i]=rbw*(N-i)/N;
				  KL_breakage.vy[i]=rbvy*(N-i)/N;
				}else
				{
				  KL_breakage.v[i]=0;
				  KL_breakage.w[i]=0;
				  KL_breakage.vy[i]=0;				
				}
				
#endif
	kin_model_evol(&KL_breakage,i);
				if ((racunajS) && ((i==1) || (fabs(KL_breakage.th[i-1]-KL_breakage.th[i])>0) || (fabs(KL_breakage.x[i-1]-KL_breakage.x[i])>0) || (fabs(KL_breakage.y[i-1]-KL_breakage.y[i])>0)) ){
					GM->mapper_point_temp.x=KL_breakage.x[i];
		   			GM->mapper_point_temp.y=KL_breakage.y[i];
		   			distance_temp_old=MAXCOST;
      //ako je tocka unutar mape
		   if(GM->check_point(GM->mapper_point_temp)) {
#if RECTANGULAR
   R_point point_ri;
   if(!PL->RealToReal(GM->mapper_point_temp, point_ri, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
      printf("Planner> point on trajectory is beyond the bounds!\n");
//      return 0;
   }
//real coordinates within a cell
	double t=KL_breakage.th[i];
	while (t>=2*M_PI) t-=2*M_PI;
	while (t<0) t+=2*M_PI;
	Pose checkme((point_ri.x+0.5),(point_ri.y+0.5),t);
	if ((cspace->checkCollision(checkme))||((ellipse)&&(Fourpointscollide((point_ri.x+0.5),(point_ri.y+0.5),t,KL_breakage.v[i],KL_breakage.vy[i],i)))){
	printf("Breakage_traj: obstacle! i=%d\t",i);
	}else
#else
	if ((DS->IsValid( GM->cell_point_temp.x, GM->cell_point_temp.y )!=1)){
	printf("obstacle! i=%d\t",i);
	}else
#endif
	{	
		distance_temp_old=computeInterpolatedCost(KL_breakage.x[i],KL_breakage.y[i],KL_breakage.th[i]);
//		if (path_alignment>=OBSTACLE){
//		  path_alignment=MAXCOST;
//		}
      
	}//za provjeru sudara s rectangle
	}//checkpoint
				KL_breakage.S[i]=distance_temp_old;
			}else{
			
				KL_breakage.S[i]=KL_breakage.S[i-1];
			}
#if IDEAL_MODEL
				if ((distance_temp_old<MAXCOST)&&(KL_breakage.S[i]!=computeInterpolatedCost(KL_breakage.x[i],KL_breakage.y[i],KL_breakage.th[i]))){
          printf("provjera prijepisa: nije jednako za KL_breakage trajektoriju %d. tocka\n",i);
        }
#endif
			if (i>0){
			   	sumvel+=GAINV*fabs(KL_breakage.vy[i])+GAINV*fabs(KL_breakage.v[i])+fabs(KL_breakage.w[i]);
			}
			if (i>0)
				path_alignment+=KL_breakage.S[i];
			if (i==N_KL-1){
				goal_angle_breakage=1-cos(KL_breakage.th[i]-desiredOrientation);
			}
	}
	if ((kriterij==0)&&(path_alignment<MAXCOST)) path_alignment=KL_breakage.S[N_KL-1];

#if IDEAL_MODEL
  if ((kriterij==1)&&(path_alignment<MAXCOST)){
   for(int i=1;i<N_KL-1;i++){//do predzadnjeg
	   if ((KL_breakage.S[1]<KL_breakage.S[N_KL-1] && (svi_uvjeti==-1)) || (KL_breakage.S[i]<KL_breakage.S[N_KL-1] && (svi_uvjeti==1)) || (KL_breakage.S[i]<decreasingslope*KL_breakage.S[i+1]) ){
		   path_alignment=MAXCOST; 
		   break;
	   }
   }
  }
#else
//  	if (path_alignment>=OBSTACLE){
//		  printf("breakage traj: obstacle cost\n");
//		  path_alignment=MAXCOST;
//		}

#endif

	return (path_alignment+rho*sumvel+epsilon*goal_angle_breakage);
}

void DynamicWindow::DoprinosProhodnost(){

	int i, j,k,imax,jmax;
	double T_breakage;

	double max_A=0.0;   //treba razvuc interval na [0,1] tocno
	double min_A=0.0;

	bool zastavica=true;
#if OMNIDRIVE
	for (k=0;k<VY_DIM+1;k++){
#endif
	for(i=0;i<V_DIM+1;i++){
		for(j=0;j<W_DIM+1;j++){
#if OMNIDRIVE
			if((TB.flag[i][j][k]==HAS_OBSTACLE)&&(TB.ocjena_prohodnost[i][j][k]>0.0)){ //za clear je vec postavljen
				T_breakage=std::max( fabs(TB.v[i]/DVX_MAX),fabs(TB.w[j]/DW_MAX));
				T_breakage=std::max(T_breakage,fabs(TB.vy[k]/DVY_MAX) );
				if (T_breakage>TB.ocjena_prohodnost[i][j][k]){
					TB.ocjena_prohodnost[i][j][k]=0.;
					TB.flag[i][j][k]=NON_ADMISSIBLE;
				}
			}
#else
			if((TB.flag[i][j]==HAS_OBSTACLE)&&(TB.ocjena_prohodnost[i][j]>0.0)){ //za clear je vec postavljen
				T_breakage=std::max( (TB.v[i]/DVX_MAX),fabs(TB.w[j]/DW_MAX) );
				if (T_breakage>TB.ocjena_prohodnost[i][j]){
					TB.ocjena_prohodnost[i][j]=0.;
					TB.flag[i][j]=NON_ADMISSIBLE;
				}
			}
#endif
		}

	}
#if OMNIDRIVE
	}
	for (k=0;k<VY_DIM+1;k++){
#endif
	for(i=0;i<V_DIM+1;i++){
		for(j=0;j<W_DIM+1;j++){
#if OMNIDRIVE
			if(((TB.flag[i][j][k]==CLEAR)||(TB.flag[i][j][k]==HAS_OBSTACLE))&&(TB.ocjena_prohodnost[i][j][k]>0.0))		{
				if ((TB.flag[i][j][k]==HAS_OBSTACLE)&&(TB.ocjena_prohodnost[i][j][k]>0.0)){//za clear je vec postavljen
						T_breakage=std::max( fabs(TB.v[i]/DVX_MAX),fabs(TB.w[j]/DW_MAX));
				T_breakage=std::max(T_breakage,fabs(TB.vy[k]/DVY_MAX) );
					TB.ocjena_prohodnost[i][j][k]=(TB.ocjena_prohodnost[i][j][k]-T_breakage)/(TMAX-T_breakage);
					if (TB.ocjena_prohodnost[i][j][k]-1.0>0.000001)
						printf("prohodnost:Veci od 1!T_breakage=%.15f,ocjena_prohodnost=%.15f\n",T_breakage,TB.ocjena_prohodnost[i][j][k]);
				}
			}
#else
			if(((TB.flag[i][j]==CLEAR)||(TB.flag[i][j]==HAS_OBSTACLE))&&(TB.ocjena_prohodnost[i][j]>0.0))		{
				if ((TB.flag[i][j]==HAS_OBSTACLE)&&(TB.ocjena_prohodnost[i][j]>0.0)){//za clear je vec postavljen
						T_breakage=std::max( (TB.v[i]/DVX_MAX),fabs(TB.w[j]/DW_MAX) );
					TB.ocjena_prohodnost[i][j]=(TB.ocjena_prohodnost[i][j]-T_breakage)/(TMAX-T_breakage);
					if (TB.ocjena_prohodnost[i][j]-1.0>0.000001)
						printf("prohodnost:Veci od 1!T_breakage=%.15f,ocjena_prohodnost=%.15f\n",T_breakage,TB.ocjena_prohodnost[i][j]);
				}
			}
#endif
		}
	}
#if OMNIDRIVE
	}
#endif
	zastavica=false;//sad za trazenje min i max
#if OMNIDRIVE
	for (k=0;k<VY_DIM+1;k++){
#endif
		for(i=0;i<V_DIM+1;i++){
	for(j=0;j<W_DIM+1;j++){
#if OMNIDRIVE
		if(((TB.flag[i][j][k]==CLEAR)||(TB.flag[i][j][k]==HAS_OBSTACLE))){
			if(zastavica){ //uhvatimo prvi CLEAR par zbog odredjivanja medje
				min_A=TB.ocjena_prohodnost[i][j][k];
				max_A=TB.ocjena_prohodnost[i][j][k];
				zastavica=false;
				imax=i;jmax=j;
			}
			if(TB.ocjena_prohodnost[i][j][k]<min_A) min_A=TB.ocjena_prohodnost[i][j][k];
			if(TB.ocjena_prohodnost[i][j][k]>max_A){
				max_A=TB.ocjena_prohodnost[i][j][k];
				imax=i;jmax=j;
			}
		}
#else
		if(((TB.flag[i][j]==CLEAR)||(TB.flag[i][j]==HAS_OBSTACLE))){
			if(zastavica){ //uhvatimo prvi CLEAR par zbog odredjivanja medje
				min_A=TB.ocjena_prohodnost[i][j];
				max_A=TB.ocjena_prohodnost[i][j];
				zastavica=false;
				imax=i;jmax=j;
			}
			if(TB.ocjena_prohodnost[i][j]<min_A) min_A=TB.ocjena_prohodnost[i][j];
			if(TB.ocjena_prohodnost[i][j]>max_A){
				max_A=TB.ocjena_prohodnost[i][j];
				imax=i;jmax=j;
			}
		}
#endif
	}
		}
#if OMNIDRIVE
	}
#endif
// 	printf("min_A=%f, max_A=%f\n",min_A,max_A);
		if ((max_A-min_A)>0.000001){
#if OMNIDRIVE
	for (k=0;k<VY_DIM+1;k++){
#endif
		for(i=0;i<V_DIM+1;i++){
			for(j=0;j<W_DIM+1;j++){
#if OMNIDRIVE
				if((TB.flag[i][j][k]==CLEAR)||((TB.flag[i][j][k]==HAS_OBSTACLE)&&(TB.ocjena_prohodnost[i][j][k]>0.0))){
					TB.ocjena_prohodnost[i][j][k]=(TB.ocjena_prohodnost[i][j][k]-min_A)/(max_A-min_A);
					if (TB.ocjena_prohodnost[i][j][k]-1.0>0.000001)
						printf("prohodnost:Veci od 1!T_breakage=%.15f,ocjena_prohodnost=%.15f\n",T_breakage,TB.ocjena_prohodnost[i][j][k]);
				}
							TB.ocjena[i][j][k]+=TB.ocjena_prohodnost[i][j][k]*Lambda;
#else
				if((TB.flag[i][j]==CLEAR)||((TB.flag[i][j]==HAS_OBSTACLE)&&(TB.ocjena_prohodnost[i][j]>0.0))){
					TB.ocjena_prohodnost[i][j]=(TB.ocjena_prohodnost[i][j]-min_A)/(max_A-min_A);
					if (TB.ocjena_prohodnost[i][j]-1.0>0.000001)
						printf("prohodnost:Veci od 1!T_breakage=%.15f,ocjena_prohodnost=%.15f\n",T_breakage,TB.ocjena_prohodnost[i][j]);
				}
							TB.ocjena[i][j]+=TB.ocjena_prohodnost[i][j]*Lambda;
#endif
			}
		}
// 		printf("najbolja prohodnost v=%f, w=%f, max_A=%f,njegova prohodnost=%f\n",TB.v[imax],TB.w[jmax],max_A,TB.ocjena_prohodnost[imax][jmax]);
#if OMNIDRIVE
	}
#endif
	
		}

	return;
}

//za path moramo posebno normirati jer su vrijednosti izvan [0,1]
void DynamicWindow::Doprinos_path(){

	int i, j,k, imax=0, jmax=0, imin=0, jmin=0, kmin=0, kmax=0;
	double max_A=0.0;
	double min_A=0.0;
	bool zastavica=true;
#if OMNIDRIVE
	for (k=0;k<VY_DIM+1;k++){
#endif
	for(i=0;i<V_DIM+1;i++){
		for(j=0;j<W_DIM+1;j++){
#if OMNIDRIVE
			if(((TB.flag[i][j][k]==CLEAR)||(TB.flag[i][j][k]==HAS_OBSTACLE))&&(TB.ocjena_path[i][j][k]>0.0)){
				if(zastavica){ //uhvatimo prvi CLEAR par zbog odredjivanja granica
					min_A=TB.ocjena_path[i][j][k];
					max_A=TB.ocjena_path[i][j][k];
					imin=i;imax=i;jmin=j;jmax=j;
					kmin=k;kmax=k;
					zastavica=false;
				}
				if(TB.ocjena_path[i][j][k]<min_A)

				{
					min_A=TB.ocjena_path[i][j][k];
					imin=i;
					jmin=j;
					kmin=k;
				}
				if(TB.ocjena_path[i][j][k]>max_A)
				{
					max_A=TB.ocjena_path[i][j][k];
					imax=i;
					jmax=j;
					kmax=k;
				}
			}
#else
			if(((TB.flag[i][j]==CLEAR)||(TB.flag[i][j]==HAS_OBSTACLE))&&(TB.ocjena_path[i][j]>0.0)){
				if(zastavica){ //uhvatimo prvi CLEAR par zbog odredjivanja granica
					min_A=TB.ocjena_path[i][j];
					max_A=TB.ocjena_path[i][j];
					imin=i;imax=i;jmin=j;jmax=j;
					zastavica=false;
				}
				if(TB.ocjena_path[i][j]<min_A)

				{
					min_A=TB.ocjena_path[i][j];
					imin=i;
					jmin=j;
				}
				if(TB.ocjena_path[i][j]>max_A)
				{
					max_A=TB.ocjena_path[i][j];
					imax=i;
					jmax=j;
				}
			}
#endif
		}
	}
#if OMNIDRIVE
	}
#endif

// 	printf("najbolje slaganje v=%f, w=%f, min_A=%f,njegova prohodnost=%f\n",TB.v[imin],TB.w[jmin],min_A,TB.ocjena_prohodnost[imin][jmin]);
#if OMNIDRIVE
	for (k=0;k<VY_DIM+1;k++){
#endif
	for(i=0;i<V_DIM+1;i++){
		for(j=0;j<W_DIM+1;j++){
#if OMNIDRIVE
			if(((TB.flag[i][j][k]==CLEAR)||(TB.flag[i][j][k]==HAS_OBSTACLE))&&(TB.ocjena_path[i][j][k]>0.0)){
				if ((imax!=imin) || (jmax!=jmin) || (kmax!=kmin) || ((max_A-min_A)>0.000001)){
					TB.ocjena_path[i][j][k]=(1.0-(TB.ocjena_path[i][j][k]-min_A)/(max_A-min_A));
				}else{
					TB.ocjena_path[i][j][k]=1.0; //znaci da imamo samo jednu slobodnu putanju
				}
				TB.ocjena[i][j][k]+=TB.ocjena_path[i][j][k]*(1-Lambda);
				if (TB.ocjena_path[i][j][k]>1.000001)
					printf("==============Veliki zbroj! %f\n",TB.ocjena_path[i][j][k]);
			}
#else
			if(((TB.flag[i][j]==CLEAR)||(TB.flag[i][j]==HAS_OBSTACLE))&&(TB.ocjena_path[i][j]>0.0)){
				if ((imax!=imin) || (jmax!=jmin) || ((max_A-min_A)>0.000001)){
					TB.ocjena_path[i][j]=(1.0-(TB.ocjena_path[i][j]-min_A)/(max_A-min_A));
				}else{
					TB.ocjena_path[i][j]=1.0; //znaci da imamo samo jednu slobodnu putanju
				}
				TB.ocjena[i][j]+=TB.ocjena_path[i][j]*(1-Lambda);
				if (TB.ocjena_path[i][j]>1.000001)
					printf("==============Veliki zbroj! %f\n",TB.ocjena_path[i][j]);
			}
#endif
		}
	}
#if OMNIDRIVE
	}
#endif
	return;
}

void DynamicWindow::Path_minimum(){

	int i, j, k, N;
	double min_A=MAXCOST,temp_v,temp_w,fi,fi_temp,fi_min;
	double kut_Erel=kut_E-RB.th;
	while (kut_Erel<-1*M_PI) kut_Erel+=2*M_PI;
	while (kut_Erel>M_PI) kut_Erel-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]

	bool zastavica=true, flag_fi=true;
	plus=false; minus=false; minus2=false;//zastavice jel je najbolji iz plus ili minus polja
//#if ROT
	plus1=false; plus2=false;
//#endif
	//bitna je i udaljenost te najblize prepreke (npr robot na sredini hodnika, kut_prepreke onda nema smisla)
#if OMNIDRIVE
	for (k=0;k<VY_DIM+1;k++)
#endif
	{
	for(i=0;i<V_DIM+1;i++){
		for(j=0;j<W_DIM+1;j++){
#if OMNIDRIVE
			if(((TB.flag[i][j][k]==CLEAR))){
				if((zastavica) || (TB.ocjena[i][j][k]<min_A)){
					min_A=TB.ocjena[i][j][k];
					temp_v=sqrt(TB.v[i]*TB.v[i]+TB.vy[k]*TB.vy[k]);
					nk=k;
#else
			if(((TB.flag[i][j]==CLEAR))){
				if((zastavica) || (TB.ocjena[i][j]<min_A)){
					min_A=TB.ocjena[i][j];
					temp_v=fabs(TB.v[i]);
#endif
					temp_w=TB.w[j];
					ni=i;
					nj=j;
					zastavica=false;
#if ROT
					plus1=false;
					plus2=false;
#endif
					plus=false;
					minus=false;
					minus2=false;
//					printf("obicni: uzima najmanji ni=%d, nj=%d: min_A=%.15f, temp_v=%f, temp_w=%f, kut_Erel=%f deg.\n",ni,nj,min_A, temp_v, temp_w*RuS, kut_Erel*RuS);
				}
#if (ISKLJUCI_ROTIRANJE)
				if (temp_v<V_TOLERANCE){
					N=(int)ceil(fabs(TB.w[j])/(DW_MAX*STEP));
					fi=(N+1)/2.*TB.w[j]*STEP;
					printf("obicni: kut fi=%f\n",fi*RuS);
					if ((fi<0.)&&(kut_Erel>0.))
						kut_Erel-=2*M_PI;
					if ((fi>0.)&&(kut_Erel<0.))
						kut_Erel+=2*M_PI;
					if (fabs(kut_Erel)>=fabs(fi)){
						fi_temp=kut_Erel-fi;
						while (fi_temp<-1*M_PI) fi_temp+=2*M_PI;
						while (fi_temp>M_PI) fi_temp-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
						fi_temp=fabs(fi_temp);
						if (flag_fi){
							flag_fi=false;
							fi_min=fi_temp;
#if OMNIDRIVE
							min_A=TB.ocjena[i][j][k];
							nk=k;
#else
							min_A=TB.ocjena[i][j];
#endif
							ni=i;
							nj=j;
							temp_v=TB.v[i];
							temp_w=TB.w[j];
							plus=false;
							minus=false;
							minus2=false;
							printf("uzima prvi fi, nj=%d, fi_min=%.15f, temp_w=%f, min_A=%f\n",nj,fi_min*RuS,temp_w*RuS,min_A);
						}else if (fi_temp<fi_min){
							fi_min=fi_temp;
#if OMNIDRIVE
							min_A=TB.ocjena[i][j][k];
							nk=k;
#else
							min_A=TB.ocjena[i][j];
#endif
							ni=i;
							nj=j;
							temp_v=TB.v[i];
							temp_w=TB.w[j];
							plus=false;
							minus=false;
							minus2=false;
							printf("uzima najveci fi, nj=%d, fi_min=%.15f, temp_w=%f, min_A=%f\n",nj,fi_min*RuS,temp_w*RuS,min_A);
						}
					}
					if ((flag_fi)&&(fabs(TB.w[j])<fabs(temp_w))){
#if OMNIDRIVE
							min_A=TB.ocjena[i][j][k];
							nk=k;
#else
							min_A=TB.ocjena[i][j];
#endif
						ni=i;
						nj=j;
						temp_v=TB.v[i];
						temp_w=TB.w[j];
						plus=false;
						minus=false;
						minus2=false;
						printf("obicni: koci, uzima najmanji w, nj=%d, temp_w=%f\n",nj,temp_w*RuS);
					}
					while (kut_Erel<-1*M_PI) kut_Erel+=2*M_PI;
					while (kut_Erel>M_PI) kut_Erel-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
				}
#endif
			}
			//plus
#if 1
#if OMNIDRIVE
			if(((TB_plus.flag[i][j][k]==CLEAR))){
				if((zastavica) || (TB_plus.ocjena[i][j][k]<min_A)){
					min_A=TB_plus.ocjena[i][j][k];
					temp_v=sqrt(TB.v[i]*TB.v[i]+TB.vy[k]*TB.vy[k]);
					nk=k;
#else
			if(((TB_plus.flag[i][j]==CLEAR))){
				if((zastavica) || (TB_plus.ocjena[i][j]<min_A)){
					min_A=TB_plus.ocjena[i][j];
					temp_v=fabs(TB.v[i]);
#endif
					ni=i;
					nj=j;
					temp_w=TB.w[j];
					zastavica=false;
#if ROT
					plus1=false;
					plus2=false;
#endif
					plus=true;
					minus=false;
					minus2=false;
//					printf("plus: uzima najmanji ni=%d, nj=%d: min_A=%.15f, temp_v=%f, temp_w=%f, kut_Erel=%f\n",ni,nj,min_A, temp_v, temp_w*RuS, kut_Erel*RuS);
				}
#if (ISKLJUCI_ROTIRANJE)
				if (temp_v<V_TOLERANCE){
					N=(int)ceil(fabs(TB.w[j])/(DW_MAX*STEP));
					fi=(N+1)/2.*TB.w[j]*STEP;
					printf("plus: kut fi=%f\n",fi*RuS);
					if ((fi<0.)&&(kut_Erel>0.))
						kut_Erel-=2*M_PI;
					if ((fi>0.)&&(kut_Erel<0.))
						kut_Erel+=2*M_PI;
					if (fabs(kut_Erel)>=fabs(fi)){
						fi_temp=kut_Erel-fi;
						while (fi_temp<-1*M_PI) fi_temp+=2*M_PI;
						while (fi_temp>M_PI) fi_temp-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
						fi_temp=fabs(fi_temp);
						if (flag_fi){
							flag_fi=false;
							fi_min=fi_temp;
#if OMNIDRIVE
							min_A=TB_plus.ocjena[i][j][k];
							nk=k;
#else
							min_A=TB_plus.ocjena[i][j];
#endif
							ni=i;
							nj=j;
							temp_v=TB.v[i];
							temp_w=TB.w[j];
							plus=true;
							minus=false;
							minus2=false;
							printf("uzima prvi fi, nj=%d, fi_min=%.15f, temp_w=%f\n",nj,fi_min*RuS,temp_w*RuS);
						}else if (fi_temp<fi_min){
							fi_min=fi_temp;
#if OMNIDRIVE
							min_A=TB_plus.ocjena[i][j][k];
							nk=k;
#else
							min_A=TB_plus.ocjena[i][j];
#endif
							ni=i;
							nj=j;
							temp_v=TB.v[i];
							temp_w=TB.w[j];
							plus=true;
							minus=false;
							minus2=false;
							printf("uzima najveci fi, nj=%d, fi_min=%.15f, temp_w=%f\n",nj,fi_min*RuS,temp_w*RuS);
						}
					}
					if ((flag_fi)&&(fabs(TB.w[j])<fabs(temp_w))){
#if OMNIDRIVE
							min_A=TB_plus.ocjena[i][j][k];
							nk=k;
#else
							min_A=TB_plus.ocjena[i][j];
#endif
						ni=i;
						nj=j;
						temp_v=TB.v[i];
						temp_w=TB.w[j];
						plus=true;
						minus=false;
						minus2=false;
						printf("plus: koci, uzima najmanji w, nj=%d, temp_w=%f\n",nj,temp_w*RuS);
					}
					while (kut_Erel<-1*M_PI) kut_Erel+=2*M_PI;
					while (kut_Erel>M_PI) kut_Erel-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
				}
#endif
			}
#endif
			//plus1
#if ROT
#if OMNIDRIVE
			if(((TB_plus1.flag[i][j][k]==CLEAR))){
				if((zastavica) || (TB_plus1.ocjena[i][j][k]<min_A)){
					min_A=TB_plus1.ocjena[i][j][k];
					temp_v=sqrt(TB.v[i]*TB.v[i]+TB.vy[k]*TB.vy[k]);
					nk=k;
#else
			if(((TB_plus1.flag[i][j]==CLEAR))){
				if((zastavica) || (TB_plus1.ocjena[i][j]<min_A)){
					min_A=TB_plus1.ocjena[i][j];
					temp_v=fabs(TB.v[i]);
#endif
					ni=i;
					nj=j;
					temp_w=TB.w[j];
					zastavica=false;
					plus1=true;
					plus2=false;
					plus=false;
					minus=false;
					minus2=false;
//					printf("plus1: uzima najmanji ni=%d, nj=%d: min_A=%.15f, temp_v=%f, temp_w=%f, kut_Erel=%f\n",ni,nj,min_A, temp_v, temp_w*RuS, kut_Erel*RuS);
				}
			}
//plus2
#if 1
#if OMNIDRIVE
			if(((TB_plus2.flag[i][j][k]==CLEAR))){
				if((zastavica) || (TB_plus2.ocjena[i][j][k]<min_A)){
					min_A=TB_plus2.ocjena[i][j][k];
					temp_v=sqrt(TB.v[i]*TB.v[i]+TB.vy[k]*TB.vy[k]);
					nk=k;
#else
			if(((TB_plus2.flag[i][j]==CLEAR))){
				if((zastavica) || (TB_plus2.ocjena[i][j]<min_A)){
					min_A=TB_plus2.ocjena[i][j];
					temp_v=fabs(TB.v[i]);
#endif
					ni=i;
					nj=j;
					temp_w=TB.w[j];
					zastavica=false;
					plus2=true;
					plus1=false;
					plus=false;
					minus=false;
					minus2=false;
//					printf("plus2: uzima najmanji ni=%d, nj=%d: min_A=%.15f, temp_v=%f, temp_w=%f, kut_Erel=%f\n",ni,nj,min_A, temp_v, temp_w*RuS, kut_Erel*RuS);
				}
			}
#endif
#endif
			//minus
#if OMNIDRIVE
			if(((TB_minus.flag[i][j][k]==CLEAR))){
				if((zastavica) || (TB_minus.ocjena[i][j][k]<min_A)){
					min_A=TB_minus.ocjena[i][j][k];
					temp_v=sqrt(TB.v[i]*TB.v[i]+TB.vy[k]*TB.vy[k]);
					nk=k;
#else
			if(((TB_minus.flag[i][j]==CLEAR))){
				if((zastavica) || (TB_minus.ocjena[i][j]<min_A)){
					min_A=TB_minus.ocjena[i][j];
					temp_v=fabs(TB.v[i]);
#endif
					ni=i;
					nj=j;
					temp_w=TB.w[j];
					zastavica=false;
#if ROT
					plus1=false;
					plus2=false;
#endif
					plus=false;
					minus=true;
					minus2=false;
//					printf("minus: uzima najmanji ni=%d, nj=%d: min_A=%.15f, temp_v=%f, temp_w=%f, kut_Erel=%f\n",ni,nj,min_A, temp_v, temp_w*RuS, kut_Erel*RuS);
				}
#if (ISKLJUCI_ROTIRANJE)
				if (temp_v<V_TOLERANCE){
					N=(int)ceil(fabs(TB.w[j])/(DW_MAX*STEP));
					fi=(N+1)/2.*TB.w[j]*STEP;
					printf("minus: kut fi=%f\n",fi*RuS);
					if ((fi<0.)&&(kut_Erel>0.))
						kut_Erel-=2*M_PI;
					if ((fi>0.)&&(kut_Erel<0.))
						kut_Erel+=2*M_PI;
					if (fabs(kut_Erel)>=fabs(fi)){
						fi_temp=kut_Erel-fi;
						while (fi_temp<-1*M_PI) fi_temp+=2*M_PI;
						while (fi_temp>M_PI) fi_temp-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
						fi_temp=fabs(fi_temp);
						if (flag_fi){
							flag_fi=false;
							fi_min=fi_temp;
#if OMNIDRIVE
							min_A=TB_minus.ocjena[i][j][k];
							nk=k;
#else
							min_A=TB_minus.ocjena[i][j];
#endif
							ni=i;
							nj=j;
							temp_v=TB.v[i];
							temp_w=TB.w[j];
							plus=false;
							minus=true;
							minus2=false;
							printf("uzima prvi fi, nj=%d, fi_min=%.15f, temp_w=%f\n",nj,fi_min*RuS,temp_w*RuS);
						}else if (fi_temp<fi_min){
							fi_min=fi_temp;
#if OMNIDRIVE
							min_A=TB_minus.ocjena[i][j][k];
							nk=k;
#else
							min_A=TB_minus.ocjena[i][j];
#endif
							ni=i;
							nj=j;
							temp_v=TB.v[i];
							temp_w=TB.w[j];
							plus=false;
							minus=true;
							minus2=false;
							printf("uzima najveci fi, nj=%d, fi_min=%.15f, temp_w=%f\n",nj,fi_min*RuS,temp_w*RuS);
						}
					}
					if ((flag_fi)&&(fabs(TB.w[j])<fabs(temp_w))){
#if OMNIDRIVE
							min_A=TB_minus.ocjena[i][j][k];
							nk=k;
#else
							min_A=TB_minus.ocjena[i][j];
#endif
						ni=i;
						nj=j;
						temp_v=TB.v[i];
						temp_w=TB.w[j];
						plus=false;
						minus=true;
						minus2=false;
						printf("minus: koci, uzima najmanji w, nj=%d, temp_w=%f\n",nj,temp_w*RuS);
					}
					while (kut_Erel<-1*M_PI) kut_Erel+=2*M_PI;
					while (kut_Erel>M_PI) kut_Erel-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
				}
#endif
			}
			//minus2
#if OMNIDRIVE
			if(((TB_minus2.flag[i][j][k]==CLEAR))){
				if((zastavica) || (TB_minus2.ocjena[i][j][k]<min_A)){
					min_A=TB_minus2.ocjena[i][j][k];
					temp_v=sqrt(TB.v[i]*TB.v[i]+TB.vy[k]*TB.vy[k]);
					nk=k;
#else
			if(((TB_minus2.flag[i][j]==CLEAR))){
				if((zastavica) || (TB_minus2.ocjena[i][j]<min_A)){
					min_A=TB_minus2.ocjena[i][j];
					temp_v=fabs(TB.v[i]);
#endif
					ni=i;
					nj=j;
					temp_w=TB.w[j];
					zastavica=false;
#if ROT
					plus1=false;
					plus2=false;
#endif
					plus=false;
					minus2=true;
					minus=false;
//					printf("minus2: uzima najmanji ni=%d, nj=%d: min_A=%.15f, temp_v=%f, temp_w=%f, kut_Erel=%f\n",ni,nj,min_A, temp_v, temp_w*RuS, kut_Erel*RuS);
				}
#if (ISKLJUCI_ROTIRANJE)
				if (temp_v<V_TOLERANCE){
					N=(int)ceil(fabs(TB.w[j])/(DW_MAX*STEP));
					fi=(N+1)/2.*TB.w[j]*STEP;
					printf("minus2: kut fi=%f\n",fi*RuS);
					if ((fi<0.)&&(kut_Erel>0.))
						kut_Erel-=2*M_PI;
					if ((fi>0.)&&(kut_Erel<0.))
						kut_Erel+=2*M_PI;
					if (fabs(kut_Erel)>=fabs(fi)){
						fi_temp=kut_Erel-fi;
						while (fi_temp<-1*M_PI) fi_temp+=2*M_PI;
						while (fi_temp>M_PI) fi_temp-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
						fi_temp=fabs(fi_temp);
						if (flag_fi){
							flag_fi=false;
							fi_min=fi_temp;
#if OMNIDRIVE
							min_A=TB_minus2.ocjena[i][j][k];
							nk=k;
#else
							min_A=TB_minus2.ocjena[i][j];
#endif
							ni=i;
							nj=j;
							temp_v=TB.v[i];
							temp_w=TB.w[j];
							plus=false;
							minus2=true;
							minus=false;
							printf("uzima prvi fi, nj=%d, fi_min=%.15f, temp_w=%f\n",nj,fi_min*RuS,temp_w*RuS);
						}else if (fi_temp<fi_min){
							fi_min=fi_temp;
#if OMNIDRIVE
							min_A=TB_minus2.ocjena[i][j][k];
							nk=k;
#else
							min_A=TB_minus2.ocjena[i][j];
#endif
							ni=i;
							nj=j;
							temp_v=TB.v[i];
							temp_w=TB.w[j];
							plus=false;
							minus2=true;
							minus=false;
							printf("uzima najveci fi, nj=%d, fi_min=%.15f, temp_w=%f\n",nj,fi_min*RuS,temp_w*RuS);
						}
					}
					if ((flag_fi)&&(fabs(TB.w[j])<fabs(temp_w))){
#if OMNIDRIVE
							min_A=TB_minus2.ocjena[i][j][k];
							nk=k;
#else
							min_A=TB_minus2.ocjena[i][j];
#endif
						ni=i;
						nj=j;
						temp_v=TB.v[i];
						temp_w=TB.w[j];
						plus=false;
						minus2=true;
						minus=false;
						printf("minus2: koci, uzima najmanji w, nj=%d, temp_w=%f\n",nj,temp_w*RuS);
					}
					while (kut_Erel<-1*M_PI) kut_Erel+=2*M_PI;
					while (kut_Erel>M_PI) kut_Erel-=2*M_PI;//svodjenje na interval [-M_PI,M_PI]
				}
#endif
			}

		}
	}
	}
	ljapunov=min_A;
	best_breakage=false;
	if (ni==-1){
		printf("no feasible trajectories in the search set!\n");
		if (flag_kl_old==false) best_breakage=true;//tu ga stavljam, ali fali uvjet ako ne stoji
	}
	best_old=false;
#if (IDEAL_MODEL==1) || 1
	if (flag_kl_old){
		if (cost_old>0){
			if ((min_A>cost_old)&&(fabs(min_A-cost_old)>fabs(min_A)/pow(10,12))){
//			if (min_A>cost_old){
				printf("The cost is lower on the old trajectory %.15f, min_A=%.15f, ni=%d, nj=%d\n",cost_old,min_A,ni,nj);
#if 0  //provjera
				if ((ni>-1)&&(nj>-1)){
				double cost;
				cost=0.;
				for (i=2;i<N_KL;i++){
					cost+=Log_plus.Skl[i][ni][nj];
				}
				printf("cost za plus je %.15f = ocjena %.15f, flag=%d\n",cost,TB_plus.ocjena[ni][nj],TB_plus.flag[ni][nj]);
				cost=0.;
				for (i=2;i<N_KL;i++){
					cost+=Log_minus.Skl[i][ni][nj];
				}
				printf("cost za minus je %.15f = ocjena %.15f, flag=%d\n",cost,TB_minus.ocjena[ni][nj],TB_minus.flag[ni][nj]);
				cost=0.;
				for (i=2;i<N_KL;i++){
					cost+=Log_minus2.Skl[i][ni][nj];
				}
				printf("cost za minus2 je %.15f = ocjena %.15f, flag=%d\n",cost,TB_minus2.ocjena[ni][nj],TB_minus2.flag[ni][nj]);
				cost=0.;
				for (i=2;i<N_KL;i++){
					cost+=Log.Skl[i][ni][nj];
				}
				printf("cost za obicni je %.15f = ocjena %.15f, flag=%d\n",cost,TB.ocjena[ni][nj],TB.flag[ni][nj]);
				cost=0.;
				for (i=2;i<N_KL;i++){
					cost+=KL_old.S[i];
				}
				printf("cost za old je %.15f = ocjena %.15f\n",cost,cost_old);
//				double tempcost=0.;
//				for(int i=0;i<N_KL;i++){
//					if (i>1)
//						tempcost+=KL_old.S[i];
//				}
//				printf("ponovo: cost za old je %.15f = ocjena %.15f\n",tempcost,cost_old);
				}
#endif
				ljapunov=cost_old;
				best_old=true;
				ni=-1; nj=-1;
			}
#if 0	
			else{
				double cost;
				if (plus){
					cost=0.;
					for (i=2;i<N_KL;i++){
						cost+=Log_plus.Skl[i][ni][nj];
					}
					printf("cost za plus je %.15f = ocjena %.15f, flag=%d\n",cost,TB_plus.ocjena[ni][nj],TB_plus.flag[ni][nj]);
				}else if (minus){
					cost=0.;
					for (i=2;i<N_KL;i++){
						cost+=Log_minus.Skl[i][ni][nj];
					}
					printf("cost za minus je %.15f = ocjena %.15f, flag=%d\n",cost,TB_minus.ocjena[ni][nj],TB_minus.flag[ni][nj]);
				}else if (minus2){
					cost=0.;
					for (i=2;i<N_KL;i++){
						cost+=Log_minus2.Skl[i][ni][nj];
					}
					printf("cost za minus2 je %.15f = ocjena %.15f, flag=%d\n",cost,TB_minus2.ocjena[ni][nj],TB_minus2.flag[ni][nj]);
				}else{
					cost=0.;
					for (i=2;i<N_KL;i++){
						cost+=Log.Skl[i][ni][nj];
					}
					printf("cost za obicni je %.15f = ocjena %.15f, flag=%d\n",cost,TB.ocjena[ni][nj],TB.flag[ni][nj]);
				}
				cost=0.;
				for (i=2;i<N_KL;i++){
					cost+=KL_old.S[i];
				}
				printf("cost za old je %.15f = ocjena %.15f\n",cost,cost_old);
				double tempcost=0.;
				for(int i=0;i<N_KL;i++){
					if (i>1)
						tempcost+=KL_old.S[i];
				}
				printf("ponovo: cost za old je %.15f = ocjena %.15f\n",tempcost,cost_old);
			}
#endif
		}
	}
#endif
#if USEBREAKAGETRAJ
	if ((ni>=0) && (cost_breakage<MAXCOST)&& (cost_breakage<ljapunov) && (fabs(ljapunov-cost_breakage)>fabs(ljapunov)/pow(10,12))) {
		printf("cost_breakage is lower=%f ljapunov=%.15f, ni=%d, nj=%d\n",cost_breakage,ljapunov,ni,nj);
		ljapunov=cost_breakage;
		best_breakage=true;
		ni=-1;nj=-1;nk=-1;
	}
#endif
	if (ljapunov>=MAXCOST){
		printf("big ljapunov, %f\n",ljapunov);
	}
//	if ((temp_v<V_TOLERANCE)&&(ni>=0)){
//		printf("only v=0 is available\n");
//	}
	if (ni>=0){
//		if (sqrt(TB.v[ni]*TB.v[ni]+TB.vy[nk]*TB.vy[nk])<V_TOLERANCE){
//			ni=-1;nj=-1;nk=-1;
//		}else
//ni slucajno to imati jer plus1 i plus2 trajektorije mogu imati pocetne brzine nula a dalje ne nul
		if ((flag_kl_old) && (fabs(min_A-cost_old)<=fabs(min_A)/pow(10,12))) {
			ni=-1;nj=-1;nk=-1;
			ljapunov=cost_old;
			best_old=true;
			printf("choosing old traj: cost of the old trajectory %.15f is similar to the cost of the best trajectory %.15f\n", cost_old, min_A);
//			if (naCilju){//ovo treba ako imam specijalna rotiranja koja su iste cijene kao stara
//				ni=-1;nj=-1;nk=-1;
//				ljapunov=cost_old;
//			}
		} else{
		if ((flag_kl_old==false) && (fabs(ljapunov-cost_breakage)<=fabs(ljapunov)/pow(10,12)) ){
  		printf("cost_breakage is equal to min_A cost_breakage=%.15f min_A=%.15f, ni=%d, nj=%d\n",cost_breakage,ljapunov,ni,nj);
  		ljapunov=cost_breakage;
  		best_breakage=true;
  		ni=-1;nj=-1;nk=-1;
		}else{
		printf("ni=%d, nj=%d, nk=%d, temp_v=%f, sqrt(vx^2+vy^2)=%f, plus=%d, plus1=%d, plus2=%d, minus=%d, minus2=%d min_A=%.15f, cost_breakage=%.15f, ljapunov=%.15f\n",ni,nj,nk, temp_v, sqrt(TB.v[ni]*TB.v[ni]+TB.vy[nk]*TB.vy[nk]),plus,plus1,plus2,minus,minus2, min_A, cost_breakage, ljapunov);
 		if (plus2){
 			printf("stani\n");
 		}
 		}
 		}
	}
			FILE *logfile;
		if ( (logfile = fopen("logger//traj","a")) == NULL ){
 			printf("Error! komb file couldn't be opened.");
 		}else{
 			fprintf(logfile,"%d %d %d %d %d %d %d\n",plus,plus1,plus2,minus,minus2,best_breakage,best_old);
 			fclose(logfile);
 		}

}


void DynamicWindow::Optimalni_par(double kut){  //kut se odredjuje u sekvenci_izvodjenja
	int i, j,k=0,stariw=-1;
	double temp_max, temp_v,temp_w,kutpre,tempref, ocjena;
// 	temp_max=0.0;postoji slucaj kad je samo brzina v=0 moguca, a u tom slucaju je ocjena=0.0, ali ne NON_ADMISSIBLE, tada nema optimalnog para
	bool zastavica=true; //treba mi zastavica za uzimanje prvog najboljeg
	kutpre=RB.th-kut_prepreke;
	//bitna je i udaljenost te najblize prepreke (npr robot na sredini hodnika, kut_prepreke onda nema smisla)
	if (kutpre<0)
		kutpre+=2*M_PI;
	if (kutpre>2*M_PI)
		kutpre-=2*M_PI;
#if OMNIDRIVE
	naStartu=false;//kakvo zakretanje!
#endif

	if (((kut<M_PI/6)&&(kut>-1*M_PI/6))||(!naStartu)){//patak
		if (naStartu){
			naStartu=false;
			ukrivustranu=0;
		}
		int imax=V_DIM+1;
		if (naCilju)
				imax=1;
#if OMNIDRIVE
	for (k=0;k<VY_DIM+1;k++){
#endif

		for(i=0;i<imax;i++){
		for(j=0;j<W_DIM+1;j++){
#if OMNIDRIVE
			if(TB.flag[i][j][k]==CLEAR){          //brzine iz proslog ciklusa su na sredini prozora (u opcem slucaju) pa odabiremo njih ako su ocjene iste
			ocjena=TB.ocjena[i][j][k];
#else
#if KRUZNI_LUK
			  if(TB.flag[i][j]>0){          //brzine iz proslog ciklusa su na sredini prozora (u opcem slucaju) pa odabiremo njih ako su ocjene iste
#else
			  if(TB.flag[i][j]==CLEAR){          //brzine iz proslog ciklusa su na sredini prozora (u opcem slucaju) pa odabiremo njih ako su ocjene iste
#endif
			ocjena=TB.ocjena[i][j];
#endif
				  if ((i==0)&&(zastavica)){
					  ni=i;
					  nj=j;
					  nk=k;
					  temp_max=ocjena;
					  temp_v=TB.v[i];
					  temp_w=TB.w[j];
					  zastavica=false;//tu vise ne bu isel
				  }
				  //pazimo da se robot ne okrece na krivu stranu ako odabere translacijsku brzinu 0, a staru rotacijsku zbog ovog gore i pazimo da ne presisa kut
				  tempref=(TB.w[j]-RB.w*exp(-STEP/WH->Tw))/(1-exp(-STEP/WH->Tw));
				  if ((ocjena==temp_max)&&(TB.v[i]==temp_v)&&(temp_v<V_TOLERANCE)&&(((fabs(tempref)<=DW_MAX/10.)&&(fabs(tempref*STEP)<=fabs(kut))&&((((kut>0)&&(TB.w[j]<temp_w))||((kut<0)&&(TB.w[j]>temp_w)))||(((fabs(temp_w*STEP)>fabs(kut))&&((kut>=0)&&(TB.w[j]<=0.)))||((kut<=0)&&(TB.w[j]>=0.)))))||((fabs(temp_w)>DW_MAX/10.)&&(fabs(TB.w[j])<fabs(temp_w))))) {

                          ni=i;
				                  nj=j;
				                  nk=k;
								  temp_max=ocjena; 
								  temp_v=TB.v[i];
								  temp_w=TB.w[j];
                        }
			if((ocjena>temp_max)||((ocjena==temp_max)&&(temp_v>V_TOLERANCE)&&((fabs(TB.v[i]-RB.v)<V_TOLERANCE)||(fabs(TB.w[j]-RB.w)<W_TOLERANCE))))//ovo je i za biranje starih brzina ali ne i ako je v==0
                        {
                          ni=i;
				                  nj=j;
				                  nk=k;
                          temp_max=ocjena;
						  temp_v=TB.v[i];
						  temp_w=TB.w[j];
			}
			
			  }
          }
	}
#if OMNIDRIVE
	}
#endif
	}else{
		i=0;//najmanja brzina v za okretanje na startu
		k=0;
		for(j=0;j<W_DIM+1;j++){
#if OMNIDRIVE
			if(TB.flag[i][j][k]==CLEAR){
				ocjena=TB.ocjena[i][j][k];
#else
			if(TB.flag[i][j]>0){
				ocjena=TB.ocjena[i][j];
#endif
				if ((zastavica)){
					ni=i;
					nj=j;
					nk=k;
					temp_max=ocjena;
					temp_v=TB.v[i];
					temp_w=TB.w[j];
					zastavica=false;//tu vise ne bu isel
				}
				tempref=(TB.w[j]-RB.w*exp(-STEP/WH->Tw))/(1-exp(-STEP/WH->Tw));
				if (((fabs(tempref)<=W_MAX/2)&&(((kut>0)&&(TB.w[j]<temp_w))||((kut<0)&&(TB.w[j]>temp_w))))||((fabs(tempref)>W_MAX/2)&&(fabs(TB.w[j])<fabs(temp_w)))) {
					ni=i;
					nj=j;
					nk=k;
					temp_max=ocjena; 
					temp_v=TB.v[i];
					temp_w=TB.w[j];
				}
			}
		}
	}
  return;
}


void DynamicWindow::Ispis_polja(){

      FILE *F;
	  FILE *F1;
	  FILE *F2;
	  FILE *F3;
	  FILE *F4, *F5, *F6;
	  FILE *F7, *F8, *F9;
	  FILE *F_temp1, *F_temp2;
	  FILE *F10;FILE *F11;FILE *F12;FILE *F13, *F14, *F15, *F16, *F17, *F18;
	  FILE *F19, *F20, *F21, *F22, *F23, *F24, *F25, *F26, *F27, *F28, *F29, *F30, *F31, *F32;
	  FILE *F41, *F42, *F43, *F44, *F61, *F62, *F63;
      int i,j,k,l;

	printf("DynamicWindow::Logging the data...\n");
    //OBSTACLE POINTOVI - X,Y koordinate (tocke na trajektorijama gdje se robot sudara s preprekom - ukljucen je safety distance)
	  if (((F=fopen((LOG_OBSTACLE_POINT_X),"wt"))!=NULL)&&((F1=fopen(LOG_OBSTACLE_POINT_Y,"wt"))!=NULL)) {
      //putanje po svim brzinama
#if OMNIDRIVE
	for (l=0;l<VY_DIM+1;l++){
#endif
		for(j=0;j<V_DIM+1;j++){
			for(k=0;k<W_DIM+1;k++){
#if OMNIDRIVE
				fprintf(F,"%f ",TB.obstacle_point_x[j][k][l]);  fprintf(F1,"%f ",TB.obstacle_point_y[j][k][l]);
#else
				fprintf(F,"%f ",TB.obstacle_point_x[j][k]);  fprintf(F1,"%f ",TB.obstacle_point_y[j][k]);
#endif
					}
			}
#if OMNIDRIVE
	}
#endif
			fprintf(F,"\n");  fprintf(F1,"\n");

			fclose(F);   fclose(F1);
	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }

      //trajektorije pokretnih prepreka - X,Y koordinate
	  if (((F=fopen(LOG_POKRETNE_PREPREKE_POINT_X,"wt"))!=NULL)&&((F1=fopen(LOG_POKRETNE_PREPREKE_POINT_Y,"wt"))!=NULL)&&((F2=fopen(LOG_POKRETNE_PREPREKE_INDEKS,"wt"))!=NULL)) {
      
		for(j=0;j<broj_pokretnih_prepreka;j++){
		fprintf(F2,"%d ",PPP[j].indeks);
			for(k=0;k<N_KL;k++){

				fprintf(F,"%f ",MO[j].x[k]);fprintf(F1,"%f ",MO[j].y[k]);
					}
			}
			fprintf(F,"\n");  fprintf(F1,"\n"); fprintf(F2,"\n");

			fclose(F);   fclose(F1); fclose(F2);
	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }

   
    //sve DW trajektorije u trenutnom ciklusu logiranja - X,Y koordinate i sve DW BREAKAGE trajektorije u trenutnom ciklusu logiranja - X,Y koordinate
      if (((F=fopen(LOG_PUTANJE_X,"wt"))!=NULL)&&((F1=fopen(LOG_PUTANJE_Y,"wt"))!=NULL)&&((F_temp1=fopen(LOG_PUTANJE_X_MINUS,"wt"))!=NULL)&&((F_temp2=fopen(LOG_PUTANJE_Y_MINUS,"wt"))!=NULL)&&((F14=fopen(LOG_PUTANJE_X_MINUS2,"wt"))!=NULL)&&((F15=fopen(LOG_PUTANJE_Y_MINUS2,"wt"))!=NULL)&&((F10=fopen(LOG_PUTANJE_V_MINUS,"wt"))!=NULL)&&((F11=fopen(LOG_PUTANJE_W_MINUS,"wt"))!=NULL)&&((F16=fopen(LOG_PUTANJE_V_MINUS2,"wt"))!=NULL)&&((F17=fopen(LOG_PUTANJE_W_MINUS2,"wt"))!=NULL)&&((F2=fopen(LOG_PUTANJE_X_PLUS,"wt"))!=NULL)&&((F3=fopen(LOG_PUTANJE_Y_PLUS,"wt"))!=NULL)&&((F12=fopen(LOG_PUTANJE_V_PLUS,"wt"))!=NULL)&&((F13=fopen(LOG_PUTANJE_W_PLUS,"wt"))!=NULL)&&((F61=fopen(LOG_PUTANJE_TH_PLUS,"wt"))!=NULL)&&((F8=fopen(LOG_PUTANJE_S_PLUS,"wt"))!=NULL)&&((F44=fopen(LOG_PUTANJE_VY_PLUS,"wt"))!=NULL)&&((F19=fopen(LOG_PUTANJE_X_PLUS1,"wt"))!=NULL)&&((F20=fopen(LOG_PUTANJE_Y_PLUS1,"wt"))!=NULL)&&((F21=fopen(LOG_PUTANJE_V_PLUS1,"wt"))!=NULL)&&((F22=fopen(LOG_PUTANJE_W_PLUS1,"wt"))!=NULL)&&((F23=fopen(LOG_PUTANJE_TH_PLUS1,"wt"))!=NULL)&&((F24=fopen(LOG_PUTANJE_S_PLUS1,"wt"))!=NULL)&&((F25=fopen(LOG_PUTANJE_VY_PLUS1,"wt"))!=NULL)&&((F26=fopen(LOG_PUTANJE_X_PLUS2,"wt"))!=NULL)&&((F27=fopen(LOG_PUTANJE_Y_PLUS2,"wt"))!=NULL)&&((F28=fopen(LOG_PUTANJE_V_PLUS2,"wt"))!=NULL)&&((F29=fopen(LOG_PUTANJE_W_PLUS2,"wt"))!=NULL)&&((F30=fopen(LOG_PUTANJE_TH_PLUS2,"wt"))!=NULL)&&((F31=fopen(LOG_PUTANJE_S_PLUS2,"wt"))!=NULL)&&((F32=fopen(LOG_PUTANJE_VY_PLUS2,"wt"))!=NULL)&&((F4=fopen(LOG_PUTANJE_V,"wt"))!=NULL)&&((F5=fopen(LOG_PUTANJE_W,"wt"))!=NULL)&&((F6=fopen(LOG_PUTANJE_TH,"wt"))!=NULL)&&((F62=fopen(LOG_PUTANJE_TH_MINUS,"wt"))!=NULL)&&((F63=fopen(LOG_PUTANJE_TH_MINUS2,"wt"))!=NULL)&&((F7=fopen(LOG_PUTANJE_S_MINUS,"wt"))!=NULL)&&((F18=fopen(LOG_PUTANJE_S_MINUS2,"wt"))!=NULL)&&((F9=fopen(LOG_PUTANJE_S,"wt"))!=NULL)&&((F41=fopen(LOG_PUTANJE_VY,"wt"))!=NULL)&&((F42=fopen(LOG_PUTANJE_VY_MINUS,"wt"))!=NULL)&&((F43=fopen(LOG_PUTANJE_VY_MINUS2,"wt"))!=NULL)) {
      //trajektorije po svim brzinama
#if OMNIDRIVE
	for (l=0;l<VY_DIM+1;l++)
#endif
	{
		for(j=0;j<V_DIM+1;j++){
			for(k=0;k<W_DIM+1;k++){

           //po svim tockama trajektorije

          for(i=0;i<N_KL;i++){
#if OMNIDRIVE
			  fprintf(F,"%f ",Log.xkl[i][j][k][l]); fprintf(F1,"%f ",Log.ykl[i][j][k][l]);
			  fprintf(F_temp1,"%f ",Log_minus.xkl[i][j][k][l]);fprintf(F_temp2,"%f ",Log_minus.ykl[i][j][k][l]);
			  fprintf(F14,"%f ",Log_minus2.xkl[i][j][k][l]);fprintf(F15,"%f ",Log_minus2.ykl[i][j][k][l]);
			  fprintf(F10,"%f ",Log_minus.vkl[i][j][k][l]); fprintf(F11,"%f ",Log_minus.wkl[i][j][k][l]);
			  fprintf(F16,"%f ",Log_minus2.vkl[i][j][k][l]); fprintf(F17,"%f ",Log_minus2.wkl[i][j][k][l]);
			  fprintf(F2,"%f ",Log_plus.xkl[i][j][k][l]); fprintf(F3,"%f ",Log_plus.ykl[i][j][k][l]);
			  fprintf(F12,"%f ",Log_plus.vkl[i][j][k][l]); fprintf(F13,"%f ",Log_plus.wkl[i][j][k][l]);
#if ROT
			  fprintf(F19,"%f ",Log_plus1.xkl[i][j][k][l]); fprintf(F20,"%f ",Log_plus1.ykl[i][j][k][l]); fprintf(F21,"%f ",Log_plus1.vkl[i][j][k][l]); fprintf(F22,"%f ",Log_plus1.wkl[i][j][k][l]); fprintf(F23,"%f ",Log_plus1.th[i][j][k][l]); fprintf(F24,"%f ",Log_plus1.Skl[i][j][k][l]); fprintf(F25,"%f ",Log_plus1.vykl[i][j][k][l]);
			  fprintf(F26,"%f ",Log_plus2.xkl[i][j][k][l]); fprintf(F27,"%f ",Log_plus2.ykl[i][j][k][l]); fprintf(F28,"%f ",Log_plus2.vkl[i][j][k][l]); fprintf(F29,"%f ",Log_plus2.wkl[i][j][k][l]); fprintf(F30,"%f ",Log_plus2.th[i][j][k][l]); fprintf(F31,"%f ",Log_plus2.Skl[i][j][k][l]); fprintf(F32,"%f ",Log_plus2.vykl[i][j][k][l]);
#endif
			  fprintf(F4,"%f ",Log.vkl[i][j][k][l]); fprintf(F5,"%f ",Log.wkl[i][j][k][l]);fprintf(F6,"%f ",Log.th[i][j][k][l]);
			  fprintf(F61,"%f ",Log_plus.th[i][j][k][l]); fprintf(F62,"%f ",Log_minus.th[i][j][k][l]);
			  fprintf(F63,"%f ",Log_minus2.th[i][j][k][l]);
			  fprintf(F7,"%f ",Log_minus.Skl[i][j][k][l]);fprintf(F8,"%f ",Log_plus.Skl[i][j][k][l]);
			  fprintf(F18,"%f ",Log_minus2.Skl[i][j][k][l]);
			  fprintf(F9,"%f ",Log.Skl[i][j][k][l]);
			  fprintf(F41,"%f ",Log.vykl[i][j][k][l]);fprintf(F42,"%f ",Log_minus.vykl[i][j][k][l]);fprintf(F43,"%f ",Log_minus2.vykl[i][j][k][l]);fprintf(F44,"%f ",Log_plus.vykl[i][j][k][l]);

#else
			  fprintf(F,"%f ",Log.xkl[i][j][k]); fprintf(F1,"%f ",Log.ykl[i][j][k]);
			  fprintf(F_temp1,"%f ",Log_minus.xkl[i][j][k]);fprintf(F_temp2,"%f ",Log_minus.ykl[i][j][k]);
			  fprintf(F14,"%f ",Log_minus2.xkl[i][j][k]);fprintf(F15,"%f ",Log_minus2.ykl[i][j][k]);
			  fprintf(F10,"%f ",Log_minus.vkl[i][j][k]); fprintf(F11,"%f ",Log_minus.wkl[i][j][k]);
			  fprintf(F16,"%f ",Log_minus2.vkl[i][j][k]); fprintf(F17,"%f ",Log_minus2.wkl[i][j][k]);
			  fprintf(F2,"%f ",Log_plus.xkl[i][j][k]); fprintf(F3,"%f ",Log_plus.ykl[i][j][k]);
			  fprintf(F12,"%f ",Log_plus.vkl[i][j][k]); fprintf(F13,"%f ",Log_plus.wkl[i][j][k]);
#if ROT
			  fprintf(F19,"%f ",Log_plus1.xkl[i][j][k]); fprintf(F20,"%f ",Log_plus1.ykl[i][j][k]); fprintf(F21,"%f ",Log_plus1.vkl[i][j][k]); fprintf(F22,"%f ",Log_plus1.wkl[i][j][k]); fprintf(F23,"%f ",Log_plus1.th[i][j][k]); fprintf(F24,"%f ",Log_plus1.Skl[i][j][k]); fprintf(F25,"%f ",Log_plus1.vykl[i][j][k]);
			  fprintf(F26,"%f ",Log_plus2.xkl[i][j][k]); fprintf(F27,"%f ",Log_plus2.ykl[i][j][k]); fprintf(F28,"%f ",Log_plus2.vkl[i][j][k]); fprintf(F29,"%f ",Log_plus2.wkl[i][j][k]); fprintf(F30,"%f ",Log_plus2.th[i][j][k]); fprintf(F31,"%f ",Log_plus2.Skl[i][j][k]); fprintf(F32,"%f ",Log_plus2.vykl[i][j][k]);
#endif
			  fprintf(F4,"%f ",Log.vkl[i][j][k]); fprintf(F5,"%f ",Log.wkl[i][j][k]);fprintf(F6,"%f ",Log.th[i][j][k]);
			  fprintf(F61,"%f ",Log_plus.th[i][j][k]); fprintf(F62,"%f ",Log_minus.th[i][j][k]);
			  fprintf(F63,"%f ",Log_minus2.th[i][j][k]);
			  fprintf(F7,"%f ",Log_minus.Skl[i][j][k]);fprintf(F8,"%f ",Log_plus.Skl[i][j][k]);
			  fprintf(F18,"%f ",Log_minus2.Skl[i][j][k]);
			  fprintf(F9,"%f ",Log.Skl[i][j][k]);
#endif
					}
					fprintf(F,"\n");  fprintf(F1,"\n");  fprintf(F2,"\n");  fprintf(F3,"\n");
					fprintf(F4,"\n");fprintf(F5,"\n");fprintf(F6,"\n"); fprintf(F_temp1,"\n");fprintf(F_temp2,"\n");
					fprintf(F61,"\n");fprintf(F62,"\n");fprintf(F63,"\n");
					fprintf(F7,"\n");fprintf(F8,"\n");fprintf(F9,"\n");
					fprintf(F10,"\n");fprintf(F11,"\n");fprintf(F12,"\n");fprintf(F13,"\n");
					fprintf(F14,"\n");fprintf(F15,"\n");fprintf(F16,"\n");fprintf(F17,"\n");fprintf(F18,"\n");fprintf(F41,"\n");fprintf(F42,"\n");fprintf(F43,"\n");fprintf(F44,"\n");
					fprintf(F19,"\n");fprintf(F20,"\n");fprintf(F21,"\n");fprintf(F22,"\n");fprintf(F23,"\n");fprintf(F24,"\n");fprintf(F25,"\n");fprintf(F26,"\n");fprintf(F27,"\n");fprintf(F28,"\n");fprintf(F29,"\n");fprintf(F30,"\n");fprintf(F31,"\n");fprintf(F32,"\n");
			}
		}

	}

		fclose(F);  fclose(F1);   fclose(F2);  fclose(F3); fclose(F4); fclose(F5); fclose(F6);
		fclose(F61); fclose(F62); fclose(F63);
		fclose(F_temp1); fclose(F_temp2); fclose(F7);fclose(F8);fclose(F9);
		fclose(F10);fclose(F11);fclose(F12);fclose(F13);
		fclose(F14); fclose(F15); fclose(F16); fclose(F17); fclose(F18); 
		fclose(F41);fclose(F42);fclose(F43);fclose(F44);
		fclose(F19);fclose(F20);fclose(F21);fclose(F22);fclose(F23);fclose(F24);fclose(F25);fclose(F26);fclose(F27);fclose(F28);fclose(F29);fclose(F30);fclose(F31);fclose(F32);
	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }

      if (((F=fopen(LOG_PUTANJE_X_OLD,"wt"))!=NULL)&&((F1=fopen(LOG_PUTANJE_Y_OLD,"wt"))!=NULL)&&((F61=fopen(LOG_PUTANJE_TH_OLD,"wt"))!=NULL)&&((F2=fopen(LOG_OPT_PUTANJE_X,"wt"))!=NULL)&&((F3=fopen(LOG_OPT_PUTANJE_Y,"wt"))!=NULL)&&((F62=fopen(LOG_OPT_PUTANJE_TH,"wt"))!=NULL)&&((F4=fopen(LOG_OPT_PUTANJE_S,"wt"))!=NULL)&&((F5=fopen(LOG_OLD_PUTANJE_S,"wt"))!=NULL)&&((F6=fopen(LOG_OLD_PUTANJE_V,"wt"))!=NULL)&&((F7=fopen(LOG_OLD_PUTANJE_VY,"wt"))!=NULL)&&((F8=fopen(LOG_OLD_PUTANJE_W,"wt"))!=NULL)&&((F9=fopen(LOG_OPT_PUTANJE_V,"wt"))!=NULL)&&((F10=fopen(LOG_OPT_PUTANJE_VY,"wt"))!=NULL)&&((F11=fopen(LOG_OPT_PUTANJE_W,"wt"))!=NULL)&&((F12=fopen(LOG_PUTANJE_X_BREAK,"wt"))!=NULL)&&((F13=fopen(LOG_PUTANJE_Y_BREAK,"wt"))!=NULL)&&((F14=fopen(LOG_PUTANJE_TH_BREAK,"wt"))!=NULL)&&((F15=fopen(LOG_BREAK_PUTANJE_S,"wt"))!=NULL)&&((F16=fopen(LOG_BREAK_PUTANJE_V,"wt"))!=NULL)&&((F17=fopen(LOG_BREAK_PUTANJE_VY,"wt"))!=NULL)&&((F18=fopen(LOG_BREAK_PUTANJE_W,"wt"))!=NULL)) {
      //trajektorije po svim brzinama

           //po svim tockama trajektorije

			      for(i=0;i<N_KL;i++){
				      fprintf(F,"%f ",KL_old.x[i]); fprintf(F1,"%f ",KL_old.y[i]);fprintf(F2,"%f ",KL.x[i]); fprintf(F3,"%f ",KL.y[i]);fprintf(F4,"%f ",KL.S[i]);fprintf(F5,"%f ",KL_old.S[i]);
				      fprintf(F6,"%f ",KL_old.v[i]);fprintf(F7,"%f ",KL_old.vy[i]);fprintf(F8,"%f ",KL_old.w[i]);
				      fprintf(F9,"%f ",KL.v[i]);fprintf(F10,"%f ",KL.vy[i]);fprintf(F11,"%f ",KL.w[i]);
				      fprintf(F61,"%f ",KL_old.th[i]);fprintf(F62,"%f ",KL.th[i]);
				      fprintf(F12,"%f ",KL_breakage.x[i]); fprintf(F13,"%f ",KL_breakage.y[i]);
				      fprintf(F14,"%f ",KL_breakage.th[i]);
				      fprintf(F15,"%f ",KL_breakage.S[i]);
				      fprintf(F16,"%f ",KL_breakage.v[i]);fprintf(F17,"%f ",KL_breakage.vy[i]);fprintf(F18,"%f ",KL_breakage.w[i]);
			      }
			      fprintf(F,"\n");  fprintf(F1,"\n"); fprintf(F2,"\n");  fprintf(F3,"\n"); fprintf(F4,"\n");fprintf(F5,"\n");fprintf(F6,"\n");fprintf(F7,"\n");fprintf(F8,"\n");fprintf(F9,"\n");fprintf(F10,"\n");fprintf(F11,"\n");fprintf(F12,"\n");fprintf(F13,"\n");fprintf(F14,"\n");fprintf(F15,"\n");fprintf(F16,"\n");fprintf(F17,"\n");fprintf(F18,"\n");
			      fprintf(F61,"\n"); fprintf(F62,"\n"); 
			      fclose(F);  fclose(F1);fclose(F2); fclose(F3); fclose(F4);fclose(F5);
			      fclose(F61);fclose(F62);fclose(F6);fclose(F7);fclose(F8);fclose(F9);fclose(F10);fclose(F11);
			      fclose(F12);fclose(F13);fclose(F14);fclose(F15);fclose(F16);fclose(F17);fclose(F18);
      }else{
	      printf("Otvaranje filea bezuspjesno!");
      }

     //BREAKAGE POINTOVI  -X,Y,TH koordinate tocaka zaustavljanja po pojedinoj trajektoriji (zapravo redundantno s ovim gore breakage trajektorijama)
 
 if (((F=fopen(LOG_BREAKAGE_POINT_X,"wt"))!=NULL)&&((F1=fopen(LOG_BREAKAGE_POINT_Y,"wt"))!=NULL)&&((F2=fopen(LOG_BREAKAGE_POINT_TH,"wt"))!=NULL)) {
      //putanje po svim brzinama
#if (OMNIDRIVE==0)
	 for(j=0;j<V_DIM+1;j++){
		 for(k=0;k<W_DIM+1;k++){

			 fprintf(F,"%f ",TB.breakage_point_x[j][k]);fprintf(F1,"%f ",TB.breakage_point_y[j][k]); fprintf(F2,"%f ",TB.breakage_point_th[j][k]);
		 }

	 }
#endif
	 fprintf(F,"\n");  fprintf(F1,"\n"); fprintf(F2,"\n");
		
	 fclose(F);  fclose(F1); fclose(F2);
 }else{
	 printf("Otvaranje filea bezuspjesno!");
 }


   //efektivna putanja u trenutnom ciklusu logiranja - X,Y koordinate
 if (((F=fopen(LOG_PATH_PUTANJE_X,"wt"))!=NULL)&&((F1=fopen(LOG_PATH_PUTANJE_Y,"wt"))!=NULL)) {
    
       for(int i=0; i<N_PATH_REDUCED;i++){
		   fprintf(F,"%f ",path_r_reduced[i].x); 
		   fprintf(F1,"%f ",path_r_reduced[i].y);
		    }
			fprintf(F,"\n");  fprintf(F1,"\n");
			fclose(F);  fclose(F1);
	  }else{
    printf("Otvaranje filea bezuspjesno!");
    }
   //dstar putanja u trenutnom ciklusu logiranja - X,Y koordinate
 if (((F=fopen(LOG_DSTAR_PATH_PUTANJE_X,"wt"))!=NULL)&&((F1=fopen(LOG_DSTAR_PATH_PUTANJE_Y,"wt"))!=NULL)) {
    
       for(int i=0; i<PL->GetPathLength();i++){
		   fprintf(F,"%f ",PL->path_r[i].x); 
		   fprintf(F1,"%f ",PL->path_r[i].y);
		    }
			fprintf(F,"\n");  fprintf(F1,"\n");
			fclose(F);  fclose(F1);
	  }else{
    printf("Otvaranje filea bezuspjesno!");
    }

      //nadjena druga tocka infleksije na globalnom geometrijskom putu - X,Y koordinate
    if((F=fopen(LOG_TOCKA_INFLEKSIJE,"wt"))!=NULL){
    
        fprintf(F,"%f %f",tocka_infleksije.x, tocka_infleksije.y);
		fprintf(F,"\n");
		fclose(F);
	  }else{
    printf("Otvaranje filea bezuspjesno!");
    }

	//sve linearne i rotacijske brzine dinamickog prozora u trenutnom ciklusu logiranja
	if (((F=fopen(LOG_LIN_BRZINE,"wt"))!=NULL)&&((F2=fopen(LOG_LINY_BRZINE,"wt"))!=NULL)&&((F1=fopen(LOG_ROT_BRZINE,"wt"))!=NULL)) {
		for(j=0;j<V_DIM+1;j++){
			fprintf(F,"%f ",TB.v[j]);
		}
      fprintf(F,"\n");
		fclose(F);
		for(j=0;j<VY_DIM+1;j++){
			fprintf(F2,"%f ",TB.vy[j]);
		}
      fprintf(F2,"\n");
		fclose(F2);
		for(j=0;j<W_DIM+1;j++){
			fprintf(F1,"%f ",TB.w[j]);
		}
		fprintf(F1,"\n");
		fclose(F1);

	  }else{
    printf("Otvaranje filea bezuspjesno!");
     }

	 //zastavice, vrijednosti ukupne ocjene, vrijednosti ocjene prohodnosti, vrijednosti ocjene poklapanja s efektivnom putanjom svakog para brzina u trenutnom ciklusu logiranja
     if (((F=fopen(LOG_TB_FLAG,"wt"))!=NULL)&&((F4=fopen(LOG_TB_FLAG_PLUS,"wt"))!=NULL)&&((F19=fopen(LOG_TB_FLAG_PLUS1,"wt"))!=NULL)&&((F20=fopen(LOG_TB_FLAG_PLUS2,"wt"))!=NULL)&&((F5=fopen(LOG_TB_FLAG_MINUS,"wt"))!=NULL)&&((F6=fopen(LOG_TB_FLAG_MINUS2,"wt"))!=NULL)&&((F1=fopen(LOG_TB_OCJENA,"wt"))!=NULL)&&((F2=fopen(LOG_TB_OCJENA_PROHODNOST,"wt"))!=NULL)&&((F3=fopen(LOG_TB_OCJENA_PATH,"wt"))!=NULL)) {
      //putanje po svim brzinama
#if OMNIDRIVE
	for (l=0;l<VY_DIM+1;l++)
#endif
	{
		for(j=0;j<V_DIM+1;j++){
			for(k=0;k<W_DIM+1;k++){
#if OMNIDRIVE
				fprintf(F,"%d ",TB.flag[j][k][l]);
				fprintf(F4,"%d ",TB_plus.flag[j][k][l]);
#if ROT
				fprintf(F19,"%d ",TB_plus1.flag[j][k][l]);
				fprintf(F20,"%d ",TB_plus2.flag[j][k][l]);
#endif
				fprintf(F5,"%d ",TB_minus.flag[j][k][l]);
				fprintf(F6,"%d ",TB_minus2.flag[j][k][l]);
				fprintf(F1,"%f ",TB.ocjena[j][k][l]);
				fprintf(F2,"%f ",TB.ocjena_prohodnost[j][k][l]);
				fprintf(F3,"%f ",TB.ocjena_path[j][k][l]);
#else
				fprintf(F,"%d ",TB.flag[j][k]);
				fprintf(F4,"%d ",TB_plus.flag[j][k]);
#if ROT
				fprintf(F19,"%d ",TB_plus1.flag[j][k]);
				fprintf(F20,"%d ",TB_plus2.flag[j][k]);
#endif
				fprintf(F5,"%d ",TB_minus.flag[j][k]);
				fprintf(F6,"%d ",TB_minus2.flag[j][k]);
				fprintf(F1,"%f ",TB.ocjena[j][k]);
				fprintf(F2,"%f ",TB.ocjena_prohodnost[j][k]);
				fprintf(F3,"%f ",TB.ocjena_path[j][k]);
#endif
			}
			fprintf(F,"\n"); fprintf(F4,"\n"); fprintf(F5,"\n");  fprintf(F1,"\n");fprintf(F2,"\n"); fprintf(F3,"\n");fprintf(F6,"\n"); fprintf(F19,"\n");fprintf(F20,"\n");
		}
	}
		fprintf(F,"\n");fprintf(F4,"\n"); fprintf(F5,"\n");  fprintf(F1,"\n"); fprintf(F2,"\n"); fprintf(F3,"\n");fprintf(F6,"\n");fprintf(F19,"\n");fprintf(F20,"\n");
		fclose(F);fclose(F4); fclose(F5); fclose(F1); fclose(F2);fclose(F3); fclose(F6);
		fclose(F19);fclose(F20);
	  }else{
    printf("Otvaranje filea bezuspjesno!");
     }

         //indeksi optimalnog para brzina (uvecani za 1 zbog crtanja u matlabu), dimenzije prostora translacijskih i rotacijskih brzina, trenutna brzina robota
    if((F=fopen(LOG,"wt"))!=NULL){
       fprintf(F,"%d %d %d %d %f %f %f %d %d", ni+1, nj+1, V_DIM+1, W_DIM+1, RB.v, RB.w, RB.vy, nk+1, VY_DIM+1);
         fprintf(F,"\n");
		
		fclose(F);
     }else{
    printf("Otvaranje filea bezuspjesno!");
    }

      //robotova trenutna pozicija (u trenutku logiranja)
    if((F=fopen(LOG_ROBOT_POSITION,"wt"))!=NULL){

					fprintf(F,"%f %f %f", RB.x, RB.y, RB.th);
      fprintf(F,"\n");
      		fclose(F);
		
   }else{
    printf("Otvaranje filea bezuspjesno!");
    }

      //laser prepreke - X,Y koordinate (u trenutnom ciklusu logiranja)
    if((F=fopen(LOG_LASER_OBSTACLES,"wt"))!=NULL){
      for(i=0;i<laser_pointer_end;i++){
          fprintf(F,"%f %f\n", OPF[i].x, OPF[i].y);
        }
      fprintf(F,"\n");
		
		fclose(F);
     }else{
    printf("Otvaranje filea bezuspjesno!");
    }
 return;
}

double DynamicWindow::round(double x){
	if ((x-floor(x))>=0.5){
		x=floor(x)+1;
	}else{
		x=floor(x);
	}
	return x;
}
