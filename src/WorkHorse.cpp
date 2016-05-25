#include "WorkHorse.h"      //Planner.h, Params.h je unutra
#include "DynamicWindow.h"      //Planner.h, Params.h
#include "GridMap.h"    //
#include "DStar.h"    //Params.h Planner
#include "moj.h" //Planner.h, Params.h, DStar.h



DynamicWindow *DW;
Planner *PL;
GridMap *GM;
extern moj *M;
DStar   *DS;


double* reallociraj_double(double *array, int new_size, bool first)
{
  if (first){
    array = (double *)malloc(new_size * sizeof( double ));
  }else{
    array = (double *)realloc( array, new_size * sizeof( double ) );
  }
  if (array==NULL){
    printf("WorkHorse> Error allocating memory FIRST (1) or REALLOC (0): %d\n",first);
    exit(1);
  }else{
    return array;
  }
//if(first){
// 	 if( (array = (double *)malloc(new_size * sizeof( double ))) == NULL ){
//			  	printf("WorkHorse> Error allocating memory FIRST!!!\n");
//         exit(1);
//         }else{
//	 	//printf("Array_value: x[0]=%f\n, x=%ld\n", array[0], array);
//		}
//}
// if( (array = (double *)realloc( array, new_size * sizeof( double ) )) == NULL ){
//			  	printf("WorkHorse> Error allocating memory REALLOC!!!\n");
//         exit(1);
//         }
//	 
//	 return array;

}


void WorkHorse::OslobodiMemoriju()
{

     //ovi podaci se uvijek koriste kod off/on-line varijante
      free(Log_robot_path.izracuni);
      free(Log_robot_path.DSizracuni);
      free(Log_robot_path.DSexplored);
      free(Log_robot_path.DWizracuni);
      free(Log_robot_path.path_x);
      free(Log_robot_path.path_y);
      free(Log_robot_path.path_th);
       free(Log_robot_path.laser_x);
 	  free(Log_robot_path.laser_y);
       free(Log_robot_path.laser2_x);
 	  free(Log_robot_path.laser2_y);
	  free(Log_robot_path.teziste_x);
	  free(Log_robot_path.teziste_y);
	  free(Log_robot_path.dwopttraj_x);
	  free(Log_robot_path.dwopttraj_y);
	  free(Log_robot_path.broj_dwopttraj);
	  free(Log_robot_path.broj_tezista);
	  free(Log_robot_path.broj_hitova);
	  free(Log_robot_path.broj_hitova2);
      free(Log_robot_path.v);
      free(Log_robot_path.vy);
      free(Log_robot_path.vy_ref);
      free(Log_robot_path.w);
      free(Log_robot_path.v_kal);
      free(Log_robot_path.w_kal);
      free(Log_robot_path.v_des);
      free(Log_robot_path.w_des);
      free(Log_robot_path.v_ref);
      free(Log_robot_path.w_ref);
      free(Log_robot_path.v_refdin);
      free(Log_robot_path.w_refdin);
      free(Log_robot_path.time);
	  free(Log_robot_path.read_time);
	  free(Log_robot_path.path_length);
	  free(Log_robot_path.path_orientation);
	  free(Log_robot_path.tocka_infleksije_x);
	  free(Log_robot_path.tocka_infleksije_y);
	  free(Log_robot_path.preslik_x);
	  free(Log_robot_path.preslik_y);
 	  free(Log_robot_path.ljapunov);
 	  free(Log_robot_path.interpolatedcost);
 	  free(Log_robot_path.replan);
 	  free(Log_robot_path.travcost);
 	  free(Log_robot_path.travcost3d);
 	  free(planner_path_run.size);
 	  
 	  free(planner_path_run.path_x);
 	  free(planner_path_run.path_y);
 	  free(planner_path_run.path_th);
 	  free(planner_path_run.path_travcost);
 	  free(planner_path_run.path_h);
#if DSTAR3D
    free(planner_path_run.path_intlo);
    free(planner_path_run.path_intup);
    free(planner_path_original.path_intlo);
    free(planner_path_original.path_intup);
#endif
 	  free(planner_path_original.path_x);
 	  free(planner_path_original.path_y);
 	  free(planner_path_original.path_th);
 	  free(planner_path_original.path_travcost);
 	  free(planner_path_original.path_h);
 	  
     //to su sve vrijednosti koje pratimo on-line

}


int WorkHorse::Logger_Init()
{
//inicijalizacija
    planner_path_original.path_x=NULL;
    planner_path_original.path_y=NULL;
    planner_path_original.path_th=NULL;
    planner_path_original.path_travcost=NULL;
    planner_path_original.path_h=NULL;
    planner_path_original.path_intlo=NULL;
    planner_path_original.path_intup=NULL;    
    planner_path_run.path_x=NULL;
    planner_path_run.path_y=NULL;
    planner_path_run.path_th=NULL;
    planner_path_run.path_travcost=NULL;
    planner_path_run.path_h=NULL;
    planner_path_run.path_intlo=NULL;
    planner_path_run.path_intup=NULL;    
    
    Log_robot_path.izracuni=NULL;
    Log_robot_path.DSizracuni=NULL;
    Log_robot_path.DSexplored=NULL;
    Log_robot_path.DWizracuni=NULL;
    Log_robot_path.path_x=NULL;
    Log_robot_path.path_y=NULL;
    Log_robot_path.path_th=NULL;
    Log_robot_path.laser_x=NULL;
    Log_robot_path.laser_y=NULL;
    Log_robot_path.laser2_x=NULL;
    Log_robot_path.laser2_y=NULL;
    Log_robot_path.teziste_x=NULL;
    Log_robot_path.teziste_y=NULL;
    Log_robot_path.broj_tezista=NULL;
    Log_robot_path.dwopttraj_x=NULL;
    Log_robot_path.dwopttraj_y=NULL;
    Log_robot_path.broj_dwopttraj=NULL;
    Log_robot_path.broj_hitova=NULL;
    Log_robot_path.broj_hitova2=NULL;
    Log_robot_path.v=NULL;
    Log_robot_path.vy=NULL;
    Log_robot_path.w=NULL;
    Log_robot_path.vy_ref=NULL;
    Log_robot_path.v_ref=NULL;
    Log_robot_path.w_ref=NULL;
    Log_robot_path.v_kal=NULL;
    Log_robot_path.w_kal=NULL;
    Log_robot_path.v_des=NULL;
    Log_robot_path.w_des=NULL;
    Log_robot_path.v_refdin=NULL;
    Log_robot_path.w_refdin=NULL;
    Log_robot_path.time=NULL;
    Log_robot_path.read_time=NULL;
    Log_robot_path.path_length=NULL;
    Log_robot_path.path_orientation=NULL;
	  Log_robot_path.tocka_infleksije_x=NULL;
	  Log_robot_path.tocka_infleksije_y=NULL;
	  Log_robot_path.ljapunov=NULL;
	  Log_robot_path.interpolatedcost=NULL;
	  Log_robot_path.replan=NULL;
	  Log_robot_path.travcost=NULL;
	  Log_robot_path.travcost3d=NULL;
	  Log_robot_path.preslik_x=NULL;
	  Log_robot_path.preslik_y=NULL;
	  planner_path_run.size=NULL;

//ovdje cemo allocirati sve strukture vezane za logger!!!
      //putanja robota
      printf("allocating new log variables\n");
      Log_robot_path.izracuni=reallociraj_double((Log_robot_path.izracuni),DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.DSizracuni=reallociraj_double((Log_robot_path.DSizracuni),DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.DSexplored=reallociraj_double((Log_robot_path.DSexplored),DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.DWizracuni=reallociraj_double((Log_robot_path.DWizracuni),DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.path_x=reallociraj_double((Log_robot_path.path_x), DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.path_y=reallociraj_double((Log_robot_path.path_y), DEFAULT_REALLOC_SIZE, true); 
	  Log_robot_path.path_th=reallociraj_double((Log_robot_path.path_th), DEFAULT_REALLOC_SIZE, true);
 	  Log_robot_path.laser_x=reallociraj_double((Log_robot_path.laser_x), DEFAULT_REALLOC_SIZE*PLAYER_LASER_MAX_SAMPLES, true);
 	  Log_robot_path.laser_y=reallociraj_double((Log_robot_path.laser_y), DEFAULT_REALLOC_SIZE*PLAYER_LASER_MAX_SAMPLES, true);
 	  Log_robot_path.laser2_x=reallociraj_double((Log_robot_path.laser2_x), DEFAULT_REALLOC_SIZE*PLAYER_LASER_MAX_SAMPLES, true);
 	  Log_robot_path.laser2_y=reallociraj_double((Log_robot_path.laser2_y), DEFAULT_REALLOC_SIZE*PLAYER_LASER_MAX_SAMPLES, true);
	  Log_robot_path.teziste_x=reallociraj_double((Log_robot_path.teziste_x), DEFAULT_REALLOC_SIZE*PLAYER_LASER_MAX_SAMPLES, true);
	  Log_robot_path.teziste_y=reallociraj_double((Log_robot_path.teziste_y), DEFAULT_REALLOC_SIZE*PLAYER_LASER_MAX_SAMPLES, true);
	  Log_robot_path.broj_tezista=reallociraj_double((Log_robot_path.broj_tezista), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.dwopttraj_x=reallociraj_double((Log_robot_path.dwopttraj_x), DEFAULT_REALLOC_SIZE*PLAYER_LASER_MAX_SAMPLES, true);
	  Log_robot_path.dwopttraj_y=reallociraj_double((Log_robot_path.dwopttraj_y), DEFAULT_REALLOC_SIZE*PLAYER_LASER_MAX_SAMPLES, true);
	  Log_robot_path.broj_dwopttraj=reallociraj_double((Log_robot_path.broj_dwopttraj), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.broj_hitova=reallociraj_double((Log_robot_path.broj_hitova), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.broj_hitova2=reallociraj_double((Log_robot_path.broj_hitova2), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.v=reallociraj_double((Log_robot_path.v), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.vy=reallociraj_double((Log_robot_path.vy), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.vy_ref=reallociraj_double((Log_robot_path.vy_ref), DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.w=reallociraj_double((Log_robot_path.w), DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.v_kal=reallociraj_double((Log_robot_path.v_kal), DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.w_kal=reallociraj_double((Log_robot_path.w_kal), DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.v_ref=reallociraj_double((Log_robot_path.v_ref), DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.w_ref=reallociraj_double((Log_robot_path.w_ref), DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.v_des=reallociraj_double((Log_robot_path.v_des), DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.w_des=reallociraj_double((Log_robot_path.w_des), DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.v_refdin=reallociraj_double((Log_robot_path.v_refdin), DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.w_refdin=reallociraj_double((Log_robot_path.w_refdin), DEFAULT_REALLOC_SIZE, true);
      Log_robot_path.time=reallociraj_double((Log_robot_path.time), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.read_time=reallociraj_double((Log_robot_path.read_time), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.path_length=reallociraj_double((Log_robot_path.path_length), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.path_orientation=reallociraj_double((Log_robot_path.path_orientation), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.tocka_infleksije_x=reallociraj_double((Log_robot_path.tocka_infleksije_x), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.tocka_infleksije_y=reallociraj_double((Log_robot_path.tocka_infleksije_y), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.ljapunov=reallociraj_double((Log_robot_path.ljapunov), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.interpolatedcost=reallociraj_double((Log_robot_path.interpolatedcost), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.replan=reallociraj_double((Log_robot_path.replan), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.travcost=reallociraj_double((Log_robot_path.travcost), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.travcost3d=reallociraj_double((Log_robot_path.travcost3d), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.preslik_x=reallociraj_double((Log_robot_path.preslik_x), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.preslik_y=reallociraj_double((Log_robot_path.preslik_y), DEFAULT_REALLOC_SIZE, true);
	  Log_robot_path.size_path=DEFAULT_REALLOC_SIZE;
	  Log_robot_path.max_path_run=DEFAULT_REALLOC_SIZE;
	  Log_robot_path.max_laser=(DEFAULT_REALLOC_SIZE-1)*PLAYER_LASER_MAX_SAMPLES;
	  Log_robot_path.max_laser2=(DEFAULT_REALLOC_SIZE-1)*PLAYER_LASER_MAX_SAMPLES;
	  Log_robot_path.max_teziste=(DEFAULT_REALLOC_SIZE-1)*PLAYER_LASER_MAX_SAMPLES;
	  Log_robot_path.max_dwopttraj=(DEFAULT_REALLOC_SIZE-1)*PLAYER_LASER_MAX_SAMPLES;
	  Log_robot_path.size_laser=0;
	  Log_robot_path.size_laser2=0;
	  Log_robot_path.size_teziste=0;
	  Log_robot_path.size_dwopttraj=0;
	  planner_path_run.size=reallociraj_double((planner_path_run.size), DEFAULT_REALLOC_SIZE, true);
      return Log_robot_path.size_path;
}

int WorkHorse::Logger_Realloc()
{
//ovdje cemo allocirati sve strukture vezane za logger!!!
      Log_robot_path.izracuni=reallociraj_double((Log_robot_path.izracuni),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.DSizracuni=reallociraj_double((Log_robot_path.DSizracuni),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.DSexplored=reallociraj_double((Log_robot_path.DSexplored),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.DWizracuni=reallociraj_double((Log_robot_path.DWizracuni),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.path_x=reallociraj_double((Log_robot_path.path_x),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.path_y=reallociraj_double((Log_robot_path.path_y),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	Log_robot_path.path_th=reallociraj_double((Log_robot_path.path_th),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	Log_robot_path.broj_hitova=reallociraj_double((Log_robot_path.broj_hitova),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	Log_robot_path.broj_hitova2=reallociraj_double((Log_robot_path.broj_hitova2),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.broj_tezista=reallociraj_double((Log_robot_path.broj_tezista),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.broj_dwopttraj=reallociraj_double((Log_robot_path.broj_dwopttraj), Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.v=reallociraj_double((Log_robot_path.v),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
Log_robot_path.vy=reallociraj_double((Log_robot_path.vy),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
Log_robot_path.vy_ref=reallociraj_double((Log_robot_path.vy_ref),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.w=reallociraj_double((Log_robot_path.w),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.v_kal=reallociraj_double((Log_robot_path.v_kal), Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.w_kal=reallociraj_double((Log_robot_path.w_kal), Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.v_ref=reallociraj_double((Log_robot_path.v_ref),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.w_ref=reallociraj_double((Log_robot_path.w_ref),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.v_des=reallociraj_double((Log_robot_path.v_des),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.w_des=reallociraj_double((Log_robot_path.w_des),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.v_refdin=reallociraj_double((Log_robot_path.v_refdin),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.w_refdin=reallociraj_double((Log_robot_path.w_refdin),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
      Log_robot_path.time=reallociraj_double((Log_robot_path.time),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.read_time=reallociraj_double((Log_robot_path.read_time),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.path_length=reallociraj_double((Log_robot_path.path_length),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.path_orientation=reallociraj_double((Log_robot_path.path_orientation),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.tocka_infleksije_x=reallociraj_double((Log_robot_path.tocka_infleksije_x),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.tocka_infleksije_y=reallociraj_double((Log_robot_path.tocka_infleksije_y),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.ljapunov=reallociraj_double((Log_robot_path.ljapunov),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.interpolatedcost=reallociraj_double((Log_robot_path.interpolatedcost),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.replan=reallociraj_double((Log_robot_path.replan),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.travcost=reallociraj_double((Log_robot_path.travcost),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.travcost3d=reallociraj_double((Log_robot_path.travcost3d),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.preslik_x=reallociraj_double((Log_robot_path.preslik_x),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
	  Log_robot_path.preslik_y=reallociraj_double((Log_robot_path.preslik_y),Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);
//	  planner_path_run.size=reallociraj_double((planner_path_run.size), Log_robot_path.size_path+REALLOC_SIZE_INCREMENT, false);

	  Log_robot_path.size_path+=REALLOC_SIZE_INCREMENT;
      return Log_robot_path.size_path;
}

void WorkHorse::LogCurrent(int process_increment)
{


   if(process_increment>=logger_current_size){
     printf("WH logcurrent:> realloc: logger_current_size=%ld, for increment=%d\n",logger_current_size,REALLOC_SIZE_INCREMENT);
   	logger_current_size=Logger_Realloc();
      }

 	  if(planner_path_run.broj>= Log_robot_path.max_path_run-4){
		  printf("WH logcurrent:> realloc: path_run.broj=%d, for increment=%d\n",planner_path_run.broj,REALLOC_SIZE_INCREMENT);
		  Log_robot_path.max_path_run+=REALLOC_SIZE_INCREMENT;
	    planner_path_run.size=reallociraj_double((planner_path_run.size), Log_robot_path.max_path_run, false);
	  }
//#if (BEZ_LASERA==1)
	  if(Log_robot_path.size_laser>= Log_robot_path.max_laser-PLAYER_LASER_MAX_SAMPLES){//zato sto sad mozemo upisati jos brdo tocaka
		  printf("WH logcurrent:> realloc: laser_size=%d, for increment=%d\n",Log_robot_path.size_laser,REALLOC_SIZE_INCREMENT*PLAYER_LASER_MAX_SAMPLES);
		  Log_robot_path.max_laser+=REALLOC_SIZE_INCREMENT*PLAYER_LASER_MAX_SAMPLES;
		  Log_robot_path.laser_x=reallociraj_double((Log_robot_path.laser_x),Log_robot_path.max_laser, false);
		  Log_robot_path.laser_y=reallociraj_double((Log_robot_path.laser_y),Log_robot_path.max_laser, false);
	  }

//#endif
//laser2
//#if (BEZ_LASERA==1)
	  if(Log_robot_path.size_laser2>= Log_robot_path.max_laser2-PLAYER_LASER_MAX_SAMPLES){//zato sto sad mozemo upisati jos brdo tocaka
		  printf("WH logcurrent:> realloc: laser_size2=%d, for increment=%d\n",Log_robot_path.size_laser2,REALLOC_SIZE_INCREMENT*PLAYER_LASER_MAX_SAMPLES);
		  Log_robot_path.max_laser2+=REALLOC_SIZE_INCREMENT*PLAYER_LASER_MAX_SAMPLES;
		  Log_robot_path.laser2_x=reallociraj_double((Log_robot_path.laser2_x),Log_robot_path.max_laser2, false);
		  Log_robot_path.laser2_y=reallociraj_double((Log_robot_path.laser2_y),Log_robot_path.max_laser2, false);
	  }
//#endif
	  if(Log_robot_path.size_teziste>= Log_robot_path.max_teziste-PLAYER_LASER_MAX_SAMPLES){
		  printf("WH logcurrent:> realloc: teziste_size=%d, for increment=%d\n",Log_robot_path.size_teziste,REALLOC_SIZE_INCREMENT*PLAYER_LASER_MAX_SAMPLES);
		  Log_robot_path.max_teziste+=REALLOC_SIZE_INCREMENT*PLAYER_LASER_MAX_SAMPLES;
		  Log_robot_path.teziste_x=reallociraj_double((Log_robot_path.teziste_x),Log_robot_path.max_teziste, false);
		  Log_robot_path.teziste_y=reallociraj_double((Log_robot_path.teziste_y),Log_robot_path.max_teziste, false);
	  }
	  if(Log_robot_path.size_dwopttraj>= Log_robot_path.max_dwopttraj-PLAYER_LASER_MAX_SAMPLES){
		  printf("WH logcurrent:> realloc: dwopttraj_size=%d, for increment=%d\n",Log_robot_path.size_dwopttraj,REALLOC_SIZE_INCREMENT*PLAYER_LASER_MAX_SAMPLES);
		  Log_robot_path.max_dwopttraj+=REALLOC_SIZE_INCREMENT*PLAYER_LASER_MAX_SAMPLES;
		  Log_robot_path.dwopttraj_x=reallociraj_double((Log_robot_path.dwopttraj_x),Log_robot_path.max_dwopttraj, false);
		  Log_robot_path.dwopttraj_y=reallociraj_double((Log_robot_path.dwopttraj_y),Log_robot_path.max_dwopttraj, false);
	  }
	  Log_robot_path.izracuni[process_increment]=vremenska_razlika;
	  Log_robot_path.DSizracuni[process_increment]=PL->search_time;
	  Log_robot_path.DSexplored[process_increment]=PL->search_explored;
	  Log_robot_path.DWizracuni[process_increment]=DW->vremenska_razlika;
     Log_robot_path.path_x[process_increment]=RB.x;
     Log_robot_path.path_y[process_increment]=RB.y;
     Log_robot_path.path_th[process_increment]=RB.th;
     Log_robot_path.v[process_increment]=RB.v;  //sve ostalo je u mm/s
     Log_robot_path.vy[process_increment]=RB.vy;  //sve ostalo je u mm/s
     Log_robot_path.vy_ref[process_increment]=vy_ref_current;
     Log_robot_path.w[process_increment]=RB.w;
     Log_robot_path.v_kal[process_increment]=v_kal_current;
     Log_robot_path.w_kal[process_increment]=w_kal_current;
     Log_robot_path.v_ref[process_increment]=v_ref_current;
     Log_robot_path.w_ref[process_increment]=w_ref_current;
     Log_robot_path.v_des[process_increment]=v_des_current;
     Log_robot_path.w_des[process_increment]=w_des_current;
     Log_robot_path.v_refdin[process_increment]=v_refdin_current;
     Log_robot_path.w_refdin[process_increment]=w_refdin_current;
//#if (LOG_LASER==1)
//	 for (int j=0;j<LB.laser_pointer_new;j++){
	 for (int j=0;j<LB.laser_manji_new;j++){
// 		 Log_robot_path.laser_x[Log_robot_path.size_laser+j]=LB.LB_pose[j].getX();
// 		 Log_robot_path.laser_y[Log_robot_path.size_laser+j]=LB.LB_pose[j].getY();
//		 Log_robot_path.laser_x[Log_robot_path.size_laser+j]=LB.LB_pose[j].x;
//		 Log_robot_path.laser_y[Log_robot_path.size_laser+j]=LB.LB_pose[j].y;
		 Log_robot_path.laser_x[Log_robot_path.size_laser+j]=LB.LB_pose_manji[j].x;
		 Log_robot_path.laser_y[Log_robot_path.size_laser+j]=LB.LB_pose_manji[j].y;
	 }
//#endif
//	 Log_robot_path.broj_hitova[process_increment]=LB.laser_pointer_new;
	 Log_robot_path.broj_hitova[process_increment]=LB.laser_manji_new;
//	 Log_robot_path.size_laser+=LB.laser_pointer_new;
	 Log_robot_path.size_laser+=LB.laser_manji_new;
	 //laser2
//#if (LOG_LASER==1)
	 for (int j=0;j<LB.laser_pointer_new;j++){
//	 for (int j=0;j<LB2.laser_manji_new;j++){
// 		 Log_robot_path.laser_x[Log_robot_path.size_laser+j]=LB.LB_pose[j].getX();
// 		 Log_robot_path.laser_y[Log_robot_path.size_laser+j]=LB.LB_pose[j].getY();
//		 Log_robot_path.laser_x[Log_robot_path.size_laser+j]=LB.LB_pose[j].x;
//		 Log_robot_path.laser_y[Log_robot_path.size_laser+j]=LB.LB_pose[j].y;
//		 Log_robot_path.laser2_x[Log_robot_path.size_laser2+j]=LB2.LB_pose_manji[j].x;
//		 Log_robot_path.laser2_y[Log_robot_path.size_laser2+j]=LB2.LB_pose_manji[j].y;
		 Log_robot_path.laser2_x[Log_robot_path.size_laser2+j]=LB.sviscanovi[j].r;
		 Log_robot_path.laser2_y[Log_robot_path.size_laser2+j]=LB.sviscanovi[j].th;
	 }
//#endif
	 Log_robot_path.broj_hitova2[process_increment]=LB.laser_pointer_new;
//	 Log_robot_path.broj_hitova2[process_increment]=LB2.laser_manji_new;
	 Log_robot_path.size_laser2+=LB.laser_pointer_new;
//	 Log_robot_path.size_laser2+=LB2.laser_manji_new;
	 for (int j=0;j<LB.broj_tezista;j++){
		 Log_robot_path.teziste_x[Log_robot_path.size_teziste+j]=LB.teziste[j].x;
		 Log_robot_path.teziste_y[Log_robot_path.size_teziste+j]=LB.teziste[j].y;
	 }
	 Log_robot_path.broj_tezista[process_increment]=LB.broj_tezista;
	 Log_robot_path.size_teziste+=LB.broj_tezista;
//opt traj
	 int numdwel=0;
	 if (DW->poceo){
	 	if ((DW->ni!=-1) && (DW->nj!=-1)){
			for (int j=0;j<N_KL;j++){
				Log_robot_path.dwopttraj_x[Log_robot_path.size_dwopttraj+j]=DW->KL.x[j];
		 		Log_robot_path.dwopttraj_y[Log_robot_path.size_dwopttraj+j]=DW->KL.y[j];
			}
			numdwel=N_KL;
		}else if (DW->best_breakage){
			for (int j=0;j<N_KL;j++){
				Log_robot_path.dwopttraj_x[Log_robot_path.size_dwopttraj+j]=DW->KL_breakage.x[j];
		 		Log_robot_path.dwopttraj_y[Log_robot_path.size_dwopttraj+j]=DW->KL_breakage.y[j];
			}
			numdwel=N_KL;
		}else if ((DW->best_old) && (DW->flag_kl_old)){
			for (int j=0;j<N_KL;j++){
				Log_robot_path.dwopttraj_x[Log_robot_path.size_dwopttraj+j]=DW->KL_old.x[j];
		 		Log_robot_path.dwopttraj_y[Log_robot_path.size_dwopttraj+j]=DW->KL_old.y[j];
			}
			numdwel=N_KL;
		}
	 }
	 Log_robot_path.broj_dwopttraj[process_increment]=numdwel;
	 Log_robot_path.size_dwopttraj+=numdwel;
    //predpostavljamo da je defaultni cycle 100ms!!!
    Log_robot_path.time[process_increment]=vremenska_razlika0;
	Log_robot_path.read_time[process_increment]=ciklus_sad;
	Log_robot_path.path_length[process_increment]=dwpath_length;//DW->path_length;
	Log_robot_path.path_orientation[process_increment]=DW->path_orientation;
	Log_robot_path.tocka_infleksije_x[process_increment]=DW->tocka_infleksije.x;
	Log_robot_path.tocka_infleksije_y[process_increment]=DW->tocka_infleksije.y;
	Log_robot_path.ljapunov[process_increment]=DW->ljapunov;
	Log_robot_path.interpolatedcost[process_increment]=interpolatedcost;
	Log_robot_path.replan[process_increment]=DS->racunaoupromjeni;
	Log_robot_path.travcost[process_increment]=DS->travcost;
	Log_robot_path.travcost3d[process_increment]=DS->travcost3d;
	Log_robot_path.preslik_x[process_increment]=DS->preslik.x;
	Log_robot_path.preslik_y[process_increment]=DS->preslik.y;
	logger_sizes[robot_path_size]=process_increment+1;
}

void WorkHorse::LoadInit(double width, double height, double resolution, double originx, double originy){

//ostali objekti se sada definiraju u INIT stanju procesa,
//sto i nije najsretnije rijesenje....
//	wld=M->wld;
	GM = new GridMap(CELL_DIM); //tu se stvori karta zauzetosti prema wld datoteci
#if (READWLD==0)
	GM->alloc_Mapfrompng(width, height, resolution, originx, originy);
#endif
	DW = new DynamicWindow();
	printf("DW\n");
	PL = new Planner();
	DS = new DStar(GM->Map_Dim_X, GM->Map_Dim_Y);
//	PL->reset();//tu se napuni DS iz GM
	          //pocetna allokacija polja
	logger_current_size=Logger_Init();
	 
}

void WorkHorse::process(){

  static int constructor_halt;
/* static ArTime start, now;*/
   static R_point current_goal;
    double goal_tolerance, theta_tolerance;
    double rbv, rbw, rbvy;
    	  struct timeval timeStart; //pocetak algoritma
	  struct timeval timeNow;  //kraj algoritma
  int mySecStart, myMSecStart,mySecNow, myMSecNow;  //za mjerenje vremena samog algoritma
    
   static int process_increment;
	switch(processState){
	//u init stanju se samo resetiraju counteri i slicne stvari ako zelimo ponovo izvesti neke fore
	case INIT:
			printf("INIT constructor_halt=%d\n",constructor_halt);
	//ceka se 5 ciklusa ako se nisu stigli napraviti konstruktori u LoadInit(void) funkciji koja se poziva prije ove
			if(constructor_halt<10){
				constructor_halt++;
			}else if (constructor_halt>10 && constructor_halt<20){
				constructor_halt++;
				if(constructor_halt==20) {
					processState=RUNNING;
//					logger_current_size=Logger_Init(); 
					constructor_halt=0;
				}
			}
			else{
				constructor_halt++;     
		 average_speed=0.0;//mjeri srednju brzinu tijekom voznje
		 brojac_brzina=0;     
	 process_increment=0;//lokalna varijabla, isto sto i cycle_number samo kaj je ova druga globalna
	 ciklus_prije=0;ciklus_sad=0;  //za mjerenje duljine ciklusa
	initial=true; //izdvajamo inicijalno racunanje puta
	myMSecStart0=0;//globalne varijable WH klase, pamti 0-ti trenutak da bi se moglo mjeriti vrijeme od samog pocetka gibanja
	mySecStart0=0; //sekunde, ovo gore su ms
	cycle_number=0; //brojac ciklusa, ne podudara se s onim u moj.cc buduci da se 5 ciklusa trosi na INIT stanje
     //u init stanju se vracamo na prvobitnu goal poziciju
	new_global_goal=true;//kad je true ide do cilja i stane
// 	new_global_goal=false;//kad je false vrati se nazad
	global_goal_workhorse.x=M->goal.x;
	global_goal_workhorse.y=M->goal.y;
	global_goal_workhorse.th=M->goal.th;//dodavanje kuta
	PL->reset();//ovdje puni DStar mapu preprekama, prepisuje iz GM i prosiruje za masku robota, a takodjer prosiruje za dodatnu cost masku ako je definirana
//        ArPose *Robot_start_pose;
//        Robot_start_pose = new ArPose();
//        Robot_start_pose->setPose(MAP_ROBOT_START_POSITION_X,MAP_ROBOT_START_POSITION_Y,MAP_ROBOT_START_POSITION_TH);
// 	      SfROBOT->moveTo(*Robot_start_pose);
	      v_ref_current=0.;v_refdin_current=0.;v_des_current=0.;delta_v_refdin=0.;delta_w_refdin=0.;
	      w_ref_current=0.;w_refdin_current=0.;w_des_current=0.;
	      vy_ref_current=0.;vy_refdin_current=0.;vy_des_current=0.;delta_vy_refdin=0.;
	      dwpath_length=0.;
// 	 processState=RUNNING;
   printf("go into RUNNING\n");
 	 }
		break;

		case REINIT: //ovo bi se trebalo pozivati iz komandne linije, nakon sto se pozove HALT, nije jos implementirano, ovako nista ne radi
  printf("REINIT\n");
  GM->reset(); //reset cuva staticke prepreke
  DS->reset();//isprazni cijelu mapu
  PL->reset();//ovdje puni DStar mapu preprekama, prepisuje iz GM i prosiruje za masku robota, a takodjer prosiruje za dodatnu cost masku ako je definirana
		  processState=HALT;

  break;
  
  case NO_PATH:
  printf("\n\nNO_PATH ramp\n"); 
  DW->naCilju=false;
  DW->rotateongoal=false;
		    DS->racunaoupromjeni=0;
     //LOGGER VREMENA  --POCETAK ALGORITMA ------------------------------
      if (gettimeofday(&timeStart, NULL) == 0)
      {
	      mySecStart = timeStart.tv_sec;
	      myMSecStart = timeStart.tv_usec / 1000;
      }
      ciklus_prije=ciklus_sad;
      vremenska_razlika0=(mySecStart-mySecStart0)*1000+(myMSecStart-myMSecStart0);
      ciklus_sad=vremenska_razlika0;   //razlika ciklus_sad - ciklus_prije bit ce duljina ciklusa, naravno za 0.-ti ciklus ne vrijedi
      cycle_number++;
vremenska_razlika=0;
//za logove kraj
        //ramp
        Ucitaj_stanje_odometrije();
        Ucitaj_stanje_lasera();
#if (OKO_REFERENCE==1)
	rbv=v_refdin_current;
	rbw=w_refdin_current;
	rbvy=vy_refdin_current;
#else
	rbv=RB.v;
	rbw=RB.w;
	rbvy=RB.vy;
#endif
  v_ref_current=0.;v_refdin_current=0.;v_des_current=0.;delta_v_refdin=0.;delta_w_refdin=0.;
  w_ref_current=0.;w_refdin_current=0.;w_des_current=0.;dwpath_length=0.;
  vy_ref_current=0.;vy_refdin_current=0.;vy_des_current=0.;delta_vy_refdin=0.;
#if (NO_PATH_RAMP==1) 
	if (fabs(rbv)>(DV_MAX*STEP)) {//v_refdin_current
		v_des_current=(rbv-(rbv)/fabs(rbv)*DV_MAX*STEP);//v_refdin_current
	}
	if (fabs(rbvy)>(DVY_MAX*STEP)) {//v_refdin_current
		vy_des_current=(rbvy-(rbvy)/fabs(rbvy)*DVY_MAX*STEP);//v_refdin_current
	}
	if (fabs(rbw)>(DW_MAX*STEP)) {//w_refdin_current
		w_des_current=(rbw-(rbw)/fabs(rbw)*DW_MAX*STEP);//w_refdin_current
	}
#endif
#if (NO_PATH_RAMP==2)
  int N,Tvx,Tvy,Tw;
  Tvx=(int)ceil((fabs(rbv))/(DVX_MAX*STEP));//vrijeme zaustavljanja
	Tvy=(int)ceil(fabs(rbvy)/(DVY_MAX*STEP));
	Tw=(int)ceil(fabs(rbw)/(DW_MAX*STEP));
	N=std::max(Tvx,Tw);
	N=std::max(N,Tvy);
	if (N>1){
	v_des_current=rbv*(N-1)/N;
	vy_des_current=rbw*(N-1)/N;
	w_des_current=rbvy*(N-1)/N;
	}else{
	v_des_current=0;
	vy_des_current=0;
	w_des_current=0;	
	} 
#endif
	v_refdin_current=v_des_current; v_ref_current=v_refdin_current*metric;
	vy_refdin_current=vy_des_current; vy_ref_current=vy_refdin_current*metric;
	w_refdin_current=w_des_current; w_ref_current=w_refdin_current;
	//end ramp
	//logirati
            LogCurrent(process_increment);
            process_increment++;
	//end log
  no_path_counter++; //cekamo opet neki broj ciklusa (definirano u Params.h, 20 ciklusa)
  if(no_path_counter>=NO_PATH_COUNTER){
 if ((v_ref_current<V_TOLERANCE*metric)&&(rbv<V_TOLERANCE) && (vy_ref_current<V_TOLERANCE*metric)&&(rbvy<V_TOLERANCE) && (fabs(w_ref_current)<W_TOLERANCE) && (fabs(rbw)<W_TOLERANCE)){
//  	Logger();

	  if (!new_global_goal){//dabar
		  global_goal_workhorse.x = M->subgoal.x;
		  global_goal_workhorse.y = M->subgoal.y;
		  global_goal_workhorse.th=M->subgoal.th;
		  printf("The new goal is subgoal (%f,%f,%f)\n",M->subgoal.x,M->subgoal.y,M->subgoal.th);
		  initial=true;
	  }//gotov dabar
	  printf("The new goal is (%f,%f,%f)\n",global_goal_workhorse.x,global_goal_workhorse.y,global_goal_workhorse.th);

  	//ovdje resetiramo GM i idemo dalje
	  GM->reset();//reset cuva staticke prepreke
	  DS->reset(); //isprazni cijelu mapu
	  PL->reset();//ovdje puni DStar mapu preprekama, prepisuje iz GM i prosiruje za masku robota, a takodjer prosiruje za dodatnu cost masku ako je definirana
	  initial=true;
	  processState=RESUMING;
	  no_path_counter=0;
  }
  }
  
  break;

  case HALT:
	  v_ref_current=0.;v_refdin_current=0.;v_des_current=0.;delta_v_refdin=0.;delta_w_refdin=0.;
	  w_ref_current=0.;w_refdin_current=0.;w_des_current=0.;dwpath_length=0.;
	  vy_ref_current=0.;vy_refdin_current=0.;vy_des_current=0.;delta_vy_refdin=0.;
	  printf("no_path_counter=%d\n",no_path_counter);
	  //sad ide automatski nazad
	  if (no_path_counter==0){
		  if (!new_global_goal){
/*			  global_goal_workhorse.x = M->start.x;
			  global_goal_workhorse.y = M->start.y;*/
			  global_goal_workhorse.x = M->subgoal.x;
			  global_goal_workhorse.y = M->subgoal.y;
			  global_goal_workhorse.th=M->subgoal.th;
			  initial=true;

// 			  new_global_goal=true;
			  GM->reset(); //reset cuva staticke prepreke
			  DS->reset();//isprazni cijelu mapu
			  PL->reset();//ovdje puni DStar mapu preprekama, prepisuje iz GM i prosiruje za masku robota, a takodjer prosiruje za dodatnu cost masku ako je 
		  }
	  }
	  if(no_path_counter<0){//tu je bilo 4
		  no_path_counter++;
	  }else{
		  no_path_counter++;
		  if (!new_global_goal){
			  new_global_goal=true;
			  processState=RESUMING;//INIT;//RESUMING;
			  no_path_counter=0;
		  }
		  if (no_path_counter==10){
			  delete DW;delete GM;delete DS;delete PL;
			  OslobodiMemoriju();
			  processState=GOAL_POSITION;
// 			  exit(1);
		  }
	  }

break;

  case GOAL_POSITION:
  printf("GOAL_POSITION\n");
  v_ref_current=0.;v_refdin_current=0.;v_des_current=0.;delta_v_refdin=0.;delta_w_refdin=0.;
  w_ref_current=0.;w_refdin_current=0.;w_des_current=0.;dwpath_length=0.;
  vy_ref_current=0.;vy_refdin_current=0.;vy_des_current=0.;delta_vy_refdin=0.;
//   processState=HALT;

  break;

  case RESUMING:
	  printf("RESUMING, counter=%d\n",no_path_counter);
	  if (no_path_counter==-1000){//DS->promjena
// 		  DS->SearchPathWitkowski();
/*		  if(DS->PathExistsWitkowski){
			  DS->getPath_backward();
			  processState=RUNNING;
		  }*/
		  printf("no_path_counter==-1000, special case!!!!\n");
		  DS->SearchPath();
		  if (DS->PathExists){
			  processState=RUNNING;
			  DS->pipodjednom=false;
			  no_path_counter=-500;
			  printf("----------------------------------------------------------------------------------------------------------\n");
		  }
		  no_path_counter--;
	  }
	  no_path_counter++; //cekamo opet neki broj ciklusa (definirano u Params.h, 20 ciklusa)
  if(no_path_counter>=1){
	  processState=RUNNING;
/*	  else{
		  processState=HALT;
	  }*/
  }
  break;

  case RUNNING:
     //LOGGER VREMENA  --POCETAK gibanja------------------------------
  if ((cycle_number==0)&&(gettimeofday(&timeStart, NULL) == 0))
      {
	      mySecStart0 = timeStart.tv_sec;
	      myMSecStart0 = timeStart.tv_usec / 1000;
        printf("zero time=%d,%d\n", myMSecStart0,mySecStart0);
      }
   //LOGGER VREMENA  --POCETAK ALGORITMA ------------------------------
      if (gettimeofday(&timeStart, NULL) == 0)
      {
	      mySecStart = timeStart.tv_sec;
	      myMSecStart = timeStart.tv_usec / 1000;
      }
      ciklus_prije=ciklus_sad;
      vremenska_razlika0=(mySecStart-mySecStart0)*1000+(myMSecStart-myMSecStart0);
      ciklus_sad=vremenska_razlika0;   //razlika ciklus_sad - ciklus_prije bit ce duljina ciklusa, naravno za 0.-ti ciklus ne vrijedi
      cycle_number++;
      DW->vremenska_razlika=0;//inicijalizacija ako nema puta
  int path_found;
  DS->brojac_izracuna=process_increment;
//   printf("process_increment=%d\n",process_increment);
  
  //odometrija - preuzima iz klase M
  Ucitaj_stanje_odometrije();
  
  //ocitanja lasera - preuzima iz klase M
  Ucitaj_stanje_lasera();

		    DS->racunaoupromjeni=0;
	  if (initial)  //racuna inicijalni put dok ga ne izracuna
	  {
		path_found=PL->Sekvenca_izvodjenja();//tu se pozove SearchPath i getPath
 
	if(path_found==1)
        {
no_path_counter=0;
			  initial=false;
			  DS->prviput=0;//to moram postaviti kad radim samo s W algoritmom
			  DW->flag_kl_old=false;
		    interpolatedcost=DW->computeInterpolatedCost(RB.x,RB.y,RB.th);
           //ako smo na pocetku pronasli goal, onda azuriramo pocetnu planner putanju
         //azuriranje pocetnih stvari....
         R_point *temp_point; int temp_length;
	 temp_length=PL->GetPathLength();
	  temp_point=PL->GetPath();
          printf("WH> path length=%d\n",temp_length); 
	  if (cycle_number==1){//samo na pocetku logira inicijalni put
          planner_path_original.path_x=reallociraj_double(planner_path_original.path_x, temp_length, true);
          planner_path_original.path_y=reallociraj_double(planner_path_original.path_y, temp_length, true);
          planner_path_original.path_th=reallociraj_double(planner_path_original.path_th, temp_length, true);
          planner_path_original.path_travcost=reallociraj_double(planner_path_original.path_travcost, temp_length, true);
          planner_path_original.path_h=reallociraj_double(planner_path_original.path_h, temp_length, true);
#if DSTAR3D
          planner_path_original.path_intlo=reallociraj_double(planner_path_original.path_intlo, temp_length, true);
          planner_path_original.path_intup=reallociraj_double(planner_path_original.path_intup, temp_length, true);
#endif
          for(int i=0;i<temp_length;i++)
          {
      		  planner_path_original.path_x[i]=temp_point[i].x;
		        planner_path_original.path_y[i]=temp_point[i].y;
		        planner_path_original.path_th[i]=temp_point[i].th;
		        planner_path_original.path_travcost[i]=PL->ori[i].travcost;
		        planner_path_original.path_h[i]=PL->ori[i].h;
#if DSTAR3D
		        planner_path_original.path_intlo[i]=PL->ori[i].intlo;
		        planner_path_original.path_intup[i]=PL->ori[i].intup;
#endif
	        }
			planner_path_original.size_path=temp_length;
//			global_goal_workhorse.x=temp_point[temp_length-1].x;
//			global_goal_workhorse.y=temp_point[temp_length-1].y;
//			global_goal_workhorse.th=temp_point[temp_length-1].th;
#if (KOMBINACIJA==1) || DSTAR_REVERSE
      //tu ce biti logiran extra path - FD* path
			temp_length=DS->getPathLength();
			I_point *i_temp_point;
			i_temp_point=DS->GetPath();
			R_point r_temp_point;

			planner_path_run.path_x=reallociraj_double(planner_path_run.path_x, (planner_path_run.size_path+temp_length), true);
			planner_path_run.path_y=reallociraj_double(planner_path_run.path_y, (planner_path_run.size_path+temp_length), true);
			for(int i=0;i<temp_length;i++)
			{
				if(!PL->IntToReal(i_temp_point[i], r_temp_point, GM->Map_Dim_X,GM->Map_Dim_Y, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM))
				{
					printf("WH> PL->IntToReal > element path[%d].x=%d , path[%d].y=%d", i, i_temp_point[i].x,i, i_temp_point[i].y);
					printf("WH> PL->IntToReal > IntToReal conversion failed!");
// 			return 0;
				}
				planner_path_run.path_x[i+planner_path_run.size_path]=r_temp_point.x;
				planner_path_run.path_y[i+planner_path_run.size_path]=r_temp_point.y;
			}
			planner_path_run.size[planner_path_run.broj]=temp_length;
			planner_path_run.broj++;
			planner_path_run.size_path+=temp_length;
#endif
	  }else{
	  	    //azuriramo trenutnu planner putanju ako je promijenjena
	    
      bool first=true;//za logiranje
      if (planner_path_run.size_path){
	      first=false;
      }

      planner_path_run.path_x=reallociraj_double(planner_path_run.path_x, (planner_path_run.size_path+temp_length), first);
      planner_path_run.path_y=reallociraj_double(planner_path_run.path_y, (planner_path_run.size_path+temp_length), first);
      planner_path_run.path_th=reallociraj_double(planner_path_run.path_th, (planner_path_run.size_path+temp_length), first);
      planner_path_run.path_travcost=reallociraj_double(planner_path_run.path_travcost, (planner_path_run.size_path+temp_length), first);
    planner_path_run.path_h=reallociraj_double(planner_path_run.path_h, (planner_path_run.size_path+temp_length), first);
#if DSTAR3D
planner_path_run.path_intlo=reallociraj_double(planner_path_run.path_intlo, (planner_path_run.size_path+temp_length), first);
planner_path_run.path_intup=reallociraj_double(planner_path_run.path_intup, (planner_path_run.size_path+temp_length), first);
#endif
      for(int i=0;i<temp_length;i++)
      {
	      planner_path_run.path_x[i+planner_path_run.size_path]=temp_point[i].x;
	      planner_path_run.path_y[i+planner_path_run.size_path]=temp_point[i].y;
	      planner_path_run.path_th[i+planner_path_run.size_path]=temp_point[i].th;
		        planner_path_run.path_travcost[i+planner_path_run.size_path]=PL->ori[i].travcost;
		        planner_path_run.path_h[i+planner_path_run.size_path]=PL->ori[i].h;
#if DSTAR3D
		        planner_path_run.path_intlo[i+planner_path_run.size_path]=PL->ori[i].intlo;
		        planner_path_run.path_intup[i+planner_path_run.size_path]=PL->ori[i].intup;
#endif
    	}
	planner_path_run.size[planner_path_run.broj]=temp_length;
//	global_goal_workhorse.x=temp_point[temp_length-1].x;
//	global_goal_workhorse.y=temp_point[temp_length-1].y;
//	global_goal_workhorse.th=temp_point[temp_length-1].th;
	planner_path_run.broj++;
    	planner_path_run.size_path+=temp_length;
#if (KOMBINACIJA==1) || DSTAR_REVERSE
      //tu ce biti logiran extra path - FD* path
	temp_length=DS->getPathLength();
	I_point *i_temp_point;
	i_temp_point=DS->GetPath();
	R_point r_temp_point;

	planner_path_run.path_x=reallociraj_double(planner_path_run.path_x, (planner_path_run.size_path+temp_length), false);
	planner_path_run.path_y=reallociraj_double(planner_path_run.path_y, (planner_path_run.size_path+temp_length), false);
	for(int i=0;i<temp_length;i++)
	{
		if(!PL->IntToReal(i_temp_point[i], r_temp_point, GM->Map_Dim_X,GM->Map_Dim_Y, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM))
		{
			printf("WH> PL->IntToReal > element path[%d].x=%d , path[%d].y=%d", i, i_temp_point[i].x,i, i_temp_point[i].y);
			printf("WH> PL->IntToReal > IntToReal conversion failed!");
// 			return 0;
		}
		planner_path_run.path_x[i+planner_path_run.size_path]=r_temp_point.x;
		planner_path_run.path_y[i+planner_path_run.size_path]=r_temp_point.y;
	}
	planner_path_run.size[planner_path_run.broj]=temp_length;
	planner_path_run.broj++;
	planner_path_run.size_path+=temp_length;
#endif
	    
	  }
        }   //if (path_found==1)
        else
        {
          if (path_found==2)
          {
            printf("WorkHorse::running> initial path is not calculated!\n");
            		DS->PathCost=OBSTACLE;//dodala zbog provjera

			v_ref_current=0.;
			w_ref_current=0.;dwpath_length=0.;
            if (gettimeofday(&timeNow, NULL) == 0)
            {
              mySecNow = timeNow.tv_sec;
              myMSecNow = timeNow.tv_usec / 1000;
            }
            vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
			vremenska_razlika0=(mySecNow-mySecStart0)*1000+(myMSecNow-myMSecStart0); //vremenski trenutak zavrsetka algoritma
            printf("WorkHorse::initial==true? - time diff msec: %d initial=%d\n\n", vremenska_razlika,initial);

            LogCurrent(process_increment);
            process_increment++;
	    no_path_counter=0;
//dodajem neka proba racunati dw svejedno
 			  initial=false;
			  DS->prviput=0;//to moram postaviti kad radim samo s W algoritmom
			  DW->flag_kl_old=false;
//zakomentiravam odlazak u halt
//	    processState=HALT;//ponovo izracunaj put prema cilju koji je ispred robota u slobodnom prostoru
//	    new_global_goal=false;
//	    Logger();
            break;
          }
          else
          {  
		  printf("WorkHorse::running> No global path was found in the beginning!!!wit=%d\n",DS->PathExistsWitkowski);
            no_path_counter=0;
//             processState=HALT;
			LogCurrent(process_increment);
			process_increment++;
 			processState=NO_PATH;
//			processState=HALT;//ponovo izracunaj put prema cilju koji je ispred robota u slobodnom prostoru
			new_global_goal=false;
					DS->PathCost=OBSTACLE;//dodala zbog provjera
//					Logger();

		break;
          }
        }
   //LOGGER VREMENA - NAKON INICIJALNOG IZVODJENJA ALGORITMA -------------------------------------------------
        if (gettimeofday(&timeNow, NULL) == 0)
        {
          mySecNow = timeNow.tv_sec;
	        myMSecNow = timeNow.tv_usec / 1000;
        }
        vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
        vremenska_razlika0=(mySecNow-mySecStart0)*1000+(myMSecNow-myMSecStart0);
        printf("WorkHorse::initial==true? After initial search - time diff msec: %d initial=%d cycle=%d\n\n", vremenska_razlika,initial,process_increment);
			v_ref_current=0.;
			w_ref_current=0.;dwpath_length=0.;
          LogCurrent(process_increment);
		  process_increment++;
		  
/*		  no_path_counter=0;
		  processState=RESUMING;*/
// 		  processState=HALT;
//		  Logger();
          break;
      }  //od initial==true
	  //sad se pocinju uracunavati brzine, pocetna je 0 naravno
	brojac_brzina++;
#if OMNIDRIVE
	average_speed+=sqrt(RB.v*RB.v+RB.vy*RB.vy);
#else
	average_speed+=RB.v;
#endif
 
     //usli smo u obradu
     //al prvo provjerimo dal smo na cilju
    

	//mjerim vrijeme
	if (gettimeofday(&timeNow, NULL) == 0)
	{
		mySecNow = timeNow.tv_sec;
		myMSecNow = timeNow.tv_usec / 1000;
	}
	vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
	printf("WH> before Planner %d ms.\n", vremenska_razlika);
    if (no_path_counter==-2000){//to je za slucaj spajanja zidova, obrisemo robot masku oko prepreke izracunate u proslom ciklusu na putu
	    int brojpam=0;
	    int robot_mask=(int)ROBOT_MASK;
	    int cost_mask=(int) COST_MASK;
	    int star_size_x=DS->GetMapSizeX();
	    int star_size_y=DS->GetMapSizeY();
	    I_point pamtiindekse[(2*(robot_mask+cost_mask)+1)*(2*(robot_mask+cost_mask)+1)*(2*robot_mask+1)];
	    for (int i=DS->prepreka.x-robot_mask;i<=DS->prepreka.x+robot_mask;i++){
		    for (int j=DS->prepreka.y-robot_mask;j<=DS->prepreka.y+robot_mask;j++){ 
			    PL->Prazni(i, j, pamtiindekse, &brojpam, star_size_x, star_size_y, robot_mask, cost_mask, true);//ovdje se puni polje pamtiindekse i mijenja se brojpam
		    }//for
	    }//for
	    PL->PrazniCostMasku(pamtiindekse, brojpam, star_size_x, star_size_y, cost_mask);//a ovdje se pomocu tog polja azurira cost maska
#if DSTAR
	    DS->SearchPath();
#else
	    DS->SearchPathWitkowski();
#endif
// 	    processState=RUNNING;
	    no_path_counter=-500;
	    printf("--------------------------spojeni zidovi-------------------------------------------\n");
    }
    if (no_path_counter!=-500){
    path_found=PL->Sekvenca_izvodjenja();
    current_goal=PL->GetGoal();
/*    if (DS->pipodjednom){   //  path_found!=1DS->promjena  (DS->novoTeziste)&&(!DS->prviput) 
		v_ref_current=0.;
		w_ref_current=0.;
		no_path_counter=-1000;
		processState=RESUMING;
// 		Logger();
// 		processState=HALT;
		break;
	}*/
    }else{
// 	    DS->pipodjednom=true;
	    if((PL->path=(DS->getPath()))==NULL){
		    printf("WorkHorse::process	> DStar path is NULL!\n");
		    path_found=2;
	    }
	    if(path_found!=2){
#if DSTAR
		PL->path_length=DS->getPathLength();
#else
		PL->path_length=DS->getPathLength_backward();
#endif
	    	if(!PL->IntToRealPath(GM->Map_Dim_X,GM->Map_Dim_Y, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM))
	    	{
		    printf("PlannerMainSequence> Path conversion from int to real failed!\n");
		    path_found=0;
	    	}
	    	if (path_found!=0){
	    		path_found=1;
	    		no_path_counter=-1000;
	    		printf("in WH called for PL path\n");
	    	}
	    }
    }
/*	if ((path_found==2)&&(DS->PathExistsWitkowski)){
		PL->path=DS->GetPathBackward();
		PL->path_length=DS->getPathLength_backward();
		if(!PL->IntToRealPath(GM->Map_Dim_X,GM->Map_Dim_Y, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM))
		{
			printf("Planner Sekvenca> Nije uspijela konverzija path iz int u real!\n");
			path_found=2;
		}else{
			path_found=1;
		}
	}*/
 	//mjerim vrijeme
 	if (gettimeofday(&timeNow, NULL) == 0)
 	{
 		mySecNow = timeNow.tv_sec;
 		myMSecNow = timeNow.tv_usec / 1000;
 	}
 	vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
 	printf("WH> after Planner %d ms.\n", vremenska_razlika);

    if( (path_found==1) ) //|| (path_found==2)) 
    {
	    if (!initial){
		    rbv=RB.v;
		    rbw=RB.w;
		    rbvy=RB.vy;
#if (DELTA_REF==0)
// 	if (fabs(delta_v_refdin)>V_TOLERANCE){
		    delta_v_refdin=rbv-delta_v_refdin;//ostvarene akceleracije
		    delta_w_refdin=rbw-delta_w_refdin;
		    delta_vy_refdin=rbvy-delta_vy_refdin;
// 	}
#else
		    delta_v_refdin=v_refdin_current-delta_v_refdin;//sad su zadane akceleracije jer je u staroj delti predzadnja referenca
		    delta_w_refdin=w_refdin_current-delta_w_refdin;
		    delta_vy_refdin=vy_refdin_current-delta_vy_refdin;
#endif
		    if (DS->racunaoupromjeni){
//			    DW->flag_kl_old=false;
		    }
		    DW->Sekvenca_izvodjenja();
		    dwpath_length=DW->path_length;//tu ga prepisujem
		    interpolatedcost=DW->computeInterpolatedCost(RB.x,RB.y,RB.th);
		    DS->kut_puta=DW->path_orientation;
#if (DELTA_REF==1)		    
		    delta_v_refdin=v_refdin_current;//ovo pamti stare reference iz n-1 koraka, tu nize ce se novu izracunati
		    delta_w_refdin=w_refdin_current;
		    delta_vy_refdin=vy_refdin_current;
#endif
#if (PO_PREDIKCIJI==1)
		    rbv=v_des_current;
		    rbw=w_des_current;
		    rbvy=vy_des_current;
#endif
		    v_des_current=DW->GetRefVelocitiesV();
		    w_des_current=DW->GetRefVelocitiesW();//radijani/s
		    vy_des_current=DW->GetRefVelocitiesVY();
// 		    printf("WH:desired: %f\n",v_des_current);
	  //prvo ukljucenje dinamike
	  	    if (Tv>0.00001){
		    v_refdin_current=((v_des_current-rbv*exp(-STEP/Tv))/(1-exp(-STEP/Tv)));
		    if (fabs(v_refdin_current)<V_TOLERANCE)
			    v_refdin_current=0.;
		    w_refdin_current=(w_des_current-rbw*exp(-STEP/Tw))/(1-exp(-STEP/Tw));
// 		    printf("WH:reference poslije v(%d)=%d (%f), w(%d)=%d\n",DW->ni,int(DW->round(v_refdin_current)),v_refdin_current,DW->nj,int(DW->round(w_refdin_current*RuS)));
	  //sad ukljucenje kalibrirane odometrije
	  //sad to racuna na pocetku sljedeceg ciklusa
		    double vr,vl;
		    vr=(2*v_refdin_current+kb*b*w_refdin_current)/(2*kr);
		    vl=(2*v_refdin_current-kb*b*w_refdin_current)/(2*kl);
// 	  v_ref_current=(DW->round(((vr+vl)/2)));
// 	  w_ref_current=(DW->round(((vr-vl)/b*RuS)));//u stupnjevima
		    v_ref_current=(vr+vl)/2000.;//u m
		    w_ref_current=(vr-vl)/b;//u radijanima
		    if (fabs(v_ref_current)<V_TOLERANCE*metric)
			    v_ref_current=0.;
		    if (fabs(w_ref_current)<W_TOLERANCE)
			    w_ref_current=0.;
	    	    }else{
	    	    v_refdin_current=v_des_current;
	    	    w_refdin_current=w_des_current;
	    	    vy_refdin_current=vy_des_current;
	    	    v_ref_current=v_refdin_current*metric;
		    w_ref_current=w_refdin_current;//radijani/s
		    vy_ref_current=vy_refdin_current*metric;
	    	    }
// 	  v_ref_current=(vr+vl)/2;
// 	  w_ref_current=(vr-vl)/b;
	  //racunanje akceleracije za sljedeci ciklus
#if (DELTA_REF==1)
//		    delta_v_refdin=v_refdin_current-RB.v;//zadane akceleracije
//		    delta_w_refdin=w_refdin_current-RB.w;
#else
		    delta_v_refdin=RB.v;
		    delta_w_refdin=RB.w;
		    delta_vy_refdin=RB.vy;
#endif
// 	  printf("delta_w_refdin %f, w_refdin_current-rbw=%f\n",delta_w_refdin,(w_refdin_current-rbw));
// 	  v_ref_current=(DW->round(v_ref_current));
// 	  w_ref_current=(DW->round(w_ref_current*RuS));
 /*  v_refdin_current=RB.v+delta_v_refdin;
		    w_refdin_current=RB.w+delta_w_refdin;
  //malo provjerica
		    if (v_refdin_current<0.)
		    v_refdin_current=0.;
		    if (v_refdin_current>V_MAX)
		    v_refdin_current=V_MAX;
		    if (w_refdin_current>W_MAX)
		    w_refdin_current=W_MAX;
		    if (w_refdin_current<W_MIN)
		    w_refdin_current=W_MIN;*/
	    }
 
/*	    M->setvel.v= v_ref_current;    //tu se postavljaju brzine za moj klasu
	    M->setvel.w=  w_ref_current;*/
//   SfROBOT->comInt(11,int(v_ref_current));//mm
//   SfROBOT->comInt(21,int(w_ref_current));//SfROBOT->comInt(21,int(w_ref_current*1.1));	
//   SfROBOT->setVel(int(v_ref_current));
//   SfROBOT->setRotVel(int(w_ref_current)); //stupnjevi

           

	    //azuriramo trenutnu planner putanju ako je promijenjena
	    if (DS->racunaoupromjeni){
      R_point *temp_point; int temp_length;

      //D* path
      temp_length=PL->GetPathLength();
      temp_point=PL->GetPath();
      bool first=true;//za logiranje
      if (planner_path_run.size_path){
	      first=false;
      }

      planner_path_run.path_x=reallociraj_double(planner_path_run.path_x, (planner_path_run.size_path+temp_length), first);
      planner_path_run.path_y=reallociraj_double(planner_path_run.path_y, (planner_path_run.size_path+temp_length), first);
      planner_path_run.path_th=reallociraj_double(planner_path_run.path_th, (planner_path_run.size_path+temp_length), first);
      planner_path_run.path_travcost=reallociraj_double(planner_path_run.path_travcost, (planner_path_run.size_path+temp_length), first);
    planner_path_run.path_h=reallociraj_double(planner_path_run.path_h, (planner_path_run.size_path+temp_length), first);
#if DSTAR3D
planner_path_run.path_intlo=reallociraj_double(planner_path_run.path_intlo, (planner_path_run.size_path+temp_length), first);
planner_path_run.path_intup=reallociraj_double(planner_path_run.path_intup, (planner_path_run.size_path+temp_length), first);
#endif
      for(int i=0;i<temp_length;i++)
      {
	      planner_path_run.path_x[i+planner_path_run.size_path]=temp_point[i].x;
	      planner_path_run.path_y[i+planner_path_run.size_path]=temp_point[i].y;
	      planner_path_run.path_th[i+planner_path_run.size_path]=temp_point[i].th;
		        planner_path_run.path_travcost[i+planner_path_run.size_path]=PL->ori[i].travcost;
		        planner_path_run.path_h[i+planner_path_run.size_path]=PL->ori[i].h;
#if DSTAR3D
		        planner_path_run.path_intlo[i+planner_path_run.size_path]=PL->ori[i].intlo;
		        planner_path_run.path_intup[i+planner_path_run.size_path]=PL->ori[i].intup;
#endif
    	}
	planner_path_run.size[planner_path_run.broj]=temp_length;
	planner_path_run.broj++;
    	planner_path_run.size_path+=temp_length;
#if (KOMBINACIJA==1) || DSTAR_REVERSE
      //tu ce biti logiran extra path - FD* path
	temp_length=DS->getPathLength();
	I_point *i_temp_point;
	i_temp_point=DS->GetPath();
	R_point r_temp_point;

	planner_path_run.path_x=reallociraj_double(planner_path_run.path_x, (planner_path_run.size_path+temp_length), first);
	planner_path_run.path_y=reallociraj_double(planner_path_run.path_y, (planner_path_run.size_path+temp_length), first);
	for(int i=0;i<temp_length;i++)
	{
		if(!PL->IntToReal(i_temp_point[i], r_temp_point, GM->Map_Dim_X,GM->Map_Dim_Y, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM))
		{
			printf("WH> PL->IntToReal > element path[%d].x=%d , path[%d].y=%d", i, i_temp_point[i].x,i, i_temp_point[i].y);
			printf("WH> PL->IntToReal > IntToReal conversion failed!");
// 			return 0;
		}
		planner_path_run.path_x[i+planner_path_run.size_path]=r_temp_point.x;
		planner_path_run.path_y[i+planner_path_run.size_path]=r_temp_point.y;
	}
	planner_path_run.size[planner_path_run.broj]=temp_length;
	planner_path_run.broj++;
	planner_path_run.size_path+=temp_length;
#endif
	    }
//           	//mjerim vrijeme
 		if (gettimeofday(&timeNow, NULL) == 0)
 		{
 			mySecNow = timeNow.tv_sec;
 			myMSecNow = timeNow.tv_usec / 1000;
 		}
 		vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
 		printf("WH> after path realloc %d ms.\n", vremenska_razlika);

    }
    //dodajem
    if (path_found==1){
    
    }
    else
    {
      if (path_found==2)
      {
      		DS->PathCost=OBSTACLE;//dodala zbog provjera

        printf("WorkHorse::running> The path is not yet calculated! time diff msec: %d \n", vremenska_razlika);
#if 0 //komentiram nek vozi po dw-u
        //ramp
#if (OKO_REFERENCE==1)
	rbv=v_refdin_current;
	rbw=w_refdin_current;
	rbvy=vy_refdin_current;
#else
	rbv=RB.v;
	rbw=RB.w;
	rbvy=RB.vy;
#endif
	v_ref_current=0.;v_refdin_current=0.;v_des_current=0.;delta_v_refdin=0.;delta_w_refdin=0.;
	w_ref_current=0.;w_refdin_current=0.;w_des_current=0.;dwpath_length=0.;
	vy_ref_current=0.;vy_refdin_current=0.;vy_des_current=0.;delta_vy_refdin=0.;
#if (NO_PATH_RAMP==1) 
	if (fabs(rbv)>(DV_MAX*STEP)) {//v_refdin_current
		v_des_current=(rbv-(rbv)/fabs(rbv)*DV_MAX*STEP);//v_refdin_current
	}
	if (fabs(rbvy)>(DVY_MAX*STEP)) {//v_refdin_current
		vy_des_current=(rbvy-(rbvy)/fabs(rbvy)*DVY_MAX*STEP);//v_refdin_current
	}
	if (fabs(rbw)>(DW_MAX*STEP)) {//w_refdin_current
		w_des_current=(rbw-(rbw)/fabs(rbw)*DW_MAX*STEP);//w_refdin_current
	}
#endif
#if (NO_PATH_RAMP==2)
  int N,Tvx,Tvy,Tw;
  Tvx=(int)ceil((fabs(rbv))/(DVX_MAX*STEP));//vrijeme zaustavljanja
	Tvy=(int)ceil(fabs(rbvy)/(DVY_MAX*STEP));
	Tw=(int)ceil(fabs(rbw)/(DW_MAX*STEP));
	N=std::max(Tvx,Tw);
	N=std::max(N,Tvy);
	v_des_current=std::max(0.,rbv*(N-1)/N);
	vy_des_current=std::max(0.,rbw*(N-1)/N);
	w_des_current=std::max(0.,rbvy*(N-1)/N); 
#endif
	v_refdin_current=v_des_current; v_ref_current=v_refdin_current*metric;
	vy_refdin_current=vy_des_current; vy_ref_current=vy_refdin_current*metric;
	w_refdin_current=w_des_current; w_ref_current=w_refdin_current;
	//end ramp
#endif
			//LOGGER VREMENA - KRAJ ALGORITMA -------------------------------------------------
		if (gettimeofday(&timeNow, NULL) == 0)
		{
			mySecNow = timeNow.tv_sec;
			myMSecNow = timeNow.tv_usec / 1000;
		}
		vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
		vremenska_razlika0=(mySecNow-mySecStart0)*1000+(myMSecNow-myMSecStart0);
       LogCurrent(process_increment);
        process_increment++;
/*		if (DS->prepreka.x==-1){
			Logger();
			processState=HALT;*/
#if RECTANGULAR || 1
			no_path_counter++;
			if (no_path_counter==1) GM->reset();
			if (no_path_counter>20){
			no_path_counter=0;
//			processState=NO_PATH;
			processState=HALT;
			 	 Logger();
 	 process_increment=0;
 	 OslobodiMemoriju();
 	 logger_current_size=Logger_Init();
 	 printf("ok\n");	 

			}
#else
	  processState=NO_PATH;
	  no_path_counter=0;
#endif
/*		}else{
			no_path_counter=-2000;
// 		processState=RESUMING;
}	*/
	// 		if (DS->watchdog_counter!=0){
// 			Logger();
// 			processState=HALT;
			new_global_goal=true;//tak da stane
// 		}
	  LogCurrent(process_increment);
	  process_increment++;

        break;

      }
        else
        {
	        printf("WorkHorse::running> No path was found!!!\n");
          		DS->PathCost=OBSTACLE;//dodala zbog provjera
	  no_path_counter=0;
        //ramp
#if (OKO_REFERENCE==1)
	rbv=v_refdin_current;
	rbw=w_refdin_current;
	rbvy=vy_refdin_current;
#else
	rbv=RB.v;
	rbw=RB.w;
	rbvy=RB.vy;
#endif
	
	v_ref_current=0.;v_refdin_current=0.;v_des_current=0.;delta_v_refdin=0.;delta_w_refdin=0.;
	w_ref_current=0.;w_refdin_current=0.;w_des_current=0.;dwpath_length=0.;
	vy_ref_current=0.;vy_refdin_current=0.;vy_des_current=0.;delta_vy_refdin=0.;
#if (NO_PATH_RAMP==1) 
	if (fabs(rbv)>(DV_MAX*STEP)) {//v_refdin_current
		v_des_current=(rbv-(rbv)/fabs(rbv)*DV_MAX*STEP);//v_refdin_current
	}
	if (fabs(rbvy)>(DVY_MAX*STEP)) {//v_refdin_current
		vy_des_current=(rbvy-(rbvy)/fabs(rbvy)*DVY_MAX*STEP);//v_refdin_current
	}
	if (fabs(rbw)>(DW_MAX*STEP)) {//w_refdin_current
		w_des_current=(rbw-(rbw)/fabs(rbw)*DW_MAX*STEP);//w_refdin_current
	}
#endif
#if (NO_PATH_RAMP==2)
  int N,Tvx,Tvy,Tw;
  Tvx=(int)ceil((fabs(rbv))/(DVX_MAX*STEP));//vrijeme zaustavljanja
	Tvy=(int)ceil(fabs(rbvy)/(DVY_MAX*STEP));
	Tw=(int)ceil(fabs(rbw)/(DW_MAX*STEP));
	N=std::max(Tvx,Tw);
	N=std::max(N,Tvy);
	if (N>1){
	v_des_current=rbv*(N-1)/N;
	vy_des_current=rbw*(N-1)/N;
	w_des_current=rbvy*(N-1)/N;
	}else{
	v_des_current=0;
	vy_des_current=0;
	w_des_current=0;	
	} 
#endif

	v_refdin_current=v_des_current; v_ref_current=v_refdin_current*metric;
	vy_refdin_current=vy_des_current; vy_ref_current=vy_refdin_current*metric;
	w_refdin_current=w_des_current; w_ref_current=w_refdin_current;
	//end ramp
	  LogCurrent(process_increment);
	  process_increment++;
	  processState=NO_PATH;
//	  Logger();
// 	  processState=HALT;
	  new_global_goal=true;//tak da stane
	  break;
        }
    }  //else od path_found!=1
    
    
// 	printf("goal_tolerance=%f\n",goal_tolerance);
	goal_tolerance=sqrt((RB.x-global_goal_workhorse.x)*(RB.x-global_goal_workhorse.x)+(RB.y-global_goal_workhorse.y)*(RB.y-global_goal_workhorse.y));
	if(((goal_tolerance<=GOAL_POSITION_TOLERANCE)&&(fabs(v_refdin_current)<V_TOLERANCE)&&(fabs(RB.v)<V_TOLERANCE)&&(fabs(vy_refdin_current)<V_TOLERANCE)&&(fabs(RB.vy)<V_TOLERANCE))||(DW->naCilju))  //pola radijusa robota (*4 je radijus robota)
    {
	    if (DW->naCilju==false)
		    DW->naCilju=true;
	    //jos provjera za kut na cilju
	    theta_tolerance=fabs(RB.th-global_goal_workhorse.th);
	    while (theta_tolerance>=M_PI)
		    theta_tolerance=fabs(theta_tolerance-2*M_PI);
      double theta_tolerance_ref=THETA_TOLERANCE;//(DW_MAX*STEP)/(W_DIM);
#if (THETAGOAL==0)
	    theta_tolerance=0.;
#else
      if ((fabs(w_refdin_current)<W_TOLERANCE)&&(fabs(RB.w)<W_TOLERANCE)&&(fabs(v_refdin_current)<V_TOLERANCE)&&(fabs(RB.v)<V_TOLERANCE))
        DW->rotateongoal=true;
	    printf("theta_tolerance=%f deg, THETA_TOLERANCE=%f deg, theta_tolerance_ref=%f deg, RB.th=%f deg, global_goal_workhorse.th=%f deg, goal_tolerance=%f nacilju=%d rotateongoal=%d v_refdin_current=%f, w_refdin_current=%f\n",theta_tolerance*RuS, THETA_TOLERANCE*RuS, theta_tolerance_ref*RuS, RB.th*RuS, global_goal_workhorse.th*RuS, goal_tolerance, DW->naCilju, DW->rotateongoal, v_refdin_current, w_refdin_current);
#endif
	//    if ((fabs(theta_tolerance)<=THETA_TOLERANCE)){//&&(fabs(w_ref_current)<W_TOLERANCE)&&(fabs(RB.w)<W_TOLERANCE)){
	    if ((fabs(theta_tolerance)<=theta_tolerance_ref)&&(fabs(w_refdin_current)<W_TOLERANCE)&&(fabs(DW->oldref.w[0])<W_TOLERANCE)&&(fabs(v_refdin_current)<V_TOLERANCE)&&(fabs(DW->oldref.v[0])<V_TOLERANCE)&&(fabs(vy_refdin_current)<V_TOLERANCE)&&(fabs(DW->oldref.vy[0])<V_TOLERANCE)){
			    DW->naCilju=false;
			    DW->rotateongoal=false;
	    v_ref_current=0.;v_refdin_current=0.;v_des_current=0.;delta_v_refdin=0.;delta_w_refdin=0.;
	    w_ref_current=0.;w_refdin_current=0.;w_des_current=0.;dwpath_length=0.;
	    vy_ref_current=0.;vy_refdin_current=0.;vy_des_current=0.;delta_vy_refdin=0.;
	    printf("goal_tolerance=%f mm, GOAL_POSITION_TOLERANCE=%f mm, RB.x=%f, RB.y=%f, RB.th=%f, global_goal_workhorse (x=%f, y=%f, th=%f)\n",(goal_tolerance), GOAL_POSITION_TOLERANCE, RB.x, RB.y, RB.th, global_goal_workhorse.x, global_goal_workhorse.y, global_goal_workhorse.th);
	    printf("+++WorkHorse::Process()> Entering goal region, process_increment=%d, cycle_number=%d++++++\n", process_increment,cycle_number);
	 LogCurrent(process_increment);
	 process_increment++;
	 no_path_counter=0;
	 processState=HALT;
	 goal_tolerance=((M->subgoal.x-M->goal.x)*(M->subgoal.x-M->goal.x)+(M->subgoal.y-M->goal.y)*(M->subgoal.y-M->goal.y));
	 if (goal_tolerance>GOAL_POSITION_TOLERANCE*GOAL_POSITION_TOLERANCE) //((M->subgoal.x!=M->goal.x)||(M->subgoal.y!=M->goal.y))
	 {
	 //ova zastavica trenutno nicem ne sluzi jer se u main-u postavlja
	 if (M->subgoal2flag){
	 	global_goal_workhorse=M->subgoal2;
	 	M->subgoal2flag=false;
	 	printf("WH> subgoal2\n");
	 }else{
	 	global_goal_workhorse=M->goal;
	 }
	 new_global_goal=false;//provjeri je li mozda stigao do subgoala umjesto do goala
//	 M->subgoal=M->goal;//ovo isto postavlja main
	 processState=NO_PATH; //jer ga ovaj prebaci u main-u
	 }else{
 	 Logger();
 	 process_increment=0;
 	 OslobodiMemoriju();
 	 logger_current_size=Logger_Init();
 	 printf("ok\n");	 
	 }
	 
      break;
	    }
    }         //od goal_tolerance
    
    
	//LOGGER VREMENA - KRAJ ALGORITMA -------------------------------------------------
  if (gettimeofday(&timeNow, NULL) == 0)
  {
    mySecNow = timeNow.tv_sec;
	  myMSecNow = timeNow.tv_usec / 1000;
  }
  vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
  vremenska_razlika0=(mySecNow-mySecStart0)*1000+(myMSecNow-myMSecStart0);
     	//ovdje se azuriraju novi podaci vezani za robota
  if (DW->ljapunov>=Log_robot_path.ljapunov[process_increment-1]){
  	printf("increase of J*=%.15f, old was %.15f\n",DW->ljapunov, Log_robot_path.ljapunov[process_increment-1]);
  }else{
  	printf("decrease of J*=%.15f\n",DW->ljapunov);  
  }

  LogCurrent(process_increment);
  process_increment++;
  if (vremenska_razlika>100) {
	  printf("WH: calculations last more than 100ms. time_diff=%d\n",vremenska_razlika);
  }
  if (0 && DW->best_old==true){
	  Logger();
  }

    if (DW->locmincnt>N_KL){
      processState=NO_PATH;
    }
//  if (process_increment==13 || process_increment==21 || process_increment==32 || process_increment==43){
//	  Logger();
//  }
    if (DW->best_breakage){
    printf("tu");
    }
/*  if(GM->num_col_points_old){
	  Logger();
  processState=HALT;
	}*/
/*  if(Log_robot_path.size_teziste>240){
	  Logger();
	  processState=HALT;
  }*/
  //-----------------------------------------------------------------------------------------------------------------
  break;

 }// end of switch
 M->setvel.v= v_ref_current;    //tu se postavljaju brzine za moj klasu
 M->setvel.w=  w_ref_current;
 M->setvel.vy= vy_ref_current;
 if (DW->poceo){
 for (int i=DELAY3DT; i>0; i--){
    DW->oldref.v[i]=DW->oldref.v[i-1];
    DW->oldref.vy[i]=DW->oldref.vy[i-1];
    DW->oldref.w[i]=DW->oldref.w[i-1];
 }
 DW->oldref.v[0]=v_refdin_current;
 DW->oldref.vy[0]=vy_refdin_current;
 DW->oldref.w[0]=w_refdin_current;
 for (int i=0; i<DELAY3DT+1; i++){
    printf("(v,vy,w)=(%f,%f,%f)\n",DW->oldref.v[i],DW->oldref.vy[i],DW->oldref.w[i]);
 }
 }
}

void WorkHorse::Ucitaj_stanje_lasera(){
	//sad upisivanje novih podataka
	int i,j;//,brojpomaka;
	double distance,x_temp,y_temp,kut;//,pomakprepreke
	i=0; j=0;LB.laser_pointer_new=0;LB.laser_manji_new=0;rmin=LASER_RANGE_MAX;
	double rmax=0.;
	int imax=0;
	bool nasaormax=false;
	//laser2
	LB2.laser_pointer_new=0;LB2.laser_manji_new=0;
	for (i=0; i<M->LB2.scan_count; i++) {
		LB2.sviscanovi[i].r=M->LB2.scan[i].r;  //ovdje su svi laseri, trebaju mi za GM
		LB2.sviscanovi[i].th=M->LB2.scan[i].th;
		LB2.LB_pose[i].x=M->LB2.point[i].x;
		LB2.LB_pose[i].y=M->LB2.point[i].y;
		if(LB2.sviscanovi[i].r<LASER_RANGE_MAX){
			//tu bi isli samo oni za trazenje tezista (ako radim provjeru gornju)
			LB2.LB_scan[j]=LB2.sviscanovi[i];
			LB2.LB_pose_manji[j]=LB2.LB_pose[i];
			j++;
		}
	}
	LB2.laser_pointer_new=i;
	LB2.laser_manji_new=j;

	//laser1
	j=0;	
	for (i=0; i<M->LB.scan_count; i++) {
		LB.sviscanovi[i].r=M->LB.scan[i].r;  //ovdje su svi laseri, trebaju mi za GM
		LB.sviscanovi[i].th=M->LB.scan[i].th;
			if (0) {//ne trebam ovo medo
			if (LB.sviscanovi[i].r<rmin){
				rmin=LB.sviscanovi[i].r;//udaljenost do najblize prepreke
			}
			
			if (!nasaormax){	
			if (LB.sviscanovi[i].r>rmax){
				rmax=LB.sviscanovi[i].r;//udaljenost do najblize prepreke
				imax=i;
				//gledanje okoline tog najdaljeg hita
				//ako na udaljenosti od 5*CELL_DIM postoji hit koji bi prosirivanjem za robot_mask popunio prostor na trenutnom th onda se trazi sljedeci max udaljeni hit
				//w=l/r=3*CELL_DIM/(5*CELL_DIM)=35 st, odnosno 70 hitova, a 4*CELL_DIM za svaki slucaj je 45 st, odnosno 90 hitova
				if ((rmax>5*CELL_DIM)&&(i>90)&&(i<M->LB.scan_count-90)){//pazi nece valjati za laser s 180 zraka
					int trazidalje=0;
					for (int k=i-90;k<i+90;k++){
						if (LB.sviscanovi[k].r<5*CELL_DIM){
							trazidalje=1;
// 							printf("ovaj rmax nije dobar, indeks=%d\n",k);
							rmax=0.;
							break;
						}
					}
					if (trazidalje==0){
						nasaormax=true;
					}
				}
			}
			}
			}//if 0 medo
		  LB.LB_pose[i].x=M->LB.point[i].x;
		  LB.LB_pose[i].y=M->LB.point[i].y;
			if(LB.sviscanovi[i].r<LASER_RANGE_MAX){
				//tu bi isli samo oni za trazenje tezista (ako radim provjeru gornju)
				LB.LB_scan[j]=LB.sviscanovi[i];
				LB.LB_pose_manji[j]=LB.LB_pose[i];
				j++;
			}
	}
		
    
      LB.laser_pointer_new=i;
      LB.laser_manji_new=j;

	

      if (0) { //ne trebam ovo medo
      R_point temp;
	      temp.x=5*CELL_DIM*cos(LB.sviscanovi[imax].th);//lokalna, pol metra ispred robota
	      temp.y=(5*CELL_DIM)*sin(LB.sviscanovi[imax].th);
	      M->subgoal.x=Lokalna_u_globalnu_x(RB.x, RB.th,temp.x,temp.y);
	      M->subgoal.y=Lokalna_u_globalnu_y(RB.y, RB.th,temp.x,temp.y);
	      //provjera da podcilj ne padne u prepreku zbog krive lokalizacije
	      I_point subgoal_i;
	      if(!PL->RealToInt(M->subgoal, subgoal_i, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)) {
		      printf("Subgoal je izvan mape!\n");
// 		      return 0;
	      }
	      int broj=4;
	      while(DS->IsValid(subgoal_i.x, subgoal_i.y)!=1){
// 		      printf("Subgoal je zauzet. Trazim drugi, blize.\n");
		      temp.x=broj*CELL_DIM*cos(LB.sviscanovi[imax].th);//lokalna, pol metra ispred robota
		      temp.y=(broj*CELL_DIM)*sin(LB.sviscanovi[imax].th);
		      M->subgoal.x=Lokalna_u_globalnu_x(RB.x, RB.th,temp.x,temp.y);
		      M->subgoal.y=Lokalna_u_globalnu_y(RB.y, RB.th,temp.x,temp.y);
		      if(!PL->RealToInt(M->subgoal, subgoal_i, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
			      printf("Subgoal je izvan mape!\n");
// 			      return 0;
		      }
		      broj--;
		      if (broj==0){
			      for (int d=0;d<8;d++){
				      subgoal_i.x=DS->StartRacunac.x+xofs[d];
				      subgoal_i.y=DS->StartRacunac.y+yofs[d];
				      if ((subgoal_i.x==DS->Start.x)&&(subgoal_i.y==DS->Start.y))
					      continue;
				      if (DS->IsValid(subgoal_i.x, subgoal_i.y)==1){
					      if(!PL->IntToReal(subgoal_i, M->subgoal, GM->Map_Dim_X,GM->Map_Dim_Y, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM))
					      {
						      printf("ne bi smio neuspjet pretvorit int u real\n");
					      }
					      break;
// 					      printf("pored startracunca (%d,%d) je ovaj subgoal nadjen (%d,%d)",DS->StartRacunac.x,DS->StartRacunac.y,subgoal_i.x,subgoal_i.y);
				      }
			      }
		      }
	      }
	      if (DS->IsValid(subgoal_i.x, subgoal_i.y)!=1){//ako bas nijednog nema slobodnog uzmi start
		      if(!PL->IntToReal(DS->StartRacunac, M->subgoal, GM->Map_Dim_X,GM->Map_Dim_Y, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM))
		      {
			      printf("ne bi smio neuspjet pretvorit int u real\n");
		      }
// 		      printf("uzimam start\n");
	      }
// 	      printf("subgoal_i=(%d,%d)\n",subgoal_i.x,subgoal_i.y);
}//if 0 medo
 	return;
}

void WorkHorse::Ucitaj_stanje_odometrije(){


  //brzine
	RB.v =M->RB.v;
	RB.w =M->RB.w;
	RB.vy =M->RB.vy;
// 	RB.v =(SfROBOT->getVel());
//   RB.w =SuR*(SfROBOT->getRotVel());//
  v_kal_current=RB.v;
  w_kal_current=RB.w;
  double vl,vr;
// 	vl=(SfROBOT->getLeftVel());
// 	vr=(SfROBOT->getRightVel());
 	//printf("prije: RB.v=%.15f, RB.w=%.15f\n",RB.v,RB.w);
	//kalibrirana odometrija
	vl=(2*RB.v-b*RB.w)/2;
	vr=(2*RB.v+b*RB.w)/2;
	if (kl>1.){
	RB.v=(kl*vl+kr*vr)/2;
	RB.w=(kr*vr-kl*vl)/(kb*b);
	}
 	//printf("poslije: RB.v=%.15f, RB.w=%.15f, sur=%.15f, rus=%.15f, M_PI=%.15f\n",RB.v,RB.w,SuR,double(RuS),double(M_PI));
//odometrija "LOKALNA" (npr. za dynamic window)
 // Ocitanje polozaja i brzine
 RB.x =M->RB.x;//(SfROBOT->getX());
 RB.y =M->RB.y;//(SfROBOT->getY());
  RB.th=M->RB.th;//(SfROBOT->getTh()); u radijanima je!
//       RB.th*=SuR;//u radiane
//PAZNJA!!! - svodjenje na [0, 2*M_PI>
	while(RB.th<0)
		RB.th+=2*M_PI;
	while(RB.th>=2*M_PI)
		RB.th-=2*M_PI;
	return;
}


WorkHorse::WorkHorse() {
     //LASER=NULL;   //pretpostavka je da laser jos nije spojen
	planner_path_run.size_path=0;
	planner_path_run.broj=0;
 LB.laser_pointer_new=0;
  LB2.laser_pointer_new=0;
metric=0.001;
RB.v=0.;
RB.w=0.;
kl=1.;
kr=1.;
kb=1.;
/*kl=0.9989;
kr=1.0011;
kb=1.0216;*/
b=320.;
Tv=0.000009;//11;//0.11;
Tw=0.000009;//0.09;
  //process state smo u svakom slucaji inicijalizirali na INIT
  processState=INIT;
  firstlog=true;
}


/**************************
 * Destruktor klase WorkHorse *
 **************************/
WorkHorse::~WorkHorse(){
	printf("WorkHorse destroyed!");
// 	this->OslobodiMemoriju();
}


//prebacivanje iz lokalnih u globalne koordinate. Rx i Rth su dio pozicije robota; tockaX i tockaY su lokalne koordinate tocke koju prebacujemo u globalnu x koordinatu
double WorkHorse::Lokalna_u_globalnu_x(double Rx, double Rth, double tockaX, double tockaY){
	double globalnaX;
	globalnaX=Rx+cos(Rth)*tockaX-sin(Rth)*tockaY;
	return globalnaX;
}

//prebacivanje iz lokalnih u globalne koordinate. Ry i Rth su dio pozicije robota; tockaX i tockaY su lokalne koordinate tocke koju prebacujemo u globalnu y koordinatu
double WorkHorse::Lokalna_u_globalnu_y(double Ry, double Rth, double tockaX, double tockaY){
	double globalnaY;
	globalnaY=Ry+sin(Rth)*tockaX+cos(Rth)*tockaY;
	return globalnaY;
}

//prebacivanje iz globalnih u lokalne koordinate. Rx, Ry i Rth je pozicija robota; tockaX i tockaY su globalne koordinate tocke koju prebacujemo u lokalnu x koordinatu
double WorkHorse::Globalna_u_lokalnu_x(double Rx, double Ry, double Rth, double tockaX, double tockaY){
	double lokalnaX;
	lokalnaX=cos(Rth)*(tockaX-Rx)+sin(Rth)*(tockaY-Ry);
	return lokalnaX;
}

//prebacivanje iz globalnih u lokalne koordinate. Rx, Ry i Rth je pozicija robota; tockaX i tockaY su globalne koordinate tocke koju prebacujemo u lokalnu y koordinatu
double WorkHorse::Globalna_u_lokalnu_y(double Rx, double Ry, double Rth, double tockaX, double tockaY){
	double lokalnaY;
	lokalnaY=-sin(Rth)*(tockaX-Rx)+cos(Rth)*(tockaY-Ry);
	return lokalnaY;
}



void WorkHorse::Logger(){
      FILE *F;
      FILE *F_temp;
	  FILE *F2;
	  FILE *F3;
      FILE *F_temp2;
	  FILE *F1;
	  FILE *F_temp1,*F4,*F5,*F6,*F7,*F8,*F9,*F10,*F11,*F12, *F13, *F14;
      int i;
	
	DW->Ispis_polja();
int logsizes=logger_sizes[0];
    printf("WorkHorse> Average speed: v_average=%f\n", average_speed/(double)brojac_brzina);
    printf("WorkHorse> Time to the goal [sec]: t_sum=%f\n", (double)brojac_brzina/10.);
    
    char op[]="wt";
if (firstlog){
    firstlog=false;
}else{
    op[0]='a';
    op[1]='\0';
}

        //UPISIVANJE vremena izracuna cijelog algoritma i  vremena svakog kraja algoritma mjerena od samog pocetka stanja RUNNINGu
    if(((F=fopen(WH_VREMENA,op))!=NULL)&&((F12=fopen(DS_VREMENA,op))!=NULL)&&((F14=fopen(DS_EXPLORED,op))!=NULL)&&((F9=fopen(DW_VREMENA,op))!=NULL)&&((F2=fopen(WH_LOG_TIME,op))!=NULL)&&((F3=fopen(WH_LOG_READ_TIME,op))!=NULL)&&((F4=fopen(EF_PATH_LENGTH,op))!=NULL)&&((F_temp=fopen(EF_PATH_ORIENTATION,op))!=NULL)&&((F_temp1=fopen(TOCKA_INFLEKSIJE_X,op))!=NULL)&&((F_temp2=fopen(TOCKA_INFLEKSIJE_Y,op))!=NULL)&&((F5=fopen(FAKELOCWH_ROBOT_GLOBALNA_PUTANJA_X,op))!=NULL)&&((F6=fopen(FAKELOCWH_ROBOT_GLOBALNA_PUTANJA_Y,op))!=NULL)&&((F7=fopen(FAKELOCWH_ROBOT_GLOBALNA_PUTANJA_TH,op))!=NULL)&&((F8=fopen(LJAPUNOV_PUTANJA,op))!=NULL)&&((F13=fopen(INTCOST_PUTANJA,op))!=NULL)&&((F10=fopen(REPLAN_PUTANJA,op))!=NULL)&&((F11=fopen(TRAVCOST_PUTANJA,op))!=NULL)&&((F1=fopen(TRAVCOST_PUTANJA3D,op))!=NULL)) {
      //putanje po svim brzinama
		for(i=0;i<logsizes;i++){             //<=!!!!!!!!!!!!!!!
			fprintf(F,"%d ",(int) Log_robot_path.izracuni[i]);fprintf(F12,"%f ",Log_robot_path.DSizracuni[i]);fprintf(F14,"%f ",Log_robot_path.DSexplored[i]);fprintf(F9,"%d ",(int) Log_robot_path.DWizracuni[i]);fprintf(F2,"%d ",(int) Log_robot_path.time[i]);fprintf(F3,"%d ",(int) Log_robot_path.read_time[i]);fprintf(F4,"%f ",Log_robot_path.path_length[i]);
			fprintf(F_temp,"%f ",Log_robot_path.path_orientation[i]);
			fprintf(F_temp1,"%f ",Log_robot_path.tocka_infleksije_x[i]);
			fprintf(F_temp2,"%f ",Log_robot_path.tocka_infleksije_y[i]);
			fprintf(F8,"%f ",Log_robot_path.ljapunov[i]);
			fprintf(F13,"%f ",Log_robot_path.interpolatedcost[i]);
			fprintf(F10,"%f ",Log_robot_path.replan[i]);
			fprintf(F11,"%f ",Log_robot_path.travcost[i]);
			fprintf(F1,"%f ",Log_robot_path.travcost3d[i]);
		}
//       fprintf(F,"\n");fprintf(F9,"\n");fprintf(F12,"\n");fprintf(F14,"\n");
//	   fprintf(F2,"\n");
//	   fprintf(F3,"\n");
//	   fprintf(F4,"\n");
//	   fprintf(F_temp,"\n");
//	   fprintf(F_temp1,"\n");
//	   fprintf(F_temp2,"\n");
//	   fprintf(F5,"\n");
//	   fprintf(F6,"\n");
//	   fprintf(F7,"\n");
//	   fprintf(F8,"\n");
//	   fprintf(F10,"\n");
//	   fprintf(F11,"\n");fprintf(F1,"\n");fprintf(F13,"\n");
	   fclose(F);   fclose(F2);fclose(F3);fclose(F4);fclose(F5);fclose(F6);fclose(F7);fclose(F8);fclose(F13);
	   fclose(F_temp);fclose(F_temp1);fclose(F_temp2);fclose(F9);fclose(F11);fclose(F1);fclose(F10); fclose(F12);fclose(F14);
	    }else{
        printf("Otvaranje fileova bezuspjesno!");
      }
   //UPISIVANJE X,Y- koordinata geometrijske GLOBALNE inicijalne PUTANJE
	  if(((F=fopen(WH_GLOBAL_PLANNER_PATH_X,op))!=NULL)&&((F4=fopen(WH_GLOBAL_PLANNER_PATH_TRAVCOST,op))!=NULL)&&((F5=fopen(WH_GLOBAL_PLANNER_PATH_H,op))!=NULL)&&((F6=fopen(WH_GLOBAL_PLANNER_PATH_INTLO,op))!=NULL)&&((F7=fopen(WH_GLOBAL_PLANNER_PATH_INTUP,op))!=NULL)&&((F2=fopen(WH_GLOBAL_PLANNER_PATH_Y,op))!=NULL) && ((F3=fopen(WH_GLOBAL_PLANNER_PATH_TH,op))!=NULL)){
      //putanje po svim brzinama
      for(i=0;i<planner_path_original.size_path;i++){             //<=!!!!!!!!!!!!!!!
		  fprintf(F,"%f ",planner_path_original.path_x[i]); fprintf(F2,"%f ",planner_path_original.path_y[i]);
		  fprintf(F3,"%f ",planner_path_original.path_th[i]);
		  fprintf(F4,"%f ",planner_path_original.path_travcost[i]);
		  fprintf(F5,"%f ",planner_path_original.path_h[i]);
#if DSTAR3D
		  fprintf(F6,"%f ",planner_path_original.path_intlo[i]);
		  fprintf(F7,"%f ",planner_path_original.path_intup[i]);
#endif
      }
//	  fprintf(F,"\n");   fprintf(F2,"\n"); fprintf(F3,"\n");
//	  fprintf(F4,"\n");  fprintf(F5,"\n");  fprintf(F6,"\n");  fprintf(F7,"\n");  

	  fclose(F);    fclose(F2); fclose(F3);
	  fclose(F4);fclose(F5);fclose(F6);fclose(F7);
	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }
   //UPISIVANJE X,Y- koordinata geometrijske GLOBALNE trenutne PUTANJE
	  if(((F=fopen(WH_GLOBAL_PLANNER_PATH_CURRENT_X,op))!=NULL)&&((F4=fopen(WH_GLOBAL_PLANNER_PATH_CURRENT_TRAVCOST,op))!=NULL)&&((F5=fopen(WH_GLOBAL_PLANNER_PATH_CURRENT_H,op))!=NULL)&&((F6=fopen(WH_GLOBAL_PLANNER_PATH_CURRENT_INTLO,op))!=NULL)&&((F7=fopen(WH_GLOBAL_PLANNER_PATH_CURRENT_INTUP,op))!=NULL)&&((F2=fopen(WH_GLOBAL_PLANNER_PATH_CURRENT_Y,op))!=NULL) && ((F3=fopen(WH_GLOBAL_PLANNER_PATH_CURRENT_TH,op))!=NULL)){
      //putanje po svim brzinama
      for(i=0;i<planner_path_run.size_path;i++){             //<=!!!!!!!!!!!!!!!
		  fprintf(F,"%f ",planner_path_run.path_x[i]);fprintf(F2,"%f ",planner_path_run.path_y[i]);
		  fprintf(F3,"%f ",planner_path_run.path_th[i]);
		  fprintf(F4,"%f ",planner_path_run.path_travcost[i]);
		  fprintf(F5,"%f ",planner_path_run.path_h[i]);
#if DSTAR3D
		  fprintf(F6,"%f ",planner_path_run.path_intlo[i]);
		  fprintf(F7,"%f ",planner_path_run.path_intup[i]);
#endif
      }
//	  fprintf(F,"\n");   fprintf(F2,"\n"); fprintf(F3,"\n");
//	  fprintf(F4,"\n");  fprintf(F5,"\n");  fprintf(F6,"\n");  fprintf(F7,"\n");  

	  fclose(F);   fclose(F2); fclose(F3);
	  fclose(F4);fclose(F5);fclose(F6);fclose(F7);
	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }

    //prosjecna brzina gibanja i vrijeme gibanja
    if((F=fopen(WH_TIME_SPEED,"wt"))!=NULL){
          
        	fprintf(F,"%f ",average_speed/(double)brojac_brzina);
		fprintf(F,"%f ",(double)brojac_brzina/10.);
       fprintf(F,"\n");

		fclose(F);
	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }
    
    //UPISIVANJE X,Y,TH,V,W,V_REF,W_REF- koordinata GLOBALNE glatke PUTANJE
      if(((F=fopen(WH_ROBOT_GLOBALNA_PUTANJA_X,op))!=NULL)&&((F2=fopen(WH_ROBOT_GLOBALNA_PUTANJA_Y,op))!=NULL)&&((F3=fopen(WH_ROBOT_GLOBALNA_PUTANJA_TH,op))!=NULL)&&((F_temp=fopen(WH_ROBOT_TRANS_VEL,op))!=NULL)&&((F10=fopen(WH_ROBOT_TRANS_VELY,op))!=NULL)&&((F11=fopen(WH_ROBOT_TRANS_VELY_REF,op))!=NULL)&&((F_temp2=fopen(WH_ROBOT_ROT_VEL,op))!=NULL)&&((F_temp1=fopen(WH_ROBOT_TRANS_VEL_REF,op))!=NULL)&&((F1=fopen(WH_ROBOT_ROT_VEL_REF,op))!=NULL)&&((F4=fopen(WH_ROBOT_TRANS_VEL_REFDIN,op))!=NULL)&&((F5=fopen(WH_ROBOT_ROT_VEL_REFDIN,op))!=NULL)&&((F6=fopen(WH_ROBOT_TRANS_VEL_KAL,op))!=NULL)&&((F7=fopen(WH_ROBOT_ROT_VEL_KAL,op))!=NULL)&&((F8=fopen(WH_ROBOT_TRANS_VEL_DES,op))!=NULL)&&((F9=fopen(WH_ROBOT_ROT_VEL_DES,op))!=NULL)){
      //putanje po svim brzinama
		for(i=0;i<logsizes;i++){             //<=!!!!!!!!!!!!!!!
			fprintf(F,"%f ",Log_robot_path.path_x[i]); fprintf(F2,"%f ",Log_robot_path.path_y[i]);fprintf(F3,"%f ",Log_robot_path.path_th[i]);
			fprintf(F_temp,"%f ",Log_robot_path.v[i]);
			fprintf(F10,"%f ",Log_robot_path.vy[i]);fprintf(F11,"%f ",Log_robot_path.vy_ref[i]);
			fprintf(F_temp2,"%f ",Log_robot_path.w[i]);
			fprintf(F_temp1,"%f ",Log_robot_path.v_ref[i]); fprintf(F1,"%f ",Log_robot_path.w_ref[i]);
			fprintf(F4,"%f ",Log_robot_path.v_refdin[i]); fprintf(F5,"%f ",Log_robot_path.w_refdin[i]);
			fprintf(F6,"%f ",Log_robot_path.v_kal[i]); fprintf(F7,"%f ",Log_robot_path.w_kal[i]);
			fprintf(F8,"%f ",Log_robot_path.v_des[i]); fprintf(F9,"%f ",Log_robot_path.w_des[i]);
		}
//		fprintf(F,"\n"); fprintf(F2,"\n");  fprintf(F3,"\n"); fprintf(F_temp,"\n");  fprintf(F_temp2,"\n");  fprintf(F_temp1,"\n"); fprintf(F1,"\n");fprintf(F4,"\n"); fprintf(F5,"\n");fprintf(F6,"\n");fprintf(F7,"\n");fprintf(F8,"\n");
//		fprintf(F9,"\n");fprintf(F10,"\n"); fprintf(F11,"\n");
		fclose(F);  fclose(F2);   fclose(F3);  fclose(F_temp);    fclose(F_temp2); fclose(F_temp1); fclose(F1);fclose(F4);
		fclose(F5);fclose(F6);fclose(F7);	fclose(F8); fclose(F9);
		fclose(F10);  fclose(F11);
      }else{
        printf("Otvaranje filea bezuspjesno!");
      }

	  //broj podataka u poljima od loggera (zapravo broj ciklusa buduci da se svaki ciklus pune polja jednim podatkom)
      if((F=fopen(VARIOUS_LOG_SIZES,op))!=NULL){
     for(i=0;i<size_logger_num;i++){
        	fprintf(F,"%d ",logger_sizes[i]);
		printf("logger_sizes[%d]=%d\n", i, logger_sizes[i]);
      }
       fprintf(F,"\n");
       fclose(F);
	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }
	  //globalne koordinate x i y lasera iz svih ciklusa, laser pointeri su broj hitova u svakom ciklusu, zbrajanjem znace indekse tih velikih polja x i y koordinata koji oznacavaju prvi hit u ciklusu
      if(((F=fopen(WH_LOG_SVILASERI_X,op))!=NULL)&&((F_temp=fopen(WH_LOG_SVILASERI_Y,op))!=NULL) &&((F2=fopen(WH_LOG_SVILASERPOINTERI,op))!=NULL)&&((F3=fopen(WH_LOG_SVIPATHPOINTERI,op))!=NULL)&&((F4=fopen(WH_LOG_DSTAR_PRESLIK_X,op))!=NULL)&&((F5=fopen(WH_LOG_DSTAR_PRESLIK_Y,op))!=NULL)&&((F6=fopen(WH_LOG_SVILASERI2_X,op))!=NULL)&&((F7=fopen(WH_LOG_SVILASERI2_Y,op))!=NULL) &&((F8=fopen(WH_LOG_SVILASERPOINTERI2,op))!=NULL)){
		  for(int i=0;i<Log_robot_path.size_laser;i++){

			  fprintf(F,"%f ",Log_robot_path.laser_x[i]);
			  fprintf(F_temp,"%f ",Log_robot_path.laser_y[i]);
		  }
		  for(int i=0;i<Log_robot_path.size_laser2;i++){

			  fprintf(F6,"%f ",Log_robot_path.laser2_x[i]);
			  fprintf(F7,"%f ",Log_robot_path.laser2_y[i]);
		  }
		  for(int i=0;i<logsizes;i++){

			  fprintf(F2,"%f ",Log_robot_path.broj_hitova[i]);
			  fprintf(F8,"%f ",Log_robot_path.broj_hitova2[i]);
			  fprintf(F4,"%f ",Log_robot_path.preslik_x[i]);
			  fprintf(F5,"%f ",Log_robot_path.preslik_y[i]);
		  }
		  for(int i=0;i<planner_path_run.broj;i++){
	      		  fprintf(F3,"%f ",planner_path_run.size[i]);
      		  }
//		  fprintf(F,"\n"); fprintf(F2,"\n");fprintf(F4,"\n");fprintf(F5,"\n");
//		  fprintf(F6,"\n");fprintf(F7,"\n");fprintf(F8,"\n");
//		  fprintf(F_temp,"\n");
		  fclose(F); fclose(F2);fclose(F3);fclose(F4);fclose(F5);
		  fclose(F_temp);
		  fclose(F6);fclose(F7);fclose(F8);
	  }else{
		  printf("Otvaranje filea bezuspjesno!");
	  }
	  //globalne koordinate x i y tezista laserskih ocitanja iz svih ciklusa, teziste pointeri su broj tezista u svakom ciklusu, zbrajanjem znace indekse tih polja x i y koordinata tezista koji znace prvo teziste u ciklusu
	  if(((F=fopen(WH_LOG_TEZISTE_X,op))!=NULL)&&((F_temp=fopen(WH_LOG_TEZISTE_Y,op))!=NULL) &&((F2=fopen(WH_LOG_TEZISTEPOINTERI,op))!=NULL)){
		  for(int i=0;i<Log_robot_path.size_teziste;i++){

			  fprintf(F,"%f ",Log_robot_path.teziste_x[i]);
			  fprintf(F_temp,"%f ",Log_robot_path.teziste_y[i]);
		  }
		  for(int i=0;i<logsizes;i++){

			  fprintf(F2,"%f ",Log_robot_path.broj_tezista[i]);
		  }
//		  fprintf(F,"\n"); fprintf(F2,"\n");
//		  fprintf(F_temp,"\n");
		  fclose(F); fclose(F2);
		  fclose(F_temp);
	  }else{
		  printf("Otvaranje filea bezuspjesno!");
	  }
	  if(((F=fopen(DW_OPT_TRAJ_X,op))!=NULL)&&((F_temp=fopen(DW_OPT_TRAJ_Y,op))!=NULL) &&((F2=fopen(DW_LOG_POINTERI,op))!=NULL)){
		  for(int i=0;i<Log_robot_path.size_dwopttraj;i++){

			  fprintf(F,"%f ",Log_robot_path.dwopttraj_x[i]);
			  fprintf(F_temp,"%f ",Log_robot_path.dwopttraj_y[i]);
		  }
		  for(int i=0;i<logsizes;i++){

			  fprintf(F2,"%f ",Log_robot_path.broj_dwopttraj[i]);
		  }
//		  fprintf(F,"\n"); fprintf(F2,"\n");
//		  fprintf(F_temp,"\n");
		  fclose(F); fclose(F2);
		  fclose(F_temp);
	  }else{
		  printf("Otvaranje filea bezuspjesno!");
	  }

	  //pocetna pozicija robota i cilj
	  if(((F=fopen(WH_LOG_START_POSITION,op))!=NULL)&&((F2=fopen(WH_LOG_GOAL_POSITION,op))!=NULL)) {
      //putanje po svim brzinama
                  //<=!!!!!!!!!!!!!!!
#if DSTAR_REVERSE
		  fprintf(F,"%d ",DS->StartRacunac.x);
		  fprintf(F,"%d ",DS->StartRacunac.y);
		  fprintf(F2,"%d ",DS->GoalRacunac.x);
		  fprintf(F2,"%d ",DS->GoalRacunac.y);
#else
		  fprintf(F,"%f ",(double)M->start.x);
		  fprintf(F,"%f ",(double)M->start.y);
		  fprintf(F2,"%f ",(double)global_goal_workhorse.x);
		  fprintf(F2,"%f %f",(double)global_goal_workhorse.y, global_goal_workhorse.th);
#endif

//		  fprintf(F,"\n");  fprintf(F2,"\n");

		  fclose(F);  fclose(F2);
	  }else{
		  printf("Otvaranje filea bezuspjesno!");
	  }

           //azuriranje Grid Mape!!!
       GridMap_cell **Map=GM->GetMap();
	   //koordinate prepreka X,Y, koje su od tih prepreka staticke (1), a koje dinamicke (2), vremenski ciklus azuriranja prepreke
    if(((F=fopen(WH_LOG_GRIDMAP_X,"wt"))!=NULL)&&((F_temp=fopen(WH_LOG_GRIDMAP_Y,"wt"))!=NULL)&&((F2=fopen(WH_LOG_GRIDMAP_STATIC,"wt"))!=NULL)&&((F_temp2=fopen(WH_LOG_GRIDMAP_TIME_STAMP,"wt"))!=NULL)){
      for(int i=0;i<GM->GetMapSizeX();i++){
       for(int j=0;j<GM->GetMapSizeY();j++){
         //ako je celija zauzeta
         if(Map[i][j].occupancy>0){
               	fprintf(F,"%f ",Map[i][j].x);
                fprintf(F_temp,"%f ",Map[i][j].y);
                fprintf(F_temp2,"%d ",Map[i][j].time_stamp);
		            if(Map[i][j].static_cell){
		              fprintf(F2,"%d ",1);
		            }else{
		              fprintf(F2,"%d ",2);
		            }

         }
       }
		  }

       fprintf(F,"\n"); fprintf(F_temp,"\n"); fprintf(F2,"\n"); fprintf(F_temp2,"\n");

  fclose(F);  fclose(F_temp); fclose(F2); fclose(F_temp2);
	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }
	  //celije koje su na mjestu sudara robota s preprekom i cijela pokretna prepreka pomaknuta za pomak tezista na mjesto sudara po trajektoriji pokretne prepreke
    	  if(((F=fopen(WH_LOG_GRIDMAP_COLMOV_X,"wt"))!=NULL)&&((F_temp=fopen(WH_LOG_GRIDMAP_COLMOV_Y,"wt"))!=NULL) ){
			  for(int i=0;i<GM->num_col_points;i++){ 
				  fprintf(F,"%f ",GM->col_moving[i].x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2);
				  fprintf(F_temp,"%f ",GM->col_moving[i].y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2);
}
	  fprintf(F,"\n"); fprintf(F_temp,"\n");
	  fclose(F); fclose(F_temp);
}else{
	  printf("Otvaranje filea bezuspjesno!");
}
	  //stare celije (1 ciklus prije) na mjestu sudara robota s preprekom i pokretna prepreka pomaknuta za pomak tezista na mjesto sudara po trajektoriji pokretne prepreke
    	  if(((F=fopen(WH_LOG_GRIDMAP_COLMOV_OLD_X,"wt"))!=NULL)&&((F_temp=fopen(WH_LOG_GRIDMAP_COLMOV_OLD_Y,"wt"))!=NULL) ){
			  for(int i=0;i<GM->num_col_points_old;i++){ 
				  fprintf(F,"%f ",GM->col_moving_old[i].x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2);
				  fprintf(F_temp,"%f ",GM->col_moving_old[i].y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2);
			  }
			  fprintf(F,"\n");  fprintf(F_temp,"\n");
			  fclose(F);  fclose(F_temp);
		  }else{
			  printf("Otvaranje filea bezuspjesno!");
		  }
		  
		  //upis trenutnih moving prepreka od lasera (koristiti pri crtanju DW trajektorija buduci da su ove izbacene iz laser polja)
		  if(((F=fopen(WH_LOG_GRIDMAP_MOVING_X,"wt"))!=NULL)&&((F_temp=fopen(WH_LOG_GRIDMAP_MOVING_Y,"wt"))!=NULL) ){
			  for(int i=0;i<GM->numnewmovings;i++){ 
				  fprintf(F,"%f ",GM->moving[i].x);
				  fprintf(F_temp,"%f ",GM->moving[i].y);
			  }
			  fprintf(F,"\n"); fprintf(F_temp,"\n");
			  fclose(F); fclose(F_temp);
		  }else{
			  printf("Otvaranje filea bezuspjesno!");
		  }

		  //upis starih moving cellova
		  if(((F=fopen(WH_LOG_STARE_MOVING_X,"wt"))!=NULL)&&((F_temp=fopen(WH_LOG_STARE_MOVING_Y,"wt"))!=NULL) && ((F1=fopen(WH_LOG_NOVE_MOVING_X,"wt"))!=NULL)&&((F2=fopen(WH_LOG_NOVE_MOVING_Y,"wt"))!=NULL)){
			  for(int i=0;i<GM->numoldmovings;i++){ 
				  fprintf(F,"%f ",GM->stare_moving[i].x);
				  fprintf(F_temp,"%f ",GM->stare_moving[i].y);
			  }
			  fprintf(F,"\n"); fprintf(F_temp,"\n");
			  fclose(F); fclose(F_temp);
			  for(int i=0;i<GM->numnewmovingcells;i++){ 
				  fprintf(F1,"%f ",GM->moving_cell[i].x);
				  fprintf(F2,"%f ",GM->moving_cell[i].y);
			  }
			  fprintf(F1,"\n"); fprintf(F2,"\n");
			  fclose(F1); fclose(F2);
		  }else{
			  printf("Otvaranje filea bezuspjesno!");
		  }

          //Azuriranje DStar mape
		  DStarCell **map=DS->GetMap();

      //svi next pointeri DStara  - X,Y su koordinate polja, a DX,DY su pomaci iz skupa {-1,0,1} ovisno o smjeru next pointera iz polja s koordinatama X,Y
		  if(((F=fopen(WH_LOG_DSTAR_X,"wt"))!=NULL)&&((F_temp=fopen(WH_LOG_DSTAR_Y,"wt"))!=NULL)&&((F2=fopen(WH_LOG_DSTAR_DX,"wt"))!=NULL)&&((F_temp2=fopen(WH_LOG_DSTAR_DY,"wt"))!=NULL)&&((F3=fopen(WH_LOG_DSTAR_X_REVERSE,"wt"))!=NULL)&&((F4=fopen(WH_LOG_DSTAR_Y_REVERSE,"wt"))!=NULL)&&((F5=fopen(WH_LOG_DSTAR_DX_REVERSE,"wt"))!=NULL)&&((F6=fopen(WH_LOG_DSTAR_DY_REVERSE,"wt"))!=NULL)){

     for(int i=0;i<DS->GetMapSizeX();i++){
       for(int j=0;j<DS->GetMapSizeY();j++){
              if ( (map[i][j]._next.x!=-1) && (map[i][j]._next.y!=-1) )
              {

                fprintf(F,"%f ",i*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2);
                fprintf(F_temp,"%f ",j*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2);
                fprintf(F2,"%f ",(map[i][j]._next.x - i)*GM->Map_Cell_Size);
                fprintf(F_temp2,"%f ",(map[i][j]._next.y - j)*GM->Map_Cell_Size);
              }
	      if ( (map[i][j]._next_reverse.x!=-1) && (map[i][j]._next_reverse.y!=-1) )
	      {

		      fprintf(F3,"%f ",i*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2);
		      fprintf(F4,"%f ",j*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2);
		      fprintf(F5,"%f ",(map[i][j]._next_reverse.x - i)*GM->Map_Cell_Size);
		      fprintf(F6,"%f ",(map[i][j]._next_reverse.y - j)*GM->Map_Cell_Size);
	      }


        }
		  }
		  fprintf(F,"\n"); fprintf(F_temp,"\n"); fprintf(F2,"\n"); fprintf(F_temp2,"\n");
		  fprintf(F3,"\n"); fprintf(F4,"\n"); fprintf(F5,"\n"); fprintf(F6,"\n");
		  fclose(F);  fclose(F_temp); fclose(F2);  fclose(F_temp2);
		  fclose(F3);  fclose(F4); fclose(F5);  fclose(F6);

	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }

      //stanja na open listi i closed listi - X i Y koordinate
    if(((F=fopen(WH_LOG_DSTAR_OPEN_X,"wt"))!=NULL)&&((F_temp=fopen(WH_LOG_DSTAR_OPEN_Y,"wt"))!=NULL)&&((F2=fopen(WH_LOG_DSTAR_CLOSED_X,"wt"))!=NULL)&&((F_temp2=fopen(WH_LOG_DSTAR_CLOSED_Y,"wt"))!=NULL) && ((F3=fopen(WH_LOG_DSTAR_CSPACE_X,"wt"))!=NULL)&&((F4=fopen(WH_LOG_DSTAR_CSPACE_Y,"wt"))!=NULL)){

     for(int i=0;i<DS->GetMapSizeX();i++){
       for(int j=0;j<DS->GetMapSizeY();j++){

              // OPEN
              if(map[i][j].tag==1)
              {

                fprintf(F,"%f ",i*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2);
                fprintf(F_temp,"%f ",j*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2);
              }
			  else if (map[i][j].tag==-1)    //CLOSED
              {
                fprintf(F2,"%f ",i*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2);
                fprintf(F_temp2,"%f ",j*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2);
              }
              //cspace
              if (map[i][j].cspace_occupied)
              {
                fprintf(F3,"%f ",i*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2);
                fprintf(F4,"%f ",j*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2);              
              }
        }
		  }
		  fprintf(F,"\n"); fprintf(F_temp,"\n"); fprintf(F2,"\n"); fprintf(F_temp2,"\n");
		  fprintf(F3,"\n");fprintf(F4,"\n");
		  fclose(F); fclose(F_temp);fclose(F2); fclose(F_temp2);
		  fclose(F3);fclose(F4);

	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }
      //koordinate inicijalnog puta D* (redundantno s onim gore iz planera)
      if(((F=fopen(WH_LOG_DSTAR_INICIJALNI_X,"wt"))!=NULL)&&((F_temp=fopen(WH_LOG_DSTAR_INICIJALNI_Y,"wt"))!=NULL) &&((F2=fopen(WH_LOG_WIT_INICIJALNI_X,"wt"))!=NULL)&&((F3=fopen(WH_LOG_WIT_INICIJALNI_Y,"wt"))!=NULL)){

		for(int i=0;i<DS->PathLengthinicijalni;i++){
			fprintf(F,"%f ",DS->pathinicijalni[i].x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2);
			fprintf(F_temp,"%f ",DS->pathinicijalni[i].y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2);
		  }
		  for(int i=0;i<DS->PathLengthWitkowski;i++){
			  fprintf(F2,"%f ",DS->path_witkowski[i].x*GM->Map_Cell_Size+GM->Map_Home.x+GM->Map_Cell_Size/2);
			  fprintf(F3,"%f ",DS->path_witkowski[i].y*GM->Map_Cell_Size+GM->Map_Home.y+GM->Map_Cell_Size/2);
		  }

		  fprintf(F,"\n"); fprintf(F_temp,"\n");fprintf(F2,"\n");fprintf(F3,"\n");
		  fclose(F);  fclose(F_temp);fclose(F2);  fclose(F3);



	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }
      //koordinate trenutnog puta D* (redundantno s onim gore iz planera)
	  if(((F=fopen(WH_LOG_DSTAR_PATH_X,"wt"))!=NULL)&&((F_temp=fopen(WH_LOG_DSTAR_PATH_Y,"wt"))!=NULL)&&((F2=fopen(WH_LOG_DSTAR_PATH_TH,"wt"))!=NULL) ){
      R_point *temp_point; int temp_length;

      //D* path
      temp_length=PL->GetPathLength();
      temp_point=PL->GetPath();

		  for(int i=0;i<temp_length;i++){
			  fprintf(F,"%f ",temp_point[i].x);
			  fprintf(F_temp,"%f ",temp_point[i].y);
			  fprintf(F2,"%f ",temp_point[i].th);
		  }

		  fprintf(F,"\n"); fprintf(F2,"\n");fprintf(F_temp,"\n");
		  fclose(F);  fclose(F_temp); fclose(F2);


	  }else{
		  printf("Otvaranje filea bezuspjesno!");
	  }
      //indeksi specijalnih zauzetih polja od cspacea
	  if(((F=fopen(WH_LOG_DSTAR_OCC_I,"wt"))!=NULL)&&((F_temp=fopen(WH_LOG_DSTAR_OCC_J,"wt"))!=NULL) ){

		  for(int i=0;i<DS->Numindicesocc;i++){
			  fprintf(F,"%d ",DS->indicesocc[i].x);
			  fprintf(F_temp,"%d ",DS->indicesocc[i].y);
		  }

		  fprintf(F,"\n"); fprintf(F_temp,"\n");
		  fclose(F);  fclose(F_temp);


	  }else{
		  printf("Otvaranje filea bezuspjesno!");
	  }

//cost mapa, mapa s azuriranim vremenskim ciklusima dolaska cvora na OPEN listu, mapa s vrijednostima funkcija f_biased, mapa s f costom, mapa s h costom, mapa s g costom, mapa s k costom
if(((F=fopen(WH_LOG_DSTAR_COST_MAP,"wt"))!=NULL)&&((F9=fopen(WH_LOG_DSTAR_SKUPINE_MAP,"wt"))!=NULL)&&((F1=fopen(WH_LOG_DSTAR_TIME_STAMP,"wt"))!=NULL)&&((F_temp1=fopen(WH_LOG_DSTAR_F_BIASED_COST_MAP,"wt"))!=NULL)&&((F3=fopen(WH_LOG_DSTAR_F_COST_MAP,"wt"))!=NULL)&&((F_temp=fopen(WH_LOG_DSTAR_H_COST_MAP,"wt"))!=NULL)&&((F2=fopen(WH_LOG_DSTAR_G_COST_MAP,"wt"))!=NULL)&&((F_temp2=fopen(WH_LOG_DSTAR_K_COST_MAP,"wt"))!=NULL)&&((F6=fopen(WH_LOG_WIT_F_COST,"wt"))!=NULL)&&((F7=fopen(WH_LOG_WIT_G_FORWARD,"wt"))!=NULL)&&((F8=fopen(WH_LOG_WIT_G_BACKWARD,"wt"))!=NULL)&&((F4=fopen(WH_LOG_DSTAR_K_COST_MAP_REVERSE,"wt"))!=NULL)&&((F5=fopen(WH_LOG_DSTAR_TIME_STAMP_FORWARD,"wt"))!=NULL)) {

     for(int i=0;i<DS->GetMapSizeX();i++){
       for(int j=0;j<DS->GetMapSizeY();j++){
		   //cost mapa
              int cost=map[i][j].traversal_cost;
              if(cost==OBSTACLE)
				  cost=COST_MASK+EMPTYC+1+DEBEL; //da se ne upisuju ogromne vrijednosti
			  fprintf(F,"%d ",cost);
			  //time stamp
			  fprintf(F1,"%d ",map[i][j].time_stamp);
	      fprintf(F9,"%d ",map[i][j].skupine);
	      fprintf(F5,"%d ",map[i][j].time_stamp_forward);
			  //f_biased
			  int fbiasedcost=map[i][j].total_cost_int_biased;
			  if (fbiasedcost>=OBSTACLE)
				  fbiasedcost=-100; //da se ne upisuju ogromne vrijednosti
			  if ((map[i][j].tag==NEW) && (map[i][j].prepreka_bool==false))
				  fbiasedcost=0; //neodredjene vrijednosti
			  fprintf(F_temp1,"%d ",fbiasedcost);
			  //f
			  int fcost=map[i][j].total_cost_int;
			  if (fcost>=OBSTACLE)
				  fcost=-100;
			  if ((map[i][j].tag==NEW) && (map[i][j].prepreka_bool==false))
				  fcost=0;
			  fprintf(F3,"%d ",fcost);
			  //h
			  int hcost=map[i][j].h_cost_int;
			  if ((hcost>=OBSTACLE) || (map[i][j].prepreka_bool==true))
				  hcost=-100;
			  if ((map[i][j].tag==NEW) && (map[i][j].prepreka_bool==false))
				  hcost=0;
			  fprintf(F_temp,"%d ",hcost);
			  //g
			  int gcost=map[i][j].g_cost_int;
			  if(map[i][j].prepreka_bool==true)
				  gcost=-100;
			  if ((map[i][j].tag==NEW) && (map[i][j].prepreka_bool==false))
				  gcost=0;
			  fprintf(F2,"%d ",gcost);
			  //k
			  int kcost=map[i][j].k_cost_int;
			  if ((kcost>=OBSTACLE) || (map[i][j].prepreka_bool==true))
				  kcost=-100;
			  if ((map[i][j].tag==NEW) && (map[i][j].prepreka_bool==false))
				  kcost=0;
			  fprintf(F_temp2,"%d ",kcost);
			  kcost=map[i][j].k_cost_int_reverse;
			  if (map[i][j].prepreka_bool==true)
				  kcost=-100;
			  fprintf(F4,"%d ",kcost);
			  //witkowski
			  			  //f
			  fcost=map[i][j].cost_sum;
			  if (fcost>=OBSTACLE)
				  fcost=-100;
			  if (((map[i][j].tag_forward==NEW) || (map[i][j].tag_backward==NEW)) &&(map[i][j].prepreka_bool==false))
				  fcost=0;
			  fprintf(F6,"%d ",fcost);
			  			  //g
			  gcost=map[i][j].cost_backward;
			  if (gcost>=OBSTACLE)
				  gcost=-100;
			  if (((map[i][j].tag_backward==NEW)) &&(map[i][j].prepreka_bool==false))
				  gcost=0;
			  fprintf(F8,"%d ",gcost);
			  			  //g
			  gcost=map[i][j].cost_forward;
			  if (gcost>=OBSTACLE)
				  gcost=-100;
			  if (((map[i][j].tag_forward==NEW)) &&(map[i][j].prepreka_bool==false))
				  gcost=0;
			  fprintf(F7,"%d ",gcost);
        }
        fprintf(F,"\n"); fprintf(F9,"\n");fprintf(F1,"\n");fprintf(F5,"\n");  fprintf(F_temp1,"\n");fprintf(F3,"\n");fprintf(F_temp,"\n");fprintf(F2,"\n");fprintf(F_temp2,"\n");
		fprintf(F6,"\n");fprintf(F7,"\n");fprintf(F8,"\n");fprintf(F4,"\n");
		  }
		  fprintf(F,"\n");fprintf(F9,"\n"); fprintf(F1,"\n");fprintf(F4,"\n");fprintf(F5,"\n"); fprintf(F_temp1,"\n");fprintf(F3,"\n");fprintf(F_temp,"\n");fprintf(F2,"\n");fprintf(F_temp2,"\n");
		  fclose(F); fclose(F9); fclose(F1); fclose(F_temp1); fclose(F3);	fclose(F_temp);  fclose(F2);  fclose(F_temp2);
		  fclose(F6);fclose(F7);fclose(F8);fclose(F4);fclose(F5);
	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }
//izracuni vremena, f cost od robota, f_biased cost od robota, h_cost od robota (jednak f jer je f=g+h, a g je 0 za robota (koji je kretajuci start))
	  if(((F_temp1=fopen(WH_LOG_DSTAR_AZURIRANI,op))!=NULL)&&((F=fopen(WH_LOG_DSTAR_IZRACUNI,op))!=NULL)&&((F1=fopen(WH_LOG_DSTAR_F_COST,op))!=NULL)&&((F2=fopen(WH_LOG_DSTAR_F_BIASED_COST,op))!=NULL)&&((F3=fopen(WH_LOG_DSTAR_H_COST,op))!=NULL)&&((F4=fopen(WH_LOG_DSTAR_BROJ_ITERACIJA,op))!=NULL)&&((F5=fopen(WH_LOG_DSTAR_BROJ_CVOROVA_NA_LISTI,op))!=NULL)&&((F6=fopen(WH_LOG_DSTAR_MAX_BROJ_CVOROVA_NA_LISTI,op))!=NULL)&&((F7=fopen(WH_LOG_DSTAR_CIJENA_PUTA,op))!=NULL)&&((F8=fopen(WH_LOG_DSTAR_DULJINA_PUTA,op))!=NULL)&&((F_temp=fopen(WH_LOG_DSTAR_DULJINA_PUTA_UM,op))!=NULL)&&((F_temp2=fopen(WH_LOG_WIT_DULJINA_PUTA_UM,op))!=NULL)) {
	      int granica;
/*	      if (DS->brojac_izracuna < DS->GetMapSizeX()*DS->GetMapSizeY()) {
		      granica=DS->brojac_izracuna;
	      } else {
		      granica=DS->GetMapSizeX()*DS->GetMapSizeY();
	      }*/
	      granica=logsizes;
	      for(int i=0;i<granica;i++){
			  fprintf(F_temp1,"%d ",DS->azurirani_polje[i]);
		 fprintf(F,"%d ",DS->izracuni[i]); 
		 fprintf(F1,"%d ",DS->f_putanje[i]); 
		 fprintf(F2,"%d ",DS->f_biased_putanje[i]); 
		 fprintf(F3,"%d ",DS->h_putanje[i]);
		 fprintf(F4,"%d ",DS->broj_iteracija[i]); 
		 fprintf(F5,"%d ",DS->broj_cvorova_na_listi[i]); 
		 fprintf(F6,"%d ",DS->max_broj_cvorova_na_listi[i]); 
		 fprintf(F7,"%d ",DS->cijena_puta[i]); 
		 fprintf(F8,"%d ",DS->duljina_puta[i]); 
		 fprintf(F_temp,"%f ",DS->duljina_puta_um[i]);
		 fprintf(F_temp2,"%f ",DS->duljina_wit_puta_um[i]);
		  }
//		  fprintf(F_temp1,"\n"); fprintf(F_temp,"\n"); fprintf(F_temp2,"\n"); fprintf(F,"\n");  fprintf(F1,"\n");   fprintf(F2,"\n"); fprintf(F3,"\n");  fprintf(F4,"\n");   fprintf(F5,"\n"); fprintf(F6,"\n"); fprintf(F7,"\n");fprintf(F8,"\n");
		  fclose(F_temp1);  fclose(F);  fclose(F1);  fclose(F2);  fclose(F3);  fclose(F4); fclose(F5); fclose(F6); fclose(F7); fclose(F8);fclose(F_temp);fclose(F_temp2);


	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }
	  //witkowski algoritam
	  if(((F_temp1=fopen(WH_LOG_WIT_AZURIRANI_BACKWARD,op))!=NULL)&&((F=fopen(WH_LOG_WIT_AZURIRANI_FORWARD,op))!=NULL)&&((F1=fopen(WH_LOG_WIT_IZRACUNIWITKOWSKI,op))!=NULL)&&((F9=fopen(WIT_PUT_I_SKUPINE_VREMENA,op))!=NULL)&&((F2=fopen(WH_LOG_WIT_BROJ_ITERACIJAWITKOWSKI,op))!=NULL)&&((F3=fopen(WH_LOG_WIT_BROJ_CVOROVA_NA_LISTI_FORWARD,op))!=NULL)&&((F4=fopen(WH_LOG_WIT_MAX_BROJ_CVOROVA_NA_LISTI_FORWARD,op))!=NULL)&&((F5=fopen(WH_LOG_WIT_CIJENA_PUTA_FORWARD,op))!=NULL)&&((F6=fopen(WH_LOG_WIT_CIJENA_PUTA_BACKWARD,op))!=NULL)&&((F7=fopen(WH_LOG_WIT_DULJINA_PUTA_FORWARD,op))!=NULL)&&((F8=fopen(WH_LOG_WIT_BROJ_CVOROVA_NA_LISTI_BACKWARD,op))!=NULL)&&((F_temp=fopen(WH_LOG_WIT_MAX_BROJ_CVOROVA_NA_LISTI_BACKWARD,op))!=NULL)&&((F_temp2=fopen(WH_LOG_WIT_DULJINA_PUTA_BACKWARD,op))!=NULL)) {
		  int granica;
		  /*if (DS->brojac_izracunaWitkowski < DS->GetMapSizeX()*DS->GetMapSizeY()) {
			  granica=DS->brojac_izracunaWitkowski;
		  } else {
			  granica=DS->GetMapSizeX()*DS->GetMapSizeY();
		  }*/
		  granica=logsizes;
		  for(int i=0;i<granica;i++){
			  fprintf(F_temp1,"%d ",DS->azurirani_backward_polje[i]); 
			  fprintf(F,"%d ",DS->azurirani_forward_polje[i]); 
			  fprintf(F1,"%d ",DS->izracuniWitkowski[i]); 
			  fprintf(F9,"%d ",DS->vremena_skupine[i]);
			  fprintf(F2,"%d ",DS->broj_iteracijaWitkowski[i]);
			  fprintf(F3,"%d ",DS->broj_cvorova_na_listi_forward[i]); 
			  fprintf(F4,"%d ",DS->max_broj_cvorova_na_listi_forward[i]); 
			  fprintf(F5,"%d ",DS->cijena_puta_forward[i]); 
			  fprintf(F6,"%d ",DS->cijena_puta_backward[i]); 
			  fprintf(F7,"%d ",DS->duljina_puta_forward[i]); 
			  fprintf(F8,"%d ",DS->broj_cvorova_na_listi_backward[i]); 
			  fprintf(F_temp,"%d ",DS->max_broj_cvorova_na_listi_backward[i]); 
			  fprintf(F_temp2,"%d ",DS->duljina_puta_backward[i]);
		  }
//		  fprintf(F_temp1,"\n"); fprintf(F,"\n");  fprintf(F1,"\n"); fprintf(F9,"\n");   fprintf(F2,"\n"); fprintf(F3,"\n");  fprintf(F4,"\n");   fprintf(F5,"\n"); fprintf(F6,"\n"); fprintf(F7,"\n");fprintf(F8,"\n"); fprintf(F_temp,"\n");fprintf(F_temp2,"\n");
		  fclose(F_temp1);  fclose(F);  fclose(F1);  fclose(F9); fclose(F2);  fclose(F3);  fclose(F4); fclose(F5); fclose(F6); fclose(F7); fclose(F8); fclose(F_temp);fclose(F_temp2);


	  }else{
		  printf("Otvaranje filea bezuspjesno!");
	  }

planner_path_original.size_path=0;
planner_path_run.size_path=0;
planner_path_run.broj=0;
Log_robot_path.size_laser=0;
Log_robot_path.size_laser2=0;
Log_robot_path.size_teziste=0;
Log_robot_path.size_dwopttraj=0;



}

