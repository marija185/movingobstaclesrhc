
#ifndef WORK_HORSE_H
#define WORK_HORSE_H
//#include <libplayerc++/playerc++.h> //novi player
//#include <libplayerinterface/player.h>  //novi player
// #include <libplayercore/player.h>  //novi player
//#include <player.h>    //za definove od lasera
//#include <playerclient.h>  // for player client stuff (time)
#include "Planner.h" //za R_point
#include "Params.h" //za includove malloc stdio...
#include <sys/time.h>
//vremena izracuna cijelog algoritma i  vremena svakog kraja algoritma mjerena od samog pocetka stanja RUNNINGu
#define WH_VREMENA			"logger//vremena_cijelog_algoritma.dat"  
#define WIT_PUT_I_SKUPINE_VREMENA	"logger//wit_put_i_skupine_vremena.dat"
#define DW_VREMENA			"logger//vremena_dw_algoritma.dat"
#define DS_VREMENA			"logger//vremena_ds_algoritma.dat"
#define DS_EXPLORED			"logger//explored_ds_algoritma.dat"
#define WH_LOG_TIME			"logger//wh_log_time.dat"
#define WH_LOG_READ_TIME			"logger//wh_log_read_time.dat"
//efektivna duljina
#define EF_PATH_LENGTH			"logger//ef_path_length.dat"
#define EF_PATH_ORIENTATION		"logger//ef_path_orientation.dat"
#define TOCKA_INFLEKSIJE_X		"logger//tocka_infleksije_x.dat"
#define TOCKA_INFLEKSIJE_Y		"logger//tocka_infleksije_y.dat"
//ljapunov
#define LJAPUNOV_PUTANJA		"logger//ljapunov_putanja.dat"
#define INTCOST_PUTANJA		"logger//intcost_putanja.dat"
#define REPLAN_PUTANJA		"logger//replan_putanja.dat"
#define TRAVCOST_PUTANJA		"logger//travcost_putanja.dat"
#define TRAVCOST_PUTANJA3D		"logger//travcost_putanja3d.dat"
 //azuriranje pocetne globalne putanje za planner (geometrijski put)
#define WH_GLOBAL_PLANNER_PATH_X			"logger//global_planner_path_x.dat"
#define WH_GLOBAL_PLANNER_PATH_Y			"logger//global_planner_path_y.dat"
#define WH_GLOBAL_PLANNER_PATH_TH			"logger//global_planner_path_th.dat"
#define WH_GLOBAL_PLANNER_PATH_TRAVCOST			"logger//global_planner_path_travcost.dat"
#define WH_GLOBAL_PLANNER_PATH_H			"logger//global_planner_path_h.dat"
#define WH_GLOBAL_PLANNER_PATH_INTLO			"logger//global_planner_path_intlo.dat"
#define WH_GLOBAL_PLANNER_PATH_INTUP			"logger//global_planner_path_intup.dat"
//azuriranje trenutne globalne putanje za planner (geometrijski put)
#define WH_GLOBAL_PLANNER_PATH_CURRENT_X			"logger//global_planner_path_current_x.dat"
#define WH_GLOBAL_PLANNER_PATH_CURRENT_Y			"logger//global_planner_path_current_y.dat"
#define WH_GLOBAL_PLANNER_PATH_CURRENT_TH			"logger//global_planner_path_current_th.dat"
#define WH_GLOBAL_PLANNER_PATH_CURRENT_TRAVCOST			"logger//global_planner_path_current_travcost.dat"
#define WH_GLOBAL_PLANNER_PATH_CURRENT_H			"logger//global_planner_path_current_h.dat"
#define WH_GLOBAL_PLANNER_PATH_CURRENT_INTLO			"logger//global_planner_path_current_intlo.dat"
#define WH_GLOBAL_PLANNER_PATH_CURRENT_INTUP			"logger//global_planner_path_current_intup.dat"
#define WH_LOG_SVIPATHPOINTERI                              "logger//wh_svipathpointeri.dat"
//logger za prosjecnu brzinu gibanja i vrijeme gibanja
#define WH_TIME_SPEED 		"logger//time_speed.dat"
//X,Y,TH,V,W,V_REF,W_REF- koordinata GLOBALNE glatke PUTANJE
#define WH_ROBOT_GLOBALNA_PUTANJA_X			"logger//robot_globalna_putanja_x.dat"
#define WH_ROBOT_GLOBALNA_PUTANJA_Y			"logger//robot_globalna_putanja_y.dat"
#define WH_ROBOT_GLOBALNA_PUTANJA_TH			"logger//robot_globalna_putanja_th.dat"
#define FAKELOCWH_ROBOT_GLOBALNA_PUTANJA_X			"logger//fakelocrobot_globalna_putanja_x.dat"
#define FAKELOCWH_ROBOT_GLOBALNA_PUTANJA_Y			"logger//fakelocrobot_globalna_putanja_y.dat"
#define FAKELOCWH_ROBOT_GLOBALNA_PUTANJA_TH			"logger//fakelocrobot_globalna_putanja_th.dat"
#define WH_ROBOT_TRANS_VEL			"logger//robot_trans_vel.dat"
#define WH_ROBOT_TRANS_VELY			"logger//robot_trans_vely.dat"
#define WH_ROBOT_ROT_VEL			"logger//robot_rot_vel.dat"
#define WH_ROBOT_TRANS_VEL_REF			"logger//robot_trans_vel_ref.dat"
#define WH_ROBOT_TRANS_VELY_REF			"logger//robot_trans_vely_ref.dat"
#define WH_ROBOT_ROT_VEL_REF			"logger//robot_rot_vel_ref.dat"
#define WH_ROBOT_TRANS_VEL_REFDIN			"logger//robot_trans_vel_refdin.dat"
#define WH_ROBOT_ROT_VEL_REFDIN			"logger//robot_rot_vel_refdin.dat"
#define WH_ROBOT_TRANS_VEL_KAL			"logger//robot_trans_vel_kal.dat"
#define WH_ROBOT_ROT_VEL_KAL			"logger//robot_rot_vel_kal.dat"
#define WH_ROBOT_TRANS_VEL_DES			"logger//robot_trans_vel_des.dat"
#define WH_ROBOT_ROT_VEL_DES			"logger//robot_rot_vel_des.dat"
//ovo je size loggera
#define VARIOUS_LOG_SIZES 		"logger//logger_sizes.dat"
// globalne koordinate x i y lasera iz svih ciklusa, laser pointeri su broj hitova u svakom ciklusu, zbrajanjem znace indekse tih velikih polja x i y koordinata koji oznacavaju prvi hit u ciklusu
#define WH_LOG_SVILASERI_X   "logger//wh_svilaseri_x.dat"   //tu su svi x laseri u cijeloj voznji
#define WH_LOG_SVILASERI_Y   "logger//wh_svilaseri_y.dat"   //tu su svi y laseri u cijeloj voznji
#define WH_LOG_SVILASERPOINTERI    "logger//wh_svilaserpointeri.dat"   //tu broj hitova tijekom cijele voznje
//laser2
#define WH_LOG_SVILASERI2_X   "logger//wh_svilaseri2_x.dat"   //tu su svi x laseri u cijeloj voznji
#define WH_LOG_SVILASERI2_Y   "logger//wh_svilaseri2_y.dat"   //tu su svi y laseri u cijeloj voznji
#define WH_LOG_SVILASERPOINTERI2    "logger//wh_svilaserpointeri2.dat"   //tu broj hitova tijekom cijele voznje
//globalne koordinate x i y tezista laserskih ocitanja iz svih ciklusa, teziste pointeri su broj tezista u svakom ciklusu, zbrajanjem znace indekse tih polja x i y koordinata tezista koji znace prvo teziste u ciklusu
#define WH_LOG_TEZISTE_X   "logger//wh_teziste_x.dat"   //tu su tezista u cijeloj voznji
#define WH_LOG_TEZISTE_Y   "logger//wh_teziste_y.dat"   //tu su tezista u cijeloj voznji
#define WH_LOG_TEZISTEPOINTERI    "logger//wh_tezistepointeri.dat"   //tu je broj tezista tijekom cijele voznje
//azuriranje trenutne opt traj iz dw
#define DW_OPT_TRAJ_X					"logger//dw_opt_traj_x.dat"
#define DW_OPT_TRAJ_Y					"logger//dw_opt_traj_y.dat"
#define DW_LOG_POINTERI                          "logger//dw_pointeri.dat"
 //pocetna pozicija robota i cilj
#define WH_LOG_START_POSITION			"logger//wh_start_position.dat"
#define WH_LOG_GOAL_POSITION			"logger//wh_goal_position.dat"
 //azuriranje Grid Mape
 //koordinate prepreka X,Y, koje su od tih prepreka staticke (1), a koje dinamicke (2), vremenski ciklus azuriranja prepreke
#define WH_LOG_GRIDMAP_X			"logger//wh_gridmap_x.dat"
#define WH_LOG_GRIDMAP_Y			"logger//wh_gridmap_y.dat"
#define WH_LOG_GRIDMAP_STATIC			"logger//wh_gridmap_static_cell.dat"    //koje su od tih prepreka staticke (1), a koje dinamicke (2)
#define WH_LOG_GRIDMAP_TIME_STAMP   "logger//wh_gridmap_time_stamp.dat"     //vremenski ciklus azuriranja prepreke
//celije koje su na mjestu sudara robota s preprekom i cijela pokretna prepreka pomaknuta za pomak tezista na mjesto sudara po trajektoriji pokretne prepreke
#define WH_LOG_GRIDMAP_COLMOV_X "logger//wh_gridmap_colmov_x.dat"   //tu su celije sudara s pokretnom preprekom i cijela pokretna translatirana
#define WH_LOG_GRIDMAP_COLMOV_Y "logger//wh_gridmap_colmov_y.dat"   //tu su celije sudara s pokretnom preprekom i cijela pokretna translatirana
//stare celije (1 ciklus prije) na mjestu sudara robota s preprekom i pokretna prepreka pomaknuta za pomak tezista na mjesto sudara po trajektoriji pokretne prepreke
#define WH_LOG_GRIDMAP_COLMOV_OLD_X "logger//wh_gridmap_colmov_old_x.dat"   //tu su stare celije sudara s pokretnom preprekom i...
#define WH_LOG_GRIDMAP_COLMOV_OLD_Y "logger//wh_gridmap_colmov_old_y.dat"   //tu su stare celije sudara s pokretnom preprekom i ...
//upis trenutnih moving prepreka od lasera (koristiti pri crtanju DW trajektorija buduci da su ove izbacene iz laser polja)
#define WH_LOG_GRIDMAP_MOVING_X  "logger//wh_gridmap_moving_x.dat"
#define WH_LOG_GRIDMAP_MOVING_Y  "logger//wh_gridmap_moving_y.dat"
//stare moving
#define WH_LOG_STARE_MOVING_X  "logger//wh_gridmap_stare_moving_x.dat"
#define WH_LOG_STARE_MOVING_Y  "logger//wh_gridmap_stare_moving_y.dat"
//nove moving
#define WH_LOG_NOVE_MOVING_X  "logger//wh_gridmap_nove_moving_x.dat"
#define WH_LOG_NOVE_MOVING_Y  "logger//wh_gridmap_nove_moving_y.dat"
//Azuriranje DStar mape
 //svi next pointeri DStara  - X,Y su koordinate polja, a DX,DY su pomaci iz skupa {-1,0,1} ovisno o smjeru next pointera iz polja s koordinatama X,Y
#define WH_LOG_DSTAR_X			"logger//wh_dstar_x.dat"
#define WH_LOG_DSTAR_Y			"logger//wh_dstar_y.dat"
#define WH_LOG_DSTAR_DX			"logger//wh_dstar_dx.dat"
#define WH_LOG_DSTAR_DY			"logger//wh_dstar_dy.dat"
#define WH_LOG_DSTAR_X_REVERSE			"logger//wh_dstar_x_reverse.dat"
#define WH_LOG_DSTAR_Y_REVERSE			"logger//wh_dstar_y_reverse.dat"
#define WH_LOG_DSTAR_DX_REVERSE			"logger//wh_dstar_dx_reverse.dat"
#define WH_LOG_DSTAR_DY_REVERSE			"logger//wh_dstar_dy_reverse.dat"
 //stanja na open listi i closed listi - X i Y koordinate
#define WH_LOG_DSTAR_OPEN_X			"logger//wh_dstar_open_x.dat"
#define WH_LOG_DSTAR_OPEN_Y			"logger//wh_dstar_open_y.dat"
#define WH_LOG_DSTAR_CLOSED_X			"logger//wh_dstar_closed_x.dat"
#define WH_LOG_DSTAR_CLOSED_Y			"logger//wh_dstar_closed_y.dat"
//occupied for cspace
#define WH_LOG_DSTAR_CSPACE_X			"logger//wh_dstar_cspace_x.dat"
#define WH_LOG_DSTAR_CSPACE_Y			"logger//wh_dstar_cspace_y.dat"

 //koordinate inicijalnog puta D*  (redundantno s onim gore iz planera)
#define WH_LOG_DSTAR_INICIJALNI_X			"logger//wh_dstar_inicijalni_x.dat"
#define WH_LOG_DSTAR_INICIJALNI_Y			"logger//wh_dstar_inicijalni_y.dat"
 //koordinate trenutnog puta D*  (redundantno s onim gore iz planera)
#define WH_LOG_DSTAR_PATH_X			"logger//wh_dstar_path_x.dat"
#define WH_LOG_DSTAR_PATH_Y			"logger//wh_dstar_path_y.dat"
#define WH_LOG_DSTAR_PATH_TH			"logger//wh_dstar_path_th.dat"
//indeksi specijalnih zauzetih polja od cspacea
#define WH_LOG_DSTAR_OCC_I			"logger//wh_dstar_occ_i.dat"
#define WH_LOG_DSTAR_OCC_J			"logger//wh_dstar_occ_j.dat"
 //koordinate preslika na putu DD*
#define WH_LOG_DSTAR_PRESLIK_X			"logger//wh_dstar_preslik_x.dat"
#define WH_LOG_DSTAR_PRESLIK_Y			"logger//wh_dstar_preslik_y.dat"
 //cost mapa, mapa s f costom, mapa s h costom, mapa s g costom, mapa s k costom
#define WH_LOG_DSTAR_COST_MAP			"logger//wh_dstar_cost_map.dat"
//skupine cvorova izmedju S tocki
#define WH_LOG_DSTAR_SKUPINE_MAP			"logger//wh_dstar_skupine_map.dat"
//time_stampovi - mapa s azuriranim vremenskim ciklusima dolaska cvora na OPEN listu
#define WH_LOG_DSTAR_TIME_STAMP     "logger//wh_dstar_time_stamp.dat"
#define WH_LOG_DSTAR_TIME_STAMP_FORWARD     "logger//wh_dstar_time_stamp_forward.dat"
//mapa s vrijednostima funkcija f_biased, f, h, g, k
#define WH_LOG_DSTAR_F_BIASED_COST_MAP			"logger//wh_dstar_f_biased_cost_map.dat"
#define WH_LOG_DSTAR_F_COST_MAP			"logger//wh_dstar_f_cost_map.dat"
#define WH_LOG_DSTAR_H_COST_MAP			"logger//wh_dstar_h_cost_map.dat"
#define WH_LOG_DSTAR_G_COST_MAP			"logger//wh_dstar_g_cost_map.dat"
#define WH_LOG_DSTAR_K_COST_MAP			"logger//wh_dstar_k_cost_map.dat"
#define WH_LOG_DSTAR_K_COST_MAP_REVERSE			"logger//wh_dstar_k_cost_map_reverse.dat"
//izracuni vremena trajanja D* algoritma, f cost od robota, f_biased cost od robota, h_cost od robota (jednak f jer je f=g+h, a g je 0 za robota (koji je kretajuci start))
#define WH_LOG_DSTAR_AZURIRANI			"logger//wh_dstar_azurirani.dat"
#define WH_LOG_DSTAR_IZRACUNI			"logger//wh_dstar_izracuni.dat"
#define WH_LOG_DSTAR_F_COST			"logger//wh_dstar_f_cost.dat"      //f_cost i h_cost na putu
#define WH_LOG_DSTAR_F_BIASED_COST			"logger//wh_dstar_f_biased_cost.dat"   //f_cost i h_cost na putu
#define WH_LOG_DSTAR_H_COST			"logger//wh_dstar_h_cost.dat"    //f_cost i h_cost na putu
#define WH_LOG_DSTAR_BROJ_ITERACIJA			"logger//wh_dstar_broj_iteracija.dat"
#define WH_LOG_DSTAR_BROJ_CVOROVA_NA_LISTI			"logger//wh_dstar_broj_cvorova_na_listi.dat"
#define WH_LOG_DSTAR_MAX_BROJ_CVOROVA_NA_LISTI			"logger//wh_dstar_max_broj_cvorova_na_listi.dat"
#define WH_LOG_DSTAR_CIJENA_PUTA			"logger//wh_dstar_cijena_puta.dat"
#define WH_LOG_DSTAR_DULJINA_PUTA			"logger//wh_dstar_duljina_puta.dat"
#define WH_LOG_DSTAR_DULJINA_PUTA_UM			"logger//wh_dstar_duljina_puta_um.dat"
//Witkowski
#define WH_LOG_WIT_IZRACUNIWITKOWSKI			"logger//wh_wit_izracuniWitkowski.dat"
#define WH_LOG_WIT_AZURIRANI_FORWARD			"logger//wh_wit_azurirani_forward.dat"
#define WH_LOG_WIT_AZURIRANI_BACKWARD			"logger//wh_wit_azurirani_backward.dat"
#define WH_LOG_WIT_BROJ_ITERACIJAWITKOWSKI			"logger//wh_wit_broj_iteracijaWitkowski.dat"
#define WH_LOG_WIT_BROJ_CVOROVA_NA_LISTI_FORWARD			"logger//wh_wit_broj_cvorova_na_listi_forward.dat"
#define WH_LOG_WIT_MAX_BROJ_CVOROVA_NA_LISTI_FORWARD			"logger//wh_wit_max_broj_cvorova_na_listi_forward.dat"
#define WH_LOG_WIT_CIJENA_PUTA_FORWARD			"logger//wh_wit_cijena_puta_forward.dat"
#define WH_LOG_WIT_DULJINA_PUTA_FORWARD			"logger//wh_wit_duljina_puta_forward.dat"
#define WH_LOG_WIT_BROJ_CVOROVA_NA_LISTI_BACKWARD			"logger//wh_wit_broj_cvorova_na_listi_backward.dat"
#define WH_LOG_WIT_MAX_BROJ_CVOROVA_NA_LISTI_BACKWARD			"logger//wh_wit_max_broj_cvorova_na_listi_backward.dat"
#define WH_LOG_WIT_CIJENA_PUTA_BACKWARD			"logger//wh_wit_cijena_puta_backward.dat"
#define WH_LOG_WIT_DULJINA_PUTA_BACKWARD			"logger//wh_wit_duljina_puta_backward.dat"
#define WH_LOG_WIT_DULJINA_PUTA_UM			"logger//wh_wit_duljina_puta_um.dat"
#define WH_LOG_WIT_F_COST			"logger//wh_wit_f_cost.dat"
#define WH_LOG_WIT_G_FORWARD			"logger//wh_wit_g_forward.dat"
#define WH_LOG_WIT_G_BACKWARD			"logger//wh_wit_g_backward.dat"
#define WH_LOG_WIT_INICIJALNI_X			"logger//wh_wit_inicijalni_x.dat"
#define WH_LOG_WIT_INICIJALNI_Y			"logger//wh_wit_inicijalni_y.dat"


//stanja procesa
#define INIT		11
#define RUNNING		12
#define DUMMY		13
#define HALT		14
#define COMPASS_INIT		15
#define RESUMING		16
#define REINIT		17
#define GOAL_POSITION		18
#define NO_PATH		19

enum{
    robot_path_size,
    size_logger_num
};

struct MB_RB{

	// Polozaj, orijentacija
	double x, y, th;

 	// Brzine
	double v, w, vy;
};

struct R_pol{
	double  r, th;
};

typedef struct{
	double *izracuni;   //vremensko trajanje cijelog algoritma u RUNNING stanju
	double *DWizracuni;   //vremensko trajanje DW algoritma u RUNNING stanju
	double *DSizracuni;   //vremensko trajanje DS algoritma u RUNNING stanju
	double *DSexplored;
	double *path_x, *path_y,*path_th;  //pozicija robota tijekom voznje
	double *laser_x, *laser_y;  //laserska ocitanja tijekom voznje, ova polja se drukcije realociraju od ostalih
	double *laser2_x, *laser2_y;  //laserska ocitanja tijekom voznje, ova polja se drukcije realociraju od ostalih
	double *teziste_x, *teziste_y; //tezista grupiranih laserskih ocitanja tijekom voznje, ova polja se drukcije realociraju od ostalih
	double *dwopttraj_x, *dwopttraj_y; //dw opt traj during the motion
	double *broj_hitova, *broj_tezista, *broj_hitova2, *broj_dwopttraj;   //broj laserskih hitova po ciklusu i broj tezista po ciklusu tijekom voznje
	double *v, *w, *vy;  //brzine robota tijekom voznje (ocitane)
	double *v_kal, *w_kal;  //brzine robota tijekom voznje (ocitane pa kalibrirane)
	double *v_ref, *w_ref, *vy_ref; //postavne brzine robota tijekom voznje (racuna DW, dinamika, kalibracija)
	double *v_refdin, *w_refdin; //postavne brzine robota tijekom voznje (racuna DW, dinamika, prije kalibracije)
	double *v_des, *w_des; //desired (zeljene) brzine robota (racuna DW)
	double *time; //vremenski trenutak zapisivanja svih gornjih podataka u polja mjereno od samog pocetka RUNNING stanja
	double *read_time; //vremenski trenutak ocitavanja brzina mjereno od samog pocetka RUNNING stanja
	double *path_length;//od DW-a
	double *path_orientation;//od DW-a
	double *tocka_infleksije_x, *tocka_infleksije_y;//od DW-a
	double *preslik_x, *preslik_y;//od DS-a
	double *ljapunov;//od DW-a
	double *interpolatedcost;
	double *replan;
	double *travcost;//od DS-a
	double *travcost3d;//od DS-a
	int size_path; //trenutna velicina polja, ako je broj ciklusa dostigao ovu velicinu onda se mora realocirati polje
	int max_path_run; //trenutna velicina polja run putanje
	int size_laser; //trenutni broj laser hitova u poljima laser_x i laser_y
	int max_laser;//trenutna velicina laser_x i laser_y polja, ako je size_laser dostigao ovu velicinu onda se moraju realocirati polja laser_x i laser_y
	int size_laser2; //trenutni broj laser hitova u poljima laser_x i laser_y
	int max_laser2;//trenutna velicina laser_x i laser_y polja, ako je size_laser dostigao ovu velicinu onda se moraju realocirati polja laser_x i laser_y
	int size_teziste; //trenutni broj tezista u poljima teziste_x i teziste_y
	int max_teziste;  //trenutna velicina teziste_x i teziste_y polja, ako je size_teziste dostigao ovu velicinu onda se moraju realocirati polja teziste_x i teziste_y
	int size_dwopttraj; //trenutni broj opt traj dw
	int max_dwopttraj; 
}WH_Logger;

typedef struct{
 double *path_x, *path_y, *path_th;
 double *path_travcost, *path_h, *path_intlo, *path_intup; 
 int size_path;
 int broj;
 double *size;//double samo zbog gotovih realoc funkcija
}WH_Logger_path;      //ovo je za trenutnu i inicijalnu planer putanju

//podrska za laser
typedef struct{
	R_point LB_pose[PLAYER_LASER_MAX_SAMPLES];//ovo su svi
	R_point LB_pose_manji[PLAYER_LASER_MAX_SAMPLES];//ovo su unutar 3 m
// double sviscanovi[36];//36*5=180, po 5 stupnjeva
	R_pol LB_scan[PLAYER_LASER_MAX_SAMPLES];//ovo su unutar 3 m
	R_pol sviscanovi[PLAYER_LASER_MAX_SAMPLES];//ovo su svi
	R_point teziste[PLAYER_LASER_MAX_SAMPLES];

	int broj_tezista;
	//bool LB_new[180];
  //int laser_pointer_start;     
  int laser_pointer_new;      //broj novih mjerenja u jednoj iteraciji
                              //maksimalni broj je 180
  int laser_manji_new;
}MB_Laser;

class WorkHorse {
public:
  MB_RB RB;
  MB_Laser LB;              //strukura sa tekucim podacima lasera
  MB_Laser LB2;              //strukura sa tekucim podacima lasera
  double laserx, lasery, laserth, laser2x, laser2y, laser2th; //laser origin
  double v_ref_current, w_ref_current;  //trenutne referentne brzine koje racuna DW
  double v_des_current, w_des_current;  //trenutne referentne brzine koje racuna DW
  double v_refdin_current, w_refdin_current;  //trenutne referentne brzine koje racuna DW, dinamika, prije kalibracije
  double vy_ref_current,vy_refdin_current,vy_des_current,delta_vy_refdin;//omnidrive
  double v_kal_current, w_kal_current;  //trenutne ocitane brzine prije kalibracije
  double delta_v_refdin,delta_w_refdin;//koju akceleraciju je zapravo htio primijeniti u k+1-vom ciklusu
  double dwpath_length;//tu cu trpati path_length iz DW-a
  double interpolatedcost;
  int vremenska_razlika;
  char *wld;
  int vremenska_razlika0;
  int ciklus_prije,ciklus_sad;    //razlika ce biti duljina trajanja prethodnog ciklusa, to koristi DW za racunanje brzine pokretne prepreke
  bool initial;   //zastavica za racunanje inicijalnog puta
  int cycle_number; //brojac ciklusa u RUNNING stanju
  int mySecStart0,myMSecStart0;
  long logger_current_size;  //alocirana (realocirana) duljina polja logera
  WH_Logger Log_robot_path;
  WH_Logger_path planner_path_original;  //planner_path_original
  WH_Logger_path planner_path_run;   //planner_path_run
  int logger_sizes[size_logger_num];  //sizes raznih polja   (tu je size_logger_num uvijek 1)
  int Logger_Init();  //pocetna alokacija polja
  int Logger_Realloc(); //realokacija
  void LogCurrent(int process_increment);  //zapisivanje u svakom ciklusu potrebnih podataka u polje logera
  void Logger(); //zapisivanje u datoteke
  void OslobodiMemoriju();   //oslobadjanje polja logera
  void Ucitaj_stanje_lasera();
  void Ucitaj_stanje_odometrije();
  double Lokalna_u_globalnu_x(double Rx, double Rth, double tockaX, double tockaY);
  double Lokalna_u_globalnu_y(double Rx, double Rth, double tockaX, double tockaY);
  double Globalna_u_lokalnu_x(double Rx, double Ry, double Rth, double tockaX, double tockaY);
  double Globalna_u_lokalnu_y(double Rx, double Ry, double Rth, double tockaX, double tockaY);
  double average_speed;  //mjeri srednju brzinu tijekom voznje
  long brojac_brzina;   //za mjerenje srednje brzine tijekom voznje
  double metric; //za prebacivanje u metre iz milimetara i obrnuto
  R_point global_goal_workhorse;
  int no_path_counter;
  bool new_global_goal;
  double kl,kr,kb,b,Tv,Tw;//parametri robota
  double rmin;//udaljenost do najblize prepreke (pise se u ucitaj_stanje_lasera)
  bool firstlog;

public:

	// Konstruktor
	WorkHorse();
	

	// Destruktor
	~WorkHorse();
  
    void LoadInit(double width, double height, double resolution, double originx, double originy);
 	int processState;
 	void process();
};

#endif
