
//created 15.06.2003

#ifndef DYNAMIC_WINDOW_H
#define DYNAMIC_WINDOW_H
#include <algorithm>
#include "WorkHorse.h" //zbog MB_RB, a on ima Planner.h i Params.h (R_point, I_point)

/**********************************
 * Definicije globalnih konstanti *
 **********************************/

//DIMENZIJE DINAMICKOG PROZORA (ukupno je dimenzija +1)  MORA biti PARAN broj za simetriju!!!
//(+1)Dimenzija translacijskih brzina dinamickog prozora
#define V_DIM		2//2			
// (+1)Dimenzija rotacijskih brzina dinamickog prozora
#define W_DIM		2//2
//omnidrive robot
#define OMNIDRIVE	0
//(+1)Dimension sideway speed
#define VY_DIM		4
//puting upside down V rotational velocity profile into the trajectory calculations (1 yes, 0 no) only for OMNIDRIVE 1, now for diff too
#define ROT		0			
//LASER   -- Broj tocaka sa lasera u memoriji
#define N_OPF PLAYER_LASER_MAX_SAMPLES
//do kuda gledamo sa laserom (iz Params.h, isti koristi i WH)
#define LASER_RANGE	LASER_RANGE_MAX
//s obzirom na maksimalnu brzinu i domet lasera odredjuje se donja granica maksimalnog slobodnog vremena
#define TMAX   (LASER_RANGE/V_MAX)
//PARAMETRI KRUZNOG LUKA - broj tocaka koji opisuje kruznu trajektoriju
#define N_KL		51 //30 51
//uguravanje w=0 u prozor brzina
#define UGURAVANJE 1
//single suma i isti broj tocaka
#define PROMJENA_KRITERIJA 1
//promjena kriterija
#define STARASUMA 0
//iskljucujem rotiranje prema E (obrnuta logika 0 je iskljucivanje)
#define ISKLJUCI_ROTIRANJE 0
//rho for slowing down
#define RHO 0//1./M_PI*0.1*0.002 //0.00001 //turn off with 0
//gain of the translational velocity (multiply with rho to see what is contribution of v,vy)
#define GAINV M_PI*0.1 //M_PI/0.00100
//objective function (1-sum, 0-last point)
#define OBJECTIVE_SUM 1
//conditions for stability 1-all points have cost ge than end point S(i)>=S(T); 0-none; -1- only S(0)>=S(T)
#define OBJECTIVE_CONDITIONS 1
//special calculation of cost function at goal
#define SPECIAL_GOAL_F 0
//including delay of 3 time steps of p3dx in kin model (3), 1 - old only 1 time step
#define DELAY3DT 1
//rotatation excludes translation and vice versa
#define ROT_EXCL_TRANS_CONSTR 0
//deadzone around cell edges (in cell sizes)
#define DEADZONE (DV_MAX*STEP*STEP/CELL_DIM*DW_MAX*STEP*STEP/0.25) //=0.002
//epsilon for orientating towards the goal orientation
#define EPSILON 0//10000. //turn off with 0
//ellipse around every point on trajectory
#define ELLIPSE 0 //turn off with 0
//without breakage trajectory
#define USEBREAKAGETRAJ 1
//nije vise kruzni luk
#define KRUZNI_LUK 0
//constraints for diagonal speed
#define DIAGCONSTR 0
//broj tocaka efektivne putanje
#define N_PATH_REDUCED N_KL
//uzeli smo DUZINU_DSTAR_PUTA istu kao i za maximalnu duzinu trajektorije dinamickog prozora
#define DUZINA_DSTAR_PUTA  (TMAX*V_MAX)
//pokretne prepreke
#define BR_PPP   100000 //max broj pokretnih prepreka detektiranih u jednom ciklusu
//promjenljivi max_path (0-prema najblizoj prepreci, 1-prema max brzini)
#define MAX_PATH 1
//acceleration constraints
#define ACC_CONSTRAINT 0
//da li se gleda razlika zadanih akceleracija (1) ili ostvarenih (0)
#define DELTA_REF 1
//da li i za alfu (kutnu akceleraciju) vrijedi dozvoljavanje samo jednog smjera kao i za translacijsku akc.
#define I_ZA_ALFU 1
//radi se prozor oko reference iz proslog ciklusa
#define OKO_REFERENCE 1
//da li se predikcija racuna po predikciji od proslog ciklusa ili po stvarnoj brzini.
#define PO_PREDIKCIJI 0
//pogledati trazenje druge tocke infleksije u funkciji FindEffectivePath()
#define ZA_PRVU 1
#define I_ZA_DRUGU 1
//Tezinski koeficijent u kriterijskoj funkciji - prohodnost i u path alignmentu: LAMBDA*prohodnost + (1-LAMBDA)*path_alignment
#define LAMBDA  0.5
//SECURITY DISTANCE za robota
#define SC1   0.//in mm
#define SC2   100.
#define SC_W 0.0
//TOLERANCIJA BRZINA -  brzinom 0 se smatra i brzina od 5mm/s
#define V_TOLERANCE  (0.01*DV_MAX)  //3 mm/s u milimetrima
#define W_TOLERANCE  (0.01*DW_MAX)  //1 deg/s u radijanima (5.*M_PI/180.)
//maximal value of the path cost in the criterium
#define MAXCOST 1000000000.

//moze biti od 1 na vise...  sampliranje globalnog geometrijskog puta tako da se izmedju dvije tocke ubace jos dvije, odn. segment se dijeli na tri dijela
#define REAL_PATH_SAMPLE 2

//ovo je mali hat_trick :)
#define MNOZI_SINKO_MNOZI 1//(3/2.) //zbog tezista otezane sume u kriteriju path alignment


//moguce stanje flagova za parove brzina
#define INITIAL 0
#define DYNAMIC_CONSTRAINTS -1
#define NON_ADMISSIBLE -2
#define KINEMATIC_CONSTRAINTS -3
#define GOAL_CONSTRAINTS -4
#define HAS_OBSTACLE 1
#define CLEAR 2


//iscrtavanje optimalne trajektorije u prozoru Stagea - svaka 5. tocka
#define DEC_KL_DRAW 5   //faktor decimacije za crtanje optimalne trajektorije


//LOGGER datoteke
//tu su upisane tocke prepreka na putanji  - X,Y koordinate (tocke na trajektorijama gdje se robot sudara s preprekom - ukljucen je safety distance)
#define LOG_OBSTACLE_POINT_X		"logger//log_obstacle_point_x.dat"
#define LOG_OBSTACLE_POINT_Y		"logger//log_obstacle_point_y.dat"
 //trajektorije pokretnih prepreka - X,Y koordinate
#define LOG_POKRETNE_PREPREKE_POINT_X		"logger//log_pokretne_prepreke_point_x.dat"
#define LOG_POKRETNE_PREPREKE_POINT_Y		"logger//log_pokretne_prepreke_point_y.dat"
#define LOG_POKRETNE_PREPREKE_INDEKS		"logger//log_pokretne_prepreke_indeks.dat"
//sve DW trajektorije u trenutnom ciklusu logiranja - X,Y koordinate i sve DW BREAKAGE trajektorije u trenutnom ciklusu logiranja - X,Y koordinate
#define LOG_PUTANJE_X			"logger//log_putanje_x.dat"
#define LOG_PUTANJE_Y			"logger//log_putanje_y.dat"
#define LOG_PUTANJE_V			"logger//log_putanje_v.dat"
#define LOG_PUTANJE_VY			"logger//log_putanje_vy.dat"
#define LOG_PUTANJE_W			"logger//log_putanje_w.dat"
#define LOG_PUTANJE_TH			"logger//log_putanje_th.dat"
#define LOG_PUTANJE_X_OLD			"logger//log_putanje_x_old.dat"
#define LOG_PUTANJE_Y_OLD			"logger//log_putanje_y_old.dat"
#define LOG_PUTANJE_TH_OLD			"logger//log_putanje_th_old.dat"
#define LOG_PUTANJE_X_BREAK			"logger//log_putanje_x_break.dat"
#define LOG_PUTANJE_Y_BREAK			"logger//log_putanje_y_break.dat"
#define LOG_PUTANJE_TH_BREAK			"logger//log_putanje_th_break.dat"
#define LOG_OPT_PUTANJE_X			"logger//log_opt_putanje_x.dat"
#define LOG_OPT_PUTANJE_Y			"logger//log_opt_putanje_y.dat"
#define LOG_OPT_PUTANJE_TH			"logger//log_opt_putanje_th.dat"
#define LOG_OPT_PUTANJE_S			"logger//log_opt_putanje_S.dat"
#define LOG_OPT_PUTANJE_V			"logger//log_opt_putanje_v.dat"
#define LOG_OPT_PUTANJE_VY			"logger//log_opt_putanje_vy.dat"
#define LOG_OPT_PUTANJE_W			"logger//log_opt_putanje_w.dat"
#define LOG_OLD_PUTANJE_S			"logger//log_old_putanje_S.dat"
#define LOG_OLD_PUTANJE_V			"logger//log_old_putanje_V.dat"
#define LOG_OLD_PUTANJE_VY			"logger//log_old_putanje_VY.dat"
#define LOG_OLD_PUTANJE_W			"logger//log_old_putanje_W.dat"
#define LOG_BREAK_PUTANJE_S			"logger//log_break_putanje_S.dat"
#define LOG_BREAK_PUTANJE_V			"logger//log_break_putanje_V.dat"
#define LOG_BREAK_PUTANJE_VY			"logger//log_break_putanje_VY.dat"
#define LOG_BREAK_PUTANJE_W			"logger//log_break_putanje_W.dat"
#define LOG_PUTANJE_X_PLUS			"logger//log_putanje_x_plus.dat"
#define LOG_PUTANJE_Y_PLUS			"logger//log_putanje_y_plus.dat"
#define LOG_PUTANJE_V_PLUS			"logger//log_putanje_v_plus.dat"
#define LOG_PUTANJE_VY_PLUS			"logger//log_putanje_vy_plus.dat"
#define LOG_PUTANJE_W_PLUS			"logger//log_putanje_w_plus.dat"
#define LOG_PUTANJE_S_PLUS			"logger//log_putanje_S_plus.dat"
#define LOG_PUTANJE_TH_PLUS			"logger//log_putanje_th_plus.dat"
#define LOG_PUTANJE_X_PLUS1			"logger//log_putanje_x_plus1.dat"
#define LOG_PUTANJE_Y_PLUS1			"logger//log_putanje_y_plus1.dat"
#define LOG_PUTANJE_V_PLUS1			"logger//log_putanje_v_plus1.dat"
#define LOG_PUTANJE_VY_PLUS1			"logger//log_putanje_vy_plus1.dat"
#define LOG_PUTANJE_W_PLUS1			"logger//log_putanje_w_plus1.dat"
#define LOG_PUTANJE_S_PLUS1			"logger//log_putanje_S_plus1.dat"
#define LOG_PUTANJE_TH_PLUS1			"logger//log_putanje_th_plus1.dat"
#define LOG_PUTANJE_X_PLUS2			"logger//log_putanje_x_plus2.dat"
#define LOG_PUTANJE_Y_PLUS2			"logger//log_putanje_y_plus2.dat"
#define LOG_PUTANJE_V_PLUS2			"logger//log_putanje_v_plus2.dat"
#define LOG_PUTANJE_VY_PLUS2			"logger//log_putanje_vy_plus2.dat"
#define LOG_PUTANJE_W_PLUS2			"logger//log_putanje_w_plus2.dat"
#define LOG_PUTANJE_S_PLUS2			"logger//log_putanje_S_plus2.dat"
#define LOG_PUTANJE_TH_PLUS2			"logger//log_putanje_th_plus2.dat"
#define LOG_PUTANJE_X_MINUS			"logger//log_putanje_x_minus.dat"
#define LOG_PUTANJE_Y_MINUS			"logger//log_putanje_y_minus.dat"
#define LOG_PUTANJE_V_MINUS			"logger//log_putanje_v_minus.dat"
#define LOG_PUTANJE_VY_MINUS			"logger//log_putanje_vy_minus.dat"
#define LOG_PUTANJE_W_MINUS			"logger//log_putanje_w_minus.dat"
#define LOG_PUTANJE_S_MINUS			"logger//log_putanje_S_minus.dat"
#define LOG_PUTANJE_TH_MINUS			"logger//log_putanje_th_minus.dat"
#define LOG_PUTANJE_TH_MINUS2			"logger//log_putanje_th_minus2.dat"
#define LOG_PUTANJE_X_MINUS2			"logger//log_putanje_x_minus2.dat"
#define LOG_PUTANJE_Y_MINUS2			"logger//log_putanje_y_minus2.dat"
#define LOG_PUTANJE_V_MINUS2			"logger//log_putanje_v_minus2.dat"
#define LOG_PUTANJE_VY_MINUS2			"logger//log_putanje_vy_minus2.dat"
#define LOG_PUTANJE_W_MINUS2			"logger//log_putanje_w_minus2.dat"
#define LOG_PUTANJE_S_MINUS2			"logger//log_putanje_S_minus2.dat"
#define LOG_PUTANJE_S			"logger//log_putanje_S.dat"
#define LOG_BREAKAGE_PUTANJE_X			"logger//log_breakage_putanje_x.dat"
#define LOG_BREAKAGE_PUTANJE_Y			"logger//log_breakage_putanje_y.dat"
//dstar putanja u trenutnom ciklusu logiranja - X,Y koordinate
#define LOG_DSTAR_PATH_PUTANJE_X			"logger//log_dstar_path_putanje_x.dat"
#define LOG_DSTAR_PATH_PUTANJE_Y			"logger//log_dstar_path_putanje_y.dat"
//BREAKAGE POINTOVI  -X,Y,TH koordinate tocaka zaustavljanja po pojedinoj trajektoriji (zapravo redundantno s ovim gore breakage trajektorijama)
#define LOG_BREAKAGE_POINT_X		"logger//log_breakage_point_x.dat"
#define LOG_BREAKAGE_POINT_Y		"logger//log_breakage_point_y.dat"
#define LOG_BREAKAGE_POINT_TH		"logger//log_breakage_point_th.dat"
//efektivna putanja u trenutnom ciklusu logiranja - X,Y koordinate
#define LOG_PATH_PUTANJE_X			"logger//log_path_putanje_x.dat"
#define LOG_PATH_PUTANJE_Y			"logger//log_path_putanje_y.dat"
//nadjena druga tocka infleksije na globalnom geometrijskom putu - X,Y koordinate
#define LOG_TOCKA_INFLEKSIJE			"logger//log_tocka_infleksije.dat"
//sve linearne i rotacijske brzine dinamickog prozora u trenutnom ciklusu logiranja
#define LOG_LIN_BRZINE		"logger//log_lin_brzine.dat"
#define LOG_LINY_BRZINE		"logger//log_liny_brzine.dat"
#define LOG_ROT_BRZINE		"logger//log_rot_brzine.dat"
//zastavice, vrijednosti ukupne ocjene, vrijednosti ocjene prohodnosti, vrijednosti ocjene poklapanja s efektivnom putanjom svakog para brzina u trenutnom ciklusu logiranja
#define LOG_TB_FLAG		"logger//log_TB_flag.dat"
#define LOG_TB_FLAG_PLUS		"logger//log_TB_flag_plus.dat"
#define LOG_TB_FLAG_PLUS1		"logger//log_TB_flag_plus1.dat"
#define LOG_TB_FLAG_PLUS2		"logger//log_TB_flag_plus2.dat"
#define LOG_TB_FLAG_MINUS		"logger//log_TB_flag_minus.dat"
#define LOG_TB_FLAG_MINUS2		"logger//log_TB_flag_minus2.dat"
#define LOG_TB_OCJENA		"logger//log_TB_ocjena.dat"
#define LOG_TB_OCJENA_PROHODNOST		"logger//log_TB_ocjena_prohodnost.dat"
#define LOG_TB_OCJENA_PATH		"logger//log_TB_ocjena_path.dat"
//indeksi optimalnog para brzina (uvecani za 1 zbog crtanja u matlabu), dimenzije prostora translacijskih i rotacijskih brzina, trenutna brzina robota
#define LOG			        "logger//log.dat"
//robotova trenutna pozicija (u trenutku logiranja)
#define LOG_ROBOT_POSITION		"logger//log_robot_position.dat"
//laser prepreke - X,Y koordinate (u trenutnom ciklusu logiranja)
#define LOG_LASER_OBSTACLES		"logger//log_laser_obstacles.dat"

// Kruzni luk
typedef struct{

	// Koordinate
	double x[N_KL];	double y[N_KL]; double th[N_KL]; double v[N_KL]; double vy[N_KL]; double w[N_KL]; double S[N_KL];
	}MB_KL;
	
typedef struct{

	double v[DELAY3DT+1];	double vy[DELAY3DT+1]; double w[DELAY3DT+1]; 
	}MB_oldref;
	

//ovo je mozda ruzno ali da mi ostanu m skripte iskoristive za prvu inacicu, poslije obrisi...
#if OMNIDRIVE
//definiranje loggera za DW
struct Logger{
	double xkl[N_KL][V_DIM+1][W_DIM+1][VY_DIM+1];     //x-pozicija tocke putanje (kruznog luka)
	double ykl[N_KL][V_DIM+1][W_DIM+1][VY_DIM+1];     //y-pozicija tocke putanje (kruznog luka)
	double th[N_KL][V_DIM+1][W_DIM+1][VY_DIM+1];     //th tocke putanje (kruznog luka)
	double vkl[N_KL][V_DIM+1][W_DIM+1][VY_DIM+1];
	double vykl[N_KL][V_DIM+1][W_DIM+1][VY_DIM+1];
	double wkl[N_KL][V_DIM+1][W_DIM+1][VY_DIM+1];
	double Skl[N_KL][V_DIM+1][W_DIM+1][VY_DIM+1];
  double xkl_breakage[N_KL][V_DIM+1][W_DIM+1][VY_DIM+1];     //x-pozicija tocke putanje za zaustavljanje(kruznog luka)
	double ykl_breakage[N_KL][V_DIM+1][W_DIM+1][VY_DIM+1];     //y-pozicija tocke putanje za zaustavljanje(kruznog luka)
  // Trajektorija za iscrtavanje optimalne putanje
  double xkl_optimal[N_KL/DEC_KL_DRAW+1];
  double ykl_optimal[N_KL/DEC_KL_DRAW+1];
};
#else
//definiranje loggera za DW
struct Logger{
	double xkl[N_KL][V_DIM+1][W_DIM+1];     //x-pozicija tocke putanje (kruznog luka)
	double ykl[N_KL][V_DIM+1][W_DIM+1];     //y-pozicija tocke putanje (kruznog luka)
	double th[N_KL][V_DIM+1][W_DIM+1];     //th tocke putanje (kruznog luka)
	double vkl[N_KL][V_DIM+1][W_DIM+1];
	double vykl[N_KL][V_DIM+1][W_DIM+1];
	double wkl[N_KL][V_DIM+1][W_DIM+1];
	double Skl[N_KL][V_DIM+1][W_DIM+1];
  double xkl_breakage[N_KL][V_DIM+1][W_DIM+1];     //x-pozicija tocke putanje za zaustavljanje(kruznog luka)
	double ykl_breakage[N_KL][V_DIM+1][W_DIM+1];     //y-pozicija tocke putanje za zaustavljanje(kruznog luka)
  // Trajektorija za iscrtavanje optimalne putanje
  double xkl_optimal[N_KL/DEC_KL_DRAW+1];
  double ykl_optimal[N_KL/DEC_KL_DRAW+1];
};
#endif

struct MB_setpoint{
  double setpoint_v;
  double setpoint_vy;
  double setpoint_w;
  double setpoint_x;
  double setpoint_y;
  double setpoint_th;
};


// Tablica svih parova brzina
typedef struct{
	//flagovi INITIAL i  KINEMATIC_CONSTRAINTS se upisuju u Odredi_dinamicki_prozor();  NON_ADMISSIBLE, HAS_OBSTACLE i CLEAR se upisuju u Prohodnost()
#if OMNIDRIVE
	int flag[V_DIM+1][W_DIM+1][VY_DIM+1];  // Tablica zastavica
//ocjene se inicijaliziraju u Odredi_dinamicki_prozor(), izracunavaju se u DoprinosProhodnost() te Doprinos_path()
	double ocjena[V_DIM+1][W_DIM+1][VY_DIM+1];     // Tablica - ukupna ocjena odredjene trajektorije
	double ocjena_prohodnost[V_DIM+1][W_DIM+1][VY_DIM+1]; //tablica prohodnosti s obzirom na prepreke
	double ocjena_path[V_DIM+1][W_DIM+1][VY_DIM+1];  // Tablica - ocjena odredjene trajektorije s obzirom na poklapanje s efektivnom putanjom
//tocke sudara na trajektoriji se odredjuju u Prohodnost()
	double obstacle_point_x[V_DIM+1][W_DIM+1][VY_DIM+1]; //tocke na preprekama gdje se robot sudara s preprekom (prosirenom za safety distance)
	double obstacle_point_y[V_DIM+1][W_DIM+1][VY_DIM+1];
  	double obstacle_point_th[V_DIM+1][W_DIM+1][VY_DIM+1];
#else
	int flag[V_DIM+1][W_DIM+1];  // Tablica zastavica
//ocjene se inicijaliziraju u Odredi_dinamicki_prozor(), izracunavaju se u DoprinosProhodnost() te Doprinos_path()
	double ocjena[V_DIM+1][W_DIM+1];     // Tablica - ukupna ocjena odredjene trajektorije
	double ocjena_prohodnost[V_DIM+1][W_DIM+1]; //tablica prohodnosti s obzirom na prepreke
	double ocjena_path[V_DIM+1][W_DIM+1];  // Tablica - ocjena odredjene trajektorije s obzirom na poklapanje s efektivnom putanjom
//tocke sudara na trajektoriji se odredjuju u Prohodnost()
	double obstacle_point_x[V_DIM+1][W_DIM+1]; //tocke na preprekama gdje se robot sudara s preprekom (prosirenom za safety distance)
  	double obstacle_point_y[V_DIM+1][W_DIM+1];
  	double obstacle_point_th[V_DIM+1][W_DIM+1];
#endif
//brzine se odredjuju u Odredi_dinamicki_prozor()
	double v[V_DIM+1]; double w[W_DIM+1]; double vy[VY_DIM+1]; //polja linearnih i rotacijskih brzina koje se nalaze u dinamickom prozoru
//radijus se odredjuje u Kruzni_luk()
	double radius[V_DIM+1][W_DIM+1];  //radijus pojedine trajektorije
	double centar_luka_x[V_DIM+1][W_DIM+1];  //centar luka pojedine trajektorije
	double centar_luka_y[V_DIM+1][W_DIM+1];  //centar luka pojedine trajektorije
//tocke zaustavljanja se odredjuju u Kruzni_luk()
	double breakage_point_x[V_DIM+1][W_DIM+1]; //tocke zaustavljanja
	double breakage_point_y[V_DIM+1][W_DIM+1];
	double breakage_point_th[V_DIM+1][W_DIM+1];
  
}MB_TB;

class DynamicWindow{

 public:
 
 R_point *GetEffectivePath(){return this->path_r_reduced;};
 double GetEffectivePathLength(){return this->path_length;};
 double GetEffectivePathOrientation(){return this->path_orientation;};
 int GetEffectivePathLengthInt(){return this->effective_path_length;};
 
 double GetRefVelocitiesV(){return this->SP.setpoint_v;};
 double GetRefVelocitiesW(){return this->SP.setpoint_w;};
 double GetRefVelocitiesVY(){return this->SP.setpoint_vy;};

  public:
  //kreiranje reprezentacije pokretnih prepreka - tocke, brzine, kutevi (smjer kretanja)
  struct pokretne_prepreke: public R_point{
	double brzina, kut, omega;
	int indeks;
  };
  typedef struct pokretne_prepreke p_p;
  p_p PPP[BR_PPP]; //polje pokretnih prepreka
  int broj_pokretnih_prepreka;
  int obrnuto,oznaka,locmincnt;
//   char port1[11],port2[11];//imena datoteka od drugih robota
  R_point OPF[N_OPF]; //polje sa tockama kao laser preprekama
  int laser_pointer_end;//zbog trenutno aktualnih prepreka
  int indeks_infleksije;
  double desiredOrientation;
  R_point tocka_infleksije;
	// Elementi kruznog luka
  MB_KL KL;
  MB_KL KL_old;//stari se pamti
  MB_KL KL_old_temp;
  MB_KL KL_breakage; //breakage trajectory
  MB_KL KL_plus, KL_minus, KL_minus2;
  MB_oldref oldref;
#if ROT
  MB_KL KL_plus1, KL_plus2;
  MB_TB TB_plus1, TB_plus2;
//  bool plus1,plus2;
  Logger Log_plus1, Log_plus2;
#endif
  bool plus1,plus2;
  bool flag_kl_old;
  bool best_old;
  bool best_breakage;
  bool blizu_cilja;
  double cost_old;
  double cost_breakage;
  int T_old;
  double ljapunov;
  R_point E, C_plus;
  bool plus,minus,minus2;
  // Elementi trajektorije pokretne prepreke (moving obstacle)
  MB_KL MO[BR_PPP];
	// Tablica parova brzina dinamickog prozora
  MB_TB TB;
  MB_TB TB_plus, TB_minus, TB_minus2;
 	// (UNIVERZALNI) Indexi za polja
  int ni, nj, nk;
	// Varijable minimalnog zaustavnog puta
  int vs,ws;//reference iz pretproslog ciklusa - za ukljanjanje pikova
  double ts, as, ss;
  int broj_DStar_celija;
  bool naStartu;
  bool poceo;
  int obstacle_index, min_obstacle_index;
  bool naCilju;//za postizanje ciljne orijentacije
  bool rotateongoal;
  int ukrivustranu;
  double Lambda;
  double kut_E;//kut prema exit pointu
  //stvari vezane za path
  R_point *path_r; //globalna geometrijska putanja
  R_point *path_r_reduced;   //efektivna putanja
  R_point *path_r_sampled;  //oversamplirana globalna geometrijska putanja
  double path_length;   //duljina efektivne putanje (uracunat 3/2*)
  int effective_path_length;  //broj elemenata polja efektivne putanje
  double path_orientation;//smjer efektivne putanje u odnosu na x globalnu os
  double kut_prepreke;//kut najblize prepreke robotu, pise se u prohodnosti, a cita u optimalnom paru
  double limit_distance;//ona udaljenost sa scvovima
  double udaljenost_prepreke;//udaljenost do najblize prepreke koja ima kut kut_prepreke
  double epsilon;//suma svih udaljenosti do najblize prepreke po svim tockama trajektorije
  bool blizu;//da li je prepreka blizu robota za manje od metar
  R_point GetTockaInfleksije(){return this->tocka_infleksije;};
  
  //podaci primljeni od drugog robota
double volatile ROBOT_X;
double volatile ROBOT_Y;
double volatile ROBOT_W;
double  volatile ROBOT_V;
double  volatile ROBOT_TH;
	public:

  //logger data
	Logger Log;  //upisuje se u LogMisc()
	Logger Log_plus, Log_minus, Log_minus2;
  
  MB_RB RB; //instanca stanja robota
  
  MB_setpoint SP;   //instanca setpoint vrijednosti brzina
  void Sekvenca_izvodjenja();
    int vremenska_razlika;
  void TimeLog(int state);
  //ubacivanje trenutnog stanja robota u strukturu
  void Azuriraj_stanje_robota();
  int FindEffectivePath();
  //ubacivanje novih mjerenja lasera u strukuru prepreka
  void Ubaci_laser();
  // Definicija dinamickog prozora
  void Odredi_dinamicki_prozor();
  // kinematic model
  void kin_model_evol(MB_KL *KLa, int i);
  //funkcija za izracun trajektorije pokretne prepreke
  void Pokretna_prepreka();
  void ChangeVariables(double x,double y,double th, double w, double v);
  bool Fourpointscollide(double x, double y, double th, double vx, double vy, int index);
  double checkDesiredOrientation(double th);
  double computeInterpolatedCost(double x, double y, double th);
  void MinimalniZaustavniPut();
  // Funkcija za proracun kruznog luka
  void Kruzni_luk();
  void Kruzni_luk_sa_zaustavljanjem();
//  void Trajectory_with_stop(); //for omnidrive
  void LogMisc(int state);
  void Prohodnost();
  double PathAlignment();
  double Old_traj();
  double Breakage_traj();
  void DoprinosProhodnost();
  void Doprinos_path();
	// Trazenje optimalnog para
  void Optimalni_par(double kut);
  void Path_minimum();
  //azuriranje log podataka na hard disk
  void Ispis_polja();
  double round(double x);
	// Konstruktor
  DynamicWindow();
	// Destruktor
  ~DynamicWindow(){
	  if(path_r_reduced) free(path_r_reduced);
	  if(path_r_sampled) free(path_r_sampled);
	  printf("DW unisten");
  }


};

#endif
