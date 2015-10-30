#ifndef GRIDMAP_H
#define GRIDMAP_H
#include <math.h>
#include <string.h>
#include "WorkHorse.h"      //Planner.h, Params.h je unutra



typedef struct: public R_point{
  int occupancy;
  int time_stamp;
 bool static_cell;
 int obstacle;
}GridMap_cell;

struct CsStack{
    double stack[10][3];
    int top;
    double T[3][3];
    };



    class GridMap{


	    public:
		    int counter;  //counter nam daje zapravo time_stamp
		    int counter_moving; //time_stamp za pokretne prepreke mora bit drugaciji ako ne zelimo upisivat u dstar mapu
		    void alloc_Map();  //stvaranje mape po dimenzijama iz wld datoteke
		    void alloc_Mapfrompng(double width, double height, double resolution, double originx, double originy);
		    void free_Map();
		    int check_point(R_point current_point);//provjera da li se nalazi tocka unutar dimenzija karte
		    int fill_Map();  //ubacivanje trenutnog laserskog ocitanja u mapu
		    int fill_Map_Init();  //prepisivanje statickih prepreka iz wld datoteke u mapu
		    void push(double x0, double y0, double th);  //pushanje novog koordinatnog sustava - prebacivanje koordinata iz novog koordinatnog sustava u globalni
		    void pop();
		    int GetMapSizeX() {return Map_Dim_X;};
		    int GetMapSizeY(){return Map_Dim_Y;};
		    GridMap_cell **GetMap(){return Map;};
		    GridMap_cell **Map;  //pointer na alociranu memoriju za mapu
		    R_point Map_Home;    //realne koordinate ishodista mape
		    double Map_Dim_X_A, Map_Dim_Y_A; //dimenzije mape u realnom iznosu
		    double Map_Cell_Size;      //velicina celije
		    int Map_Dim_X, Map_Dim_Y; //dimenzije mape u integeru (maksimalni indeksi)
		    I_point cell_point_temp;  //pomocne varijable kod pretvaranja iz realnih koordinata u indekse
		    R_point mapper_point_temp;
  //celije na koje utjecu nova laserska ocitanja - trazenje pokretnih prepreka
  //polje pokretnih prepreka - odredjene trazenjem skupova ne statickih laserskih povezanih hitova, ima ih numnewcells
		    R_point *moving;//[PLAYER_LASER_MAX_SAMPLES]; //realne koordinate
		    R_point *statickiHitovi;//[PLAYER_LASER_MAX_SAMPLES]; //realne koordinate
		    R_point *moving_cell;//[PLAYER_LASER_MAX_SAMPLES]; //realne koordinate, jedan hit po celiji
		    int *moving_index;//[101];//indeksima razdvajamo grupirane pokretne prepreke, ima ih kao numnewcg
		    int *moving_cell_index;//[101];//indeksima razdvajamo grupirane pokretne prepreke (samo celije), ima ih kao numnewcg
		    R_point *stare_moving;//[PLAYER_LASER_MAX_SAMPLES];  //prepisane iz moving_celije polja iz proslog ciklusa
		    R_point *stare_moving_pom;//[PLAYER_LASER_MAX_SAMPLES];  //prepisane iz moving_celije polja iz proslog ciklusa
		    I_point *col_moving;//[101]; //celije na mjestu sudara s pokretnom preprekom (to je pokretna iz moving_celije translatirana na mjesto sudara), izracunato u DW, ima ih num_col_points
		    I_point *col_moving_old;//[101];  //stare takve (iz proslog ciklusa), u PL se prepisuju (kasni jedan ciklus buduci da se DW izvodi zadnji u ciklusu)
		    R_point *stare_cg;//[101];      //stara tezista (iz proslog ciklusa)
		    R_point *cg;//[101];                //nova tezista - nastala grupiranjem laserskih hitova koji nisu staticke prepreke, ima ih numnewcg
		    I_point *indeksimapepunjenje; //pamtim indekse u mapi laserskih hitova da ne trcim prek cijele mape u planeru
		    I_point *indeksimapepraznjenje; //pamtim indekse u mapi laserskih hitova da ne trcim prek cijele mape u planeru
		    I_point *indeksizanepunjenje;//u DW se upisuju
//   I_point novi[PLAYER_LASER_MAX_SAMPLES]; //za crtanje
		    int brojnovih;//za crtanje
		    int numindeksimapepunjenje;
		    int numindeksimapepraznjenje;
		    int numindeksizanepunjenje;
		    int numnewmovings,numnewmovingcells,num_col_points,numnewstatic;
		    int numoldmovings,pocetak,num_col_points_old;
		    int numnewcg,numoldcg;
		    int nonempty;
  //parametri pretvorbe wld u grid
//   double linije[900][4]; //e ako ima vise linija u wld datoteci nadrapao je
		    double **linije; //sve mijenjam u maloce
		    CsStack cstransf;
  //Konstruktor i destruktor klase
		    void reset();
		    GridMap(double cell_size);
		    ~GridMap()
		    {
			    free_Map();
			    printf("GM unisten");
	
		    };
    };
#endif
