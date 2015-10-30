#include <math.h>
//#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include "Params.h"             //zbog MIN, parametara konverzije
//#include <playerclient.h>  // for player client stuff       (time)
//#include <libplayerc++/playerc++.h> //novi player
#include "Planner.h" //zbog I_point
#include <sys/time.h>
#include <bucketedqueue.h>
#include <iostream>

//#define COSTDIAGONAL 1  
//#define COSTSTRAIGHT 1      
//#define COSTROTATION 1     
#define COSTDIAGONAL 14//56  
#define COSTSTRAIGHT 10//40      
#define COSTROTATION 0     
//#define DSTAR_EIGHT_CONNECTED 1

#define LESS(a1, a2, b1, b2) ((a1) < (b1) ? 1 : ((a1) == (b1)) && ((a2) < (b2)) ? 1 : 0)
// vector [a1 a2] is less than vector [b1 b2] according to the components
#define LESSEQ(a1, a2, b1, b2) ((a1) < (b1) ? 1 : ((a1) == (b1)) && ((a2) <= (b2)) ? 1 : 0)
#define LESSEQ3(a1, a2, a3, b1, b2, b3) ((a1) < (b1) ? 1 : (((a1) == (b1)) && ((a2) < (b2)) ? 1 : (((a2) == (b2)) && ((a3) <= (b3)) ? 1 : 0 )))
#define LESS3(a1, a2, a3, b1, b2, b3) ((a1) < (b1) ? 1 : (((a1) == (b1)) && ((a2) < (b2)) ? 1 : (((a2) == (b2)) && ((a3) < (b3)) ? 1 : 0 )))
// vektor [a1 a2 a3] je manji ili jednak vektoru [b1 b2 b3]
//vec je definiran u /usr/include/sys/param.h #define MIN(a, b) ((a) < (b) ? (a) : (b))
#define EMPTYC 1
#define OBSTACLE 100000000
#define NEW 0
#define OPEN 1
#define CLOSED -1
//without costmask in planning (1), with (0)
#define NO_COSTMASK 0
//for Witkowski algorithm - sorting the queue
#define SORTIRANJE 0
#define NOVI_RED 0
#define NOVI_RED_SORT 0
#define JAVIPETLJU 0

#define MAXORI 110 //number of discretized orientations

#define USEBUCKET 0 //lista 0, ne radi za replaniranje

//smjerovi s obzirom na trenutni cvor (pomaci)                                                                  (-1,1)    (0,1)   (1,1)             5.   6.   7.
static const int xofs[ 8 ] = { -1,1,0,0, 1, -1, -1, 1 };
static const int yofs[ 8 ] = { 0,0,1,-1, 1, 1, -1, -1 };
//static const int xofsOri[ 10 ] = { -1, 0, 1, -1, 1, -1, 0, 1, 0, 0 };
//static const int yofsOri[ 10 ] = { -1, -1, -1, 0, 0, 1, 1, 1, 0, 0 };
//static const int thofsOri[ 10 ] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, -1 };

static const int xofsOri[ 10 ] = { -1, 0, 1, 0, 0, 0, 1, -1, -1, 1 };
static const int yofsOri[ 10 ] = { 0, -1, 0, 1, 0, 0, 1, 1, -1, -1 };
static const int thofsOri[ 10 ] = { 0, 0, 0, 0, 1, -1, 0, 0, 0, 0 };

//                                                    (-1,-1)  (0,-1)  (1,-1)            0.   1.   2.
//smjerovi s obzirom na trenutni cvor (pomaci za dvije celije)                                         (-2,-2)  (-1,-2)   (0,-2)  (1,-2)  (2,-2) 0. 1.   2.   3.  4. itd okolo
static const int x2ofs[ 16 ] = { -2, -1, 0, 1, 2, -2, 2, -2, 2, -2, 2, -2, -1, 0, 1, 2};
static const int y2ofs[ 16 ] = { -2, -2, -2, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 2, 2, 2};
//smjerovi s obzirom na trenutni cvor (pomaci za tri celije)                                       (-3,-3)  (-2,-3)  (-1,-3)   (0,-3)  (1,-3)  (2,-3) (3,-3) 0. 1.   2.   3.  4. 5. 6. itd okolo
static const int x3ofs[ 24 ] = { -3, -2, -1, 0, 1, 2, 3, -3, 3, -3, 3, -3, 3, -3, 3, -3, 3, -3, -2, -1, 0, 1, 2, 3};
static const int y3ofs[ 24 ] = { -3, -3, -3, -3, -3, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 3, 3, 3, 3, 3};
//smjerovi s obzirom na trenutni cvor (pomaci za cetiri celije)
static const int x4ofs[ 32 ] = { -4, -3, -2, -1, 0, 1, 2, 3, 4, -4, 4, -4, 4, -4, 4, -4, 4, -4, 4, -4, 4, -4, 4, -4, -3, -2, -1, 0, 1, 2, 3, 4};
static const int y4ofs[ 32 ] = { -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4};
static const int x5ofs[ 40 ] = { -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5,-5, 5, -5, 5, -5, 5, -5, 5, -5, 5, -5, 5, -5, 5, -5, 5, -5, 5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5};
static const int y5ofs[ 40 ] = { -5, -5, -5, -5, -5,-5,-5,-5,-5,-5,-5,-4,-4,-3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};

//cijena puta u pojedinim smjerovima
static const int travCost[ 8 ] =
    { COSTSTRAIGHT, COSTSTRAIGHT, COSTSTRAIGHT, COSTSTRAIGHT,
      COSTDIAGONAL, COSTDIAGONAL, COSTDIAGONAL, COSTDIAGONAL};

//static const int travCostOri[ 10 ] =
//    { COSTDIAGONAL, COSTSTRAIGHT, COSTDIAGONAL,
//      COSTSTRAIGHT, COSTSTRAIGHT,
//      COSTDIAGONAL, COSTSTRAIGHT, COSTDIAGONAL, COSTSTRAIGHT, COSTSTRAIGHT };

static const int travCostOri[ 10 ] =
    { COSTSTRAIGHT, COSTSTRAIGHT, COSTSTRAIGHT, COSTSTRAIGHT,
      COSTROTATION, COSTROTATION,
      COSTDIAGONAL, COSTDIAGONAL, COSTDIAGONAL, COSTDIAGONAL};

      //for open list
      struct at {
	      I_point element;
	      struct at *sljed; 
      };
      typedef struct at atom;


//blacklist for avoiding negative circles



class DStarCell{

  public:

        I_point _next;     //point to the next best neighbor
	I_point _maska; //for costmask calculations
	int preprekaokolo;//how many obstacles is around it within the robot mask
	int preprekablizu;//how many obstacles is around it within the costmask
	int h_cost_int;           //cost of the path from the goal
        int k_cost_int;			// key value
        int g_cost_int; //heuristics towards the start
        int total_cost_int; //this is f=g+k
        int total_cost_int_biased; // not used
        int promjena;	//changed occupancy to occupied (2), to free (1), no change (0)
        bool prepreka_bool;            //is it occupied
        bool cspace_occupied;//is it occupied for all orientations
        int time_stamp;           //cycle time index of inserting the node
	int path_counter;//cycle time index for the nodes in the path
	int tag;                  //tag on open list (NEW, OPEN, CLOSED)
	int traversal_cost; //EMPTYC, OBSTACLE, and between values for the costmask
	int traversal_cost_stari; //old value

//ful 3D search
	int h_cost_intOri[MAXORI];           //cost of the path from the goal
        int k_cost_intOri[MAXORI];			// key value
        I_point _nextOri[MAXORI];     //point to the next best neighbor
	int tagOri[MAXORI];           //cost of the path from the goal
		
		//witkowski and twd*
		I_point _next_reverse;     //twd*
		int time_stamp_forward; //twd
		int h_cost_int_reverse; //twd*
		int k_cost_int_reverse;//twd*
		int tag_reverse; //twd*
		int cost_forward;
		int cost_k_forward;
		int cost_backward;
		int cost_k_backward;
		int cost_sum;
		int tag_forward;
		int tag_backward;
		I_point _next_forward, _next_backward;
		int skupine;
		int cvorovi;

        DStarCell()
        {
          _next.x=-1; _next.y=-1; //as a "NULL" values
	  _next_reverse.x=-1; _next_reverse.y=-1; //as a "NULL" values
		  _maska.x=-1; _maska.y=-1;
		  preprekaokolo=0;
		  preprekablizu=0;
		  h_cost_int=OBSTACLE;  //"infinity" cost
          k_cost_int=OBSTACLE;
          g_cost_int=0;
	  k_cost_int_reverse=0;
	  h_cost_int_reverse=0;
		  total_cost_int=OBSTACLE;  //total cost
          total_cost_int_biased=OBSTACLE;
          promjena=0;
          prepreka_bool=false;    //initially ass. all is free
      	  cspace_occupied=false;
      	  time_stamp=0;
	  time_stamp_forward=0;
	  path_counter=0;
	  tag=NEW;
	  tag_reverse=NEW;
	      traversal_cost=EMPTYC;
	      traversal_cost_stari=EMPTYC;
      	  
		  //witkowski algoritam
		  cost_forward=OBSTACLE;
		  cost_k_forward=OBSTACLE;
		  cost_backward=OBSTACLE;
		  cost_k_backward=OBSTACLE;
		  cost_sum=OBSTACLE;
		  tag_forward=NEW;
		  tag_backward=NEW;
		  _next_forward.x=-1; _next_forward.y=-1; //as "NULL" values
		  _next_backward.x=-1; _next_backward.y=-1; //as "NULL" values
		  skupine=0;
		  cvorovi=0;

        };
};

class DStar{

  public:
	  std::vector<I_point> blacklist;
	  I_point Start; //current robot cell
	  I_point Goal;
	  int MapSizeX, MapSizeY;
	  int NumElemLista;     //number of elements on open list
	  int c;                    //cost of moving between two neighbor nodes in the map
	  DStarCell **map;
	  atom *glava;		// open lst
	  I_point *path;
	  I_point MinCostElemLista;          //
	  I_point negative_cell;

//for 3D search
	int maxOri;
  BucketPrioQueue<I_point*> queue;

//not important
	  R_point start_ri;// current real robot position within the cell
	  I_point prviStart;
	  I_point StartRacunac; //the same as Start 
	  I_point GoalRacunac; //not used
	  I_point StartCurr; //not used
	  I_point prepreka;//
	  I_point nemanext;//
	  I_point petlja;
	  I_point w0;//pocetka tocka W puta
	  I_point bliziPut, najboljiPut[8], stariPut[100];   //za crtanje
	  double starikut_prvi,starikut_drugi,kut_robota,kut_puta;
	  I_point susjediNebliskih[24];
	  int br;
	  int oznakaZaCrtanje;
	  int los_cell_num;
	  int goal_index;
	  int travcost;
	  int travcost3d;

  int NumElemListaReverse;     //twd*
  int NumAtom;//not used
  int NumElemPunjenja;  //num changes to occupied
  int NumElemPraznjenja;  //num changes to empty
  int NumElemCostmaska;  //num changes of cost values
  bool pipodjednom;
  int dcurr;  //not used
  int Robot_f_biased, Robot_f, Robot_h, Robot_f_biased_stari;
  int time_stamp_counter;        //brojac pozivanja algoritma (u initu se inkrementira)
  int old_path_counter;
  int prviput;             //initial planning flag
  int promjena;             //change in the map
  bool novoTeziste; //not used
  bool Start_zauzet;//not used
  int inicijalni_put;    //for logging
  int watchdog_counter;     //num iterations of while loop, 0 means over
  int racunaoupromjeni;
  struct timeval timeStart;
  struct timeval timeNow;
  int mySecStart, myMSecStart,mySecNow, myMSecNow;
  int vremenska_razlika;
  int *izracuni, *azurirani_polje, *f_biased_putanje, *f_putanje, *h_putanje;
  int *broj_iteracija, *broj_cvorova_na_listi, *max_broj_cvorova_na_listi, *cijena_puta, *duljina_puta;
  double *duljina_puta_um, *duljina_wit_puta_um;
  int brojac_izracuna; //broji izracune algoritma, nije jednak broju ciklusa!
  int max_broj_cvorova;
  int azurirani;
  int azurirani_reverse;
  int Numindicesocc;

  atom *glava_reverse;		// twd*
  I_point listapraznjenja[PLAYER_LASER_MAX_SAMPLES];
  I_point listapunjenja[PLAYER_LASER_MAX_SAMPLES];
  I_point listacostmaska[PLAYER_LASER_MAX_SAMPLES];
  I_point *pathinicijalni;
  I_point *pathstari;
  I_point *minfpoints;
  bool PathExists; int PathLength, PathLengthStari; int PathLengthinicijalni;int PathCost; int Numminfpts;

  //witkowski
  atom *ulaz_forward;		// glava liste
  atom *ulaz_backward;		// glava liste
  atom *izlaz_forward;		// glava liste
  atom *izlaz_backward;		// glava liste
  atom *novi_ulaz_forward;		// glava liste
  atom *novi_ulaz_backward;		// glava liste
  atom *novi_izlaz_forward;		// glava liste
  atom *novi_izlaz_backward;		// glava liste
  int NumElemLista_forward;
  int NumElemLista_backward;
  int prviputWitkowski;             //oznaka inicijalnog izvodjenja
  int radiponovo;
  int azurirani_forward;
  int azurirani_backward;
  I_point element_forward;
  I_point element_backward;
  I_point *path_witkowski;
  int PathLengthWitkowski;
  I_point *path_witkowski_segment;
  I_point *path_witkowski_segment_old;
  I_point *sve_konkavne_tocke;
  int broj_konkavnih_tocaka;
  int PathLengthWitkowskiSegment;
  int PathLengthWitkowskiSegmentOld;
  I_point *path_forward;
  int PathLength_forward;
  I_point *path_backward;
  int PathLength_backward;
  bool PathExistsWitkowski;
  FILE	*logfile;
  bool forward,backward,trazi_jos_backward,trazi_jos_forward;
  int Robot_backward, Robot_sum_backward, Robot_forward, Robot_sum_forward;
  int broj_for,broj_bac;
  int *izracuniWitkowski, *azurirani_forward_polje, *azurirani_backward_polje, *vremena_skupine;
  int *broj_iteracijaWitkowski, *broj_cvorova_na_listi_forward, *max_broj_cvorova_na_listi_forward, *cijena_puta_forward, *duljina_puta_forward;
  int *broj_cvorova_na_listi_backward, *max_broj_cvorova_na_listi_backward, *cijena_puta_backward, *duljina_puta_backward;
  int brojac_izracunaWitkowski; //broji izracune algoritma, nije jednak broju ciklusa!
  R_point preslik;
  I_point goal_wit,goal_wit_old;
  I_point *indicesocc;
  
  //functions
  int	Init( int start_x, int start_y, int goal_x, int goal_y);//check for start and goal
  void arc_cost(int X_cell_x, int X_cell_y, int Y_cell_x, int Y_cell_y); //set global value c
  int   IsValid(int x, int y); //out of map (0), occupied (2), free (1)
  void reset();
  bool   SearchPath(); //main D* function, call processState()
  void	processState(); 
  void  insertNode( I_point element, int hnew ); //d*
  int dodaj (atom **glavap, I_point element);//insert into sorted list
  int brisi (atom **glavap, I_point element);//delete node with element==element
  I_point* getPath();  //follow the next pointers
  
  //functions for orientation search
  void processStateOri();
  bool SearchPathOri();
  void insertNodeOri(I_point element, int hnew);
  int dodajOri(atom **glavap, I_point element);
  int IsValidOri(I_point X); //out of map (0), occupied (2), free (1)
  int PathCostOri(I_point X); //value h
  int howmanyOriCollides(I_point X);
  int InitOri( I_point start, I_point goal);//check for start and goal
  I_point* getPathOri();  //follow the next pointers
  void arc_costOri(I_point X, I_point Y);
  

//not important
  void resetWitkowski();
  void resetReverse();
  bool   SearchPathReverse(); //twd
  bool   SearchPathWitkowski();
  void	processStateReverse(); //twd
  void	processStateWitkowski();
  void  insertNodeReverse( I_point element, int hnew ); //twd*
  void    insertNodeWitkowski( I_point element, int costnew ,int smjer);
  int dodaj_reverse (atom **glavap, I_point element);//twd
  int brisi_do_costa (atom** glavap, int h_cost);
  atom *trazi (atom *glava, int stamp);//not used
  atom *trazi_reverse (atom *glava, int stamp);//not used
  void ispisi (atom *glava);//print
  void ispisiRed (atom *glava);
  int DodajURed (I_point element, atom **ulaz, atom **izlaz);
  int DodajURedSortirano (I_point element, atom **ulaz, atom **izlaz, int smjer);
  int SkiniIzReda (atom **ulaz, atom **izlaz);
  int brisiIzReda (I_point element, atom **ulaz, atom **izlaz);
  int traziCost (atom **ulaz, atom **izlaz, I_point *element, int smjer);
  int    pathCostEstimate( I_point P1, I_point P2 ); //racuna g_cost_int
  int    pathCostEstimateDiagonalStraight( I_point P1, I_point P2 );
  void traziSusjedniPut(int brojPolja);
  I_point* getPathWitkowski();  //
  I_point* getPath_forward();  //
  I_point* getPath_backward();  //
  void  Free();
   int getPathLength() {return (int)PathLength;};
   int getPathLengthWitkowski() {return (int)PathLengthWitkowski;};
   int getPathLengthWitkowskiSegment() {return (int)PathLengthWitkowskiSegment;};
   int RadiSkupinuCvorova(int i, int j, int *brojskupine);
   int getPathLength_forward() {return (int)PathLength_forward;};
   I_point *GetPathForward(){return path_forward;}; //veliko G je razlika
   int getPathLength_backward() {return (int)PathLength_backward;};
   I_point *GetPathBackward(){return path_backward;};  //veliko G je razlika
   I_point *GetPath(){return path;};  //veliko G je razlika
//funkcije za dohvacanje internih podataka
   int GetMapSizeX() {return (int)MapSizeX;};
   int GetMapSizeY(){return (int)MapSizeY;};
   DStarCell **GetMap(){return map;};

  DStar(int size_x, int size_y)
  {
	  blacklist.reserve(300);
          if(size_x>0 && size_y>0)
          {
              MapSizeX=size_x; MapSizeY=size_y;
              map = (DStarCell **)malloc(size_x*sizeof(DStarCell *)) ;
               for (int i=0; i<size_x; i++)
                  map[i]=new DStarCell[size_y];
                  
	      glava = NULL; //open list
      	      path = (I_point *)malloc(MAXORI*size_x*size_y*sizeof(I_point)) ;
            

//for two-way D*
	      glava_reverse = NULL;
	     //for witkowski
	      ulaz_forward = NULL;
		  ulaz_backward = NULL;
		  izlaz_forward = NULL;
		  izlaz_backward = NULL;
		  novi_ulaz_forward= NULL;		
		  novi_ulaz_backward= NULL;		
		  novi_izlaz_forward= NULL;		
		  novi_izlaz_backward= NULL;
		  		
			  path_forward = (I_point *)malloc(size_x*size_y*sizeof(I_point)) ;
			  path_backward = (I_point *)malloc(size_x*size_y*sizeof(I_point)) ;
			  path_witkowski = (I_point *)malloc(size_x*size_y*sizeof(I_point)) ;
			  path_witkowski_segment = (I_point *)malloc(size_x*size_y*sizeof(I_point)) ;
			  path_witkowski_segment_old = (I_point *)malloc(size_x*size_y*sizeof(I_point)) ;
			  sve_konkavne_tocke = (I_point *)malloc(size_x*size_y*sizeof(I_point)) ;
			  pathinicijalni = (I_point *)malloc(size_x*size_y*sizeof(I_point)) ;
	      pathstari = (I_point *)malloc(size_x*size_y*sizeof(I_point)) ;
	      minfpoints= (I_point *)malloc(size_x*size_y*sizeof(I_point)) ;
	      indicesocc = (I_point *)malloc(size_x*size_y*sizeof(I_point)) ;
              reset();
              
              
                              //log things   
	       izracuni = (int *)malloc(size_x*size_y*sizeof(int)) ;
		   azurirani_polje = (int *)malloc(size_x*size_y*sizeof(int)) ;
              f_putanje = (int *)malloc(size_x*size_y*sizeof(int)) ;
              f_biased_putanje = (int *)malloc(size_x*size_y*sizeof(int)) ;
              h_putanje = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      broj_iteracija = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      broj_cvorova_na_listi = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      max_broj_cvorova_na_listi = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      cijena_puta = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      duljina_puta = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      duljina_puta_um = (double *)malloc(size_x*size_y*sizeof(double)) ;
	      duljina_wit_puta_um = (double *)malloc(size_x*size_y*sizeof(double)) ;
	      izracuniWitkowski = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      vremena_skupine = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      broj_iteracijaWitkowski = (int *)malloc(size_x*size_y*sizeof(int)) ;
		  azurirani_forward_polje = (int *)malloc(size_x*size_y*sizeof(int)) ;
		  azurirani_backward_polje = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      broj_cvorova_na_listi_forward = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      broj_cvorova_na_listi_backward = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      max_broj_cvorova_na_listi_backward = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      max_broj_cvorova_na_listi_forward = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      cijena_puta_forward = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      duljina_puta_forward = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      cijena_puta_backward = (int *)malloc(size_x*size_y*sizeof(int)) ;
	      duljina_puta_backward = (int *)malloc(size_x*size_y*sizeof(int)) ;


          }
          else
          {
            printf("DStarOptim> Invalid map size");
            //fprintf(stderr,"AStarOptim> Invalid map size");
            //exit(1);
          }                 

        printf("The DStar object is created\n");
  };


  
  ~DStar()
  {
  Free();
  printf("DStar destroyed");

  };

};




