#ifndef PARAMS_H
#define PARAMS_H
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>  // for atoi(3)
//#include <values.h>  // for MAXINT
#include <limits.h>  // for INT_MAX
#include <string.h> /* for strcmp() */
#include <unistd.h> /* for usleep() */

//#ifndef MIN
//#define MIN(a,b) ((a < b) ? (a) : (b))
//#endif
//#ifndef MAX
//#define MAX(a,b) ((a > b) ? (a) : (b))
//#endif

#define PLAYER_LASER_MAX_SAMPLES 2000
#define RuS			180./M_PI //(57.295779513)
#define SuR			M_PI/180. //(0.0174532925)

//logging stuff
#define REALLOC_SIZE_INCREMENT 100   //resizing is done every 10sec
#define DEFAULT_REALLOC_SIZE 1000

//maximal velocities in milimeters!!! 	
//#define	V_MAX		400. //total max velocity
//#define DV_MAX		300. //total max acceleration
//#define V_MIN		-400.
//#define	VY_MAX		400. //omnidrive
//#define	VX_MAX		400. 
//#define VX_MIN		-400.
//#define VY_MIN		-400.
//#define DVX_MAX		300.
//#define DVY_MAX		300. //omnidrive

//#define	V_MAX		200. //total max velocity
//#define DV_MAX		150. //total max acceleration
//#define V_MIN		0.
//#define	VY_MAX		200. //omnidrive
//#define	VX_MAX		200. //diff
//#define VX_MIN		0.
//#define VY_MIN		0.
//#define DVX_MAX		150. //diff
//#define DVY_MAX		150. //omnidrive
extern double V_MAX, DV_MAX, V_MIN, VY_MAX, VX_MAX, VX_MIN, VY_MIN, DVX_MAX, DVY_MAX, W_MAX, W_MIN, DW_MAX;


//radians
//#define W_MAX		(30.*M_PI/180.)
//#define W_MIN		(-30.*M_PI/180.)
//#define DW_MAX		(20.*M_PI/180.)
//duration of a time step in seconds
#define STEP		0.1

//robot (1) or simulator (0) or simulator with the localization and single robot (2)
#define ROBOT 2
//slowing down for no path by ramp (1) by circular ramp (2) or by breaks (0)
#define NO_PATH_RAMP 2 
//waiting for some number of cycles in no_path state
#define NO_PATH_COUNTER 3 //16
//using the goal orientation
#define THETAGOAL 1
//not using odometry topic 1 (NO_LASER must be 1), next position is calculated according to the model
#define IDEAL_MODEL 0
//not using laser readings, only static map - 1, 0 is using laser 
#define NO_LASER 0

//width of the robot (acc. y axis)
#define ROBOT_MASK (ceil(RR/CELL_DIM))
//length of the robot (acc. x axis)
#define ROBOT_MASKY (ceil(RRY/CELL_DIM))
//using higher costs around obstacles
#define COST_MASK 0//8//4//2 //(ROBOT_MASKY-ROBOT_MASK)
//#define COST_MASK (ROBOT_MASKY-ROBOT_MASK+2)
//#define COST_MASK (ROBOT_MASK+2)
//podebljavam taj jedan cost_mask da ne bude 2 nego 3 (EMPTY+COST_MASK+DEBEL), stavi u 0 kad ne zelis koristiti
#define DEBEL 0//4//10//10
#define LOW_COST 0
 //maska oko prepreka upisana u dstar mapu kao vrijednost koja se od EMPTY (=1) inkrementalno povecava sto je celija blize prepreci (4 znaci 4 celija oko prepreke)

#define DSTAR 1
// 3D search
#define DSTAR3D 0
// 3D search plus orientations (use with DSTAR3D 1 and USE3DCOST 0)
#define DSTAR3DORI 0
//cost traversals in 3d, influence only for 3D search
#define USE3DCOST 0
//maxdistance cost used in cells
#define MAXDISTANCE (ROBOT_MASKY-1)
// interpolation of cost of 2D search
#define DSTARINT 1
// interpolation of cost of 3D search, use only for 3D search
#define DSTAR3DINT 0
//test full3d search with ORI costs
#define TEST3DSEARCH 0
//koristenje reverznog D* prije D*
#define DSTAR_REVERSE 0
//iscrpno pretrazivanje D* algoritma (reverzni vec je)
#define ISCRPNI 1
//bez koristenja heuristike, za jedan i za drugi algoritam
#define BEZ_HEU 1
//koristenje kombinacije witkowskog i D*, ono prije iz clanka s ecmr-a
#define KOMBINACIJA 0

//kad nema kartu onda prepoznaje zidove 1
#define TRAZI_ZID 0
//kad roboti komuniciraju preko datoteke
#define DATOTEKA 0

#define VERBOSE_DYNAMIC_WINDOW 0
#define VERBOSE_DYNAMIC_WINDOW_WARNING 1

#define LASER_RANGE_MAX 3000.   //ovo koristi WH za spremanje laserskih podataka i DW i GM za provjeru
#define RES 0.008727 //radijani: svakih tolko radijana je nova zraka, za player-stage

//dimenzija celija
#define CELL_DIM  100.//25.//50.//100.

//grid occupancy treshold
#define GRID_MAP_OCCUPANCY_TRESHOLD 1

//GABARITI ROBOTA
//is the robot rectangular shape (1 yes, 0 circular)
#define RECTANGULAR	0
#define RR			356.//301.406//260.//560.//356.//paper 156//expbig 456.
#define RRY			300.//498.//543.405//840.//598.//paper 798//expbig 698.
#define GOAL_POSITION_TOLERANCE (CELL_DIM/2.*sqrt(2.))
#define THETA_TOLERANCE (5*M_PI/180.)
//reading from the world line file (wld) or from map (png)
#define READWLD 0
//reading from the map topic (png) or parameters
#define LOADMAP 1
#define PRAZAN 1
#if (PRAZAN==1)
#define WORLD "karta/rectangularemptymap.wld"
#else
#define WORLD "karta/sim-WLD.wld"
#endif

#define MAP_GOAL_POSITION_X -17000
#define MAP_GOAL_POSITION_Y 1000

#define MAP_START_POSITION_X -24533
#define MAP_START_POSITION_Y 1074
#define MAP_START_POSITION_TH -64.609

#endif
