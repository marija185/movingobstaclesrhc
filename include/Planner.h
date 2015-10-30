
//created 15.06.2003

#ifndef PLANNER_H
#define PLANNER_H
#include "Params.h" //za includove malloc stdio...
#include <algorithm>
//#include "DStar.h"    //Params.h

struct R_point{
	double  x, y, th;
};

struct I_point{
	int  x, y, th;
	bool operator == (I_point a){
		if (x==a.x && y==a.y)
			return 1;
			else
				return 0;
	}


	int operator - (I_point a){
		return std::max(abs(x- a.x),abs(y-a.y));
	}
};

struct ori_point{
	int  travcost, h, intlo, intup;
};

//robot maska se dobiva kao kvadratna maska (za sada)
//RR polumjer robota je definiran u dynamic window headeru

class Planner{
  public:
	  I_point *path; //path dobiven iz DS
	  ori_point *ori;//path from ori search
	  R_point     *path_r; //int to real
  int         path_length;
  int      path_length_max;
  int ciklus_col_points; //zapamti ciklus u kojem se pojavljuje col point
  double search_time;
  int search_explored;
  //ista prica, ali sa razlicitim tipovima podataka
  R_point start_r, global_goal_r; //realne koordinate, apsolutni iznos
  I_point start_i, global_goal_i; //integerske pozicije unutar mape
  int counter_debug;
  double starix,stariy;
  int Sekvenca_izvodjenja();
  int RealToInt(R_point &real_point, I_point &int_point, double map_size_x, double map_size_y, double map_home_x, double map_home_y, double cell_dim);
  int RealToReal(R_point &real_point, R_point &r_point, double map_size_x, double map_size_y, double map_home_x, double map_home_y, double cell_dim);
  int IntToReal(I_point &int_point, R_point &real_point, int map_size_x, int map_size_y, double map_home_x, double map_home_y, double cell_dim);
  int IntToRealPath(int map_size_x, int map_size_y, double map_home_x, double map_home_y, double cell_dim);
  void Puni(int i, int j, int star_size_x, int star_size_y, int robot_mask, int cost_mask);
  void Prazni(int i, int j,I_point *pamtiindekse,int *brojpam, int star_size_x, int star_size_y, int robot_mask, int cost_mask, bool startzauzet);
  void PrazniCostMasku(I_point *pamtiindekse, int brojpam, int star_size_x, int star_size_y, int cost_mask);
  
  int GetPathLength(){return path_length;};
  R_point *GetPath(){return path_r;};
   R_point GetGoal(){ return global_goal_r;};
   void SetGlobalGoal(R_point global_goal){ this->global_goal_r=global_goal_r;};
  void reset();
  I_point *pamtiindekse;
  Planner();
  ~Planner(){
  	//free(path);
	//free(path_r);
	  free(pamtiindekse);
	  printf("Planner unisten");
  }
};

#endif


