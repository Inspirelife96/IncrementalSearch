#ifndef __GLOBALVARIABLES_H__
#define __GLOBALVARIABLES_H__


#include <vector>

/*******************************************************************************************************************/
//------------------------------------------
//Flags - enable or disable features


#define INCREMENTAL_SEARCH_ALGORITHM
#define RL_ALGORITHM

#define EIGHT_CONNECTED_GRIDWORLD
//#define FOUR_CONNECTED_GRIDWORLD

//#define SHOW_DEBUG_INFO false
//#define SHOW_DEBUG_INFO true

#define MANHATTAN	1
#define EUCLIDEAN	2

//------------------------------------------


//-------------------------------------------------------------------------------
#ifdef FOUR_CONNECTED_GRIDWORLD
 
	//4-connected gridworld
	#define DIRECTIONS 4
	const struct {
	  int x;
	  int y;
	} neighbours[4]={{0, -1}, {-1, 0}, {1, 0}, {0, 1}};

	/////////////////////////////////////////////////////

#endif

const double SQRT_2 =  1.4142135623731;

//-------------------------------------------------------------------------------
#ifdef EIGHT_CONNECTED_GRIDWORLD

	//8-connected gridworld
	#define DIRECTIONS 8
	
	//movement sequence, used in the journal
	const struct {
	  int x;
	  int y;
	} neighbours[8]={ {-1,-1}, {0, -1}, {1, -1}, 
					{-1, 0}, {1, 0}, 
					{-1, 1}, {0, 1}, {1, 1} };
			
	//clockwise, starting at 3 o'clock			
	//~ const struct {
	  //~ int x;
	  //~ int y;
	//~ } succ[8]={ {1,0}, {1, 1}, {0,1}, {-1, 1}, {-1, 0}, {-1,-1}, {0, -1}, {1, -1} };
		
#endif
//-------------------------------------------------------------------------------

//------------------------------------------
	
extern int numberOfExpandedStates;
extern int numberOfVertexAccesses;
extern int maxQLength;	
extern int qLengthAfterSearch;
	
extern bool MAP_INITIALISED;
extern bool PRECALCULATED_GRIDWORLD_READY;

//~ extern bool USE_EUCLIDEAN_DISTANCE_HEURISTIC;
extern unsigned int HEURISTIC;

//Robot soccer dimensions	
extern int GRIDWORLD_ROWS; 
extern int GRIDWORLD_COLS;

//////////////////////////////////////////////////////////////////////////////
//REINFORCEMENT LEARNING

//extern int MAX_ACTIONS;

//end_REINFORCEMENT LEARNING
//////////////////////////////////////////////////////////////////////////////
using namespace std;

enum cellType{TRAVERSABLE=0, BLOCKED=1, UNKNOWN=9};
enum vertexStatus{UNEXPLORED=0, EXPANDED=1, ACCESSED=2};

 struct CellPosition
{
	int row;
	int col;
};

 struct Coordinates
{
	int x, y;
};


typedef struct {
  int y;
  int x;
} loc_t;


struct vertex
{
	
#ifdef INCREMENTAL_SEARCH_ALGORITHM	
    double rhs;
    double g;
	 int c;
	 double h;
	 double f;
	 double key[2];
	 vertex* move[DIRECTIONS]; 
    double linkCost[DIRECTIONS];	
#endif	
	
	
	
	
#ifdef RL_ALGORITHM
	
	#ifdef EIGHT_CONNECTED_GRIDWORLD
    double Q[8];
		
	#elif FOUR_CONNECTED_GRIDWORLD
    double Q[4];
	#endif
	
	 double sumQ;
	 double maxQ;
	
	 int reward;
#endif	
	
	 //--------------------------------------------------------------------------------- 
	 //TYPE: 0 - traversable, 1 - blocked, 9 - unknown, 6 - start vertex, 7 - goal vertex
    char type; 
	 //---------------------------------------------------------------------------------
	 int row;
	 int col;
	 char status; 
	 
	 int x1,y1,x2,y2;
	 Coordinates centre; //centre x, centre y
}; 

extern int MAX_MOVES;

extern vector<vector<vertex> > map;
extern vertex startVertex;
extern vertex goalVertex;



typedef struct vector<CellPosition> PathType;

/*******************************************************************************************************************/
extern int fieldX1, fieldY1, fieldX2, fieldY2; //playing field boundaries
extern float WORLD_MAXX;
extern float WORLD_MAXY;
// colour constants
extern int BACKGROUND_COLOUR;
extern int LINE_COLOUR;

extern int cellWidth;
extern int cellHeight;

/********************************************************************************************************************/
//8-connected gridworld
#define DIRECTIONS 8 //clockwise sequence of moves (8-connected gridworld)

#define INF  1000000

struct LpaStarCell
{
    //LpaStarCell* move[DIRECTIONS];
    //LpaStarCell* predecessor[DIRECTIONS];
	// double linkCost[DIRECTIONS];
    LpaStarCell* parent;
    LpaStarCell* trace;
    short obstacle;
    int x, y;

	 double g;
    double rhs;
	 double h;
    double key[2];

    //additional part
    double s_g;    // 起点到此点的距离( 由g和h可以得到f，此处f省略，f=g+h )  
    double s_h;    // 启发函数预测的此点到终点的距离  
   // struct AStarNode * s_parent;    // 父节点  
    int s_is_in_closetable;     // 是否在close表中  
    int s_is_in_opentable;    
	
	 //~ int g;
    //~ int rhs;
    //~ int key[2];
	
	
	 //---------------------
	 //TYPE: 0 - traversable, 1 - blocked, 9 - unknown, 6 - start vertex, 7 - goal vertex
	 char type;
	 //----------------------
	

	
    int generated;
    int heapindex;
};


struct AStarNode
{
    short obstacle;

 	int s_x;    // 坐标(最终输出路径需要)  
    int s_y;  
    double s_g;    // 起点到此点的距离( 由g和h可以得到f，此处f省略，f=g+h )  
    double s_h;    // 启发函数预测的此点到终点的距离  
    struct AStarNode * s_parent;    // 父节点  
    int s_is_in_closetable;     // 是否在close表中  
    int s_is_in_opentable;      // 是否在open表中 
	
	 //---------------------
	 //TYPE: 0 - traversable, 1 - blocked, 9 - unknown, 6 - start vertex, 7 - goal vertex
	 char type;
	 //----------------------
	

	
    int generated;
    int heapindex;
};

extern bool SHOW_MAP_DETAILS;

/********************************************************************************************************************/



#endif
