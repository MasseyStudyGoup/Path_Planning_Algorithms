///////////////////////////////////////////////////////////////////////////////////////////
//
//	 	      Program Name: globalVariables 
//
//	 		        History:  date of revision
//                         13/Aug/2017
//                         28/July/2015  
//                         03/Aug/2014  
//
//      Start-up code by:    n.h.reyes@massey.ac.nz
//
///////////////////////////////////////////////////////////////////////////////////////////

#ifndef __GLOBALVARIABLES_H__
#define __GLOBALVARIABLES_H__

#include <vector>

/*******************************************************************************************************************/
//------------------------------------------
//Flags - enable or disable features
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
}neighbours[4]= { {0, -1}, {-1, 0}, {1, 0}, {0, 1}};

/////////////////////////////////////////////////////

#endif

const double SQRT_2 = 1.4142135623731;

//-------------------------------------------------------------------------------
#ifdef EIGHT_CONNECTED_GRIDWORLD

//8-connected gridworld
#define DIRECTIONS 8

//movement sequence, used in the journal
const struct {
	int x;
	int y;
} neighbours[8] = { { -1, -1 }, { 0, -1 }, { 1, -1 }, { -1, 0 }, { 1, 0 }, { -1,
		1 }, { 0, 1 }, { 1, 1 } };

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

#define ALGO_LPASTAR    0x01
#define ALGO_DSTARTLIET 0X02
extern char g_algorithm;
//////////////////////////////////////////////////////////////////////////////
//REINFORCEMENT LEARNING

//extern int MAX_ACTIONS;

//end_REINFORCEMENT LEARNING
//////////////////////////////////////////////////////////////////////////////
using namespace std;
const char T_TRAVERSABLE = '0';
const char T_BLOCKED = '1';
const char T_START = '6';
const char T_GOAL = '7';
const char T_UNKNOWN = '9';

enum cellType {
	TRAVERSABLE = 0, BLOCKED = 1, START = 6, GOAL = 7, UNKNOWN = 9
};
enum vertexStatus {
	UNEXPLORED = 0, EXPANDED = 1, ACCESSED = 2
};

struct CellPosition {
	int row;
	int col;
};

struct Coordinates {
	int x, y;
};

typedef struct {
	int y;
	int x;
} loc_t;

struct vertex {
	double rhs;
	double g;
	// int c;
	double h;
	double f;
	double key[2];
	vertex* move[DIRECTIONS];
	double linkCost[DIRECTIONS];

	//--------------------------------------------------------------------------------- 
	//TYPE: 0 - traversable, 1 - blocked, 9 - unknown, 6 - start vertex, 7 - goal vertex
	char type;
	//---------------------------------------------------------------------------------
	int row;
	int col;
	char status;

	int x1, y1, x2, y2;
	Coordinates centre; //centre x, centre y
};

extern int MAX_MOVES;

extern vector<vector<vertex> > map;
extern vertex startVertex;
extern vertex goalVertex;

extern vector<vertex*> g_changed;

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

extern int NAPOLEON;

/********************************************************************************************************************/
//8-connected gridworld
#define DIRECTIONS 8 //clockwise sequence of moves (8-connected gridworld)

#define INF  1000000

struct MazeCell {
	double linkCost[DIRECTIONS];
	int row, col;
	double g;
	double rhs;
	double h;
	double key[2];
	char type;

	bool equals(MazeCell* other) {
		if (other == nullptr)
			return false;
		return (this->row == other->row) && (this->col == other->col);
	}

	bool equals(int row, int col) {
		return (this->row == row) && (this->col == col);
	}

	void copyFrom(const vertex& v) {
		type = v.type;
		row = v.row;
		col = v.col;
		for (int i = 0; i < DIRECTIONS; i++)
			linkCost[i] = v.linkCost[i];
		//no need to copy other fields
	}

	void copyTo(vertex* v) {
		v->g = g;
		v->rhs = rhs;
		v->h = h;
		v->type = type;
		v->key[0] = key[0];
		v->key[1] = key[1];
		for (int i = 0; i < DIRECTIONS; i++)
			linkCost[i] = v->linkCost[i];
		//no need to copy row and col
	}
};

struct LpaStarCell {
	LpaStarCell* move[DIRECTIONS];

	double linkCost[DIRECTIONS];

	int x, y;

	double g;
	double rhs;
	double h;
	double key[2];

	//---------------------
	//TYPE: 0 - traversable, 1 - blocked, 9 - unknown, 6 - start vertex, 7 - goal vertex
	char type;
	//----------------------

};

extern bool SHOW_MAP_DETAILS;

/********************************************************************************************************************/
#endif
