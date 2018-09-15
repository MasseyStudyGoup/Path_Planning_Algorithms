///////////////////////////////////////////////////////////////////////////////////////////
//
//
//  
//                        
//
//	 	      Program Name: Incremental Search 
//	 	       Description: start-up codes for simulating LPA* and D*Lite
//                        - implements a gridworld class that loads a gridworld from file, and is
//                          modifiable through a user-interface 
//
//        Run Parameters: 
//
//    Keys for Operation: 
//
//	 		        History:  date of revision
//                         13/Aug/2017
//                         28/July/2015  
//                         03/Aug/2014  
//
//      Start-up code by:    n.h.reyes@massey.ac.nz
//
///////////////////////////////////////////////////////////////////////////////////////////

#include <windows.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <deque>
#include <set>
#include <vector>
#include <algorithm>

//-------------------------
#include "globalVariables.h"
#include "transform.h"
#include "graphics.h"
#include "DStarLite.h"
#include "LPAstar.h"
#include "IdaStar.h"
#include "gridworld.h"
#include "Util.h"

// colour constants
int BACKGROUND_COLOUR;
int LINE_COLOUR;

int robotWidth;
int GRIDWORLD_ROWS; //duplicated in GridWorld
int GRIDWORLD_COLS; //duplicated in GridWorld

//----------------------------
unsigned int HEURISTIC;
//~ bool USE_EUCLIDEAN_DISTANCE_HEURISTIC;
int numberOfExpandedStates;
int MAX_MOVES;
int maxQLength;
int qLengthAfterSearch;

vector<vertex*> g_changed;
char g_algorithm = ALGO_DSTARTLIET; //ALGO_LPASTAR; //ALGO_DSTARTLIET //ALGO_IDASTAR;
///////////////////////////////////////////////////////////////////////////////
DStarLite* g_dsl = nullptr;
LpaStar* g_lpas = nullptr;
IdaStar* g_idas = nullptr;
GridWorld grid_world;
bool findPath();
bool SHOW_MAP_DETAILS;
///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////
void updateData(bool fromMazeToMap) {
	if (g_lpas != nullptr && g_algorithm == ALGO_LPASTAR) { //LP A*
		for (int i = 0; i < grid_world.getGridWorldRows(); i++) {
			for (int j = 0; j < grid_world.getGridWorldCols(); j++) {
				if (fromMazeToMap) {
					g_lpas->m_maze[i][j].copyTo(&grid_world.map[i][j]);
				} else {
					g_lpas->m_maze[i][j].copyFrom(grid_world.map[i][j]);
				}
			}
		}
	} else if (g_dsl != nullptr) {
		for (int i = 0; i < grid_world.getGridWorldRows(); i++) {
			for (int j = 0; j < grid_world.getGridWorldCols(); j++) {
				if (fromMazeToMap) {
					g_dsl->m_maze[i][j].copyTo(&grid_world.map[i][j]);
				} else {
					g_dsl->m_maze[i][j].copyFrom(grid_world.map[i][j]);
				}
			}
		}
	} else if (g_idas != nullptr){
		for (int i = 0; i < grid_world.getGridWorldRows(); i++) {
			for (int j = 0; j < grid_world.getGridWorldCols(); j++) {
				if (fromMazeToMap) {
					g_idas->m_maze[i][j].copyTo(&grid_world.map[i][j]);
				} else {
					g_idas->m_maze[i][j].copyFrom(grid_world.map[i][j]);
				}
			}
		}
	}
}
void updateH() {
	if (g_algorithm == ALGO_LPASTAR) {
		g_lpas->updateH();
	} else {
		g_dsl->updateH();

	}
	updateData(true); //from maze to map
}

void updateKey() {
	if (g_algorithm == ALGO_LPASTAR) {
		g_lpas->updateKey();
	} else {
		g_dsl->updateKey();
	}
	updateData(true); //from maze to map
}

//return nullptr if selected an vertex on the boundry
vertex* getSelectedVertex(int rowSelected, int colSelected) {
	if ((rowSelected > 1) && (rowSelected < GRIDWORLD_ROWS) && (colSelected > 1)
			&& (colSelected < GRIDWORLD_COLS)) {
		return grid_world.getVertex(rowSelected - 1, colSelected - 1);
	}

	return nullptr;
}

void drawInformationPanel(int x, int y, char* info) {
	///////////////////////////////////////////////////////////////////////////////////////////
	settextstyle(SMALL_FONT, HORIZ_DIR, 4);
	settextjustify(LEFT_TEXT, CENTER_TEXT);
	setcolor(YELLOW);
	outtextxy(x, y, info);
	///////////////////////////////////////////////////////////////////////////////////////////
}

void runSimulation(char *fileName) {
	WorldBoundaryType worldBoundary; //duplicated in GridWorld
	DevBoundaryType deviceBoundary; //duplicated in GridWorld
	bool ANIMATE_MOUSE_FLAG = false;
	bool validCellSelected = false;
	static BOOL page = false;
	int mX, mY;
	float worldX, worldY;
	worldX = 0.0f;
	worldY = 0.0f;

	int action = -1;
	//-----------------------
	CellPosition p;
	int rowSelected, colSelected;
	//-----------------------
	rowSelected = -1;
	colSelected = -1;

	int mouseRadius = 1;

	srand(time(NULL));  // Seed the random number generator

	//Initialise the world boundaries
	grid_world.initSystemOfCoordinates();
	grid_world.loadMapAndDisplay(fileName);
	grid_world.initialiseMapConnections();

	vertex start = grid_world.getStartVertex();
	vertex goal = grid_world.getGoalVertex();
	cout << "(start.col = " << start.col << ", start.row = " << start.row << ")"
			<< endl;
	cout << "(goal.col = " << goal.col << ", goal.row = " << goal.row << ")"
			<< endl;

	//----------------------------------------------------------------
	worldBoundary = grid_world.getWorldBoundary();
	deviceBoundary = grid_world.getDeviceBoundary();
	GRIDWORLD_ROWS = grid_world.getGridWorldRows();
	GRIDWORLD_COLS = grid_world.getGridWorldCols();

	// keep running the program until the ESC key is pressed   
	while ((GetAsyncKeyState(VK_ESCAPE)) == 0) {
		setactivepage(page);
		cleardevice();

		action = getKey();
		if (action == 105) {
			SHOW_MAP_DETAILS = true;
		} else if (action == 104) {
			SHOW_MAP_DETAILS = false;
		} else {
			//do nothing
		}

		if (SHOW_MAP_DETAILS) {
			grid_world.displayMapWithDetails();
		} else {
			grid_world.displayMap();
		}

		vertex* v = nullptr;
		switch (action) {
		case 1000:
			break;
		case 1001:  //ENTER KEY
			if (findPath()) {
				g_changed.empty();
				updateData(true); //from maze to map
				grid_world.setSearchStatus(1);
			} else {
				grid_world.setSearchStatus(-1);
			}
			break;
		case 1: //Block selected cell
			v = getSelectedVertex(rowSelected, colSelected);
			if (v != nullptr && v->type != T_BLOCKED) {
				grid_world.setMapTypeValue(rowSelected - 1, colSelected - 1,
						T_BLOCKED);
				grid_world.initialiseMapConnections();
				//re-search
				grid_world.setSearchStatus(0);
				//store the changed vertex
				g_changed.push_back(v);
			}
			rowSelected = -1;
			colSelected = -1;
			action = -1;
			break;
		case 103:
			if (grid_world.getSearchStatus() == 1){
				if (g_algorithm == ALGO_IDASTAR){
					IdaStarPath* mazePath = g_idas->getPath();
					vector<vertex*> path;
					for (int i = 0; i < mazePath->cells.size(); i++) {
						path.push_back(grid_world.getVertex(mazePath->cells[i]->row, mazePath->cells[i]->col));

					}
					grid_world.displayPath(path);
				}
				else{
					grid_world.displayPath();
				}
			}
			break;
		case 105:
			grid_world.displayMapWithKeyDetails();
			break;
		case 106:
			if (g_algorithm != ALGO_IDASTAR) {
				g_algorithm = ALGO_IDASTAR;
				grid_world.setSearchStatus(0);
			}
			break;
		case 107:
			if (g_algorithm != ALGO_LPASTAR) {
				g_algorithm = ALGO_LPASTAR;
				grid_world.setSearchStatus(0);
			}
			break;
		case 108:
			if (g_algorithm != ALGO_DSTARTLIET) {
				g_algorithm = ALGO_DSTARTLIET;
				grid_world.setSearchStatus(0);
			}
			break;
		case 15:
			if (rowSelected != -1 && colSelected != -1) {
				grid_world.displayVertexConnections(colSelected - 1,
						rowSelected - 1);
				rowSelected = -1;
				colSelected = -1;
			} else {
				cout
						<< "invalid new START vertex, please select a new START vertex first."
						<< endl;
				break;
			}
			action = -1;
			break;
		case 16:
			if (grid_world.isGridMapInitialised()) {
				grid_world.displayMapConnections();
			}
			action = -1;
			break;
		case 6: //set cell as new START vertex
		{
			// retrieve current START vertex
			vertex s = grid_world.getStartVertex();
			if ((rowSelected > 1) && (rowSelected < GRIDWORLD_ROWS)
					&& (colSelected > 1) && (colSelected < GRIDWORLD_COLS)) {
				if ((s.row != -1) && (s.col != -1)) {
					//set current START VERTEX to an ordinary TRAVERSABLE CELL
					grid_world.setMapTypeValue(s.row, s.col, '0');
					grid_world.initialiseMapConnections();
				} else {
					cout << "invalid START vertex" << endl;
					break;
				}
			}
			//--------------------------------------------
			//set selected cell as the NEW START VERTEX
			if ((rowSelected > 1) && (rowSelected < GRIDWORLD_ROWS)
					&& (colSelected > 1) && (colSelected < GRIDWORLD_COLS)) {
				grid_world.setMapTypeValue(rowSelected - 1, colSelected - 1,
						'6');
				s.row = rowSelected - 1;
				s.col = colSelected - 1;
				grid_world.setStartVertex(s);
				grid_world.setSearchStatus(0);

				rowSelected = -1;
				colSelected = -1;
			} else {
				cout
						<< "invalid new START vertex, please select a new START vertex first."
						<< endl;
				break;
			}
			//--------------------------------------------
			action = -1;
			break;
		}
		case 7: //set cell as new GOAL vertex
		{
			// retrieve current GOAL vertex
			vertex s = grid_world.getGoalVertex();
			if ((rowSelected > 1) && (rowSelected < GRIDWORLD_ROWS)
					&& (colSelected > 1) && (colSelected < GRIDWORLD_COLS)) {
				if ((s.row > 1) && (s.row < GRIDWORLD_ROWS) && (s.col > 1)
						&& (s.col < GRIDWORLD_COLS)) {
					//set current GOAL VERTEX to an ordinary TRAVERSABLE CELL
					grid_world.setMapTypeValue(s.row, s.col, '0');
					//ok, proceed
				} else {
					cout << "invalid GOAL vertex" << endl;
					action = -1;
					break;
				}
			}
			//--------------------------------------------
			//set selected cell as the NEW GOAL VERTEX
			if ((rowSelected > 1) && (rowSelected < GRIDWORLD_ROWS)
					&& (colSelected > 1) && (colSelected < GRIDWORLD_COLS)) {
				grid_world.setMapTypeValue(rowSelected - 1, colSelected - 1,
						'7');
				s.row = rowSelected - 1;
				s.col = colSelected - 1;
				grid_world.setSearchStatus(0);
				grid_world.setGoalVertex(s);
				grid_world.initialiseMapConnections();

				rowSelected = -1;
				colSelected = -1;
			} else {
				cout
						<< "invalid new GOAL vertex, please select a new GOAL vertex first."
						<< endl;
				action = -1;
				break;
			}
			action = -1;
			break;
		}
		case 109:
			updateData(false); //from map to maze
			action = -1;
			break;
		case 110:
			updateH();
			action = -1;
			break;
		case 9: //display g-values only
			//(bool display_g, bool display_rhs, bool display_h, bool display_key)
			grid_world.displayMapWithSelectedDetails(true, false, false, false);
			action = -1;
			break;
		case 10: //display h-values only
			grid_world.displayMapWithSelectedDetails(false, false, true, false);
			action = -1;
			break;
		case 11: //display key-values only
			updateKey();
			grid_world.displayMapWithSelectedDetails(false, false, false, true);
			action = -1;
			break;
		case 12: //make cell Traversable
			v = getSelectedVertex(rowSelected, colSelected);
			if (v->type != T_TRAVERSABLE) {
				grid_world.setMapTypeValue(rowSelected - 1, colSelected - 1,
						T_TRAVERSABLE);
				grid_world.initialiseMapConnections();
				grid_world.setSearchStatus(0);
				g_changed.push_back(v);
			}
			rowSelected = -1;
			colSelected = -1;
			action = -1;
			break;
		case 14:
			grid_world.displayMapWithPositionDetails();
			action = -1;
			break;
			//~ default: //Display grid without details
			//~ grid_world.displayMap();

		};
		//----------------------------------------------------------------------------------------------------------------
		// Mouse handling
		if (mousedown()) {

			ANIMATE_MOUSE_FLAG = true;

			mX = mousecurrentx();
			mY = mousecurrenty();

			//if the goal selected is within the playing field boundaries
			if (mX >= grid_world.getFieldX1() && mX <= grid_world.getGridMaxX()
					&& mY >= grid_world.getFieldY1()
					&& mY <= grid_world.getGridMaxY()) {
				circle(mX, mY, 3);
				validCellSelected = true;
			} else {
				validCellSelected = false;
			}
		} //end of mousedown()

		if (ANIMATE_MOUSE_FLAG) {
			//draw Cross-hair to mark Goal
			setcolor(RED);
			circle(mX, mY, 20);
			line(mX, mY - 20, mX, mY + 20);
			line(mX - 20, mY, mX + 20, mY);
			//end of draw Cross-hair
			// special effect to display concentric circles locating the target
			setcolor(YELLOW);
			if (mouseRadius < 40) {
				mouseRadius += 1;
			}
			circle(mX, mY, mouseRadius);
			//Sleep(50);
			if (mouseRadius >= 40) {
				ANIMATE_MOUSE_FLAG = false;
				mouseRadius = 0;
			}
			//end of special effect
		}

		/////////////////////////////////////////////////////////////////////////////
		char info[80];
		float wX, wY;

		wX = xWorld(worldBoundary, deviceBoundary, mX);
		wY = yWorld(worldBoundary, deviceBoundary, mY);
		sprintf(info, "x: %d, y: %d", mX, mY);
		drawInformationPanel(grid_world.getFieldX2(),
				grid_world.getFieldY1() + textheight("H") * 2, info);

		sprintf(info, "wX: %3.0f, wY: %3.0f", wX, wY);
		drawInformationPanel(grid_world.getFieldX2(),
				grid_world.getFieldY1() + textheight("H") * 5, info);
		/////////////////////////////////////////////////////////////////////////////
		if (validCellSelected) {
			p = grid_world.getCellPosition_markCell(mX, mY);

			rowSelected = p.row;
			colSelected = p.col;

			sprintf(info, "row: %d, col: %d", rowSelected, colSelected);
			drawInformationPanel(grid_world.getFieldX2(),
					grid_world.getFieldY1() + textheight("H") * 6, info);

		}
		setvisualpage(page);
		page = !page;  //switch to another page
	}
}
///////////////////////////////////////////////////////////////////////////////////////
//
// EXAMPLE:  main grid_Dstar_journal.txt MANHATTAN
int main(int argc, char *argv[]) {
	char gridFileName[80];
	string heuristic(argv[2]);
	std::transform(heuristic.begin(), heuristic.end(), heuristic.begin(),
			::tolower);

	if (argc == 3) {
		strcpy(gridFileName, argv[1]);

		//heuristic function selection
		if ((heuristic.compare("euclidean") == 0)
				|| (heuristic.compare("e") == 0)) {
			HEURISTIC = EUCLIDEAN;
			cout << "Heuristics = EUCLIDEAN" << endl;
		}
		if ((heuristic.compare("manhattan") == 0)
				|| (heuristic.compare("m") == 0)) {
			HEURISTIC = MANHATTAN;
			cout << "Heuristics = MANHATTAN" << endl;
		}
	} else {
		cout
				<< "\n==================================================================================="
				<< endl;
		//cout << endl << endl;
		cout
				<< "                 << Incremental Search v.1.0   by n.h.reyes@massey.ac.nz>>                                "
				<< endl;
		cout
				<< "==================================================================================="
				<< endl;
		cout << "Syntax error: Missing parameters:  gridworld heuristic"
				<< endl;
		cout
				<< "\nPlease follow the following syntax:  main <gridworld> {MANHATTAN, EUCLIDEAN}"
				<< endl;
		cout << "\ne.g.  main grid_Dstar_journal.txt MANHATTAN " << endl;
		cout
				<< "\n==================================================================================="
				<< endl;

	}

	int graphDriver = 0, graphMode = 0;

	//initgraph(&graphDriver, &graphMode, "", 1440, 900); // Start Window
	//initgraph(&graphDriver, &graphMode, "", 1280, 1024); // Start Window
	//initgraph(&graphDriver, &graphMode, "", 1360, 768); // Start Window - LAPTOP SCREEN
	initgraph(&graphDriver, &graphMode, "", 1920, 1080); // Start Window - Full-HD

	BACKGROUND_COLOUR = WHITE;
	LINE_COLOUR = GREEN;

	GRIDWORLD_ROWS = 0; //7; //6; //duplicated in GridWorld
	GRIDWORLD_COLS = 0; //15;//13; //duplicated in GridWorld
	SHOW_MAP_DETAILS = false;

	try {
		runSimulation(gridFileName);
	}

	//Type of the exceptions thrown by the standard definitions of operator new and operator new[]
	//when they fail to allocate the requested storage space
	catch (std::bad_alloc & ba) {
		std::cerr << "out of memory caught: " << ba.what() << endl;
	}

	catch (exception & e) {
		cout << "Standard exception: " << e.what() << endl;
	}

	catch (...) {
		cout << "Unknown exception caught!\n";
	}

	cout << "----<< The End.>>----" << endl;

	return 0;
}


bool findPath() { // this findPath is defined and used by the main function.
	vertex start = grid_world.getStartVertex();
	vertex goal = grid_world.getGoalVertex();

	if (g_algorithm == ALGO_LPASTAR) { //LP A*
		g_lpas = new LpaStar(grid_world.getGridWorldRows(),
				grid_world.getGridWorldCols());
		g_lpas->setStart(start.row, start.col);
		g_lpas->setGoal(goal.row, goal.col);
		updateData(false);
		return g_lpas->findPath();
	} else if (g_algorithm == ALGO_DSTARTLIET) { // must be D* star lite
		g_dsl = new DStarLite(grid_world.getGridWorldRows(),
				grid_world.getGridWorldCols());
		g_dsl->setStart(start.row, start.col);
		g_dsl->setGoal(goal.row, goal.col);
		updateData(false); //copy data from map to maze
		return g_dsl->findPath(); //g_dsl is the instance of our defined class DstarLite
		
	} else{
		g_idas = new IdaStar(grid_world.getGridWorldRows(),
				grid_world.getGridWorldCols());
		g_idas->setStart(start.row, start.col);
		g_idas->setGoal(goal.row, goal.col);
		updateData(false); //from map to maze
		return g_idas->findPath();
>>>>>>> master
	}

	return true;
}
