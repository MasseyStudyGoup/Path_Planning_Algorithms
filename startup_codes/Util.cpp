#include "Util.h"

#include <iostream>
#include <queue>
#include <windows.h>

#include "globalVariables.h"
#include "LPAstar.h"
#include "gridworld.h"
#include "PriorityQueue.h"
#include "DStarLite.h"

using namespace std;

int getKey() {

	if (GetAsyncKeyState(VK_UP) < 0) { //UP ARROW
		return 200;
	}

	if (GetAsyncKeyState(VK_DOWN) < 0) { //DOWN ARROW
		return 201;
	}

	if (GetAsyncKeyState(VK_F3) < 0) {
		return 103;
	}

	if (GetAsyncKeyState(VK_F4) < 0) {
		return 104;
	}

	if (GetAsyncKeyState(VK_F5) < 0) {
		return 105;
	}

	if (GetAsyncKeyState(VK_F6) < 0) {
		//execute A* with strict expanded list
		return 106;
	}
	if (GetAsyncKeyState(VK_F7) < 0) {
		//execute LPA*
		return 107;
	}
	if (GetAsyncKeyState(VK_F8) < 0) {
		//execute D*Lite
		return 108;
	}

	//copy display map to algorithm's maze
	if (GetAsyncKeyState(VK_F9) < 0) {
		return 109;
	}

	//copy algorithm's maze to display map
	if (GetAsyncKeyState(VK_F10) < 0) {
		return 110;
	}

	if (GetAsyncKeyState(0x53) < 0) { //S-key (start cell)
		return 6;
	}

	if (GetAsyncKeyState(0x58) < 0) { //X-key (goal cell)
		return 7;
	}

	if (GetAsyncKeyState(0x42) < 0) { //B-key (block cell)
		return 1;
	}

	if (GetAsyncKeyState(0x47) < 0) {  //G-key
		return 9;
	}

	if (GetAsyncKeyState(0x48) < 0) {  //H-key
		return 10;
	}

	if (GetAsyncKeyState(0x4B) < 0) {  //K-key
		return 11;
	}

	if (GetAsyncKeyState(0x55) < 0) { //U-key (Unblock cell)
		return 12;
	}

	if (GetAsyncKeyState(0x50) < 0) { //P-key (position of cells)
		return 14;
	}

	if (GetAsyncKeyState(0x43) < 0) { //C-key (connections of cells)
		return 15;
	}

	if (GetAsyncKeyState(0x4D) < 0) { //M-key (entire map connections)
		return 16;
	}

	if (GetAsyncKeyState(0x52) < 0) { //R-key (REINFORCEMENT LEARNING - reward values)
		return 17;
	}

	if (GetAsyncKeyState(0x51) < 0) { //Q-key (REINFORCEMENT LEARNING - maxQ-values)
		return 18;
	}

	if (GetAsyncKeyState(VK_SPACE) < 0) { //SPACE BAR
		return 1000;
	}
	short state = GetAsyncKeyState(VK_RETURN);
	if (GetAsyncKeyState(VK_RETURN) < 0) { //ENTER KEY
		return 1001;
	}

	return -1;
}

void copyVertex(vertex* pTarget, vertex* pSource){
	if (pTarget == nullptr || pSource == nullptr)
		return;
	memcpy(pTarget, pSource, sizeof(vertex));
	for (int i=0; i<DIRECTIONS; i++)
		pTarget->move[i] = nullptr;
}


void testPriorityQueue() {
	double a[8];
	double b[8];
	memcpy(a, b, sizeof(b));

	MazeCell v1;
	v1.row = 0;
	v1.col = 0;
	v1.key[0] = 1;
	v1.key[1] = 1;

	MazeCell v2;
	v2.row = 0;
	v2.col = 1;
	v2.key[0] = 2;
	v2.key[1] = 1;

	MazeCell v3;
	v3.row = 1;
	v3.col = 0;
	v3.key[0] = 1;
	v3.key[1] = 2;

	PriorityQueue pqv;

	pqv.insert(&v1);
	pqv.insert(&v2);
	pqv.insert(&v3);
	pqv.find(1, 0);
	MazeCell* pV = pqv.top();
	MazeCell* p1 = pqv.pop();
	pV = pqv.pop();
}


void test() {
	double key1[2] = {1, 2};
	double key2[2] = {3, 4};
	DStarLite::copyKey(key1, key2);
	DStarLite dsLite(7,5);
	//dsLite.initialize(2, 1, 5, 3);
}
