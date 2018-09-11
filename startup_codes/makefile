Main.exe		: 	Main.o  transform.o LPAstar.o DStarLite.o gridworld.o graphics.o Util.o PriorityQueue.o
	g++ -g -o Main.exe Main.o transform.o LPAstar.o DStarLite.o gridworld.o graphics.o Util.o PriorityQueue.o -l gdi32 
			
Main.o		:	Main.cpp graphics.h transform.h LPAstar.h DStarLite.h gridworld.h globalvariables.h Util.h PriorityQueue.h
	g++ -g -c -std=c++11 -O0    Main.cpp

transform.o		:	 transform.cpp transform.h
	g++ -g -c -std=c++11 -O0    transform.cpp	

LPAstar.o	:	 LPAstar.cpp LPAstar.h
	g++ -g -c -std=c++11 -O0    LPAstar.cpp

DStarLite.o	:	 DStarLite.cpp DStarLite.h
	g++ -g -c -std=c++11 -O0    DStarLite.cpp

gridworld.o	:	 gridworld.cpp gridworld.h
	g++ -g -c -std=c++11 -O0    gridworld.cpp

graphics.o		:	 graphics.cpp graphics.h
	g++ -g -c -std=c++11 -O0    graphics.cpp

Util.o:		Util.cpp Util.h
	g++ -g -c -std=c++11 -O0    Util.cpp

PriorityQueue.o:	PriorityQueue.cpp PriorityQueue.h
	g++ -g -c -std=c++11 -O0    PriorityQueue.cpp		
		
clean:
	del *.o
	del *.exe

