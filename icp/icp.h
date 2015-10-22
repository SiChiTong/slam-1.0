#ifndef ICP2_H
#define ICP2_H

#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <cmath>
#include <cv.h>
#include <highgui.h>
#include <cstdlib>
#include "CTimestamp.h"

#include <pthread.h>

#define LASERRAYS 361
#define POPULATION 2000 //me 1000 kalo, std 5000 arketo-15000 elafra veltiwsi alla poli argo
#define ANAPOSO 10 //15 std, 18-20 ginetai me bestf=40, 30 genies, 3000 population
#define DIASPORA 20//20
#define DIASPORAANGLE 30000.0//30000.0

#define DIASPORAANGLEPLUS 10000.0/DIASPORAANGLE/2

#define d_LASERANG 180.0
#define d_LASERANGRAD 3.14159265
#define d_LASERMAX 8.30//7.30

#define ocgd 0.07 //0.03

#define EXPANSION 300				//me ti rithmisi gia gwnies kales kai 3000 population, 6 anaposo, 20,30000 diaspores kai 0.2 gwnia kalo 						//alla kanei lathos stis eutheies (0.07 ocgd) xwris 90-270

						//me 90-270 x10 kai allagmenoi oi rithmoi xrwmatismou-kalitero stin eutheia xeirotero stis 							//gwnies

struct point{
	int x,y;
	float theta;
	float getDist(const point &p);
	void transform(point *p,int dx,int dy,float theta);
};

struct laserScan{
	float scan[LASERRAYS];
	point p[LASERRAYS];
};

struct laserScanLog{
	std::vector<laserScan> scans;
};

struct transformation{
	int dx,dy;
	float theta;
	const transformation & operator+=(const transformation &op2);
	void clear(void);
};

struct pair{
	point p1,p2;
};

struct person{
	transformation t;
	float fitness;
};

class icp{

	laserScanLog lscans;
	laserScan previous;
	laserScan original;
	laserScan transformed;
	transformation T;

	unsigned int nextScanIndex;

public:

	unsigned int MAPSIZEX,MAPSIZEY,STARTX,STARTY;

	person popul[POPULATION];
	float bestFitness;
	transformation bestTransformation;
transformation old;

	transformation robotPose;
	void updateMapColor(void);
	void initialisePopulation(void);
 float v;
int flag;
int flag2;
float max3,max4,min3,min4;
	void updateFitnesses(void);
	void transformGA(const transformation &t);
	void showScansGA(void);
	void runIteration(void);

	void runGA(void);

	bool fixNewScans(void);

	void updateGAMap(void);

	void fixNewGeneration(void);

	std::string mapTitle;
	IplImage *mapDisplay;
	unsigned char **map;
	unsigned char *data;	

	void fixLastGeneration(void);

	icp();
	~icp(){}
	void readScans(char *log);
	
	void showMap(void);

	float meanIcpTime;
	
	
/*
 * 
 * name: expandMap
 * id=0 :right / id=1 : down / id=2:left / id=3:up
 */
	void expandMap(unsigned int id);
};

#endif

