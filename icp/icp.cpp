#include <icp.h>
#include <icp.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
using namespace std;

const transformation & transformation::operator+=(const transformation &op2){
	dx+=op2.dx;
	dy+=op2.dy;
	theta+=op2.theta;
	return *this;
}

void transformation::clear(void){
	dx=0;
	dy=0;
	theta=0;
}

icp::icp(void){
	robotPose.dx=0;
	robotPose.dy=0;
	robotPose.theta=0;
	
	MAPSIZEX=MAPSIZEY=724;
	STARTX=STARTY=362;
	
	mapTitle="Global Map";
	mapDisplay = cvCreateImage(cvSize(MAPSIZEX,MAPSIZEY), IPL_DEPTH_8U,3);
	cvNamedWindow(mapTitle.c_str(),CV_WINDOW_AUTOSIZE);
	
	map=new unsigned char *[MAPSIZEX];
	for(unsigned int i=0;i<MAPSIZEX;i++)
		map[i]=new unsigned char [MAPSIZEY];
		
	for(unsigned int i=0;i<MAPSIZEX;i++)
		for(unsigned int j=0;j<MAPSIZEY;j++)
			map[i][j]=127;
	
	meanIcpTime=0;
	//min=10000;
//max=-10000;

}

void icp::readScans(char *log){
	using namespace std;
	laserScan singleScan;

	ifstream myfile(log);

	unsigned int counter;

	string num;
	while(!myfile.eof()){
		counter=0;
		while (counter!=361){
			num.clear();
			unsigned char c='.';
			do{
				c=myfile.get();
				num+=c;
			}while(c!=',');
			num.erase(num.size()-1,1);
			counter++;
			istringstream snum(num);
			snum>>singleScan.scan[counter-1];
		}
		myfile.get();
		myfile.get();
		lscans.scans.push_back(singleScan);
	}
	cout<<lscans.scans.size()<<" scans loaded\n";
	myfile.close();

	previous=lscans.scans[0];
	original=lscans.scans[1];

	nextScanIndex=2;
//int max,max2=0;
	for (unsigned int i=0;i<LASERRAYS;i++){
		previous.p[i].theta=-d_LASERANGRAD/2+i*d_LASERANGRAD/LASERRAYS;
		previous.p[i].x=previous.scan[i]/ocgd*cos(previous.p[i].theta);
		previous.p[i].y=previous.scan[i]/ocgd*sin(previous.p[i].theta);
		original.p[i].theta=-d_LASERANGRAD/2+i*d_LASERANGRAD/LASERRAYS;
		original.p[i].x=original.scan[i]/ocgd*cos(original.p[i].theta);
		original.p[i].y=original.scan[i]/ocgd*sin(original.p[i].theta);
	//if (original.p[i].x<max){max=original.p[i].x;}
	//if (original.p[i].y<max){max2=original.p[i].y;}
		
	}
	//std::cout<<max<<"\n";
	//std::cout<<max2<<"\n";
	transformed=original;
//std::cout<<"\nNew scans:";		
//for (unsigned int i=0;i<LASERRAYS;i=i+2){
//std::cout<<"\nROMx(0)("<<i/2<<")<=std_logic_vector(conv_signed("<<original.p[i].x<<",10));";
//std::cout<<"\nOriginal.p["<<i<<"].x:"<<original.p[i].x;
//std::cout<<"\nOriginal.p["<<i<<"].y:"<<original.p[i].y;
//std::cout<<"\nOriginal.scan["<<i<<"]:"<<original.scan[i];
//}
//for (unsigned int i=0;i<LASERRAYS;i=i+2){
//std::cout<<"\nROMy(0)("<<i/2<<")<=std_logic_vector(conv_signed("<<original.p[i].y<<",10));";
//}
}

float point::getDist(const point &p){
	return (this->x-p.x)*(this->x-p.x)+(this->y-p.y)*(this->y-p.y);
}

void point::transform(point *p,int dx,int dy,float theta){
	int tempx=this->x,tempy=this->y;
	p->x=tempx*cos(theta)-tempy*sin(theta)+dx;
	p->y=tempx*sin(theta)+tempy*cos(theta)+dy;
}

void icp::initialisePopulation(void){
	srand(time(0));
	
if (bestTransformation.theta>0.15 || bestTransformation.theta<-0.15){
	popul[0].t.dx=0;
	popul[0].t.dy=0;
	popul[0].t.theta=0;
flag2=0;
}
else{
popul[0].t.dx=rand()%DIASPORA-DIASPORA/2;
popul[0].t.dy=rand()%DIASPORA-DIASPORA/2;
popul[0].t.theta=rand()%10000/DIASPORAANGLE -DIASPORAANGLEPLUS;
flag2=1;
}
	for(unsigned int i=1;i<POPULATION;i++){
		popul[i].t.dx=rand()%DIASPORA-DIASPORA/2;
		popul[i].t.dy=rand()%DIASPORA-DIASPORA/2;
		popul[i].t.theta=rand()%10000/DIASPORAANGLE -DIASPORAANGLEPLUS;
	//std::cout<<popul[i].t.theta<<"\n";
//std::cout<<popul[i].t.dx<<"\n";
//std::cout<<popul[i].t.dy<<"\n";
	}
}

void icp::updateFitnesses(void){
	int tempx,tempy;
	unsigned int i;
	float sinth,costh,tttx,ttty;
	transformation temp;
	bool expansionRight;


//float min[24]={10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000};
//float max[24]={-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000};
//float min2[24]={10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000};
//float max2[24]={-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000};
	
	for(i=0;i<POPULATION;i++)
	{
		popul[i].fitness=0;

		temp.dx=robotPose.dx+popul[i].t.dx;
		temp.dy=robotPose.dy+popul[i].t.dy;
		temp.theta=robotPose.theta+popul[i].t.theta;	
			
		for(unsigned int j=0;j<LASERRAYS;j+=ANAPOSO){
			expansionRight=false;
			tempx=original.p[j].x;
			tempy=original.p[j].y;
		//std::cout<<original.p[j].x<<" ";
			sinth=sin(temp.theta);
			costh=cos(temp.theta);
			tttx=tempx*costh-tempy*sinth+temp.dx;
			ttty=tempx*sinth+tempy*costh+temp.dy;
//if (ttty<min){min=ttty;}
//if (ttty>max){max=ttty;}			


			while(!expansionRight){
				expansionRight=true;
				if(tttx>=(int)(MAPSIZEX-STARTX)){
					expandMap(0);	expansionRight=false; std::cout<<"Expand right\n";
				}
				if(tttx<=-((int)STARTX)){
					expandMap(2);	expansionRight=false; std::cout<<"Expand left\n";
				}
				if(ttty>=(int)(MAPSIZEY-STARTY)){
					expandMap(1);	expansionRight=false; std::cout<<"Expand down\n";
				}
				if(ttty<=-((int)STARTY)){
					expandMap(3);	expansionRight=false; std::cout<<"Expand up\n";
				}
			}
			tttx+=STARTX;
			ttty+=STARTY;
//if (i==0){
//std::cout<<((unsigned int)tttx)<<",";
//std::cout<<((unsigned int)ttty)<<"\n";
//}
//if (ttty<min3){min3=ttty;}
//if (ttty>max3){max3=ttty;}
//if (tttx<min4){min4=tttx;}
//if (tttx>max4){max4=tttx;}	

//if (j==180 or j==0 or j==15){
//if (ttty<min[j/15]){min[j/15]=ttty;}
//if (ttty>max[j/15]){max[j/15]=ttty;}
//if (tttx<min2[j/15]){min2[j/15]=tttx;}
//if (tttx>max2[j/15]){max2[j/15]=tttx;}	
//std::cout<<((unsigned int)tttx)<<",";
//}
//if ((j>105 && j<255) && flag2==1){popul[i].fitness+=10*(255-map[(unsigned int)tttx][(unsigned int)ttty])/255.0;}
//else{
			popul[i].fitness+=(255-map[(unsigned int)tttx][(unsigned int)ttty])/255.0;
//}
			
		}
//if (popul[i].t.dx<2 && popul[i].t.dy<2 && popul[i].t.dx>-2 && popul[i].t.dy>-2 && popul[i].t.theta<0.1 && popul[i].t.theta>-0.1){
//popul[i].fitness=0;
//}
		if(bestFitness<popul[i].fitness){
			

			//if(bestTransformation.theta<0.2 && bestTransformation.theta>-0.2){//0.4
					bestFitness=popul[i].fitness;
					bestTransformation=popul[i].t;
if((bestTransformation.theta<0.2 && bestTransformation.theta>-0.2)){flag=1;}//0.2
else {flag=0;} 
			//}
		}
		//if(bestFitness>23) break; //23 std, 30++ ligo kalitera
	}	
	//std::cout<<"Best Fitness:"<<bestFitness<<"\n";


//for (int k=0;k<24;k++){
///if ((max[k]-min[k])*(max2[k]-min2[k])>v){v=(max[k]-min[k])*(max2[k]-min2[k]);}
//if ((max[k]-min[k])>v){v=(max[k]-min[k]);}
//}
//delete [] min;
//delete [] max;
//delete [] min2;
//delete [] max2;
//if ((max-min)*(max2-min2)>v){v=(max-min)*(max2-min2);}
//std::cout<<(max3-min3)*(max4-min4)<<"\n\n";
//std::cout<<"("<<min4<<","<<max4<<")"<<"("<<min3<<","<<max3<<")"<<"\n";
//std::cout<<robotPose.dx+250<<","<<robotPose.dy+250<<","<<robotPose.theta<<"\n";
}	

void icp::runIteration(void){
	initialisePopulation();

	int counter=0;
/*
std::cout<<"\n\nPopulation "<<counter<<":\n";
for (int j=0;j<500;j++){
std::cout<<"("<<popul[j].t.dx<<","<<popul[j].t.dy<<","<<popul[j].t.theta<<"),";
}
*/
//max3=-10000;
//max4=-10000;
//min3=10000;
//min4=10000;
	while(counter<19){ //19 std, 30 kalitero mallon
		bestFitness=0;
		bestTransformation.dx=bestTransformation.dy=bestTransformation.theta=0;
		
		updateFitnesses();
//std::cout<<v<<"\n\n";
		fixNewGeneration();
//if (counter==3) {std::cout<<popul[400].t.dx<<",";
//std::cout<<popul[400].t.dy<<"\n";
//std::cout<<popul[300].t.dx<<",";
//std::cout<<popul[300].t.dy<<"\n";
//std::cout<<popul[100].t.dx<<",";
//std::cout<<popul[100].t.dy<<"\n";}
//std::cout<<"\nPOPULATION:\n";
//if (counter<5){
//for(unsigned int i=0;i<POPULATION;i++){
		//popul[i].t.dx
		//popul[i].t.dy
		//popul[i].t.theta
//std::cout<<"Counter:"<<counter<<"\n";
	//std::cout<<popul[i].t.theta<<"\n";
//std::cout<<popul[i].t.dx<<"\n";
//std::cout<<popul[i].t.dy<<"\n\n";
	//}}
		counter++;

	}

//std::cout<<"("<<min4<<","<<max4<<")"<<"("<<min3<<","<<max3<<")"<<"\n";
//if ((max3-min3)*(max4-min4)>v){v=(max3-min3)*(max4-min4);}
//std::cout<<v<<"\n\n";
}

bool icp::fixNewScans(void){
	if(nextScanIndex==lscans.scans.size()) return true;
	previous=original;
	original=lscans.scans[nextScanIndex];
	++nextScanIndex;
//int max=-10000,max2=-10000,min=10000,min2=10000;	
	for (unsigned int i=0;i<LASERRAYS;i++){
		original.p[i].theta=-d_LASERANGRAD/2+i*d_LASERANGRAD/LASERRAYS;
		original.p[i].x=original.scan[i]/ocgd*cos(original.p[i].theta);
		original.p[i].y=original.scan[i]/ocgd*sin(original.p[i].theta);
	//if (original.p[i].x<min){min=original.p[i].x;}
	//if (original.p[i].y<min2){min2=original.p[i].y;}
	//if (original.p[i].x>max){max=original.p[i].x;}
	//if (original.p[i].y>max2){max2=original.p[i].y;}
				
	}
		

	//std::cout<<min<<"\n";
	//std::cout<<min2<<"\n";
	//std::cout<<max<<"\n";
	//std::cout<<max2<<"\n";
	transformed=original;	
	return false;
}

void icp::runGA(void){
	bool done=false;
	unsigned int counter=0;
	//CTimestamp time;
	//double t=time.getTimestampMilli();
	//std::cout<<t<<"\n";
   FILE *fOut;

    fOut = fopen("output.txt", "w");	
//int max=0,min=0;
v=-10000;

//std::cout<<"\nFirst scan:";		
//for (unsigned int i=0;i<LASERRAYS;i=i+18){
//std::cout<<"\nOriginal.p["<<i<<"].x:"<<original.p[i].x;
//std::cout<<"\nOriginal.p["<<i<<"].y:"<<original.p[i].y;


//}

old.dx=old.dy=old.theta=0;	
	while(!done){
		
std::cout<<"\n";
//std::cout<<"\nNew scans:";		

/*
for (unsigned int i=0;i<LASERRAYS;i=i+2){
std::cout<<"\nROM("<<counter<<")("<<i/2<<")<=std_logic_vector(conv_signed("<<(int)((float)original.scan[i]/ocgd)<<",10));";
}
std::cout<<"\n";	
for (unsigned int i=0;i<LASERRAYS;i=i+2){
std::cout<<"\nROMx("<<counter<<")("<<i/2<<")<=std_logic_vector(conv_signed("<<original.p[i].x<<",10));";
}
std::cout<<"\n";
for (unsigned int i=0;i<LASERRAYS;i=i+2){
std::cout<<"\nROMy("<<counter<<")("<<i/2<<")<=std_logic_vector(conv_signed("<<original.p[i].y<<",10));";
}		
*/



		runIteration();
/*
std::cout<<"\n\nPopulation "<<counter<<":\n";
for (int j=0;j<500;j++){
std::cout<<"("<<popul[j].t.dx<<","<<popul[j].t.dy<<","<<popul[j].t.theta<<"),";
}
*/
//std::cout<<robotPose.dx+250<<",";	
//std::cout<<robotPose.dy+250<<",";
//std::cout<<robotPose.theta<<"\n";

/*std::cout<<"\n\nBest "<<counter<<":";
std::cout<<"\nBest dx:"<<bestTransformation.dx;	
std::cout<<"\nBest dy:"<<bestTransformation.dy;
std::cout<<"\nBest theta:"<<bestTransformation.theta;
*/	
	
//if (bestTransformation.dx<min){min=bestTransformation.dx;}
//if (bestTransformation.dy>max){max=bestTransformation.dy;}
		

		robotPose.dx+=bestTransformation.dx;
		robotPose.dy+=bestTransformation.dy;
		robotPose.theta+=bestTransformation.theta;

//if (bestTransformation.theta<0.15 && bestTransformation.theta>-0.15 && old.dx)


old.dx=bestTransformation.dx;
old.dy=bestTransformation.dy;
old.theta=bestTransformation.theta;



/*
		std::cout<<"\n\nPose "<<counter<<":";
		std::cout<<"\nROBOT X: "<<robotPose.dx+STARTX;		
		std::cout<<"\nROBOT Y: "<<robotPose.dy+STARTY;
		std::cout<<"\nROBOT THETA: "<<robotPose.theta;	
*/

//std::cout<<"\nCounter: "<<counter<<"\n";
		updateMapColor();
/*		std::cout<<"\nAfter map update "<<counter<<":\n";
		std::cout<<"MAP:";
if (counter<20){		
		for(unsigned int k=0;k<MAPSIZEX;k++){
		
			std::cout<<"\n"	;	
			for(unsigned int j=0;j<MAPSIZEY;j++){
				std::cout<<(int)map[j][k]<<",";
			
			}
		}
}*/
		//std::cout<<"\nROBOT X: "<<robotPose.dx+STARTX;		
		//std::cout<<"\nROBOT Y: "<<robotPose.dy+STARTY;
		//std::cout<<"\nROBOT THETA: "<<robotPose.theta;

		done=fixNewScans();



	
//for (unsigned int i=0;i<LASERRAYS;i++){
//std::cout<<"\nOriginal.p["<<i<<"].x:"<<original.p[i].x;
//std::cout<<"\nOriginal.p["<<i<<"].y:"<<original.p[i].y;
//std::cout<<"\nOriginal.scan["<<i<<"]:"<<original.scan[i];
//}		
		
		counter++;

	showMap();
		std::cout<<"\nCounter: "<<counter<<"/"<<lscans.scans.size()<<"\n";



	}	
	
//std::cout<<min<<"\n";
//std::cout<<max<<"\n";

	//std::cout<<"Mean execution time:"<<(time.getTimestampMilli()-t)/((float)counter)<<"\n";
	std::string outFileName="/home/greg/Desktop/map10.png";
for (int i = 0; i < MAPSIZEX; i++){
	for (int j=1; j<MAPSIZEY; j++){           // save array to after.txt
            fprintf(fOut, "%u", map[i,j]);
		if (j==MAPSIZEY-1){
			fprintf(fOut, "\n");}
		else{		
		fprintf(fOut, ",");}
	}
 }
        fclose(fOut);

showMap();
	if(!cvSaveImage(outFileName.c_str(),mapDisplay)) printf("Could not save: %s\n",outFileName.c_str());
}

void icp::fixNewGeneration(void){
	popul[0].t.dx=bestTransformation.dx;//elitism
	popul[0].t.dy=bestTransformation.dy;
	popul[0].t.theta=bestTransformation.theta;
	unsigned int k,l;
	bool done=false;
	unsigned int coun=0;
	for(unsigned int i=1;i<POPULATION/4;i++){//elitism 2, me std 20 kalo
		popul[i].t.dx=(bestTransformation.dx)+rand()%DIASPORA/2-DIASPORA/4;
		popul[i].t.dy=(bestTransformation.dy)+rand()%DIASPORA/2-DIASPORA/4;
		popul[i].t.theta=(bestTransformation.theta)+rand()%10000/DIASPORAANGLE-DIASPORAANGLEPLUS;
	}


for(unsigned int i=POPULATION/4;i<POPULATION/2;i++){//elitism 2, me std 20 kalo
		popul[i].t.dx=(old.dx)+rand()%DIASPORA/2-DIASPORA/4;
		popul[i].t.dy=(old.dy)+rand()%DIASPORA/2-DIASPORA/4;
		popul[i].t.theta=(bestTransformation.theta)+rand()%10000/DIASPORAANGLE-DIASPORAANGLEPLUS;
	}

/*
	for(unsigned int i=POPULATION/20;i<POPULATION/4;i++){//crossover
		//done=false;	coun=0;
		//while(!done){
			//k=rand()%POPULATION;
			//l=rand()%POPULATION;
			//if(popul[k].fitness>popul[i].fitness){
				popul[i].t.dx=(old.dx)+rand()%DIASPORA/2-DIASPORA/4;//(popul[k].t.dx+popul[l].t.dx)/2;//rand()%DIASPORA/2-DIASPORA/4;
				popul[i].t.dy=(old.dy)+rand()%DIASPORA/2-DIASPORA/4;//(popul[k].t.dy+popul[l].t.dy)/2;//rand()%DIASPORA/2-DIASPORA/4;
				popul[i].t.theta=(old.theta)+rand()%10000/DIASPORAANGLE-DIASPORAANGLEPLUS;//(popul[k].t.theta+popul[l].t.theta)/2;//rand()%10000/DIASPORAANGLE/2-DIASPORAANGLEPLUS/2;
				//done=true;			
			//}
			//coun++;
			//if(coun==5) done=true;
		//}
	}
*/
	for(unsigned int i=POPULATION/2;i<POPULATION;i++){//mutation
			done=false;	coun=0;
			while(!done){
				k=rand()%POPULATION;
				//l=rand()%POPULATION;
				if(popul[k].fitness>popul[i].fitness){
					popul[i].t.dx=popul[k].t.dx+rand()%DIASPORA/2-DIASPORA/4;
					popul[i].t.dy=popul[k].t.dy+rand()%DIASPORA/2-DIASPORA/4;
					popul[i].t.theta=popul[k].t.theta+rand()%10000/DIASPORAANGLE/2-DIASPORAANGLEPLUS/2;
					done=true;			
				}
				coun++;
				if(coun==5) done=true;
			}
		}
}

void icp::updateMapColor(void){
	int R=0;
	float TH = -d_LASERANG/2.0;
	int dmeasure;
//std::cout<<"\nInside map update:\n";
//std::cout<<"Robot dx:"<<robotPose.dx<<"\n";
//std::cout<<"Robot dy:"<<robotPose.dy<<"\n";
//std::cout<<"Robot theta:"<<robotPose.theta<<"\n";
//std::cout<<"STARTX:"<<STARTX<<"\n";
//std::cout<<"STARTY:"<<STARTY<<"\n";
	while(TH<d_LASERANG/2.0){
//std::cout<<"OUTER LOOP"<<"\n";
//std::cout<<"new TH:"<<TH<<"\n";

		int xt,yt,measid;
		measid=(TH+d_LASERANG/2.0)/d_LASERANG*LASERRAYS;
//std::cout<<"new measid:"<<measid<<"\n";
		xt=transformed.p[measid].x;
		yt=transformed.p[measid].y;
//std::cout<<"new xt , yt:"<<xt<<" , "<<yt<<"\n";
		dmeasure=(int)((float)original.scan[measid]/ocgd);

//std::cout<<"dmeasure["<<measid<<"]:"<<dmeasure<<"\n";
//std::cout<<"original.scan:"<<original.scan[measid]<<"\n";
//std::cout<<"new dmeasure:"<<dmeasure<<"\n";
//std::cout<<"Angle:"<<robotPose.theta+(float)measid/(float)LASERRAYS*d_LASERANGRAD-d_LASERANGRAD/2<<"\n";
//std::cout<<"Cos:"<<cos(robotPose.theta+(float)measid/(float)LASERRAYS*d_LASERANGRAD-d_LASERANGRAD/2)<<"\n";
//std::cout<<"Sin:"<<sin(robotPose.theta+(float)measid/(float)LASERRAYS*d_LASERANGRAD-d_LASERANGRAD/2)<<"\n";
		while(R<d_LASERMAX/ocgd-3){
//std::cout<<"INNER LOOP"<<"\n";
//std::cout<<"new R:"<<R<<"\n";


			float xpoint,ypoint;
			xpoint=R*cos(robotPose.theta+(float)measid/(float)LASERRAYS*d_LASERANGRAD-d_LASERANGRAD/2) + robotPose.dx+STARTX;
			ypoint=R*sin(robotPose.theta+(float)measid/(float)LASERRAYS*d_LASERANGRAD-d_LASERANGRAD/2) + robotPose.dy+STARTY;
//std::cout<<"new xpoint, ypoint:"<<xpoint<<", "<<ypoint<<"\n";			
			//0.1 kai 0.7 kalo, 0.2 kai 0.9 enallaktiko, 0.15 kai 0.8 mesi lisi

			if(dmeasure>R || (xt==0 && yt==0)){
//std::cout<<"1st case\n";
				int tt=map[(unsigned int)xpoint][(unsigned int)ypoint];
//std::cout<<"READ tt:"<<tt<<"\n";
				
				if (flag==0){
				
				if (tt>=127){tt+=(255-tt)*0.2;}//02
				else{tt+=(255-tt)*0.05;}//005

				}
				else{
				if (tt>=127){tt+=(255-tt)*0.05;}//005
				else{tt+=(255-tt)*0.02;}//002
				//tt+=(255-tt)*0.1;//0.1 aksioprepes, 0.05 std
				}

//std::cout<<"WRITE tt:"<<tt<<"\n";
				map[(unsigned int)xpoint][(unsigned int)ypoint]=tt;
			}
			
			if( dmeasure+1>R && dmeasure-2<R ){
//std::cout<<"2nd case\n";
				int tt=map[(unsigned int)xpoint][(unsigned int)ypoint];
//std::cout<<"READ tt:"<<tt<<"\n";
				if (flag==1){
				if (tt>=127){tt+=(0-tt)*0.2;}//02
				else{tt+=(0-tt)*0.3;}		//03				//me to default kalo stis gwnies, kako stis eutheies otan 													//einai
				//tt+=(0-tt)*0.7;//0.7 aksioprepes, 0.3 std			//makreis diadromoi
				}								//0.5, 0.05, 0.2, 0.01, 0.4, 0.8, 0.4, 0.8 kalitero stin 													//eutheia

				else{
				if (tt>=127){tt+=(0-tt)*0.4;}//04
				else{tt+=(0-tt)*0.7;}//07
				//tt+=(0-tt)*0.5;
				}
//std::cout<<"WRITE tt:"<<tt<<"\n";
				map[(unsigned int)xpoint][(unsigned int)ypoint]=tt;
			}
			
			R++;
		}
		TH+=1;
		R=1;
	}
}

void icp::showMap(void){
	//To be deleted
	cvReleaseImage(&mapDisplay);
	mapDisplay = cvCreateImage(cvSize(MAPSIZEX,MAPSIZEY), IPL_DEPTH_8U,3);
	
	
	CvScalar pixel;
	for(unsigned int k=0;k<MAPSIZEX;k++){
		for(unsigned int j=0;j<MAPSIZEY;j++){
			pixel.val[0]= pixel.val[1]=pixel.val[2]=map[k][j];
			cvSet2D(mapDisplay,j,k, pixel);
		}
	}
		
	pixel.val[0]= 0;
	pixel.val[1]= 0;
	pixel.val[2]=255;

	unsigned int xx,yy;

	xx=robotPose.dx+STARTX;		
	yy=robotPose.dy+STARTY;	

	cvSet2D(mapDisplay,yy,xx,pixel);
	cvSet2D(mapDisplay,yy,xx-1, pixel);
	cvSet2D(mapDisplay,yy,xx+1, pixel);
	cvSet2D(mapDisplay,yy+1, xx, pixel);
	cvSet2D(mapDisplay,yy-1, xx, pixel);

	cvShowImage(mapTitle.c_str(), mapDisplay);
	cvWaitKey(10);	
}

void icp::expandMap(unsigned int id){
	unsigned char **tempMap;
	//cvReleaseImage(&mapDisplay);
	if(id==0 || id==2){	//Needs horizontal expansion - increase X
		tempMap=new unsigned char*[MAPSIZEX+EXPANSION];
		for(unsigned int i=0;i<MAPSIZEX+EXPANSION;i++)
			tempMap[i]=new unsigned char[MAPSIZEY];
		if(id==0){	//Expand right
			for(unsigned int i=0;i<MAPSIZEX;i++)
				for(unsigned int j=0;j<MAPSIZEY;j++)
					tempMap[i][j]=map[i][j];
			for(unsigned int i=MAPSIZEX;i<MAPSIZEX+EXPANSION;i++)
				for(unsigned int j=0;j<MAPSIZEY;j++)
					tempMap[i][j]=127;
		}
		else{	//Expand left
			for(unsigned int i=EXPANSION;i<MAPSIZEX+EXPANSION;i++)
				for(unsigned int j=0;j<MAPSIZEY;j++)
					tempMap[i][j]=map[i-EXPANSION][j];
			for(unsigned int i=0;i<EXPANSION;i++)
				for(unsigned int j=0;j<MAPSIZEY;j++)
					tempMap[i][j]=127;	
			STARTX+=EXPANSION;			
		}
		for(unsigned int i=0;i<MAPSIZEX;i++)
			delete [] map[i];
		delete [] map;
		MAPSIZEX+=EXPANSION;
	}
	if(id==1 || id==3){	//Needs vertical expansion - increase Y
		tempMap=new unsigned char*[MAPSIZEX];
		for(unsigned int i=0;i<MAPSIZEX;i++)
			tempMap[i]=new unsigned char[MAPSIZEY+EXPANSION];
		if(id==1){	//Expand down
			for(unsigned int i=0;i<MAPSIZEX;i++)
				for(unsigned int j=0;j<MAPSIZEY;j++)
					tempMap[i][j]=map[i][j];
			for(unsigned int i=0;i<MAPSIZEX;i++)
				for(unsigned int j=MAPSIZEY;j<MAPSIZEY+EXPANSION;j++)
					tempMap[i][j]=127;
		}
		else{	//Expand up
			for(unsigned int i=0;i<MAPSIZEX;i++)
				for(unsigned int j=EXPANSION;j<MAPSIZEY+EXPANSION;j++)
					tempMap[i][j]=map[i][j-EXPANSION];
			for(unsigned int i=0;i<MAPSIZEX;i++)
				for(unsigned int j=0;j<EXPANSION;j++)
					tempMap[i][j]=127;	
			STARTY+=EXPANSION;			
		}
		for(unsigned int i=0;i<MAPSIZEX;i++)
			delete [] map[i];
		delete [] map;
		MAPSIZEY+=EXPANSION;
	}

	map=new unsigned char* [MAPSIZEX];
	for(unsigned int i=0;i<MAPSIZEX;i++)
		map[i]=new unsigned char [MAPSIZEY];
		
	for(unsigned int i=0;i<MAPSIZEX;i++)
		for(unsigned int j=0;j<MAPSIZEY;j++)
			map[i][j]=tempMap[i][j];
			
	for(unsigned int i=0;i<MAPSIZEX;i++)
		delete [] tempMap[i];
	delete [] tempMap;
	//mapDisplay=cvCreateImage(cvSize(MAPSIZEX,MAPSIZEY), IPL_DEPTH_8U,3);

}
