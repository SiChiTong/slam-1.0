#include "icp.h"

int main (int argc, char *argv[]){
	icp test;
	
	if(argc==2)
		test.readScans(argv[1]);
	else
		test.readScans((char *)"../scans/scans_simple_corner");

	test.runGA();

	std::cout<<"Finished!\n";
	cvWaitKey(0);

	return 0;
}
