#include <iostream>
#define APP_1

#ifdef APP_1
#include "AlignImageHD.h"
#include "LineReconstructor.h"
#endif


int main(int argc, char** argv)
{

#ifdef APP_1
	int step = atoi(argv[1]);
	int method = atoi(argv[2]);
	if (step == 0)
	{	
		cout << ("Start Registration........... ") << endl;
		AlignImageHD mss;
		
		mss.preprocess(step, method);
		mss.processRegistHDAndMSSImages(); 
	}
	else if (step == 3)
	{
		cout << ("Start Reconstruction........... ") << endl;
		LineRecon3D rd;
		rd.preprocess(step, method);
		rd.processRecon3D();
	}
#endif
}
