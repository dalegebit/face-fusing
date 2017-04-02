#include <iostream>
#include <fstream>
#include "FaceMorph.h"
#include "image_utils.h"

using namespace std;

// Test 
// ---------

int main()
{

	FaceMorph fm(Img("res/pics/1.bmp"), "res/control_pts/1cp.txt", Img("res/pics/2.bmp"), "res/control_pts/2cp.txt", 11);
	fm.getMorphMiddles();
	fm.plot();
	fm.saveMiddles("example/1to2");

	return 0;
}
