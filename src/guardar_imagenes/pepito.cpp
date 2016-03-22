#include "pepito.hpp"
#include <iostream>

using namespace cv;
using namespace std;

void funcion_pepito(void)
{
	cout << "\n\n adentro de funcion pepito \n \n";
	
	Mat mat = Mat::zeros(4,4,CV_8UC1);
	
	namedWindow("ventana");
	
	imshow("ventana", mat);
	waitKey(0);	
}
