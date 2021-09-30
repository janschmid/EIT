#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>
#include "readKML.h"

int main(){
	
	KMLClass klm;
	
	std::vector<struct coordinate>coordinates;
	coordinates = klm.extract_coordinates();
	for(int i=0;i<coordinates.size();i++){
		printf("%f %f %f\n", coordinates[i].latitude, coordinates[i].longitude, coordinates[i].altitude);
	}

	return 0;
}
