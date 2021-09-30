#include <string>
#include <fstream>
#include <iostream>
#include <vector>

#include "readKML.h"

int main(){
	
	KMLClass klm;
	
	std::vector<struct coordinate>coordinates;
	coordinates = klm.extract_coordinates();
	double hej = 1.123456789;	
	for(int i=0;i<coordinates.size();i++){
		std::cout << hej << " " << coordinates[i].latitude << " " << coordinates[i].longitude << " " << coordinates[i].altitude << "\n";
	}

	return 0;
}
