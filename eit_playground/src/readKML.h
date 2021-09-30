#include <string>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <vector>

struct coordinate{
	double latitude;
	double longitude;
	double altitude;
};

class KMLClass{
public:
	std::vector<struct coordinate> extract_coordinates(){
		std::vector<struct coordinate>coordinates;
		struct coordinate crd;
	
		std::string tempString;
		size_t pos;
		int i =0;
		std::string line;
		std::ifstream missionFile ("MissionPlan.kml");
		if(missionFile.is_open()){
			while(getline(missionFile, line)){
				if(line.rfind("Lat:", 0)==0){
					pos = line.find_first_of(' ');
					tempString = line.substr(pos+1);
					crd.latitude = std::stod(tempString);
					i++;
				}else if(line.rfind("Lon:",0)==0){
					pos = line.find_first_of(' ');
					tempString = line.substr(pos+1);
					crd.longitude = std::stod(tempString);
					i++;
				}else if(line.rfind("Alt AMSL:",0)==0){
					//Above mean sea level - no nothing for now
				}else if(line.rfind("Alt Rel:",0)==0){
					pos = line.find_first_of(' ');
					tempString = line.substr(pos+1);
					pos = tempString.find_first_of(' ');
					tempString = tempString.substr(pos+1);
					crd.altitude = std::stod(tempString);
					i++;
				}
				if(i==3){
					coordinates.push_back(crd);
					i=0;
				}
			}
		}
		return coordinates;
	}
};
