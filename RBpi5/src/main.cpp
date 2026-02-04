// Actual Includes
#include <iostream>
#include <thread>

// Own Includes
#include "MotorController.h"
#include "CamController.h"
#include "LidarController.h"

//Object Creation
MotorController myMotorController("/dev/ttyACM0");
CamController myCamController;
LidarController myLidarController;

//Globals
const int ModuleCount = 3;


void setup(){
	std::cout << "[MAIN] Hello World! and now all modules: "<< ModuleCount << std::endl;
	myMotorController.test();
	myCamController.Test();
	myLidarController.Test();
}

int main() {
 	setup();   
    return 0;
}
