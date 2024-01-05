#include "robot.h"

robot_information_t robotInf;


void robotInit(){
	robotInf.robotMode = ROBOT_INIT;
	robotInf.modeStep = ROBOT_INIT_IMU;  //用来测量三轴姿态角（角速度） 以及加速度
}