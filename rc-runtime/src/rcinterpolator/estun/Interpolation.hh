#ifndef __INTERPOLATION_HH__
#define __INTERPOLATION_HH__

#include <deque>
#include "calInverseKin.hh"



void AxisInterp(const AxisPos_Deg &originalPos,
				const AxisPos_Deg &targetPos,
				double Ts, 
				double velPerc,
				double accPerc, 
				RobotConfig &param);

int interp_compute(ROBOT_INST &temp_inst);
#endif