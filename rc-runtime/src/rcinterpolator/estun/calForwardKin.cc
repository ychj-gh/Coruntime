#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
//#include <deque>
#include <stdlib.h>
#include "inst_type.h"
#include "calInverseKin.hh"
#include "Interpolation.hh"

tmatrix toAxisTransMatrix(const DH_param  &param)
{
	tmatrix transf = dmatrix::Identity(4,4);
	tmatrix TransXZ = dmatrix::Identity(4,4);
	tmatrix RotX = dmatrix::Identity(4,4);
	tmatrix RotZ = dmatrix::Identity(4,4);
	Eigen::Matrix3d rotX,rotZ;
	rotX = Eigen::AngleAxisd(param.Alpha*M_PI/180.0,Eigen::Vector3d::UnitX());
	rotZ = Eigen::AngleAxisd(param.Theta*M_PI/180.0,Eigen::Vector3d::UnitZ());
	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 3; ++j)
		{
			RotX(i,j) = rotX(i,j);
			RotZ(i,j) = rotZ(i,j);
		}
	}
	TransXZ(0,3) = param.a;
	TransXZ(2,3) = param.d; 
	transf = RotZ * TransXZ * RotX;
	return transf;
}

// calculate euler angle
Euler_Deg tr2rpy(const tmatrix& T)
{
	Eigen::Matrix3d  tmp = T.topLeftCorner(3,3);
	// std::cout << tmp << std::endl;
	// std::cout << tmp.eulerAngles(0,1,2) << std::endl;
	return 180.0/M_PI*tmp.eulerAngles(0,1,2);  // 0 Axis X; 1 Axis Y; 2 Axis Z
}


// tmatrix calForwardKin(const AxisPos_Deg &PosAxis, Robot_param*  axes, XyzPose &PosCart)
// {
// 	dmatrix baseMatrix(4,4);
// 	baseMatrix = dmatrix::Identity(4,4);    // dan wei zheng
// 	tmatrix transMatrix(baseMatrix);
// 	//std::vector<DH_param *> A(axes);
// 	//std::vector<Robot_param *> A(axes);
// 	for(int i = 0; i < 6; ++i)
// 	{
// 		axes[i].DH_p.Theta = axes[i].DH_p.offset + PosAxis(i);
// 		//std::cout << " Theta: " << axes[i].DH_p.Theta << std::endl;
// 		transMatrix = transMatrix * toAxisTransMatrix(axes[i].DH_p);
// 	}
// 	//std::cout << transMatrix << std::endl;
// 	PosCart << transMatrix.topRightCorner(3,1),tr2rpy(transMatrix);
// 	return transMatrix;
// 	// std::cout << PosCart << std::endl;
// 	// std::cout << std::endl;
// }


tmatrix calForwardKin(const AxisPos_Deg &PosAxis, RobotConfig &param,/*Robot_param*  axes*/ XyzPose &PosCart)
{
	dmatrix baseMatrix(4,4);
	baseMatrix = dmatrix::Identity(4,4);    // dan wei zheng
	tmatrix transMatrix(baseMatrix);
	tmatrix transtool;
	//std::vector<DH_param *> A(axes);
	//std::vector<Robot_param *> A(axes);
	for(int i = 0; i < 6; ++i)
	{
		//axes[i].DH_p.Theta = axes[i].DH_p.offset + PosAxis(i);
		param.Axis[i].DH_p.Theta = param.Axis[i].DH_p.offset + PosAxis(i);
		//std::cout << " Theta: " << axes[i].DH_p.Theta << std::endl;
		// transMatrix = transMatrix * toAxisTransMatrix(axes[i].DH_p);
		transMatrix = transMatrix * toAxisTransMatrix(param.Axis[i].DH_p);
	
	}
	//std::cout << transMatrix << std::endl;
	for(int i = 0; i < 4; ++i)
		for(int j = 0; j < 4; ++j)
			transtool(i,j) = param.transTool[i][j];
	transMatrix = transMatrix * transtool;
	PosCart << transMatrix.topRightCorner(3,1),tr2rpy(transMatrix);
	return transMatrix;
	// std::cout << PosCart << std::endl;
	// std::cout << std::endl;
}

