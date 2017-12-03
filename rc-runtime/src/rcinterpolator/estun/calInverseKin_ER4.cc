#include "inst_type.h"
#include <stdio.h>
#include <vector>
#include <iostream>

tmatrix TermPos2TransMatrix(const XyzPose& termPos)
{
	tmatrix RotX = dmatrix::Identity(4,4);
	tmatrix RotY = dmatrix::Identity(4,4);
	tmatrix RotZ = dmatrix::Identity(4,4);
	tmatrix TransXYZ = dmatrix::Identity(4,4);
	tmatrix transMatrix;
	Eigen::Matrix3d rotX,rotY,rotZ;
	rotX = Eigen::AngleAxisd(termPos(3)*M_PI/180.0,Eigen::Vector3d::UnitX()); 
	rotY = Eigen::AngleAxisd(termPos(4)*M_PI/180.0,Eigen::Vector3d::UnitY());
	rotZ = Eigen::AngleAxisd(termPos(5)*M_PI/180.0,Eigen::Vector3d::UnitZ());
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
		{
			RotX(i,j) = rotX(i,j);
			RotY(i,j) = rotY(i,j);
			RotZ(i,j) = rotZ(i,j);
		}
	TransXYZ(0,3) = termPos(0);
	TransXYZ(1,3) = termPos(1);
	TransXYZ(2,3) = termPos(2);
	//transMatrix = TransXYZ * RotZ * RotY * RotX; //Kuka
	transMatrix = TransXYZ * RotX * RotY * RotZ;   //Estun 
	//std::cout << transMatrix << std::endl;
	return transMatrix;	
}

double calcRealAngle(double curAng, double candidateAng1,double candidateAng2) 
{
	double curAngCopy(curAng);
	double* allAng[3] ={ &candidateAng1, &candidateAng2, &curAngCopy };
	for (int i = 0; i < 3; ++i)
	{
		while (*allAng[i]> M_PI)
		{
			*allAng[i] -= 2*M_PI;
		}
		while (*allAng[i] < -M_PI)
		{
			*allAng[i] += 2*M_PI;
		}
	}
	double gap[2] =
	{ 0, 0 };
	bool dir[2] =
	{ true, true };
	for (int i = 0; i < 2; ++i)
	{
		if (*allAng[i] >= curAngCopy)/*=*/
		{
			gap[i] = *allAng[i]-curAngCopy;
			dir[i] = true;
		}
		else
		{
			gap[i] = curAngCopy-*allAng[i];
			dir[i] = false;
		}
		if (gap[i]> M_PI)
		{
			gap[i] = 2*M_PI-gap[i];
			dir[i] = !dir[i];
		}
	}

	if (gap[0] <= gap[1])/*=*/
	{
		return dir[0] ? curAng+gap[0] : curAng-gap[0];
	}
	else
	{
		return dir[1] ? curAng+gap[1] : curAng-gap[1];
	}
}

// Only Suitable for the same Robotic Structure
int calInverseKin_ER4(const tmatrix& transMatrix, RobotConfig& param,/*Robot_param*  axes,*/ //const std::vector<Robot_param *> &axes,
								const AxisPos_Deg& posLast, AxisPos_Deg& posTar)
{
	//dmatrix T(transMatrix);
	tmatrix transtool;
	for(int i = 0; i < 4; ++i)
		for(int j = 0; j < 4; ++j)
			transtool(i,j) = param.transTool[i][j];
	dmatrix T = transMatrix * transtool.inverse();
	AxisPos_Deg pLast(M_PI * posLast / 180.0);

	double nx(T(0,0)), ny(T(1,0)), nz(T(2,0));
	double ox(T(0,1)), oy(T(1,1)), oz(T(2,1));
	double ax(T(0,2)), ay(T(1,2)), az(T(2,2));
	double px(T(0,3)/1000.0), py(T(1,3)/1000.0), pz(T(2,3)/1000.0);

	// according to DH_Parmater
	// double d1(axes[0].DH_p.d/1000.0);
	// double d4(axes[3].DH_p.d/1000.0);
	// double d6(axes[5].DH_p.d/1000.0);
	// double a1(axes[0].DH_p.a/1000.0);
	// double a2(axes[1].DH_p.a/1000.0);
	// double a3(axes[2].DH_p.a/1000.0);

	double d1(param.Axis[0].DH_p.d/1000.0);
	double d4(param.Axis[3].DH_p.d/1000.0);
	double d6(param.Axis[5].DH_p.d/1000.0);
	double a1(param.Axis[0].DH_p.a/1000.0);
	double a2(param.Axis[1].DH_p.a/1000.0);
	double a3(param.Axis[2].DH_p.a/1000.0);

	//std::cout << d1 <<" "<< d4 <<" "<< d6 <<" " << a1 <<" "<< a2 <<" "<< a3 << std::endl;
	// solve for theta1
	double theta1_1;
	if((fabs((d6*ay + py)) < 10e-13 ) && (fabs((d6*ax + px)) < 10e-13))
			theta1_1 = pLast(0);
		else
			theta1_1 = atan2((d6*ay + py),(d6*ax + px));
		double theta1_2 = theta1_1;
		double theta1;

		if (theta1_1 <= 0.0L)
			theta1_2 = theta1_1 + M_PI;
		else
			theta1_2 = theta1_1 - M_PI;

		theta1 = calcRealAngle(pLast(0), theta1_1, theta1_2);
		// the limit of theta1 according to the reference
		if((theta1 < -M_PI) || (theta1 > M_PI))
		{
			theta1 = pLast(0);
			printf("theta1 exceeds pos limit.\n");
			return -1;
			//return INVERSE_KIN_NOT_REACHABLE;
		}
		posTar(0) = 180.0 * theta1 / M_PI;

		//solve for theta3
		double k1 = d6*(cos(theta1)*ax + sin(theta1)*ay) +  
						cos(theta1)*px + sin(theta1)*py - a1;
		double k2 = d6*az + pz - d1;
		double k3 = pow(k1, 2) + pow(k2, 2) - pow(a2, 2) - pow(a3, 2) - pow(d4, 2);
		double k4 = k3/(2*a2);
		double temp_var = pow(a3, 2) + pow(d4, 2) - pow(k4, 2);
		if(temp_var < 0.0L)
		{
			printf("Theta3 can not be solved, so can not reach the point!\n");
			//return INVERSE_KIN_NOT_REACHABLE;
			return -1;
		}
		double delta = sqrt(pow(a3, 2) + pow(d4, 2) - pow(k4, 2));
		double theta3_1 = atan2(d4, a3) + atan2(delta, k4);
		double theta3_2 = atan2(d4, a3) - atan2(delta, k4);
		double theta3;

		theta3 = calcRealAngle(pLast(2), theta3_1, theta3_2);
		// the limit of theta3 according to the reference
		if((theta3 < -200.0L/180.0L*M_PI) || (theta3 > 80.0L/180.0L*M_PI))
		{
			theta3 = pLast(2);
			printf("theta3 exceeds pos limit.\n");
			return -1;
			//return INVERSE_KIN_NOT_REACHABLE;
		}
		posTar(2) = 180.0 * theta3 / M_PI;

		//solve for theta2
		k1 = cos(theta1)*px + sin(theta1)*py - a1 + d6*(cos(theta1)*ax + sin(theta1)*ay);
		k2 = d1 - pz - d6*az;
		double a = d4*cos(theta3) - a3*sin(theta3);
		double b = d4*sin(theta3) + a2 +a3*cos(theta3);
		//已经加入了theta2的offset -pi/2    theta(运算) = theta(电机) - pi/2
	    double theta2_1;
	    if((fabs(a*k1 + b*k2) < 10e-13)  && (fabs(b*k1 - a*k2) < 10e-13))
	    	theta2_1 = pLast(1);
	    else
	    	theta2_1 = atan2((a*k1 + b*k2),(b*k1 - a*k2)) + M_PI/2.0;
	    double theta2;

	    theta2 = calcRealAngle(pLast(1), theta2_1, theta2_1);
	    // the limit of theta2 according to the reference
		if((theta2 < -70.0L/180.0L*M_PI) || (theta2 > 160.0L/180.0L*M_PI))
		{
			theta2 = pLast(1);
			printf("theta2 exceeds pos limit.\n");
			return -1;
			//return INVERSE_KIN_NOT_REACHABLE;
		}
		posTar(1) = 180.0 * theta2 / M_PI;

		k1 = -sin(theta1)*ax + cos(theta1)*ay;
		k2 = cos(theta1)*cos(theta2 - M_PI/2.0 + theta3)*ax + sin(theta1)*
				cos(theta2 - M_PI/2.0 + theta3)*ay - sin(theta2 - M_PI/2.0 + theta3)*az;

		double theta4;
		double theta4_2;
		//此处的判断阈值不能过小，过小的话，当0/0时，它无法识别出来
		if((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
		{
			theta4 = pLast(3);
			//cout << "A" << endl;
		}
		else
		{
			double theta4_1 = atan2(-k1,-k2);
			if(theta4_1 > 0.0L)
				theta4_2 = theta4_1 - M_PI;
			else
				theta4_2 = theta4_1 + M_PI;
			theta4 = calcRealAngle(pLast(3), theta4_1, theta4_2);
		}

		// the limit of theta4 according to the reference
		if((theta4 < -2.0L*M_PI) || (theta4 > 2.0L*M_PI))
		{
			theta4 = pLast(3);
			printf("theta4 exceeds pos limit.\n");
			return -1;
			//return INVERSE_KIN_NOT_REACHABLE;
		}
		posTar(3) = 180.0 * theta4 / M_PI;


		//solve for theta5
		double k1_1 = -sin(theta1)*sin(theta4) + cos(theta1)*cos(theta4)*cos(theta2 - M_PI/2.0 + theta3);
		double k1_2 = cos(theta1)*sin(theta4) + sin(theta1)*cos(theta4)*cos(theta2 - M_PI/2.0 + theta3);
		double k1_3 = -cos(theta4)*sin(theta2 - M_PI/2.0 + theta3);
		k1 = k1_1*ax + k1_2*ay + k1_3*az;
		k2 = -cos(theta1)*sin(theta2 - M_PI/2.0 + theta3)*ax - sin(theta1)*sin(theta2 - M_PI/2.0 + theta3)*
				ay - cos(theta2 - M_PI/2.0 + theta3)*az;
		double theta5_1;
		if((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
			theta5_1 = pLast(4);
		else
			theta5_1 = atan2(-k1, k2);
		double theta5;
		
		theta5 = calcRealAngle(pLast(4), theta5_1, theta5_1);	
		// the limit of theta5 according to the reference
		if((theta5 < -135.0L/180.0L*M_PI) || (theta5 > 135.0L/180.0L*M_PI))
		{
			theta5 = pLast(4);
			printf("theta5 exceeds pos limit.\n");
			return -1;
			//return INVERSE_KIN_NOT_REACHABLE;
		}
		posTar(4) = 180.0 * theta5 / M_PI; 
		
		//solve for theta6
		k1_1 = -cos(theta1)*sin(theta4)*cos(theta2 - M_PI/2.0 + theta3) - cos(theta4)*sin(theta1);
		k1_2 = cos(theta1)*cos(theta4) - sin(theta1)*sin(theta4)*cos(theta2 - M_PI/2.0 + theta3);
		k1_3 = sin(theta4)*sin(theta2 - M_PI/2.0 + theta3);
		k1 = k1_1*nx + k1_2*ny + k1_3*nz;
		k2 = k1_1*ox + k1_2*oy + k1_3*oz;
		double theta6_1;
		if((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
			theta6_1 = pLast(5);
		else
			theta6_1 = atan2(k1, -k2);
		double theta6;
		
		theta6 = calcRealAngle(pLast(5), theta6_1, theta6_1);
		// the limit of theta6 according to the reference
		if((theta6 < -2.0L*M_PI) || (theta6 > 2.0L*M_PI))
		{
			theta6 = pLast(5);
			printf("theta6 exceeds pos limit.\n");
			return -1;
			//eturn INVERSE_KIN_NOT_REACHABLE;
		}
		posTar(5) = 180.0 * theta6 / M_PI;
}