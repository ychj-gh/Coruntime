#ifndef __CALINVERSEKIN_HH__
#define __CALINVERSEKIN_HH__

tmatrix TermPos2TransMatrix(const XyzPose& termPos);
// void calInverseKin_ER4(const tmatrix& transMatrix, const std::vector<Robot_param *> &axes,
// 								const AxisPos_Deg& posLast, AxisPos_Deg& posTar);
int calInverseKin_ER4(const tmatrix& transMatrix, RobotConfig& param,//const std::vector<Robot_param *> &axes,
								const AxisPos_Deg& posLast, AxisPos_Deg& posTar);

tmatrix calForwardKin(const AxisPos_Deg &PosAxis, RobotConfig& param,/*Robot_param*  axes,*/ XyzPose &PosCart);

Euler_Deg tr2rpy(const tmatrix& T);

inline void frametrans(const XyzPose& oriPos, XyzPose& tarPos)
{
	tmatrix oriMat = TermPos2TransMatrix(oriPos);
	tmatrix tarMat = TermPos2TransMatrix(tarPos);
	tarMat = oriMat * tarMat;
	tarPos << tarMat.topRightCorner(3,1),tr2rpy(tarMat);
}
#endif