#include "inst_type.h"
#include "../../opmanager.hh"
#include <iostream>
#include <vector>
#include <deque>

void  getQuinticCoe(const double &start_deg, const double &end_deg, const double &start_vel, const double &end_vel,
					const double &start_acc, const double &end_acc, const double &tf, dvector &coe)
{
	dmatrix A(6,6);
	dvector B(6);
	double t2 = tf * tf;
	double t3 = t2 * tf;
	double t4 = t3 * tf;
	double t5 = t4 * tf;
	B << start_deg, end_deg, start_vel, end_vel, start_acc, end_acc;
	A << 1,0,0,0,0,0,
		1,tf,t2,t3,t4,t5,
		0,1,0,0,0,0,
		0,1,2*tf,3*t2,4*t3,5*t4,
		0,0,2,0,0,0,
		0,0,2,6*tf,12*t2,20*t3;
	coe = A.lu().solve(B);	
}


void QuinticPolynomi(const double &t, const dvector &coe, double &ans)
{
	double t2 = t * t;
	double t3 = t2 * t;
	double t4 = t3 * t;
	double t5 = t4 * t;
	ans = coe[0] + coe[1] * t + coe[2] * t2 + coe[3] * t3 + coe[4] * t4 + coe[5] * t5;
}


//======================================================================================================================
//@ targetPos 	: n axes degree of target position
//@ originalPos	: n axes degree of start  position
//@ Ts  			: Interpolation period
//@ velPerc		: speed percentage  20,30....
//@ accPerc		: accelerate percentage 
//@ PosSeq		: n axes interpolation sequence
//@ AxCnt		: num of axes
//====================================================================================================================== 
void AxisInterp(const AxisPos_Deg &originalPos,
				const AxisPos_Deg &targetPos,
				double Ts, 
				double velPerc,
				double accPerc, 
				RobotConfig &param)
{
	fprintf(stderr,"|>> ============= enter axis interp ===========>\n");
	std::cout << originalPos << std::endl;
	std::cout << targetPos << std::endl;
	size_t n = param.axis_count;
	AxisPos_Deg ChangePos = targetPos - originalPos;
	dvector t(n);
	double  Tmax = 0;
	dvector tA(n);
	double  Tacc = 0;

	// calculate real percentage
	if(velPerc <= 0.5 || velPerc > 95.0)
	return;
	velPerc = velPerc / 100.0;
	if(accPerc <= 0.5 || accPerc > 95.0)
	return;
	accPerc = accPerc / 100.0; 

	// find the longest time
	for(size_t ai = 0; ai != n; ++ai)
		t[ai] = fabs(ChangePos[ai])/(velPerc * param.Axis[ai].Lim_p.vellim);
	Tmax = t[0];
	for(size_t ai = 1; ai != n; ++ai)
		if(Tmax < t[ai])    Tmax = t[ai];
	if(Tmax == 0)
	{
		std::cout << "No need to move !" << std::endl;
		return;
	}

	// find the longest accelerate time
	for(size_t ai = 0; ai != n; ++ai)
		tA[ai] = (velPerc * param.Axis[ai].Lim_p.vellim) / (accPerc * param.Axis[ai].Lim_p.acclim);
	Tacc = tA[0];
	for(size_t ai = 1; ai != n; ++ai)
		if(Tacc < tA[ai])		Tacc = tA[ai];
	if(Tacc == 0)
	{
		std::cout << "Tacc = 0" << std::endl;
		return;
	} 	

	// calculate each axis actual const velocity
	double tf = Tmax + Tacc;
	double Tconst = tf - 2 * Tacc;
	dvector velConst(n);

	if(Tconst < 0)
	{
		dvector Vmax(n);
		for(size_t ai = 0; ai != n; ++ai)
			Vmax[ai] = sqrt(fabs(ChangePos[ai]) * accPerc * param.Axis[ai].Lim_p.acclim);
		for(size_t ai = 0; ai != n; ++ai)
		{
			tA[ai] = Vmax[ai] / (accPerc * param.Axis[ai].Lim_p.acclim);
		}
		Tacc = tA[0];
		for(size_t ai = 1; ai != n; ++ai)
			if(Tacc < tA[ai])   Tacc = tA[ai];
		tf = 2 * Tacc;
		Tconst = 0;
		Tmax = Tacc;
		for(size_t ai = 0; ai != n; ++ai)
			velConst[ai] = ChangePos[ai] / Tacc;
	}
	else
		for(size_t ai = 0; ai != n; ++ai)
			velConst[ai] = ChangePos[ai] / Tmax;

	// calculate interpolation points
	size_t N1 = static_cast<size_t>(Tacc/Ts);
	size_t N2 = static_cast<size_t>(Tmax/Ts);
	size_t N3 = static_cast<size_t>(tf/Ts);

	

	// calculate Accelerate Period Quintic
	dvector accPosEnd(n);
	std::vector<dvector> coeA(n);
	for(size_t ai = 0; ai != n; ++ai)
	{
		accPosEnd[ai] = originalPos[ai] + velConst[ai] * Tacc / 2.0;
		getQuinticCoe(originalPos[ai], accPosEnd[ai], 0, velConst[ai],0,0,Tacc,coeA[ai]);
	}

	// calculate Decelerate Period Quintic
	dvector decPosStart(n);
	std::vector<dvector> coeD(n);
	for(size_t ai = 0; ai != n; ++ai)
	{
		decPosStart[ai] = accPosEnd[ai] + velConst[ai] * Tconst;
		getQuinticCoe(decPosStart[ai],targetPos[ai],velConst[ai],0,0,0,Tacc,coeD[ai]);
	}

	for(size_t i = 0; i != N3+1; ++i)
	{	
		AxisPos_Deg tmp(n);
		double t = i * Ts;  // Time
		if(i <= N1)
			for(size_t ai = 0; ai != n; ++ai)
				QuinticPolynomi(t,coeA[ai],tmp[ai]);
		else if(i <= N2)
			for(size_t ai = 0; ai != n; ++ai)
				tmp[ai] = accPosEnd[ai] + velConst[ai] * (t - Tacc);
		else if(i <= N3)
			for(size_t ai = 0; ai != n; ++ai)
				QuinticPolynomi(t-Tmax,coeD[ai],tmp[ai]);

		interp_step(tmp);
	}

	// fprintf(stderr,"<< ============= go away axis interp =========== <<|\n");

}

//=========================================================================================================
// 单轴关节点动插补
// 除停止段每 Ts * N 个周期计算一次，即每个计算周期插补点个数为 N；
// @ incAcc    每个计算周期增加的加速度 
// @ procedure 判断进入哪个阶段(刚运行时进入变加速阶段，匀速段会自动进入，
//		 		当收到停止信号，将procedure 手动置为 STOP)
//=========================================================================================================
void Joint_JogInterp(size_t index,  
					int direction,
					double velPerc, 
					double accPerc, 
					AxisPos_Deg &oriDeg, 
					double &oriVel, 
					double &oriAcc, 
					double incAcc, 
					double Ts, 
					size_t N, // N Ts inc incAcc 
					enum JogProc &procedure, 
					RobotConfig &param)
{
	double tN = Ts * N;
	if(direction == 0) direction = -1;
	else direction = 1;
	double maxVel = direction * velPerc / 100.0 * param.Axis[index].Lim_p.vellim;
	double maxAcc = direction * accPerc / 100.0 * param.Axis[index].Lim_p.acclim;
	incAcc = direction * incAcc;
	size_t n = oriDeg.rows();
	switch(procedure)
	{
		case JOG_INCONSTACC:
		{
			double acc = oriAcc + incAcc;
			if(direction > 0 && acc > maxAcc)  acc = maxAcc;
			else if(direction < 0 && acc < maxAcc) acc = maxAcc;
			double a0 = oriDeg[index];
			double a1 = oriVel;
			double a3 = 2 * acc / (3 * tN);
			double a4 = - acc / (3 * tN * tN);
			double Vtmp = a1 + 3 * a3 * tN * tN + 4 * a4 * tN * tN * tN;
			if(fabs(Vtmp) <= fabs(maxVel))
			{
				for(size_t i = 0; i < N; ++ i)
				{
					double t = i * Ts;
					AxisPos_Deg tmp(n);
					tmp[index] = a0 + a1 * t + a3 * t * t * t + a4 * t * t * t * t;
					if(direction > 0 && tmp[index] > param.Axis[index].Lim_p.pos_max)
						tmp[index] = param.Axis[index].Lim_p.pos_max;
					else if(direction < 0 && tmp[index] < param.Axis[index].Lim_p.pos_min)
						tmp[index] = param.Axis[index].Lim_p.pos_min;
					for(size_t ai = 0; ai != oriDeg.rows(); ++ai)
						if(ai != index) tmp[ai] = oriDeg[ai];
					// PosSeq.push_back(tmp);
					interp_step(tmp);
				}
				double Ptmp = a0 + a1 * tN + a3 * tN * tN * tN + a4 * tN * tN * tN * tN;
				oriDeg[index] = Ptmp;
				oriVel = Vtmp;
				oriAcc = acc;
			}	
			else
			{
				double a3 = (maxVel - oriVel) / (tN * tN);
				double a4 = (oriVel - maxVel) / (2 * tN * tN * tN);
				for(size_t i =0; i < N; ++i)
				{
					double t = i * Ts;
					AxisPos_Deg tmp(n);
					tmp[index] = a0 + a1 * t + a3 * t * t * t + a4 * t * t * t * t;
					if(direction > 0 && tmp[index] > param.Axis[index].Lim_p.pos_max)
						tmp[index] = param.Axis[index].Lim_p.pos_max;
					else if(direction < 0 && tmp[index] < param.Axis[index].Lim_p.pos_min)
						tmp[index] = param.Axis[index].Lim_p.pos_min;
					for(size_t ai = 0; ai != n; ++ai)
						if(ai != index) tmp[ai] = oriDeg[ai];
					// PosSeq.push_back(tmp);
					interp_step(tmp);
				}
				double Ptmp = a0 + a1 * tN + a3 * tN * tN * tN + a4 * tN * tN * tN * tN;
				oriDeg[index] = Ptmp;
				oriVel = maxVel;
				oriAcc = 0;
				procedure = JOG_CONSTVEL;
			}
			break;
		}
		case JOG_CONSTVEL:
		{
			for(size_t i =0; i < N; ++i)
			{
				double t = i * Ts;
				AxisPos_Deg tmp(n);
				tmp[index] = oriDeg[index] + maxVel * t;
				if(direction > 0 && tmp[index] > param.Axis[index].Lim_p.pos_max)
					tmp[index] = param.Axis[index].Lim_p.pos_max;
				else if(direction < 0 && tmp[index] < param.Axis[index].Lim_p.pos_min)
					tmp[index] = param.Axis[index].Lim_p.pos_min;
				for(size_t ai = 0; ai != n; ++ai)
					if(ai != index) tmp[ai] = oriDeg[ai];
				// PosSeq.push_back(tmp);
				interp_step(tmp);
			}	
			double Ptmp = oriDeg[index] + maxVel * tN;
			oriDeg[index] = Ptmp;
			oriVel = maxVel;
			break;
		}
		case JOG_STOP:
		{
			double a0 = oriDeg[index];
			double a1 = oriVel;
			maxAcc = 5 * maxAcc;
			double a3 = -4 * maxAcc * maxAcc / (9 * oriVel);
			double a4 = 4 * maxAcc * maxAcc * maxAcc / (27 * oriVel * oriVel);
			double deltaT = 3 * oriVel / (2 * maxAcc);
			size_t N2 = deltaT / Ts;
			AxisPos_Deg tmp(n);
			for(size_t i = 0; i < N2; ++i)
			{	
				double t = i * Ts;
				tmp[index] = a0 + a1 * t + a3 * t * t * t + a4 * t * t * t * t;
				if(direction > 0 && tmp[index] > param.Axis[index].Lim_p.pos_max)
					tmp[index] = param.Axis[index].Lim_p.pos_max;
				else if(direction < 0 && tmp[index] < param.Axis[index].Lim_p.pos_min)
					tmp[index] = param.Axis[index].Lim_p.pos_min;
				for(size_t ai = 0; ai != n; ++ai)
					if(ai != index) tmp[ai] = oriDeg[ai];
				// PosSeq.push_back(tmp);
				interp_step(tmp);
			}
			tmp[index] = a0 + a1 * deltaT + a3 * deltaT * deltaT * deltaT + a4 * deltaT * deltaT * deltaT * deltaT;
			if(direction > 0 && tmp[index] > param.Axis[index].Lim_p.pos_max)
				tmp[index] = param.Axis[index].Lim_p.pos_max;
			else if(direction < 0 && tmp[index] < param.Axis[index].Lim_p.pos_min)
				tmp[index] = param.Axis[index].Lim_p.pos_min;
			for(size_t ai = 0; ai != n; ++ai)
				if(ai != index) tmp[ai] = oriDeg[ai];
			// PosSeq.push_back(tmp);
			interp_step(tmp);
			oriDeg[index] = tmp[index];
			oriVel = 0;
			oriAcc = 0;
			break;
		}
	}
}


void ad_Interp( double maxVel, 
				double maxAcc, 
				double &oriDeg, 
				double &tarDeg, 
				double &oriVel, 
				double &oriAcc, 
				int dir, 
				double incAcc, 
				double Ts, 
				size_t N, 
				enum JogProc &procedure, 
				std::vector<double> &SinSeq)
{
	double tN = Ts * N;
	double tmp;
	switch(procedure)
	{
		case JOG_INCONSTACC:
		{
			double acc = oriAcc + dir * incAcc;
			if(fabs(acc) > maxAcc)  acc = maxAcc * acc / fabs(acc);
			double a0 = oriDeg;
			double a1 = oriVel;
			double a3 = 2 * acc / (3 * tN);
			double a4 = - acc / (3 * tN * tN);
			double Vtmp = a1 + 3 * a3 * tN * tN + 4 * a4 * tN * tN * tN;
			if(fabs(Vtmp) <= maxVel)
			{
				for(size_t i = 0; i < N; ++ i)
				{
					double t = i * Ts;
					tmp = a0 + a1 * t + a3 * t * t * t + a4 * t * t * t * t;
					if((oriDeg - tarDeg)*(tmp - tarDeg) <= 0) tmp = tarDeg; 
					SinSeq.push_back(tmp);
				}
				double Ptmp = a0 + a1 * tN + a3 * tN * tN * tN + a4 * tN * tN * tN * tN;
				if((oriDeg - tarDeg)*(Ptmp - tarDeg) <= 0) Ptmp = tarDeg;
				oriDeg = Ptmp;
				oriVel = Vtmp;
				oriAcc = acc;
			}	
			else
			{
				double a3 = (dir * maxVel - oriVel) / (tN * tN);
				double a4 = (oriVel - dir * maxVel) / (2 * tN * tN * tN);
				for(size_t i =0; i < N; ++i)
				{
					double t = i * Ts;
					tmp = a0 + a1 * t + a3 * t * t * t + a4 * t * t * t * t;
					if((oriDeg - tarDeg)*(tmp - tarDeg) <= 0) tmp = tarDeg; 
					SinSeq.push_back(tmp);
				}
				double Ptmp = a0 + a1 * tN + a3 * tN * tN * tN + a4 * tN * tN * tN * tN;
				if((oriDeg - tarDeg)*(Ptmp - tarDeg) <= 0) Ptmp = tarDeg;
				oriDeg = Ptmp;
				oriVel = maxVel;
				oriAcc = 0;
				procedure = JOG_CONSTVEL;
			}
			break;
		}
		case JOG_CONSTVEL:
		{
			for(size_t i =0; i < N; ++i)
			{
				double t = i * Ts;
				tmp = oriDeg + maxVel * t;
				if((oriDeg - tarDeg)*(tmp - tarDeg) <= 0) tmp = tarDeg; 
				SinSeq.push_back(tmp);
			}	
			double Ptmp = oriDeg + maxVel * tN;
			if((oriDeg - tarDeg)*(Ptmp - tarDeg) <= 0) Ptmp = tarDeg;  
			oriDeg = Ptmp;
			oriVel = maxVel;
			break;
		}
		case JOG_STOP:
		{
			double a0 = oriDeg;
			double a1 = oriVel;
			double a3 = -4 * maxAcc * maxAcc / (9 * oriVel);
			double a4 = 4 * maxAcc * maxAcc * maxAcc / (27 * oriVel * oriVel);
			double deltaT = 3 * oriVel / (2 * maxAcc);
			size_t N2 = deltaT / Ts;
			for(size_t i = 0; i < N2; ++i)
			{	
				double t = i * Ts;
				tmp = a0 + a1 * t + a3 * t * t * t + a4 * t * t * t * t;
				if((oriDeg - tarDeg)*(tmp - tarDeg) <= 0) tmp = tarDeg; 
				SinSeq.push_back(tmp);
			}
			tmp = a0 + a1 * deltaT + a3 * deltaT * deltaT * deltaT + a4 * deltaT * deltaT * deltaT * deltaT;
			if((oriDeg - tarDeg)*(tmp - tarDeg) <= 0) tmp = tarDeg; 
			SinSeq.push_back(tmp);
			oriDeg = tmp;
			oriVel = 0;
			oriAcc = 0;
			break;
		}
	}
}

void Axis_Adjust_Interp( AxisPos_Deg &oriDeg, 
						AxisPos_Deg &tarDeg, 
						double velPerc,
 						double *oriVel, 
 						double *oriAcc, 
 						double incAcc, 
 						double Ts, 
 						size_t N, // N Ts inc incAcc 
 						enum JogProc *procedure, 
 						RobotConfig &param,
 						AxisPos_Deg &tmp
 						)
{
	RTIME start = rt_timer_read();
	int dir[6];
	double maxVel[6], maxAcc[6];
	std::vector<double> SinSeq[6];
	// AxisPos_Deg tmp(6);
	for(int index = 0; index < 6; ++index)
	{
		maxVel[index] = velPerc / 100.0 * param.Axis[index].Lim_p.vellim;
		maxAcc[index] = param.Axis[index].Lim_p.acclim;
		if(tarDeg[index] > oriDeg[index])  		dir[index] = 1;
		else if(tarDeg[index] == oriDeg[index])	dir[index] = 0;
		else 					  				dir[index] = -1;

		if(oriVel[index]*dir[index] < 0)  
		{
			// std::cout << "In Deg: " << oriDeg[index] << "In tar Deg " << tarDeg[index] << std::endl;
			// std::cout << "proc " << procedure[index] << std::endl;
			// std::cout << "dir " << dir[index] << std::endl;
			procedure[index] = JOG_INCONSTACC;
			oriAcc[index] = dir[index] * fabs(oriAcc[index]); // acch = dir * incAcc;
			// std::cout << "Axis " << index + 1 << " reached " << std::endl;
			// std::cout << "Accelerate = " << oriAcc[index] << std::endl;
		}
		else if(dir[index] == 0)
		{
			procedure[index] = JOG_INCONSTACC;
			oriVel[index] = 0;
			oriAcc[index] = -dir[index] * incAcc;
			// std::cout << "index:" << index+1 << " Equal!" << std::endl; 
		}
		ad_Interp( maxVel[index], 
				   maxAcc[index], 
				   oriDeg[index], 
				   tarDeg[index],
				   oriVel[index], 
				   oriAcc[index],
				   dir[index],
				   incAcc,
				   Ts,
				   N, 
				   procedure[index], 
				   SinSeq[index]); 			  // keep
	}
	RTIME end = rt_timer_read();
	std::cout <<" time : " << end-start <<" ns" << std::endl;
	for(int ia = 0; ia < SinSeq[0].size(); ++ia)
	{
		for(int index = 0; index < 6; ++index)
			tmp[index] = SinSeq[index][ia];
		std::cout << tmp.transpose() << std::endl;
		// PosSeq.push_back(tmp);
		interp_step(tmp);
	}

}



void Cart_Adjust_Interp( AxisPos_Deg &oriDeg, 
						 XyzPose &tarPos, 
						 int coor, 
						 double velPerc,
 						 double *oriVel, 
 						 double *oriAcc, 
 						 double incAcc, 
 						 double Ts, 
 						 size_t N, // N Ts inc incAcc 
 						 enum JogProc *procedure, 
 						 RobotConfig &param,
 						 AxisPos_Deg &tmp)
{
	printf("=================== enter =======================\n");
	int dir[6];
	double maxVel[6], maxAcc[6];
	std::vector<double> SinSeq[6];
	XyzPose oriPos;
	XyzPose pPos;
	// AxisPos_Deg tmp(6);
	AxisPos_Deg tarDeg(6);
	tmatrix tarMat = TermPos2TransMatrix(tarPos);
	if(coor == 2)
	{
		tmatrix oriMat = calForwardKin(oriDeg, param, oriPos);
		tarMat = oriMat * tarMat;
	}
	pPos << tarMat.topRightCorner(3,1),tr2rpy(tarMat);
	std::cout << "tarPos = " << pPos.transpose() << std::endl;
	if(calInverseKin_ER4(tarMat, param, oriDeg, tarDeg) == -1)
	{
		tarDeg = oriDeg;
		std::cout << "can't move to target pos!" << std::endl;
		return;
	}

	for(int index = 0; index < 6; ++index)
	{
		maxVel[index] = velPerc / 100.0 * param.Axis[index].Lim_p.vellim;
		maxAcc[index] = param.Axis[index].Lim_p.acclim;
		if(tarDeg[index] > oriDeg[index])  		dir[index] = 1;
		else if(tarDeg[index] == oriDeg[index])	dir[index] = 0;
		else 					  				dir[index] = -1;

		if(oriVel[index]*dir[index] < 0)  
		{
			// std::cout << "In Deg: " << oriDeg[index] << "In tar Deg " << tarDeg[index] << std::endl;
			// std::cout << "proc " << procedure[index] << std::endl;
			// std::cout << "dir " << dir[index] << std::endl;
			procedure[index] = JOG_INCONSTACC;
			oriAcc[index] = dir[index] * fabs(oriAcc[index]); // acch = dir * incAcc;
			// std::cout << "Axis " << index + 1 << " reached " << std::endl;
			// std::cout << "Accelerate = " << oriAcc[index] << std::endl;
		}
		else if(dir[index] == 0)
		{
			procedure[index] = JOG_INCONSTACC;
			oriVel[index] = 0;
			oriAcc[index] = -dir[index] * incAcc;
			// std::cout << "index:" << index+1 << " Equal!" << std::endl; 
		}
		ad_Interp( maxVel[index], 
		   maxAcc[index], 
		   oriDeg[index], 
		   tarDeg[index],
		   oriVel[index], 
		   oriAcc[index],
		   dir[index],
		   incAcc,
		   Ts,
		   N, 
		   procedure[index], 
		   SinSeq[index]); 			  // keep
	}
	for(int ia = 0; ia < SinSeq[0].size(); ++ia)
	{
		for(int index = 0; index < 6; ++index)
			tmp[index] = SinSeq[index][ia];
		// PosSeq.push_back(tmp);
		// std::cout << tmp.transpose() << std::endl;
		XyzPose Ptmp;
		calForwardKin(tmp, param, Ptmp);
		std::cout << Ptmp.transpose() << std::endl;
		interp_step(tmp);
	}

}

tmatrix calForwardKin(const AxisPos_Deg &PosAxis, RobotConfig &param,/*Robot_param*  axes,*/ XyzPose &PosCart);

/* VelLim and AccLim should be two dimenson*/
void Cart_JogInterp(size_t index, 
					int direction,
					int coor, 
					double velPerc, 
					double *VelLim/* mm/s */,
					double *AccLim,
				    AxisPos_Deg &oriDeg, 
				    double &oriVel, 
				    double &oriAcc, 
				    double incAcc, 
				    double Ts, 
				    size_t N, 
				    enum JogProc &procedure, 
				    RobotConfig  &param)
{
	//std::cout << "start cart jog" << std::endl;
	double tN = Ts * N;
	if(direction == 0)	direction = -1;
	else 				direction = 1;
	double maxVel, maxAcc;
	size_t n = oriDeg.rows();
	AxisPos_Deg last_tmp(oriDeg);
	AxisPos_Deg tmp(n);
	XyzPose OutPos;
	if(index < 3)
	{
		 maxVel = direction * velPerc / 100.0 * VelLim[0];
		 maxAcc = direction * AccLim[0];
	}
	else 
	{
		maxVel = direction * velPerc / 100.0 * VelLim[1];
		maxAcc = direction * AccLim[1];
	}
	incAcc = direction * incAcc;
	XyzPose oriPos;
	tmatrix transT = calForwardKin(oriDeg, param, oriPos);

	//std::cout << " before switch " << std::endl;
	switch(procedure)
	{
		case JOG_INCONSTACC:
		{
			double acc = oriAcc + incAcc;
			if(fabs(acc) > fabs(maxAcc))  acc = maxAcc;
			double a1 = oriVel;
			double a3 = 2 * acc / (3 * tN);
			double a4 = - acc / (3 * tN * tN);
			double Vtmp = a1 + 3 * a3 * tN * tN + 4 * a4 * tN * tN * tN;
			if(fabs(Vtmp) <= fabs(maxVel))
			{
				for(size_t i = 0; i < N; ++ i)
				{
					double t = i * Ts;
					XyzPose Ttmp;
					tmatrix transTtmp;
					Ttmp[index] = a1 * t + a3 * t * t * t + a4 * t * t * t * t;
					for(size_t ai = 0; ai != 6; ++ai)
						if(ai != index) Ttmp[ai] = 0;
					tmatrix deltaT = TermPos2TransMatrix(Ttmp);
					if(coor == BASE)  		transTtmp = deltaT * transT;
					else if(coor == TOOL)		transTtmp = transT * deltaT;
					if(calInverseKin_ER4(transTtmp, param, last_tmp, tmp) == -1)
					{
						tmp = last_tmp;
						std::cout << " Can't move to target !" << std::endl;
						break;
					}
					calForwardKin(tmp, param, OutPos);		/* for test */
					// std::cout << param.Axis[0].DH_p.d << " " << param.Axis[1].DH_p.d << " " << param.Axis[2].DH_p.d << " "
					// << param.Axis[3].DH_p.d <<" " << param.Axis[4].DH_p.d << " " << param.Axis[5].DH_p.d << std::endl;
					std::cout << "Pos = " << OutPos.transpose() << std::endl;
					last_tmp = tmp;
					// DegSeq.push_back(tmp);
					// PosSeq.push_back(OutPos);
					//std::cout << "Deg = " << tmp.transpose() << std::endl;
					interp_step(tmp);
				}
				oriDeg = tmp;
				oriVel = Vtmp;
				oriAcc = acc;
			}
			else
			{
				double a3 = (maxVel - oriVel) / (tN * tN);
				double a4 = (oriVel - maxVel) / (2 * tN * tN * tN);
				for(size_t i =0; i < N; ++i)
				{
					double t = i * Ts;
					XyzPose Ttmp;
					tmatrix transTtmp;
					Ttmp[index] = a1 * t + a3 * t * t * t + a4 * t * t * t * t;
					for(size_t ai = 0; ai != 6; ++ai)
						if(ai != index) Ttmp[ai] = 0;
					tmatrix deltaT = TermPos2TransMatrix(Ttmp);
					if(coor == BASE)  		transTtmp = deltaT * transT;
					else if(coor == TOOL)		transTtmp = transT * deltaT;
					if(calInverseKin_ER4(transTtmp, param, last_tmp, tmp) == -1)
					{
						tmp = last_tmp;
						std::cout << " Can't move to target !" << std::endl;
						std::cout << " Last_tmp = " << last_tmp.transpose() << std::endl;
						break;
					}
					calForwardKin(tmp, param, OutPos);		/* for test */
					// std::cout << param.Axis[0].DH_p.d << " " << param.Axis[1].DH_p.d << " " << param.Axis[2].DH_p.d << " "
					// << param.Axis[3].DH_p.d <<" " << param.Axis[4].DH_p.d << " " << param.Axis[5].DH_p.d << std::endl;
					std::cout << "Pos = " << OutPos.transpose() << std::endl;
					last_tmp = tmp;
					// DegSeq.push_back(tmp);
					// PosSeq.push_back(OutPos);
					//std::cout << "Deg = " << tmp.transpose() << std::endl;
					interp_step(tmp);
				}
				oriDeg = tmp;
				oriVel = maxVel;
				oriAcc = 0;
				procedure = JOG_CONSTVEL;
			}
			break;	
		}
		case JOG_CONSTVEL:
		{
			for(size_t i =0; i < N; ++i)
			{
				double t = i * Ts;
				XyzPose Ttmp;
				tmatrix transTtmp;
				Ttmp[index] = maxVel * t;
				for(size_t ai = 0; ai != 6; ++ai)
					if(ai != index) Ttmp[ai] = 0;
				tmatrix deltaT = TermPos2TransMatrix(Ttmp);
				if(coor == BASE)  		transTtmp = deltaT * transT;
				else if(coor == TOOL)		transTtmp = transT * deltaT;
				if(calInverseKin_ER4(transTtmp, param, last_tmp, tmp) == -1)
				{
					tmp = last_tmp;
					std::cout << " Can't move to target !" << std::endl;
					std::cout << " Last_tmp = " << last_tmp.transpose() << std::endl;
					break;
				}
				calForwardKin(tmp, param, OutPos);		/* for test */
				// std::cout << param.Axis[0].DH_p.d << " " << param.Axis[1].DH_p.d << " " << param.Axis[2].DH_p.d << " "
				// 	<< param.Axis[3].DH_p.d <<" " << param.Axis[4].DH_p.d << " " << param.Axis[5].DH_p.d << std::endl;
				std::cout << "Pos = " << OutPos.transpose() << std::endl;
				last_tmp = tmp;
				// DegSeq.push_back(tmp);
				// PosSeq.push_back(OutPos);
				//std::cout << "Deg = " << tmp.transpose() << std::endl;
				interp_step(tmp);
			}	
			oriDeg = tmp;
			oriVel = maxVel;
			break;
		}
		case JOG_STOP:
		{
			double a1 = oriVel;
			double a3 = -4 * maxAcc * maxAcc / (9 * oriVel);
			double a4 = 4 * maxAcc * maxAcc * maxAcc / (27 * oriVel * oriVel);
			double deltaTime = 3 * oriVel / (2 * maxAcc);
			size_t N2 = deltaTime / Ts;
			XyzPose Ttmp;
			tmatrix transTtmp;
			for(size_t i = 0; i < N2; ++i)
			{	
				double t = i * Ts;
				Ttmp[index] = a1 * t + a3 * t * t * t + a4 * t * t * t * t;
				for(size_t ai = 0; ai != 6; ++ai)
					if(ai != index) Ttmp[ai] = 0;
				tmatrix deltaT = TermPos2TransMatrix(Ttmp);
				if(coor == BASE)  		transTtmp = deltaT * transT;
				else if(coor == TOOL)		transTtmp = transT * deltaT;
				if(calInverseKin_ER4(transTtmp, param, last_tmp, tmp) == -1)
				{
					tmp = last_tmp;
					std::cout << " Can't move to target !" << std::endl;
					std::cout << " Last_tmp = " << last_tmp.transpose() << std::endl;
					//break;
				}
				calForwardKin(tmp, param, OutPos);		/* for test */
				// std::cout << param.Axis[0].DH_p.d << " " << param.Axis[1].DH_p.d << " " << param.Axis[2].DH_p.d << " "
				// 	<< param.Axis[3].DH_p.d <<" " << param.Axis[4].DH_p.d << " " << param.Axis[5].DH_p.d << std::endl;
				std::cout << "Pos = " << OutPos.transpose() << std::endl;
				last_tmp = tmp;
				// DegSeq.push_back(tmp);
				// PosSeq.push_back(OutPos);
				//std::cout << "Deg = " << tmp.transpose() << std::endl;
				interp_step(tmp);
			}
			Ttmp[index] = a1 * deltaTime + a3 * deltaTime * deltaTime * deltaTime + a4 * deltaTime * deltaTime * deltaTime * deltaTime;
			for(size_t ai = 0; ai != n; ++ai)
				if(ai != index) Ttmp[ai] = 0;
			tmatrix deltaT = TermPos2TransMatrix(Ttmp);
			if(coor == BASE)  		transTtmp = deltaT * transT;
			else if(coor == TOOL)		transTtmp = transT * deltaT;
			if(calInverseKin_ER4(transTtmp, param, last_tmp, tmp) == -1)
			{
				tmp = last_tmp;
				std::cout << " Can't move to target !" << std::endl;
				std::cout << " Last_tmp = " << last_tmp.transpose() << std::endl;
				//break;
			}
			calForwardKin(tmp, param, OutPos);		/* for test */
			// std::cout << param.Axis[0].DH_p.d << " " << param.Axis[1].DH_p.d << " " << param.Axis[2].DH_p.d << " "
			// 		<< param.Axis[3].DH_p.d <<" " << param.Axis[4].DH_p.d << " " << param.Axis[5].DH_p.d << std::endl;
			std::cout << "Pos = " << OutPos.transpose() << std::endl;
			last_tmp = tmp;
			// DegSeq.push_back(tmp);
			// PosSeq.push_back(OutPos);
			//std::cout << "Deg = " << tmp.transpose() << std::endl;
			interp_step(tmp);
			oriDeg = tmp;
			oriVel = 0;
			oriAcc = 0;
			break;
		}
	}
}

void LineInterp(const XyzPose &oriPos, 
				XyzPose &tarPos, 
				double lmaxVel/* mm/s */,
				double lmaxAcc,
				RobotConfig &param,
				double oriVel, 
				double Ts, 
				double percVel,
				int coor)
{
	size_t n = param.axis_count;
	// TerminalPose oriPos;
	// calForwardKin(oriDeg,axes,oriPos);

	if(coor == TOOL) 	frametrans(oriPos,tarPos);
	std::cout << " position interpolation " << std::endl;
	std::cout << " oriPos = " << oriPos.transpose() << std::endl;
	std::cout << " tarPos = " << tarPos.transpose() << std::endl;
	/******************************* position interpolation ****************************************/
	double ox = oriPos[0],oy = oriPos[1], oz = oriPos[2];
	double tx = tarPos[0],ty = tarPos[1], tz = tarPos[2];
	double rundis = sqrt((ox-tx)*(ox-tx) + (oy-ty)*(oy-ty) + (oz-tz)*(oz-tz));
	// calculate accelerate time
	double Vf = percVel / 100.0 * lmaxVel;
	double Tacc = 3 * fabs(Vf - oriVel) / (2 * lmaxAcc);
	double Tdec = 3 * Vf / (2 * lmaxAcc);
	double A_dis = lmaxAcc / 3 * Tacc * Tacc + oriVel * Tacc;
	double D_dis = lmaxAcc / 3 * Tdec * Tdec;
	double Tconst = (rundis - A_dis - D_dis) / Vf;
	std::vector<dvector> Pcoe(2);
	dvector  coe(6);
	coe << 0,oriVel,0, (2*lmaxAcc/(3*Tacc)), (-lmaxAcc/(3*Tacc*Tacc)),0;
	Pcoe[0] = coe;
	coe << (A_dis + Tconst*Vf) ,Vf,0, (-2*lmaxAcc / (3*Tdec)), (lmaxAcc/(3*Tdec*Tdec)),0;
	Pcoe[1] = coe;

	if(rundis < A_dis + D_dis)
	{
		Vf = sqrt(lmaxAcc * rundis);
		if(oriVel >= Vf)
		{	/* zhiyou jiansuduan */
			Tacc = 0;
			Tconst = 0;
			Tdec = 2 * oriVel / lmaxAcc;  // jiasudu jian ban 
			getQuinticCoe(0,rundis,oriVel,0,0,0,Tdec,Pcoe[1]);
		}
		else
		{	/* jiasudu jianban */
			Vf = sqrt(0.5 * lmaxAcc * rundis + 0.5 * oriVel * oriVel);
			Tacc = 2 * (Vf - oriVel) / lmaxAcc;
			Tdec = 2 * Vf / lmaxAcc;
			Tconst = 0;
			A_dis = (oriVel + Vf) * Tacc / 2;
			std::cout << A_dis << std::endl;
			getQuinticCoe(0,A_dis,oriVel,Vf,0,0,Tacc,Pcoe[0]);
			getQuinticCoe(A_dis,rundis,Vf,0,0,0,Tdec,Pcoe[1]);
		}
	}
	double tf = Tacc + Tconst + Tdec;
	size_t N1 = static_cast<size_t> (Tacc/Ts);
	size_t N2 = static_cast<size_t> ((Tacc + Tconst)/Ts);
	size_t N3 = static_cast<size_t> (tf/Ts);
	/******************************* position interpolation ****************************************/
	std::cout << " posture interpolation " << std::endl;
	/*******************************  posture interpolation  ****************************************/
	std::vector<dvector> Acoe(3);
	double ofi = oriPos[3],otheta = oriPos[4], opusi = oriPos[5];
	double tfi = tarPos[3],ttheta = tarPos[4], tpusi = tarPos[5];
	if(tfi - ofi < -180) 		tfi += 360;
	else if(tfi - ofi > 180)	tfi -= 360;
	if(ttheta-otheta < -180)	ttheta += 360;
	else if(ttheta-otheta >180)	ttheta -= 360;
	if(tpusi - opusi < -180)	tpusi += 360;
	else if(tpusi-opusi >180)	tpusi -= 360;
	getQuinticCoe(ofi,tfi,0,0,0,0,tf,Acoe[0]);
	getQuinticCoe(otheta,ttheta,0,0,0,0,tf,Acoe[1]);
	getQuinticCoe(opusi,tpusi,0,0,0,0,tf,Acoe[2]);
	
	/*******************************  posture interpolation  ****************************************/

	std::cout << " calculate each point " << std::endl;
	AxisPos_Deg last_tmp(n), zero_tmp(n),oriDeg(n),tarDeg(n);    // one argument in calInverseKin_ER4
	zero_tmp << 0,0,0,0,0,0;
	tmatrix transT = TermPos2TransMatrix(oriPos);
	calInverseKin_ER4(transT, param, zero_tmp, last_tmp);
	oriDeg = last_tmp;

	for(size_t i = 0; i != N3+1; ++i)
	{	
		/*******************************  calculate each point    ****************************************/	
		double t = i * Ts; 	 // Time
		double dtmp;		 	 // distance tmp
		XyzPose Ttmp;	 // TerminalPose tmp
		AxisPos_Deg  tmp(n);
		if(i <= N1)
			QuinticPolynomi(t,Pcoe[0],dtmp);
		else if(i <= N2)
			dtmp = A_dis + Vf * (t - Tacc);
		else if(i <= N3)
			QuinticPolynomi(t-Tacc-Tconst,Pcoe[1],dtmp);
		double r1 = dtmp / rundis;
		Ttmp[0] = (1 - r1) * ox + r1 * tx;
		Ttmp[1] = (1 - r1) * oy + r1 * ty;
		Ttmp[2] = (1 - r1) * oz + r1 * tz;
		QuinticPolynomi(t,Acoe[0],Ttmp[3]);
		QuinticPolynomi(t,Acoe[1],Ttmp[4]);
		QuinticPolynomi(t,Acoe[2],Ttmp[5]);

		/*******************************  calculate each InverseKin   ****************************************/	
		tmatrix transT = TermPos2TransMatrix(Ttmp);
		if(calInverseKin_ER4(transT, param,last_tmp, tmp) == -1)
		{
			tmp = last_tmp;
			std::cout << " Can't move to target !" << std::endl;
			break;
		}
		last_tmp = tmp;
		std::cout << tmp.transpose() << std::endl;
		interp_step(tmp);
	}

	/******************************* print some useful information *******************************************/
	//tarDeg = DegSeq.back();
	std::cout << " oriPos ";
	std::cout << oriPos.transpose() << std::endl;
	std::cout << " oriDeg ";
	std::cout << oriDeg.transpose() << std::endl;
	std::cout << " tarPos ";
	std::cout << tarPos.transpose() << std::endl;
	// std::cout << " tarDeg ";
	// std::cout << tarDeg.transpose() << std::endl;
	std::cout << " Tacc = " << Tacc << "  Tconst =  " << Tconst << "  Tdec =  " << Tdec << std::endl;
}



int interp_compute(ROBOT_INST &temp_inst){
	switch(temp_inst.ri_type){
		case PTP: {
			AxisInterp(temp_inst.args[0].apv, 
					   temp_inst.args[1].apv,
					   rc_core.Ts,
					   rc_core.vper*0.5,
					   rc_core.aper,
					   rc_runtime_param
						);
			break;
		}
		case LIN: {
			LineInterp(temp_inst.args[0].cpv, 
					   temp_inst.args[1].cpv,
					   500,
					   100,
					   rc_runtime_param,
					   0,
					   rc_core.Ts,
					   rc_core.vper,
					   rc_core.coordinate
					   );
			break;
		}
		case CIRC:

			break;
		case JOINTJOG:{
			double oriVel = 0;
			double oriAcc = 0;
			int count = 0;
			std::cout << "pos " << temp_inst.args[0].apv.transpose() << std::endl;
			do{
				Joint_JogInterp(temp_inst.args[0].jjp.jointindex,  
					temp_inst.args[0].jjp.direction,
					rc_core.vper, 
					rc_core.aper, 
					temp_inst.args[0].apv, 
					oriVel, 
					oriAcc, 
					2, 
					rc_core.Ts, 
					20, // N Ts inc incAcc 
					rc_core.procedure, 
					rc_runtime_param);
					count ++;
					printf("**************************************%d*****************\n",count);

			} while(rc_core.jog_startup);
			rc_core.procedure = JOG_STOP;
			Joint_JogInterp(temp_inst.args[0].jjp.jointindex,  
					temp_inst.args[0].jjp.direction,
					rc_core.vper, 
					rc_core.aper, 
					temp_inst.args[0].apv, 
					oriVel, 
					oriAcc, 
					2, 
					rc_core.Ts, 
					20, // N Ts inc incAcc 
					rc_core.procedure, 
					rc_runtime_param);
			break;
		}
		case CARTJOG: {
			double Vellim[2] = {200,50};
			double Acclim[2] = {80,80};
			double oriVel = 0;
			double oriAcc = 0;
			int count = 0;
			do{
				std::cout << "pos " << temp_inst.args[0].apv.transpose() << std::endl;
			 	Cart_JogInterp(temp_inst.args[0].jjp.jointindex, 
						temp_inst.args[0].jjp.direction,
						rc_core.coordinate, 
						rc_core.vper,
						Vellim/* mm/s */,
						Acclim,
				        temp_inst.args[0].apv, 
				        oriVel, 
				        oriAcc, 
				        2, 
				        rc_core.Ts, 
				        20, 
				        rc_core.procedure,
				        rc_runtime_param);
			 			count ++;
						printf("**************************************%d*****************\n",count);
			}while(rc_core.jog_startup);
			rc_core.procedure = JOG_STOP;
			Cart_JogInterp(temp_inst.args[0].jjp.jointindex, 
						temp_inst.args[0].jjp.direction,
						rc_core.coordinate, 
						rc_core.vper, 
						Vellim/* mm/s */,
						Acclim,
				        temp_inst.args[0].apv, 
				        oriVel, 
				        oriAcc, 
				        2, 
				        rc_core.Ts, 
				        20, 
				        rc_core.procedure,
				        rc_runtime_param);
			break;
		}
		case AXIS_ADJUST: 
		{
			double oriVel[6] = {0};
			double oriAcc[6] = {0};
			enum JogProc proc[6] = {JOG_INCONSTACC,JOG_INCONSTACC,JOG_INCONSTACC,
									JOG_INCONSTACC,JOG_INCONSTACC,JOG_INCONSTACC};
			AxisPos_Deg tmp(6);
			bool first_time = true;
			bool flag = false;
			do {
				if(!first_time) {
					temp_inst.args[0].apv = tmp;
				}
				first_time = false;
				std::cout << "oripos = " << temp_inst.args[0].apv << std::endl;
				Axis_Adjust_Interp(temp_inst.args[0].apv, 
						   temp_inst.args[1].apv, 
						   rc_core.vper,
 						   oriVel, 
 						   oriAcc, 
 						   2, 
				           rc_core.Ts,  
 						   20, // N Ts inc incAcc 
 						   proc, 
 						   rc_runtime_param,
 						   tmp);
				
				inst_buffer_read_nonblock(temp_inst, flag);

			} while(flag);
			break;
		}
		case CART_ADJUST:
		{
			double oriVel[6] = {0};
			double oriAcc[6] = {0};
			enum JogProc proc[6] = {JOG_INCONSTACC,JOG_INCONSTACC,JOG_INCONSTACC,
									JOG_INCONSTACC,JOG_INCONSTACC,JOG_INCONSTACC};
			AxisPos_Deg tmp(6);
			bool first_time = true;
			bool flag = false;
			do
			{
				if(!first_time) {
					temp_inst.args[0].apv = tmp;
				}
				first_time = false;
				Cart_Adjust_Interp(temp_inst.args[0].apv, 
			  					 temp_inst.args[1].cpv,
			  					 temp_inst.args[0].jjp.refsys,
			  				 	 1,
				   				 oriVel, 
				   				 oriAcc, 
				   				 2, 
	           					 rc_core.Ts,  
				   				 20, // N Ts inc incAcc 
				   				 proc, 
				  			     rc_runtime_param,
				  			     tmp);
				printf("******************** Adjust  ************************\n");
				inst_buffer_read_nonblock(temp_inst, flag);
			}while(flag);
			break;
		}
	}
}
