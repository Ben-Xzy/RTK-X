#include"mathf.h"
using namespace std;
void inputRDot(XMatrix& RDot, double OMEGAk, double ik, double orbit_xk, double orbit_yk)
{
	RDot(0, 0) = cos(OMEGAk);   RDot(0, 1) = -sin(OMEGAk) * cos(ik);   RDot(0, 2) = -(orbit_xk * sin(OMEGAk) + orbit_yk * cos(OMEGAk) * cos(ik));  RDot(0, 3) = orbit_yk * sin(OMEGAk) * sin(ik);
	RDot(1, 0) = sin(OMEGAk);   RDot(1, 1) = cos(OMEGAk) * cos(ik);   RDot(1, 2) = (orbit_xk * cos(OMEGAk) - orbit_yk * sin(OMEGAk) * cos(ik));  RDot(1, 3) = -orbit_yk * cos(OMEGAk) * sin(ik);
	RDot(2, 0) = 0;   RDot(2, 1) = sin(ik);   RDot(2, 2) = 0;  RDot(2, 3) = orbit_yk * cos(ik);
}
void inputRx(XMatrix& Rx, double phiX)
{
	/*Rx mat*/
	Rx(0, 0) = 1; Rx(0, 1) = 0; Rx(0, 2) = 0;
	Rx(1, 0) = 0; Rx(1, 1) = cos(phiX); Rx(1, 2) = sin(phiX);
	Rx(2, 0) = 0; Rx(2, 1) = -sin(phiX); Rx(2, 2) = cos(phiX);
}
void inputRz(XMatrix& Rz, double phiZ)
{
	/*Rz mat*/
	Rz(0, 0) = cos(phiZ); Rz(0, 1) = sin(phiZ); Rz(0, 2) = 0;
	Rz(1, 0) = -sin(phiZ); Rz(1, 1) = cos(phiZ); Rz(1, 2) = 0;
	Rz(2, 0) = 0; Rz(2, 1) = 0; Rz(2, 2) = 1;
}
void inputRzDot(XMatrix& RzDot, double Coe, double tk)
{
	RzDot(0, 0) = -sin(Coe * tk) * Coe; RzDot(0, 1) = cos(Coe * tk) * Coe; RzDot(0, 2) = 0;
	RzDot(1, 0) = -cos(Coe * tk) * Coe; RzDot(1, 1) = -sin(Coe * tk) * Coe; RzDot(1, 2) = 0;
	RzDot(2, 0) = RzDot(2, 1) = RzDot(2, 2) = 0;
}
void CalRho(double pos_r[], double pos_s[], double& rho)
{
	rho = sqrt(pow(pos_r[0] - pos_s[0], 2) + pow(pos_r[1] - pos_s[1], 2) + pow(pos_r[2] - pos_s[2], 2));
}
bool TurboEdit(double dMW_N, double dGF, double N_theta,double GF_Prcple,bool &errFlag)
{
	if (fabs(dMW_N) > sqrt(N_theta) * 4.0)
	{
		if (fabs(dMW_N) > sqrt(N_theta) * 60)
		{
			errFlag = false;
		}
		return false;
	}
	if (fabs(dGF) > 2.0 * GF_Prcple)
	{
		if (fabs(dGF) > sqrt(N_theta) * 20)
		{
			errFlag == false;
		}
		return false;
	}
	return true;
}