#include"mathf.h"
/****************
Least Squares Correction Numbers:
B: Variable coefficient matrix
P: Weight matrix
W: Residual array
x: the number of result corrections
Q: Covariance matrix
******************/
void LSQCalx(XMatrix B_r, XMatrix P_r, XMatrix W_r, XMatrix& x, XMatrix& Q)
{
	XMatrix B = B_r, P = P_r, W = W_r;
	XMatrix temp;
	B.MatrixTrans();
	temp = B * P;
	B.MatrixTrans();
	temp = temp * B;
	temp.MatrixInv();
	Q = temp;
	B.MatrixTrans();
	x = Q * B * P * W;
}
/***************
Least Squares Accuracy Evaluation:
ls: LSQ Necessary Matrix Collection: B, P, W, X, etc
theta: Medium error
PDOP: Geometric Accuracy Factor
model: Positioning mode, SPP(0), RTK(1)
*****************/
void LSQCalPrCis(LSQ& ls, double& theta, double& PDOP, int model)
{
	int s = 0, t = 0;//Total number of observations, number of necessary observations
	XMatrix vPv;
	XMatrix Qnn_t;
	t = ls.x.row;
	ls.v = ls.B * ls.x - ls.W;
	s = ls.v.row;
	ls.v.MatrixTrans();
	vPv = ls.v * ls.P;
	ls.v.MatrixTrans();
	vPv = vPv * ls.v;
	PDOP = sqrt(ls.Q(0, 0) * ls.Q(0, 0) + ls.Q(1, 1) * ls.Q(1, 1) + ls.Q(2, 2) * ls.Q(2, 2));
	theta = sqrt(vPv(0, 0) / (s - t));
	if (model == 1)
	{
		for (int i = 0; i < ls.Q.row - 3; i++)
		{
			for (int j = 0; j < ls.Q.row - 3; j++)
			{
				Qnn_t(i, j) = ls.Q(i + 3, j + 3);
			}
		}
		ls.Qnn = Qnn_t;
	}
}
void LSQDstroy(LSQ& ls)
{
	std::vector<double>().swap(ls.B.matrix); ls.B.row = ls.B.col = 0;
	std::vector<double>().swap(ls.P.matrix); ls.P.row = ls.P.col = 0;
	std::vector<double>().swap(ls.W.matrix); ls.W.row = ls.W.col = 0;
	std::vector<double>().swap(ls.v.matrix); ls.v.row = ls.v.col = 0;
	std::vector<double>().swap(ls.Q.matrix); ls.Q.row = ls.Q.col = 0;
	std::vector<double>().swap(ls.Qnn.matrix); ls.Qnn.row = ls.Qnn.col = 0;
	std::vector<double>().swap(ls.x.matrix); ls.x.row = ls.x.col = 0;

}