#include"mathf.h"
/****************
最小二乘获得改正数：
B:变量系数矩阵
P：权矩阵
W：残差阵
x：结果改正数
Q：协方差阵
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
最小二乘精度评定：
ls:LSQ必要矩阵集合：B，P，W，x等等
theta:中误差
PDOP:几何精度因子
model:定位模式，SPP（0），RTK（1）
*****************/
void LSQCalPrCis(LSQ& ls, double& theta, double& PDOP, int model)
{
	int s = 0, t = 0;//总观测数，必要观测数
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