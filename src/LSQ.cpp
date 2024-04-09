#include"mathf.h"
/****************
��С���˻�ø�������
B:����ϵ������
P��Ȩ����
W���в���
x�����������
Q��Э������
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
��С���˾���������
ls:LSQ��Ҫ���󼯺ϣ�B��P��W��x�ȵ�
theta:�����
PDOP:���ξ�������
model:��λģʽ��SPP��0����RTK��1��
*****************/
void LSQCalPrCis(LSQ& ls, double& theta, double& PDOP, int model)
{
	int s = 0, t = 0;//�ܹ۲�������Ҫ�۲���
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