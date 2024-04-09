#include"Matrix.h"
using namespace std;
bool XMatrix::MatrixPlus(XMatrix& newV, XMatrix& res)
{
	int row1 = this->row;
	int col1 = this->col;
	if (row1 == newV.row && col1 == newV.col)
	{
		res.col = col;
		res.row = row;
		for (int i = 0; i < row1; i++)
		{
			for (int j = 0; j < col1; j++)
			{
			
				res(i,j) = this->matrix[i * col1 + j] + newV.matrix[i * col1 + j];
			}
		}
	}
	else {
		cout << "The matrix addition operation is in the wrong format" << endl;
	}
	return true;
}
XMatrix XMatrix::operator+(XMatrix &newV)
{
	XMatrix res;
	row = this->row;
	col = this->col;
	if (row == newV.row && col == newV.col)
	{
		res.col = col;
		res.row = row;
		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < col; j++)
			{
				res(i,j) = this->matrix[i * col + j] + newV.matrix[i * col + j];
			}
		}
	}
	else {
		cout << "The matrix addition operation is in the wrong format" << endl;
	}

	return res;
}
bool XMatrix::MatrixMius(XMatrix& newV, XMatrix& res)
{
	row = this->row;
	col = this->col;
	if (row == newV.row && col == newV.col)
	{
		res.col = col;
		res.row = row;
		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < col; j++)
			{
				res(i,j) = this->matrix[i * col + j] - newV.matrix[i * col + j];
			}
		}
	}
	else {
		cout << "The matrix subtraction operation is formatted incorrectly" << endl;
	}
	return true;
}
XMatrix XMatrix::operator-(XMatrix& newV)
{
	XMatrix res;
	row = this->row;
	col = this->col;
	if (row == newV.row && col == newV.col)
	{
		res.col = col;
		res.row = row;
		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < col; j++)
			{
				res(i,j) = this->matrix[i * col + j] - newV.matrix[i * col + j];
			}
		}
	}
	else {
		cout << "The matrix addition operation is in the wrong format" << endl;
	}

	return res;
}
bool XMatrix::MatrixMul(XMatrix& newV, XMatrix& res)
{
	row = this->row;
	col = this->col;
	if (col == newV.row )
	{
		res.col = newV.col;
		res.row = this->row;
		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < newV.col; j++)
			{
				double temp = 0;
				for (int m = 0;m < col; m++)
				{
					temp = temp + this->matrix[i * col + m] * newV.matrix[m * newV.col + j];
				}
				res(i,j) = temp;
			}
		}
	}
	else {
		cout << "The matrix multiplication operation is formatted incorrectly" << endl;
	}
	return true;
}
XMatrix XMatrix::operator*(XMatrix& newV)
{
	XMatrix res;
	row = this->row;
	col = this->col;
	if (col == newV.row)
	{
		res.row = row;
		res.col = newV.col;
		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < newV.col; j++)
			{
				double temp = 0;
				for (int m = 0; m < col; m++)
				{
					temp = temp + this->matrix[i * col + m] * newV.matrix[m * newV.col + j];
				}
				res(i,j) = temp;
			}
		}
	}
	else {
		cout << "The matrix multiplication operation is formatted incorrectly" << endl;
	}
	return res;
}
bool XMatrix::MatrixInv()//矩阵求逆
{
	vector<double> a(this->matrix.size());
	vector<double> b(a.size());
	//res.col = this->col;
	//res.row = this->row;
	//res.matrix.resize(this->matrix.size());
	int n = row;
	copy(this->matrix.begin(), this->matrix.end(), a.begin());
	


	if (this->col == this->row)
	{


		int i, j, k, l, u, v, is[150], js[150];   /* matrix dimension <= 10 */
		double d, p;

		if (n <= 0)
		{
			printf("Error dimension in MatrixInv!\n");
			return 0;
		}

		/* 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变 */
		for (i = 0; i < n; i++)
		{
			for (j = 0; j < n; j++)
			{
				b[i * n + j] = a[i * n + j];
			}
		}

		for (k = 0; k < n; k++)
		{
			d = 0.0;
			for (i = k; i < n; i++)   /* 查找右下角方阵中主元素的位置 */
			{
				for (j = k; j < n; j++)
				{
					l = n * i + j;
					p = fabs(b[l]);
					if (p > d)
					{
						d = p;
						is[k] = i;
						js[k] = j;
					}
				}
			}

			if (d < DBL_EPSILON)   /* 主元素接近于0，矩阵不可逆 */
			{
				printf("Divided by 0 in MatrixInv!\n");
				return 0;
			}

			if (is[k] != k)  /* 对主元素所在的行与右下角方阵的首行进行调换 */
			{
				for (j = 0; j < n; j++)
				{
					u = k * n + j;
					v = is[k] * n + j;
					p = b[u];
					b[u] = b[v];
					b[v] = p;
				}
			}

			if (js[k] != k)  /* 对主元素所在的列与右下角方阵的首列进行调换 */
			{
				for (i = 0; i < n; i++)
				{
					u = i * n + k;
					v = i * n + js[k];
					p = b[u];
					b[u] = b[v];
					b[v] = p;
				}
			}

			l = k * n + k;
			b[l] = 1.0 / b[l];  /* 初等行变换 */
			for (j = 0; j < n; j++)
			{
				if (j != k)
				{
					u = k * n + j;
					b[u] = b[u] * b[l];
				}
			}
			for (i = 0; i < n; i++)
			{
				if (i != k)
				{
					for (j = 0; j < n; j++)
					{
						if (j != k)
						{
							u = i * n + j;
							b[u] = b[u] - b[i * n + k] * b[k * n + j];
						}
					}
				}
			}
			for (i = 0; i < n; i++)
			{
				if (i != k)
				{
					u = i * n + k;
					b[u] = -b[u] * b[l];
				}
			}
		}

		for (k = n - 1; k >= 0; k--)  /* 将上面的行列调换重新恢复 */
		{
			if (js[k] != k)
			{
				for (j = 0; j < n; j++)
				{
					u = k * n + j;
					v = js[k] * n + j;
					p = b[u];
					b[u] = b[v];
					b[v] = p;
				}
			}
			if (is[k] != k)
			{
				for (i = 0; i < n; i++)
				{
					u = i * n + k;
					v = is[k] + i * n;
					p = b[u];
					b[u] = b[v];
					b[v] = p;
				}
			}
		}
		copy(b.begin(), b.end(),this->matrix.begin());
	}
	else
	{
	cout << "The matrix format does not conform to the inversion rules" << endl;
	}
	return true;
}
void XMatrix::MatrixDis()
{
	cout.precision(5);
	cout<<std::fixed;
	col = this->col;
	row = this->row;
	cout << "[  ";
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			cout << this->matrix[i * col + j]<<"  ";
		}
		if (i < row - 1)
		{
			cout << endl<<"  ";
		}
	}

	cout << "]"<<endl;
}
XMatrix::XMatrix(double value[], int row1, int col1)

{
	this->row = row1;
	this->col = col1;
	this->matrix.resize(row * col);
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			matrix[i * col + j] = value[i * col + j];
		}
	}
}
XMatrix::XMatrix(int row1,int col1)
{
	this->row = row1;
	this->col = col1;
	this->matrix.resize(row * col);
}
XMatrix::XMatrix()
{ 
	this->row = 1; 
	this->col = 1;
}
double& XMatrix::operator()(int r, int c)
{/*由于是从0开始索引，故对应值要+1*/
	int row1 = this->row;
	int col1 = this->col;
	this->matrix.resize(row1 * col1);
	if (r+1<=row1 && c+1<=col1)
	{
		return this->matrix[r *col1+c];
	}
	if(c+1<=col1)
	{
		MatrixResize((r+1) , col1);
		this->row = r+1;
		this->col = col1;
		return   this->matrix[r *col1+ c];
	}
	else if (c+1 > col && r+1 >row)
	{
		MatrixResize((r + 1) , (c + 1));
		this->row = r + 1;
		this->col = c + 1;
		return   this->matrix[r * (c+1) + c];//c+1才是实际列数，c只是列的序号(从0开始）
	}
	else if (c + 1 > col && r +1 <= row)
	{
		MatrixResize(row1 , c + 1);
		this->row = row1;
		this->col = c + 1;
		return   this->matrix[r * (c + 1) + c];//c+1才是实际列数，c只是列的序号(从0开始）
	}
}
double XMatrix::operator()(int r, int c) const
{
	int row1 = this->row;
	int col1 = this->col;
	if (r > row1 || c > col1)
	{
		cout << "The input matrix index is out of bounds" << endl;
	}
	else
	{
		return  this->matrix[r * col1+c];
	}
}
bool XMatrix::MatrixTrans()
{
	int row = this->row;
	int col = this->col;//暂时储存原行列数
	int num = 0;
	if (row > col)
	{ 
	num = this->row;
		//for (int m = 0; m < this->row; m++)
		//{
		//	for (int n = 0; n < row - col; n++) 
		//	{
		//	this->matrix.insert(this->matrix.begin() + m*this->col+col, 0);
		//	}
		//}
	MatrixResize(num, num);
	}//多出的位置用0填充
	else
	{
	num = this->col;
	MatrixResize(num , num);
	}           //先令行列数变为一致
	for (int i = 0; i < num; i++)
	{
		for (int j = i; j < num; j++)
		{
			double TransTemp=this->matrix[i*num+j];
			this->matrix[i * num + j] = this->matrix[j * num + i];
			this->matrix[j * num + i] = TransTemp;
		}
	}
	//if (col > row)//如果是列数大于行数，需要删掉互换后的一些列
	//{
	//	for (int m = 0; m < this->row; m++)
	//	{
	//		for (int n = 0; n < col - row; n++)
	//		{
	//			if (m * row + this->col - 1 - n == this->matrix.size()-1)
	//			{
	//				this->matrix.pop_back();
	//			}
	//			else {
	//				this->matrix.erase(this->matrix.begin() + m * row + this->col - 1 - n);
	//			}
	//		}
	//	}
	//}
	//else { this->matrix.resize(row * col); }
	MatrixResize(col, row);
	this->col = row;
	this->row = col;//转置，将矩阵形状行列数互换
	return true;
}
void XMatrix::MatrixResize(int Corrow,int Corcol)
{
	matrix.resize(this->col * this->row);//提高容错，预防矩阵未初始化
	/*先处理列*/
	int ColIndex = Corcol - this->col;//若ColIndex>0则相当于扩列，反之为缩列
	if (ColIndex > 0)//扩列
	{
		for (int m = 0; m < this->row; m++)
		{
			for (int n = 0; n < ColIndex; n++)
			{
				this->matrix.insert(this->matrix.begin() + (m +1)*this->col+m*ColIndex , 0);
			}
		}
	}
	else//缩列
	{
		ColIndex = -ColIndex;
		for (int m = 0; m < this->row; m++)
		{
			for (int n = 0; n <ColIndex; n++)
			{
				if (m * Corcol + this->col - 1 - n == this->matrix.size() - 1)
				{
					this->matrix.pop_back();
				}
				else {
					this->matrix.erase(this->matrix.begin() + m *Corcol + this->col - 1 - n);
				}
			}
		}
	}
	this->col = Corcol;
	/*再处理行*/
	this->matrix.resize(Corrow * this->col);
	this->row = Corrow;
}
/***************
数组类型的矩阵乘法：
提高程序兼容性
*************/
void MatrixMultiply(int r1, int c1, int r2, int c2, double Z[], double a[], double z[])
{
	XMatrix Z_mat(Z,r1,c1);
	XMatrix a_mat(a, r2, c2);
	XMatrix z_mat;
	z_mat = Z_mat * a_mat;
	Matrix2Array(z_mat, z);
}
void MatrixInv(int n, double Z[], double ZT[])
{
	XMatrix  Z_mat(Z, n, n);
	Z_mat.MatrixInv();
	Matrix2Array(Z_mat,ZT);
}
void Matrix2Array(XMatrix X, double *a)
{
	for (int i = 0; i < X.row; i++)
	{
		for (int j = 0; j < X.col; j++)
		{
			a[i * X.col + j] = X(i, j);
		}
	}
}
void EyeMat(int n, XMatrix& m)
{
	for (int i = 0; i < n; i++)
	{
		m(i, i) = 1;
	}
}