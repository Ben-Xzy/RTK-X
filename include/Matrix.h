#ifndef MATRIX_H
#define MATRIX_H
#include<vector>
#include"math.h"
#include"string.h"
#include<iostream>

class XMatrix 
{
public:
	
	int row;
	int col;
	std::vector <double>  matrix;

	XMatrix(double value[], int row1, int col1);
	XMatrix(int row1, int col1);
	XMatrix();
	double& operator()(int r, int c);//返回引用类型,便于调用元素并修改
	double operator()(int r, int c) const;//返回值类型，便于查看元素,不会修改任何元素
	bool MatrixMul(XMatrix& newV, XMatrix& res);//res代表输出的矩阵结果
	XMatrix operator*(XMatrix& newV);
	bool MatrixPlus(XMatrix& newV, XMatrix& res);
	XMatrix operator+(XMatrix& newV);
	bool MatrixMius(XMatrix& newV, XMatrix& res);
	XMatrix operator-(XMatrix& newV);
	bool MatrixInv();
	void MatrixDis();
	bool MatrixTrans();
	void MatrixResize(int Corrow,int Corcol);
};
void MatrixMultiply(int r1, int c1, int r2, int c2, double Z[],double a[],double z[]);
void MatrixInv(int n,double Z[],double ZT[]);
void Matrix2Array(XMatrix X, double* a);
void EyeMat(int n, XMatrix& m);
#endif // !MATRIX_H
