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
	double& operator()(int r, int c);//Returns a reference type that makes it easy to call the element and modify it
	double operator()(int r, int c) const;//Returns a value type that makes it easy to view the element and does not modify any element
	bool MatrixMul(XMatrix& newV, XMatrix& res);
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
