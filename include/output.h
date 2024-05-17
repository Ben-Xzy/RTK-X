#ifndef OUTPUT_H
#define OUTPUT_H
#include<iostream>
#include"DataClassSet.h"
#include<ostream>
#include<fstream>
#include"param.h"
void OutPutResult(EPOCHOBSDATA* EachObs, POSRES Res, ofstream& outfile, int flag);
void OutputRTK(GPSTIME T, DDCEPOCHOBS D, ofstream& outfile, string s, POSRES& r);
void LibOutput(GPSTIME t, DDCEPOCHOBS &D, ofstream& outfile,RTKEKF &e);
void LibOutput(GPSTIME t, DDCEPOCHOBS& D, ofstream& outfile, POSRES& r);
#endif