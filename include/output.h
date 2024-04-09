#pragma once
#include<iostream>
#include"DataClassSet.h"
#include<ostream>
#include<fstream>
#include"param.h"
void OutPutResult(EPOCHOBSDATA* EachObs, POSRES Res, ofstream& outfile, int flag);
void OutputRTK(GPSTIME T, DDCEPOCHOBS D, ofstream& outfile, string s);
