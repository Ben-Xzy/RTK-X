#ifndef READBINARY
#define READBINARY
#include <iostream>
#include <cmath>
#include <string>
#include"DataClassSet.h"
#include"param.h"

using namespace std;
static unsigned short U2(unsigned char* p) ;
static unsigned int   U4(unsigned char* p) ;
static int            I4(unsigned char* p) ;
static float          R4(unsigned char* p) ;
static double         R8(unsigned char* p) ;
unsigned int crc32(unsigned char* buff, int len);
int ReadRange(unsigned char* buff, EPOCHOBSDATA* obs);
int ReadGPSEphem(unsigned char* buff,GPSEPHREC *GPSep);
int ReadBDSEphem(unsigned char* buff,GPSEPHREC *BDSep);
int ReadRawEphem(unsigned char* buff);
int ReadRangeCMP(unsigned char* buff);
int ReadPSRPOS(unsigned char* buff,POSRES* posres);
int ReadBestPOS(unsigned char* buff, POSRES* posres);
//int DecodeNovOem7Dat(char* filename, EPOCHOBSDATA* obs, GPSEPHREC GPSep[], GPSEPHREC BDSep[]);
int DecodeNovOem7Dat(unsigned char Buff[], int& Len,EPOCHOBSDATA* obs, GPSEPHREC GPSep[], GPSEPHREC BDSep[],POSRES *posres);
void JudgeFreq(EPOCHOBSDATA* Epoch);
#endif