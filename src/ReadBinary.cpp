#include"ReadBinary.h"
#include"SPP.h"
using namespace std;

/*转换函数，字节转换成特定类型,因为每个Field Name会有不同的字节数*/
static unsigned short U2(unsigned char* p) { unsigned short u; memcpy(&u, p, 2); return u; } 
static unsigned int   U4(unsigned char* p) { unsigned int   u; memcpy(&u, p, 4); return u; }
static int            I4(unsigned char* p) { int            i; memcpy(&i, p, 4); return i; }
static float          R4(unsigned char* p) { float          r; memcpy(&r, p, 4); return r; }
static double         R8(unsigned char* p) { double         r; memcpy(&r, p, 8); return r; }
unsigned crc32(unsigned char* buff, int len)
{
	int i, j;
	unsigned int crc = 0;
	for (i = 0; i < len; i++)
	{
		crc ^= buff[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
			else crc >>= 1;
		}
	}
	return crc;
}
/****************
实时数据流读取时，不一定会充满Maxrawlen个字节，
所以要以传入的d为最大容量，d输出的时候是截断长度大小
*******************/
int DecodeNovOem7Dat(unsigned char buff[], int& d, EPOCHOBSDATA* obs, GPSEPHREC GPSep[], GPSEPHREC BDSep[],POSRES *posres)
{
        /*正式开始读取缓存区内容部分*/
        bool RangeFlag = false;/*判断是否读到观测数据*/
        int max = d;
        d = 0;//将d重置为零，便于后续判断是否有残留数据块
        for (int i = 0; i < max - 2; i++)/*进行数据头同步*/
        {
            if (buff[i + 0] == SYNC1 && buff[i + 1] == SYNC2 && buff[i + 2] == SYNC3)
            {
                unsigned char* RealbuffAddress = buff + i;/*临时在该循环中继承文件头开始的地址*/
                /*确认数据头后，开始正式的解码*/
                int a = 4;/*Bina*/
                unsigned short msgID = U2(RealbuffAddress + a);     a += 2; /*MessageID 占两个字节*/
                char msgtype = I1(RealbuffAddress + a);
                /*判断读取是二进制文件还是ASCII码文件,0为binary，1为ascll，2为NMEA，3为reserved*/
                int type = (msgtype >> 4) & (0x3);         a += 2;
                unsigned short msglen = U2(RealbuffAddress + a);    a += 6;
                unsigned short week = U2(RealbuffAddress + a);      a += 2;
                unsigned int ms = U4(RealbuffAddress + a);
                /*判断读取数据块是否为最后一块被截断的数据*/
                if (msglen + i + HeadLen+4  > max)
                {
                    d = max - i;
                }
                else
                {
                    /*crc检验*/
                    unsigned int CRC0 = U4(RealbuffAddress + msglen + HeadLen);/*获取文件数据中的CRC码*/
                    unsigned int CRC = crc32(RealbuffAddress, msglen + HeadLen);
                    if (CRC0 != CRC)                                   //检验错误直接读取下一组数据
                    {
                        cout << "CRC check error!" << endl;
                        continue;
                    }
                    GPSEPHREC tempEp;
                    switch (msgID)
                    {
                    case ID_GPSephem:
                        
                        ReadGPSEphem(RealbuffAddress, &tempEp);
                        if (tempEp.PRN > 0 && tempEp.PRN < 40)//只有有效的星历才计数
                        {
                            GPSep[tempEp.PRN] = tempEp;
                            /*         cout << GPSRecord << endl;*/
                        }
                        break;
                    case ID_BDSephem:
                        ReadBDSEphem(RealbuffAddress, &tempEp);
                        if (tempEp.PRN > 0 && tempEp.PRN < 70)
                        {
                            BDSep[tempEp.PRN] = tempEp;
                            /*    cout << BDSRecord << endl;*/
                        }
                        break;
                    case ID_Rawephem:
                        ReadRawEphem(RealbuffAddress); break;
                    case ID_Range:
                        GetFormlock(obs);
                        memset(obs->Satobs, 0, MAXCHANNUM * sizeof(SATOBSDATA));
                        memset(obs->SatPvT, 0, MAXCHANNUM * sizeof(SATMIDRES));
                        obs->SatNum = 0;
                        obs->Time.Week = week;
                        obs->Time.SecOfWeek = ms / 1000.0;//这里注意量级
        /*                 cout << obs->Time.Week << "\t" << obs->Time.SecOfWeek << endl;*/
                        ReadRange(RealbuffAddress, obs);
                        d = max-(msglen + i + HeadLen);
                        RangeFlag = true;
                        break;
                    case ID_RangeCMP:
                        ReadRangeCMP(RealbuffAddress); break;
                    case ID_BestPos:
                        if (!posres->realPosFlag)
                        {
                            ReadBestPOS(RealbuffAddress, posres);
                        }
                        break;
                    default:break;
                    }
                }
                if (d > 0)
                {
                    for (int j = 0; j < d; j++)
                    {
                        buff[j] = buff[max - d + j];
                    }
                    break;/*将截断的数据放在缓存区的开头*/
                }
            }
            else continue;
        }
        if (RangeFlag == true) 
        { 
            JudgeFreq(obs);/*判断所得观测数据是单品还是双频*/
            return 1;
        }
        else { return -1; }//读完缓存区的数据后，存取截断数据进行下一次读取
}

int ReadRange(unsigned char* buff, EPOCHOBSDATA* obs)
{
    unsigned char* H = buff + HeadLen;
    unsigned int NumOfObs = U4(H);
    int LInd = 0;
    double LValue = 0;
    bool CountFlag = true;
    for (unsigned int i = 0; i < NumOfObs; i++)
    {
        CountFlag = true;/*每次读观测数据时先将计数标志置true*/
        int n = 0;
        GNSSSys SYS;
        unsigned short PRN = U2(H + 4);
        unsigned int track = U4(H + 44);                        /*跟踪情况集合*/
        unsigned int sys = (track >> 0x10) & (0x7);                 /*判断系统*/
        unsigned int phase_lock = (track >> (0xA)) & (0x1);			/*探测载波相位的失锁情况*/
        unsigned int parity = (track >> (0xB)) & (0x1);            /*周跳质量检测*/
        unsigned int code_lock = (track >> (0xC)) & (0x1);          /*探测伪距码的失锁情况*/
        unsigned int sigtype = (track >> 0x15) & (0x1F);
        if (sys == 0) SYS = GPS;
        else if (sys == 4) SYS = BDS;
        else SYS = UNKS;
        for (int j = 0; j < MAXCHANNUM; j++)  /*memset后enum也会被默认赋值0，故将UNKS放在第一位可使默认为UNKS*/
        {
            if (SYS == obs->Satobs[j].System && PRN == obs->Satobs[j].Prn)
            {
                n = j; 
                CountFlag = false;
                break;
            }
            if (obs->Satobs[j].System == UNKS && obs->Satobs[j].Prn == 0)
            {
                n = j;
                break;
            }
        }
        double psr = R8(H + 8);                                   /*伪距测量值*/
        double adr =-1.0* R8(H + 20);                                  /*载波相位测量值,要乘-1.0，多普勒一样*/
        double Doppler = -1.0 * R4(H + 32);
        double SD_Psr = R4(H + 16);
        double SD_Adr = R4(H + 28);
        double SNR = R4(H + 36);
        //memcpy(obs->Satobs[n].FormLocktime, obs->Satobs[n].locktime,2*sizeof(double));
        double locktime = R4(H + 40);


        H += 44;
        if (code_lock == 0 && phase_lock == 0)   /*以后可以再加上其余的判定条件，例如SNR、Locktime*/
        {
            obs->Satobs[n].Valid = false;
            continue;
        }

        if (SYS == GPS)
        {
            switch (sigtype)
            {
            case 0:                            //L1
                LInd = 0;
                LValue = GPS_L1;
                break;
            case 9:                            //L2
                LInd = 1;
                LValue = GPS_L2;
                break;
            default:continue;
            }
        }
        else if (SYS == BDS)
        {
            switch (sigtype)
            {
            case 0:                            //B1I with D1
                LInd = 0;
                LValue = BDS_B1I;
                break;
            case 2:                            //B3I with D2
                LInd = 1;
                LValue = BDS_B3I;
                break;
            case 4:                            //B1I with D1
                LInd = 0;
                LValue = BDS_B1I;
                break;
            case 6:                            //B3I with D2
                LInd = 1;
                LValue = BDS_B3I;
                break;
            default:continue;
            }
        }
        else if (SYS == UNKS)
        {
            continue;
        }
        obs->Satobs[n].Prn = PRN;
        obs->Satobs[n].System = SYS;
        obs->Satobs[n].P[LInd] = psr;
        obs->Satobs[n].L[LInd] = adr * CLight / LValue;
        obs->Satobs[n].D[LInd] = Doppler * CLight / LValue;
        obs->Satobs[n].Parity[LInd] = parity;
        obs->Satobs[n].SD_Psr[LInd] = SD_Psr;
        obs->Satobs[n].SD_Adr[LInd] = SD_Adr;
        obs->Satobs[n].SNR[LInd] = SNR;
        //memcpy(obs->Satobs[n].FormLocktime, obs->Satobs[n].locktime,2*sizeof(double));
        obs->Satobs[n].locktime[LInd] = locktime;
        if (CountFlag)
        {
            obs->SatNum++;
        }
    }
   return 0;
}
int ReadGPSEphem(unsigned char* buff, GPSEPHREC* GPSep1)
{
    unsigned char* H = buff + HeadLen;
    unsigned int PRN = U4(H);
    unsigned int health = U4(H + 12);
    unsigned int IODE1 = U4(H + 16);
    int week = I4(H + 24);
    double toe = R8(H + 32);
    double A = R8(H + 40);
    double M0 = R8(H + 56);
    double deltaN = R8(H + 48);
    double ecc = R8(H + 64);
    double omega = R8(H + 72);
    double cuc = R8(H + 80);
    double cus = R8(H + 88);
    double crc = R8(H + 96);
    double crs = R8(H + 104);
    double cic = R8(H + 112);
    double cis = R8(H + 120);
    double I0 = R8(H + 128);
    double I0_Dot = R8(H + 136);
    double OMEGA0 = R8(H + 144);
    double OMEGA_dot = R8(H + 152);
    unsigned int iodc = U4(H + 160);
    double toc = R8(H + 164);
    double tgd = R8(H + 172);
    double af0 = R8(H + 180);
    double af1 = R8(H + 188);
    double af2 = R8(H + 196);

    /*赋给结果结构体*/
    GPSEPHREC* GPSep = GPSep1;
    GPSep->PRN = PRN;
    GPSep->System = GPS;
    GPSep->SVHealth = health;
    GPSep->A = A;
    GPSep->M0 = M0;
    GPSep->TOC.SecOfWeek = toc; GPSep->TOC.Week = week;
    GPSep->TOE.SecOfWeek = toe; GPSep->TOE.Week = week;
    GPSep->ClkBias = af0; GPSep->ClkDrift = af1; GPSep->ClkDriftRate = af2;
    GPSep->IODC = iodc;
    GPSep->IODE = IODE1;
    GPSep->e = ecc;
    GPSep->OMEGA0 = OMEGA0; GPSep->OMEGADot = OMEGA_dot;
    GPSep->omega = omega;
    GPSep->Cic = cic; GPSep->Cis = cis; GPSep->Crc = crc;
    GPSep->Cuc = cuc; GPSep->Cus = cus; GPSep->Crs = crs;
    GPSep->i0 = I0; GPSep->iDot = I0_Dot;
    GPSep->TGD1 = tgd;
    GPSep->DeltaN = deltaN;

 /*   cout << "GPS:" << week << endl;*/
    return 0;
}
int ReadBDSEphem(unsigned char* buff, GPSEPHREC* BDSep1)
{
    unsigned char* H = buff + HeadLen;
    unsigned int PRN = U4(H);
    int week = I4(H + 4);
    unsigned int health = U4(H + 16);
    double tgd1 = R8(H + 20);
    double tgd2 = R8(H + 28);
    unsigned int Aodc = U4(H + 36);
    unsigned int toc = U4(H + 40);
    double af0 = R8(H + 44);
    double af1 = R8(H + 52);
    double af2 = R8(H + 60);
    unsigned int AODE = U4(H + 68);
    unsigned int toe = U4(H + 72);
    double RootA = R8(H + 76);
    double ecc = R8(H + 84);
    double omega = R8(H + 92);
    double deltaN = R8(H + 100);
    double M0 = R8(H + 108);
    double OMEGA0 = R8(H + 116);
    double OMEGA_dot = R8(H + 124);
    double I0 = R8(H + 132);
    double I0_Dot = R8(H + 140);
    double cuc = R8(H + 148);
    double cus = R8(H + 156);
    double crc = R8(H + 164);
    double crs = R8(H + 172);
    double cic = R8(H + 180);
    double cis = R8(H + 188);
  


    /*赋给结果结构体*/
    GPSEPHREC* BDSep = BDSep1;
    BDSep->PRN = PRN;
    BDSep->System = BDS;
    BDSep->SVHealth = health;
    BDSep->A = RootA*RootA;
    BDSep->M0 = M0;
    BDSep->TOC.SecOfWeek = (double)toc; BDSep->TOC.Week = week;
    BDSep->TOE.SecOfWeek = (double)toe; BDSep->TOE.Week = week;
    BDSep->ClkBias = af0; BDSep->ClkDrift = af1; BDSep->ClkDriftRate = af2;
    BDSep->IODC = Aodc;
    BDSep->IODE = AODE;
    BDSep->e = ecc;
    BDSep->OMEGA0 = OMEGA0; BDSep->OMEGADot = OMEGA_dot;
    BDSep->omega = omega;
    BDSep->Cic = cic; BDSep->Cis = cis; BDSep->Crc = crc;
    BDSep->Cuc = cuc; BDSep->Cus = cus; BDSep->Crs = crs;
    BDSep->i0 = I0; BDSep->iDot = I0_Dot;
    BDSep->TGD1 = tgd1; 
    BDSep->DeltaN = deltaN;

 /*   cout <<"BDS:" << week << endl;*/
    return 0;
}
int ReadRawEphem(unsigned char* buff)
{
    return 0;
}
int ReadRangeCMP(unsigned char* buff)
{
    return 0;
}
int ReadPSRPOS(unsigned char* buff, POSRES* posres)
{
    unsigned char* H = buff + HeadLen;
    double Blh[3],Xyz[3];
    Blh[0] = R8(H + 8);
    Blh[1] = R8(H + 16);
    Blh[2] = R8(H + 24);
    Blh2Xyz(Blh, Xyz);
    memcpy(posres->RealPos, Xyz, 3 * sizeof(double));
    posres->realPosFlag = true;
    return 0;

}
int ReadBestPOS(unsigned char* buff, POSRES* posres)
{
    unsigned char* H = buff + HeadLen;
    double Blh[3], Xyz[3];
    Blh[0] = R8(H + 8);
    Blh[1] = R8(H + 16);
    Blh[2] = R8(H + 24);
    double undulation = R4(H + 32);
    if (posres->unduFlag)
    {
        Blh[2] = Blh[2] + undulation;
    }
    Blh2Xyz(Blh, Xyz);
    memcpy(posres->RealPos, Xyz, 3 * sizeof(double));
    posres->realPosFlag = true;
    return 0;

}
void JudgeFreq(EPOCHOBSDATA* Epoch)
{
    for (int i = 0; i < MAXCHANNUM; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            if (abs(Epoch->Satobs[i].P[j]) > 1e4)
            {
                Epoch->Satobs[i].fFlag[j] = 1;
            }
            else { Epoch->Satobs[i].fFlag[j] = 0; }
        }
    }
}
void GetFormlock(EPOCHOBSDATA* s)
{
    for (int i = 0; i < s->SatNum; i++)
    {
        if (s->Satobs[i].System == GPS)
        {
            s->FormLocktime[2 * s->Satobs[i].Prn] = s->Satobs[i].locktime[0];
            s->FormLocktime[2 * s->Satobs[i].Prn + 1] = s->Satobs[i].locktime[1];
        }
        else if (s->Satobs[i].System == BDS)
        {
            s->FormLocktime[2*MAXGPSNUM +2 * s->Satobs[i].Prn] = s->Satobs[i].locktime[0];
            s->FormLocktime[2*MAXGPSNUM +2 * s->Satobs[i].Prn + 1] = s->Satobs[i].locktime[1];
        }
    }
}


