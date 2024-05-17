#include"ReadBinary.h"
#include"SPP.h"
using namespace std;

/*ת���������ֽ�ת�����ض�����,��Ϊÿ��Field Name���в�ͬ���ֽ���*/
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
ʵʱ��������ȡʱ����һ�������Maxrawlen���ֽڣ�
����Ҫ�Դ����dΪ���������d�����ʱ���ǽضϳ��ȴ�С
*******************/
int DecodeNovOem7Dat(unsigned char buff[], int& d, EPOCHOBSDATA* obs, GPSEPHREC GPSep[], GPSEPHREC BDSep[],POSRES *posres)
{
        /*��ʽ��ʼ��ȡ���������ݲ���*/
        bool RangeFlag = false;/*�ж��Ƿ�����۲�����*/
        int max = d;
        d = 0;//��d����Ϊ�㣬���ں����ж��Ƿ��в������ݿ�
        for (int i = 0; i < max - 2; i++)/*��������ͷͬ��*/
        {
            if (buff[i + 0] == SYNC1 && buff[i + 1] == SYNC2 && buff[i + 2] == SYNC3)
            {
                unsigned char* RealbuffAddress = buff + i;/*��ʱ�ڸ�ѭ���м̳��ļ�ͷ��ʼ�ĵ�ַ*/
                /*ȷ������ͷ�󣬿�ʼ��ʽ�Ľ���*/
                int a = 4;/*Bina*/
                unsigned short msgID = U2(RealbuffAddress + a);     a += 2; /*MessageID ռ�����ֽ�*/
                char msgtype = I1(RealbuffAddress + a);
                /*�ж϶�ȡ�Ƕ������ļ�����ASCII���ļ�,0Ϊbinary��1Ϊascll��2ΪNMEA��3Ϊreserved*/
                int type = (msgtype >> 4) & (0x3);         a += 2;
                unsigned short msglen = U2(RealbuffAddress + a);    a += 6;
                unsigned short week = U2(RealbuffAddress + a);      a += 2;
                unsigned int ms = U4(RealbuffAddress + a);
                /*�ж϶�ȡ���ݿ��Ƿ�Ϊ���һ�鱻�ضϵ�����*/
                if (msglen + i + HeadLen+4  > max)
                {
                    d = max - i;
                }
                else
                {
                    /*crc����*/
                    unsigned int CRC0 = U4(RealbuffAddress + msglen + HeadLen);/*��ȡ�ļ������е�CRC��*/
                    unsigned int CRC = crc32(RealbuffAddress, msglen + HeadLen);
                    if (CRC0 != CRC)                                   //�������ֱ�Ӷ�ȡ��һ������
                    {
                        cout << "CRC check error!" << endl;
                        continue;
                    }
                    GPSEPHREC tempEp;
                    switch (msgID)
                    {
                    case ID_GPSephem:
                        
                        ReadGPSEphem(RealbuffAddress, &tempEp);
                        if (tempEp.PRN > 0 && tempEp.PRN < 40)//ֻ����Ч�������ż���
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
                        obs->Time.SecOfWeek = ms / 1000.0;//����ע������
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
                    break;/*���ضϵ����ݷ��ڻ������Ŀ�ͷ*/
                }
            }
            else continue;
        }
        if (RangeFlag == true) 
        { 
            JudgeFreq(obs);/*�ж����ù۲������ǵ�Ʒ����˫Ƶ*/
            return 1;
        }
        else { return -1; }//���껺���������ݺ󣬴�ȡ�ض����ݽ�����һ�ζ�ȡ
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
        CountFlag = true;/*ÿ�ζ��۲�����ʱ�Ƚ�������־��true*/
        int n = 0;
        GNSSSys SYS;
        unsigned short PRN = U2(H + 4);
        unsigned int track = U4(H + 44);                        /*�����������*/
        unsigned int sys = (track >> 0x10) & (0x7);                 /*�ж�ϵͳ*/
        unsigned int phase_lock = (track >> (0xA)) & (0x1);			/*̽���ز���λ��ʧ�����*/
        unsigned int parity = (track >> (0xB)) & (0x1);            /*�����������*/
        unsigned int code_lock = (track >> (0xC)) & (0x1);          /*̽��α�����ʧ�����*/
        unsigned int sigtype = (track >> 0x15) & (0x1F);
        if (sys == 0) SYS = GPS;
        else if (sys == 4) SYS = BDS;
        else SYS = UNKS;
        for (int j = 0; j < MAXCHANNUM; j++)  /*memset��enumҲ�ᱻĬ�ϸ�ֵ0���ʽ�UNKS���ڵ�һλ��ʹĬ��ΪUNKS*/
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
        double psr = R8(H + 8);                                   /*α�����ֵ*/
        double adr =-1.0* R8(H + 20);                                  /*�ز���λ����ֵ,Ҫ��-1.0��������һ��*/
        double Doppler = -1.0 * R4(H + 32);
        double SD_Psr = R4(H + 16);
        double SD_Adr = R4(H + 28);
        double SNR = R4(H + 36);
        //memcpy(obs->Satobs[n].FormLocktime, obs->Satobs[n].locktime,2*sizeof(double));
        double locktime = R4(H + 40);


        H += 44;
        if (code_lock == 0 && phase_lock == 0)   /*�Ժ�����ټ���������ж�����������SNR��Locktime*/
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

    /*��������ṹ��*/
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
  


    /*��������ṹ��*/
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


