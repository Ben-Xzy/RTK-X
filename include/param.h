#define MAXCHANNUM 97 /*本来是最大96个，我们舍弃索引为0的元素，故设为97个，第一个不用*/
#define MAXGPSPRN 33/*第一个不用*/
#define MAXBDSPRN 65/*第一个不同*/
#define MAXGPSNUM 32
#define MAXBDSNUM 64
#define CLight   2.99792458e8
#define GPS_GM  3.986005e14
#define GPS_OMEGAEARTH  7.2921151467e-5
#define BDS_GM  3.986004418e14
#define BDS_OMEGAEARTH  7.2921150e-5
#define InitialValue 0.0 //坐标初值#pragma once

#define MAXRawLen       50000
#define POLYCRC32       0xEDB88320u /* CRC32 polynomial */
#define SYNC1           0xAA    /* oem7/6/4 message start sync code 1 */
#define SYNC2           0x44    /* oem7/6/4 message start sync code 2 */
#define SYNC3           0x12    /* oem7/6/4 message start sync code 3 */
#define HeadLen         28      /* oem7/6/4 message header length (bytes) */

/* message IDs */
#define ID_GPSephem     7       /*GPS epoch emeris*/
#define ID_BDSephem     1696    /* oem7/6 decoded bds ephemeris */  
#define ID_RangeCMP     140     /* oem7/6/4 range compressed */
#define ID_Range        43      /* oem7/6/4 range measurement */
#define ID_Rawephem     41      /* oem7/6/4 raw ephemeris */
#define ID_Ion_UTC      8       /* oem7/6/4 iono and utc data */
#define ID_PSRPOS       47
#define ID_PSRXYZ       243
#define ID_BestPos      42

#define WL1             0.1902936727984
#define WL2             0.2442102134246
#define MAXVAL          8388608.0

#define SQR(x)          ((x)*(x))

#define C_velocity      299792458
#define GPS_L1          1575.420e6
#define GPS_L2          1227.600e6
#define BDS_B1I         1561.098e6
#define BDS_B3I         1268.520e6

#define H0 0 /*海平面*/
#define T0 288.16 /*温度*/
#define P0 1013.25 /*气压*/
#define RH0 0.5 /*相对湿度*/
#define GPSLIMT 7500 /*GPS过期判断*/
#define BDSLIMT 3900 /*BDS过期判断*/

#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))
