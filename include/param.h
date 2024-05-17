#ifndef PARAM_H
#define PARAM_H
#define MAXCHANNUM 97 /*Originally, it was a maximum of 96, but we discarded the element with an index of 0, so we set it to 97, and the first one was not used*/
#define MAXGPSPRN 33/*discard fir,只是为了便于创建能容纳全部GPS卫星的数组，BDSPRN同理*/
#define MAXBDSPRN 65
#define MAXGPSNUM 32
#define MAXBDSNUM 64
#define CLight   2.99792458e8
#define GPS_GM  3.986005e14
#define GPS_OMEGAEARTH  7.2921151467e-5
#define BDS_GM  3.986004418e14
#define BDS_OMEGAEARTH  7.2921150e-5
#define InitialValue 0.0 //Initial value of Pos

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

#define H0 0 /*sea level*/
#define T0 288.16 /*temperature*/
#define P0 1013.25 /*atmospheric pressure*/
#define RH0 0.5 /*relative humidity*/
#define GPSLIMT 7500 /*GPS expiration judgment*/
#define BDSLIMT 3900 /*BDS expiration judgment*/

#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))
#endif