#ifndef __PCC_H__
#define __PCC_H__

#define VER_MAJOR                 0
#define VER_MINOR                 0
#define VER_PATCH                 1

#define DEFAULT_MSOP_PORT         51180
#define DEFAULT_DIFOP_PORT        51080

#define MAXSIZE                   1500
#define WHILE_NUM                 1
#define LIDAR_NUM                 5
#define MAX_POINT_NUM_IN_BLOCK    LIDAR_NUM /* 下面三个宏在结构体中被用到，无法适配config配置中的//可以将结构体的该部分设计成指针，然后通过malloc方式申请内存 */
#define MAX_BLOCK_NUM             12        /* 设置小于3有问题，还需要回头看看 */
#define ROLL_NUM                  2         /* 配置的回波次数 */

static  uint8_t   bufOrigenAddr[MAXSIZE] = {0};
static  uint16_t  pkgSn = 0;
//注意字节对齐/命名规范
typedef struct
{
    uint8_t           lidarType;
    uint8_t           msgSource;
    uint8_t           lidarNum;
    uint8_t           maxBlockNum;
    uint8_t           rollNum;
    uint8_t           waitForDifop;
    uint8_t           useLidarClock;
    uint16_t          msopPort;
    uint16_t          difopPort;  
    uint16_t          startAngle;  
    uint16_t          endAngle;    
    float             minDistance;
    float             maxDistance;
    char             *pointCloudSendIp;  //需要子网掩码？
    char             *packetSendIp;
    char             *pcapPath;
}LidarParamConfig;

static LidarParamConfig lidarParamConfig = {0};

typedef struct
{
    uint16_t          distance;
    uint16_t          azimuth;
    uint16_t          elevation;
    uint8_t           reflectivity;
    uint16_t          rsv;
}PointT;

typedef struct
{
    uint8_t           channelNum;
    uint8_t           timeOffSet;
    uint8_t           returnSn; //can be omitted
    uint8_t           rsv;
    PointT            pointT[MAX_POINT_NUM_IN_BLOCK];      
}DataBlock;

//40 byte
typedef struct
{
    uint32_t           headCode;             //package identification
    uint16_t           pkgLen;                //package paload len
    uint16_t           pkgSn;                 //package sequnce num
    uint16_t           lidarType;             //identify lidar type
    uint16_t           protocolVersion;       //identify the using protocol version
    uint8_t            timestamp[10];         //identify the timestamp for point cloud
    uint8_t            measurenmentMode;      //identify city or high way road
    uint8_t            laserNum;              //the laser scan at the same time
    uint8_t            blockNum;              //the block num in one MSOP pkg
    uint8_t            waveMode;              
    uint8_t            timeSyncMode;          //self clock signal、 sync clock signal
    uint8_t            timeSyncState;         //state of clock signal
    uint8_t            memsTmp;               //the temperature of lidar
    uint8_t            slotNum;               //muti lidar identified by the slot num
    uint8_t            rsv[10];
}PclPackageHead;

typedef struct
{
    uint8_t            rsv[4];
}PclPackageTail;

typedef struct
{
    PclPackageHead     pclPackageHead;
    DataBlock          dataBlock[MAX_BLOCK_NUM][ROLL_NUM];
    PclPackageTail     pclPackageTail;
}PclPackage;

#endif /* __PCC_H__ */