#ifndef __PCC_H__
#define __PCC_H__

#ifdef __cplusplus
extern "C" {
#endif

#define VER_MAJOR                 0
#define VER_MINOR                 0
#define VER_PATCH                 1

#define DEFAULT_MSOP_PORT         51180
#define DEFAULT_DIFOP_PORT        51080

#define MAXSIZE                   1500
#define WHILE_NUM                 1
#define LIDAR_NUM                 5         /* 默认 25 */
#define MAX_POINT_NUM_IN_BLOCK    LIDAR_NUM /* 下面三个宏在结构体中被用到，无法适配config配置中的//可以将结构体的该部分设计成指针，然后通过malloc方式申请内存 */
#define MAX_BLOCK_NUM             15   
#define ROLL_NUM                  2  //配置的回波次数

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE  (1)
#endif
typedef int StatusCode; 

//注意字节对齐/命名规范
typedef struct
{
    uint8_t           lidar_type;
    uint8_t           msg_source;
    uint8_t           lidar_num;
    uint8_t           max_block_num;
    uint8_t           roll_num;
    uint8_t           wait_for_difop;
    uint8_t           use_lidar_clock;
    uint16_t          msop_port;
    uint16_t          difop_port;
    uint16_t          start_angle;  
    uint16_t          end_angle;    
    float             min_distance;
    float             max_distance;
    uint8_t           point_cloud_send_ip[10];  //需要子网掩码？
    uint8_t           packet_send_ip[10];
    uint8_t           *pcap_path;
} LidarParamConfig;

typedef struct
{
    uint16_t          distance;      /* 球坐标系径向距离 radius（单位 mm） */
    uint16_t          azimuth;       /* 球坐标系水平夹角，方位角（分辨率 0.01°） */
    uint16_t          elevation;     /* 球坐标系垂直夹角，俯仰角/极角（分辨率 0.01°） */
    uint8_t           reflectivity;  /* 反射强度 intensity */
    uint16_t          rsv;           /* 保留 */
} PointT;

typedef struct
{
    uint8_t           channelNum;
    uint8_t           timeOffSet;
    uint8_t           returnSn; //can be omitted
    uint8_t           rsv;
    PointT            pointT[MAX_POINT_NUM_IN_BLOCK];      
} DataBlock;

//40 byte
typedef struct
{
    uint32_t           head_code;             //package identification
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
} PclPackageHead;

typedef struct
{
    uint8_t            rsv[4];
} PclPackageTail;

typedef struct
{
    PclPackageHead     pclPackageHead;
    DataBlock          dataBlock[MAX_BLOCK_NUM][ROLL_NUM];
    PclPackageTail     pclPackageTail;

} PclPackage;

void PackPclPkgTmp(PclPackage *pclPackage, uint16_t sn);
uint8_t* PackPclPkgToBuf(uint8_t *buf, PclPackage *pclPackage, uint16_t pkgSn);

int get_packet_length(uint8_t *buf, uint16_t *plen);

void PrintpOrigineAddr(uint8_t *bufAddr);
void ParasePclPackage(uint8_t *buf, PclPackage *pclPackage);

#ifdef __cplusplus
}
#endif

#endif /* __PCC_H__ */