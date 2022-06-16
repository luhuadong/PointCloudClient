#ifndef __PCC2_H__
#define __PCC2_H__

#ifdef __cplusplus
extern "C" {
#endif

#define VER_MAJOR                 0
#define VER_MINOR                 0
#define VER_PATCH                 1

#define PANDAR128_BLOCK_NUM       (2)
#define PANDAR128_LASER_NUM       (128)

#define DEFAULT_MSOP_PORT         (51180)
#define DEFAULT_DIFOP_PORT        (51080)

#define MAXSIZE                   (1500)
#define WHILE_NUM                 (1)
#define LASER_NUM                 (8)         /* 默认 25 */
#define MAX_POINT_NUM_IN_BLOCK    LASER_NUM
#define MAX_BLOCK_NUM             (4)
#define ROLL_NUM                  (3)         /* 配置的回波次数，最大3回波 */

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE  (1)
#endif
typedef int StatusCode; 


#pragma pack(push, 1)
//! @brief class representing the raw measure
/*
   0               1               2               3
   0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |           Distance         |            Intensity             |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
typedef struct
{
    uint16_t Distance;
    uint8_t Intensity;
} Pandar128Unit;
#pragma pack(pop)


#pragma pack(push, 1)
//! @brief class representing the raw block
/*
   0               1               2
   0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
  |             Azimuth            |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-

 */
typedef struct
{
    uint16_t Azimuth;
    Pandar128Unit units[PANDAR128_LASER_NUM];
} Pandar128Block;
#pragma pack(pop)

/* Custom */
#pragma pack(push, 1)
typedef struct
{
    uint16_t distance;      /* 球坐标系径向距离 radius（单位 mm） */
    uint16_t azimuth;       /* 球坐标系水平夹角，方位角（分辨率 0.01°） */
    uint16_t elevation;     /* 球坐标系垂直夹角，俯仰角/极角（分辨率 0.01°） */
    uint8_t  intensity;     /* 反射强度 intensity */
    uint16_t reserved;      /* 保留 */
} PointT;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint8_t channelNum;
    uint8_t timeOffSet;
    uint8_t returnSn; //can be omitted
    uint8_t reserved;
    PointT  pointT[MAX_POINT_NUM_IN_BLOCK];   /* 8 x 9 = 72 Bytes */
} AsensingBlock;                              /* Total 76 Bytes */
#pragma pack(pop)

#pragma pack(push, 1)
//! @brief class representing the Raw packet
/*
   0               1               2               3
   0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |               Sob              |  VersionMajor | VersionMinor |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |    DistUnit    |     Flags     |   LaserNum    |  BlockNum    |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |    EchoCount   |   EchoNum     |          Reserved            |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
typedef struct
{
    uint32_t Sob;
    uint16_t PkgLen;
    uint16_t SeqNum;
    uint16_t LidarType;
    uint8_t  VersionMajor;
    uint8_t  VersionMinor;

    uint8_t  UTCTime0;
    uint8_t  UTCTime1;
    uint8_t  UTCTime2;
    uint8_t  UTCTime3;
    uint8_t  UTCTime4;
    uint8_t  UTCTime5;
    uint32_t Timestamp;

    uint8_t  MeasureMode;
    uint8_t  LaserNum;
    uint8_t  BlockNum;
    uint8_t  EchoCount;
    uint8_t  TimeSyncMode;
    uint8_t  TimeSyncStat;
    uint8_t  MemsTemp;
    uint8_t  SlotNum;
    uint32_t FrameID;

    uint16_t Reserved1;
    uint16_t Reserved2;
    uint16_t Reserved3;

} AsensingHeader;
#pragma pack(pop)

#pragma pack(push, 1)
//! @brief class representing the Raw packet
/*
   0               1               2               3
   0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |    Reserved1   |   Reserved1   |  Reserved1    |  Reserved2   |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |    Reserved2   |   Reserved2   | ShutdownFlag  |  Reserved3   |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |    Reserved3   |   Reserved3   |         MotorSpeed           |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                            Timestamp                          |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |   ReturnMode   | FactoryInfo   |   UTCTime     |   UTCTime    |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |     UTCTime    |    UTCTime    |   UTCTime     |  UTCTime     |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                             SeqNum                            |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
typedef struct
{
    uint8_t Reserved11;
    uint8_t Reserved12;
    uint8_t Reserved13;

    uint8_t Reserved21;
    uint8_t Reserved22;
    uint8_t Reserved23;

    uint16_t Padding1;
    uint16_t Padding2;
    uint8_t Padding3;

    uint8_t ShutdownFlag;

    //uint8_t Reserved3[3];

    uint8_t ReturnMode;

    uint16_t MotorSpeed;

    //uint8_t FactoryInfo;

    uint32_t SeqNum;
} AsensingTail;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    AsensingHeader header;
    AsensingBlock blocks[MAX_BLOCK_NUM * ROLL_NUM];
    uint32_t crc;
    uint8_t functionSafety[17];
    AsensingTail tail;

} AsensingPacket;
#pragma pack(pop)

// void PackPclPkgTmp(PclPackage *pclPackage, uint16_t sn);
// uint8_t* PackPclPkgToBuf(uint8_t *buf, PclPackage *pclPackage, uint16_t pkgSn);

// int get_packet_length(uint8_t *buf, uint16_t *plen);

// void PrintpOrigineAddr(uint8_t *bufAddr);
// void ParasePclPackage(uint8_t *buf, PclPackage *pclPackage);

#ifdef __cplusplus
}
#endif

#endif /* __PCC2_H__ */