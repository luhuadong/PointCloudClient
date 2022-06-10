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
    uint16_t Sob;
    uint8_t VersionMajor;
    uint8_t VersionMinor;
    uint8_t DistUnit;
    uint8_t Flags;
    uint8_t LaserNum;
    uint8_t BlockNum;
    uint8_t EchoCount;
    uint8_t EchoNum;
    uint16_t Reserved;
} Pandar128Header;
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

    uint8_t UTCTime0;
    uint8_t UTCTime1;
    uint8_t UTCTime2;
    uint8_t UTCTime3;
    uint8_t UTCTime4;
    uint8_t UTCTime5;

    uint32_t Timestamp;

    //uint8_t FactoryInfo;

    uint32_t SeqNum;
} Pandar128Tail;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    Pandar128Header header;
    Pandar128Block blocks[PANDAR128_BLOCK_NUM];
    uint32_t crc;
    uint8_t functionSafety[17];
    Pandar128Tail tail;

} Pandar128Packet;
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