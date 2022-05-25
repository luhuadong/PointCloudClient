/*
 * Copyright (c) 2014-2022, Asensing Group
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-05-10     luhuadong    The first version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <getopt.h>
#include <errno.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "cJSON.h"

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
    uint8_t           *pointCloudSendIp;  //需要子网掩码？
    uint8_t           *packetSendIp;
    uint8_t           *pcapPath;
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

void Set_1_Byte(uint8_t **buf, uint8_t value)
{
    *(uint8_t *)(*buf) = value;
    (*buf)++;
    return;
}

void Set_2_Byte(uint8_t **buf, uint16_t value)
{
    // **buf = value;
    *(uint16_t *)(*buf) = value;
    *buf = *buf + 2;
    return;
}

void Set_4_Byte(uint8_t **buf, uint32_t value)
{
    *(uint32_t *)(*buf) = value;
    *buf = *buf + 4;
    return;
}

void PackPclPkgDataBlockTmp(PclPackage *pclPackage)
{
    for(uint8_t blockLoop = 0; blockLoop < MAX_BLOCK_NUM; blockLoop++)
    {
        DataBlock *dataBlock_tmp = &pclPackage->dataBlock[blockLoop][0];
        for(uint8_t rollLoop = 0; rollLoop < ROLL_NUM; rollLoop++)
        {
            if((blockLoop % 3 == 0) && (rollLoop % 2 == 0))
            {
                dataBlock_tmp[rollLoop].channelNum = 5;
                dataBlock_tmp[rollLoop].timeOffSet = 0;
                dataBlock_tmp[rollLoop].returnSn   = 0;
            }
            if((blockLoop % 3 == 0) && (rollLoop % 2 == 1))
            {
                dataBlock_tmp[rollLoop].channelNum = 0;
                dataBlock_tmp[rollLoop].timeOffSet = 0;
                dataBlock_tmp[rollLoop].returnSn   = 0;
            }
            if((blockLoop % 3 == 1) && (rollLoop % 2 == 0))
            {
                dataBlock_tmp[rollLoop].channelNum = 0;
                dataBlock_tmp[rollLoop].timeOffSet = 0;
                dataBlock_tmp[rollLoop].returnSn   = 0;
            }
            if((blockLoop % 3 == 1) && (rollLoop % 2 == 1))
            {
                dataBlock_tmp[rollLoop].channelNum = 5;
                dataBlock_tmp[rollLoop].timeOffSet = 0;
                dataBlock_tmp[rollLoop].returnSn   = 0;
            }
            if((blockLoop % 3 == 2) && (rollLoop % 2 == 0))
            {
                dataBlock_tmp[rollLoop].channelNum = 3;
                dataBlock_tmp[rollLoop].timeOffSet = 0;
                dataBlock_tmp[rollLoop].returnSn   = 0;
            }
            if((blockLoop % 3 == 2) && (rollLoop % 2 == 1))
            {
                dataBlock_tmp[rollLoop].channelNum = 3;
                dataBlock_tmp[rollLoop].timeOffSet = 0;
                dataBlock_tmp[rollLoop].returnSn   = 0;
            }

            PointT *pointT_tmp = &dataBlock_tmp[rollLoop].pointT[0];
            for(uint8_t pointTLoop = 0; pointTLoop < lidarParamConfig.lidarNum; pointTLoop++)
            {
                pointT_tmp[pointTLoop].distance       = 15;
                pointT_tmp[pointTLoop].azimuth        = 75;
                pointT_tmp[pointTLoop].elevation      = 13;
                pointT_tmp[pointTLoop].reflectivity   = 3;
            }
        }
    }
}

void PackPclPkgTmp(PclPackage *pclPackage, uint8_t while_num_for_client)
{
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ head
    //4byte code
    pclPackage->pclPackageHead.pkgLen = 1108;
    pclPackage->pclPackageHead.pkgSn = while_num_for_client;
    pclPackage->pclPackageHead.lidarType = 1;
    pclPackage->pclPackageHead.protocolVersion = 11;

    // pclPackage->pclPackageHead.timestamp[loop];//10byte time stamps
    pclPackage->pclPackageHead.measurenmentMode = 3;
    pclPackage->pclPackageHead.laserNum = 5;
    pclPackage->pclPackageHead.blockNum = MAX_BLOCK_NUM;
    pclPackage->pclPackageHead.waveMode = 0;
    pclPackage->pclPackageHead.timeSyncMode = 1;
    pclPackage->pclPackageHead.timeSyncState = 0;
    pclPackage->pclPackageHead.memsTmp = 20;
    pclPackage->pclPackageHead.slotNum = 1;

    PackPclPkgDataBlockTmp(pclPackage);

    pclPackage->pclPackageTail.rsv[0] = 0x12;
    pclPackage->pclPackageTail.rsv[1] = 0x34;
    pclPackage->pclPackageTail.rsv[2] = 0x56;
    pclPackage->pclPackageTail.rsv[3] = 0x78;

    return;
}


uint8_t* PackPclPkgHead(uint8_t *buf, PclPackage *pclPackage)
{
    Set_1_Byte(&buf, 0x55);
    Set_1_Byte(&buf, 0xaa);
    Set_1_Byte(&buf, 0x5a);
    Set_1_Byte(&buf, 0xa5);
    printf("++++++++++++++++++++  000 %p\n", buf);

    Set_2_Byte(&buf, pclPackage->pclPackageHead.pkgLen);
    Set_2_Byte(&buf, pclPackage->pclPackageHead.pkgSn);
    Set_2_Byte(&buf, pclPackage->pclPackageHead.lidarType);
    Set_2_Byte(&buf, pclPackage->pclPackageHead.protocolVersion);
    printf("++++++++++++++++++++  001 %p\n", buf);
    for(uint8_t loop = 0; loop < 10; loop++)
    {
        Set_1_Byte(&buf, pclPackage->pclPackageHead.timestamp[loop]);
    }
    printf("++++++++++++++++++++  002 %p\n", buf);

    Set_1_Byte(&buf, pclPackage->pclPackageHead.measurenmentMode);
    Set_1_Byte(&buf, pclPackage->pclPackageHead.laserNum);
    Set_1_Byte(&buf, pclPackage->pclPackageHead.blockNum);
    Set_1_Byte(&buf, pclPackage->pclPackageHead.waveMode);
    Set_1_Byte(&buf, pclPackage->pclPackageHead.timeSyncMode);
    Set_1_Byte(&buf, pclPackage->pclPackageHead.timeSyncState);
    Set_1_Byte(&buf, pclPackage->pclPackageHead.memsTmp);
    Set_1_Byte(&buf, pclPackage->pclPackageHead.slotNum);
    printf("++++++++++++++++++++  003 %p\n", buf);

    for(uint8_t loop = 0; loop < 10; loop++)
    {
        Set_1_Byte(&buf, pclPackage->pclPackageHead.rsv[loop]);
    }
    printf("++++++++++++++++++++  004 %p\n", buf);
    return buf;
}

uint8_t* PackPclPkgPointT(uint8_t *buf, PointT *pointT_tmp, uint8_t channelNum)
{
    // lidar num should be dynamic adjustment to real detected point num
    for(uint8_t pointTLoop = 0; pointTLoop < channelNum; pointTLoop++)
    {
        Set_2_Byte(&buf, pointT_tmp[pointTLoop].distance);
        Set_2_Byte(&buf, pointT_tmp[pointTLoop].azimuth);
        Set_2_Byte(&buf, pointT_tmp[pointTLoop].elevation);
        Set_1_Byte(&buf, pointT_tmp[pointTLoop].reflectivity);
        Set_2_Byte(&buf, pointT_tmp[pointTLoop].rsv);
    }
    return buf;
}

uint8_t* PackPclPkgDataBlock(uint8_t *buf, DataBlock *dataBlock_tmp)
{
    uint8_t timeOffSetSetFlag = 0;

    for(uint8_t rollLoop = 0; rollLoop < ROLL_NUM; rollLoop++)
    {
        Set_1_Byte(&buf, dataBlock_tmp[rollLoop].channelNum);
        if(dataBlock_tmp[rollLoop].channelNum == 0)
        {
            printf("detection for roll %u is empty!\n", rollLoop);
            continue;
        }
        if(timeOffSetSetFlag == 0)//one time_Offset needed for one block
        {
            Set_1_Byte(&buf, dataBlock_tmp[rollLoop].timeOffSet);
            timeOffSetSetFlag = 1;
        }
        Set_1_Byte(&buf, dataBlock_tmp[rollLoop].returnSn);

        PointT *pointT_tmp = &dataBlock_tmp[rollLoop].pointT[0];
        uint8_t channelNum = dataBlock_tmp[rollLoop].channelNum;
        buf = PackPclPkgPointT(buf, pointT_tmp, channelNum);
    }
    return buf;
}

uint8_t* PackPclPkgPayload(uint8_t *buf, PclPackage *pclPackage)
{
    for(uint8_t blockLoop = 0; blockLoop < MAX_BLOCK_NUM; blockLoop++)
    {
        DataBlock *dataBlock_tmp = &pclPackage->dataBlock[blockLoop][0];
        buf = PackPclPkgDataBlock(buf, dataBlock_tmp);
    }
    return buf;
}

uint8_t* PackPclPkgTeil(uint8_t *buf, PclPackage *pclPackage)
{
    Set_1_Byte(&buf, pclPackage->pclPackageTail.rsv[0]);
    Set_1_Byte(&buf, pclPackage->pclPackageTail.rsv[1]);
    Set_1_Byte(&buf, pclPackage->pclPackageTail.rsv[2]);
    Set_1_Byte(&buf, pclPackage->pclPackageTail.rsv[3]);
    return buf;
}

uint8_t* PackPclPkgToBuf(uint8_t *buf, PclPackage *pclPackage)
{
    uint8_t *tmp_buf_1 = buf;

    printf("0000000000000000 %p\n", buf);
    buf = PackPclPkgHead(buf, pclPackage);
    uint8_t *tmp_buf_2 = buf;
    if((tmp_buf_2 - tmp_buf_1) == 40)
    {
        printf("head offset is OK! \n");
    }
    printf("0000000000000000 %p\n", buf);

    buf = PackPclPkgPayload(buf, pclPackage);
    uint16_t pkgPayloadLen = buf - tmp_buf_1 - sizeof(PclPackageHead);
    pkgSn++;
    tmp_buf_1 = tmp_buf_1 + 4;
    Set_2_Byte(&tmp_buf_1, pkgPayloadLen);
    tmp_buf_1 = tmp_buf_1 + 2;
    Set_2_Byte(&tmp_buf_1, pkgSn);
    
    buf = PackPclPkgTeil(buf, pclPackage);
    return buf;
}


uint32_t read_file(const char* filename, char** content)
{
    // open in read binary mode
    FILE* file = fopen(filename, "rb");
    if (file == NULL) {
        fprintf(stderr, "read file fail: %s\n", filename);
        return -1;
    }

    // get the length
    fseek(file, 0, SEEK_END);
    long length = ftell(file);
    fseek(file, 0, SEEK_SET);

    // allocate content buffer
    *content = (char*)malloc((size_t)length + sizeof(""));

    // read the file into memory
    size_t read_chars = fread(*content, sizeof(char), (size_t)length, file);
    if ((long)read_chars != length) {
        fprintf(stderr, "length dismatch: %zu, %ld\n", read_chars, length);
        free(*content);
        return -1;
    }
    (*content)[read_chars] = '\0';

    fclose(file);
    return 0;
}

uint32_t LidarConfigInit()
{
    const char* filename = "lidarConfig.json";
    char *json = NULL;
    if (read_file(filename, &json) != 0) return -1;
    if ((json == NULL) || (json[0] == '\0') || (json[1] == '\0')) {
        fprintf(stderr, "file content is null\n");
        return -1;
    }

    cJSON* items = cJSON_Parse(json);
    if (items == NULL) {
        fprintf(stderr, "pasre json file fail\n");
        return -1;
    }

    char* printed_json = cJSON_PrintUnformatted(items);
    if (printed_json == NULL) {
        fprintf(stderr, "print json fail\n");
        return -1;
    }
    printf("%s\n\n", printed_json);
    free(printed_json);
    //将JSON结构所占用的数据空间释放

    printf("000000000000000000000000000000000000000000 init 1 \n");

    cJSON* item = cJSON_GetObjectItem(items, "lidar_type");
    lidarParamConfig.lidarType = item->valueint;
    item = cJSON_GetObjectItem(items, "msg_source");
    lidarParamConfig.msgSource = item->valueint;
    item = cJSON_GetObjectItem(items, "lidar_num");
    lidarParamConfig.lidarNum = item->valueint;
    item = cJSON_GetObjectItem(items, "max_block_num");
    lidarParamConfig.maxBlockNum = item->valueint;
    item = cJSON_GetObjectItem(items, "roll_num");
    lidarParamConfig.rollNum = item->valueint;
    item = cJSON_GetObjectItem(items, "msop_port");
    lidarParamConfig.msopPort = item->valueint;
    item = cJSON_GetObjectItem(items, "difop_port");
    lidarParamConfig.difopPort = item->valueint;
    item = cJSON_GetObjectItem(items, "start_angle");
    lidarParamConfig.startAngle = item->valueint;
    item = cJSON_GetObjectItem(items, "end_angle");
    lidarParamConfig.endAngle = item->valueint;
    item = cJSON_GetObjectItem(items, "min_distance");
    lidarParamConfig.minDistance = item->valueint;
    item = cJSON_GetObjectItem(items, "max_distance");
    lidarParamConfig.maxDistance = item->valueint;
    item = cJSON_GetObjectItem(items, "wait_for_difop");
    lidarParamConfig.waitForDifop = item->valueint;
    item = cJSON_GetObjectItem(items, "use_lidar_clock");
    lidarParamConfig.useLidarClock = item->valueint;

    item = cJSON_GetObjectItem(items, "point_cloud_send_ip");
    char *msopIp = item->valuestring;
    lidarParamConfig.pointCloudSendIp = msopIp;

    item = cJSON_GetObjectItem(items, "packet_send_ip");
    char *mispIp = item->valuestring;
    lidarParamConfig.packetSendIp = mispIp;

    item = cJSON_GetObjectItem(items, "pcap_path");
    char *pcapPath = item->valuestring;
    lidarParamConfig.pcapPath = pcapPath;

    cJSON_Delete(items);

    printf("0000000000000000000 point_cloud_send_ip: %s\n", lidarParamConfig.pointCloudSendIp);
    printf("0000000000000000000 packet_send_ip: %s\n", lidarParamConfig.packetSendIp);
    printf("0000000000000000000 pcap_path: %s\n", lidarParamConfig.pcapPath);

    return 0;
}

static void show_usage(const char *cmd)
{
    printf("Usage: %s [options] ... \n", cmd);
    printf("This is a demo for how to use options\n\n");
    printf("  -h, --help           display this help and exit\n");
    printf("  -v, --version        output version information and exit\n");
    printf("  -w, --who=NAME       tell me what is your NAME\n");
    printf("  -s, --say=CONTENT    what CONTENT do you want to say\n\n");

    exit(0);
}

static void show_version(void)
{
    printf("version %d.%d.%d\n", VER_MAJOR, VER_MINOR, VER_PATCH);
    exit(0);
}

int main(int argc, char **argv)
{
    int option;
    char *ip = NULL;
    char *port = NULL;

    const char * const short_options = "hvi:p:";
    const struct option long_options[] = {

        { "help",    0, NULL, 'h' },
        { "version", 0, NULL, 'v' },
        { "ip",      1, NULL, 'i' },
        { "port",    1, NULL, 'p' },
        { NULL,      0, NULL,  0  }
    };

    while ((option = getopt_long(argc, argv, short_options, long_options, NULL)) != -1) 
    {
        switch (option)
        {
        case 'h':
            show_usage(argv[0]);
            break;
        case 'v':
            show_version();
            break;
        case 'i':
            ip = strdup(optarg);
            break;
        case 'p':
            port = strdup(optarg);
            break;
        case '?':
        default :
            printf("Error: option invalid\n");
            exit(EXIT_FAILURE);
            break;
        }
    }

    printf("IP: %s, Port: %s\n", ip, port);
    exit(0);


    struct sockaddr_in server_addr;
    uint32_t cli_fd;
    uint32_t sr, sw;
    uint32_t while_num_for_client = 1;

    LidarConfigInit();

    PclPackage pclPackage = {0};

    if((cli_fd = socket(AF_INET,SOCK_STREAM,0)) < 0)
    {
        printf("socket error:%d:%s\n",errno,strerror(errno));
        return -1;
    }
    printf("socket OK fd[%d]\n",cli_fd);

    bzero(&server_addr,sizeof(server_addr));

    server_addr.sin_family=AF_INET;
    server_addr.sin_port=htons(atoi(argv[1]));
    inet_aton(argv[2],&server_addr.sin_addr);

    if(connect(cli_fd,(struct sockaddr *)&server_addr,sizeof(server_addr))<0)
    {
        printf("connect error:%d:%s\n",errno,strerror(errno));
        close(cli_fd);
        return -1;
    }
    printf("connect to server %s:%d OK\n",argv[2],atoi(argv[1]));


    while(while_num_for_client++)
    {
        bzero(bufOrigenAddr,sizeof(bufOrigenAddr));
        uint8_t *buf = bufOrigenAddr;

        PackPclPkgTmp(&pclPackage, while_num_for_client - 2);

        PackPclPkgToBuf(buf, &pclPackage);
        
        if((sw=write(cli_fd,&bufOrigenAddr,sizeof(bufOrigenAddr)))<=0)
        {
            printf("write error:%d:%s\n",errno,strerror(errno));
            close(cli_fd);
            return -1;
        }
        printf("================================================================= head \n");

        printf("write %d bytes data to server headcode0:        %x\n",(int)sizeof(buf), bufOrigenAddr[0]);
        printf("write %d bytes data to server headcode1:        %x\n",(int)sizeof(buf), bufOrigenAddr[1]);
        printf("write %d bytes data to server headcode2:        %x\n",(int)sizeof(buf), bufOrigenAddr[2]);
        printf("write %d bytes data to server headcode3:        %x\n",(int)sizeof(buf), bufOrigenAddr[3]);
        printf("write %d bytes data to server pkgLen:           %u\n",(int)sizeof(buf), *(uint16_t *)(bufOrigenAddr+4));
        printf("write %d bytes data to server pkgSn:            %u\n",(int)sizeof(buf), *(uint16_t *)(bufOrigenAddr+6));
        printf("write %d bytes data to server lidarType:        %u\n",(int)sizeof(buf), *(uint16_t *)(bufOrigenAddr+8));
        printf("write %d bytes data to server protocolVertion:  %u\n",(int)sizeof(buf), *(uint16_t *)(bufOrigenAddr+10));

        printf("write %d bytes data to server measurenmentMode: %u\n",(int)sizeof(buf), *(uint8_t *)(bufOrigenAddr+22));
        printf("write %d bytes data to server laserNum:         %u\n",(int)sizeof(buf), *(uint8_t *)(bufOrigenAddr+23));
        printf("write %d bytes data to server blockNum:         %u\n",(int)sizeof(buf), *(uint8_t *)(bufOrigenAddr+24));
        printf("write %d bytes data to server waveMode:         %u\n",(int)sizeof(buf), *(uint8_t *)(bufOrigenAddr+25));
        printf("write %d bytes data to server timeSyncMode:     %u\n",(int)sizeof(buf), *(uint8_t *)(bufOrigenAddr+26));
        printf("write %d bytes data to server timeSyncState:    %u\n",(int)sizeof(buf), *(uint8_t *)(bufOrigenAddr+27));
        printf("write %d bytes data to server memsTmp:          %u\n",(int)sizeof(buf), *(uint8_t *)(bufOrigenAddr+28));
        printf("write %d bytes data to server slotNum:          %u\n",(int)sizeof(buf), *(uint8_t *)(bufOrigenAddr+29));
        printf("================================================================= clock 0\n");

        for(uint8_t blockLoop = 0; blockLoop < 3; blockLoop++)
        {
            printf("================================================================= block\n");

            for (int rollLoop = 0; rollLoop < 2; rollLoop++)
            {
                printf("----------------------------------------------------------------- roll\n");

                printf("read %d bytes data from client channelNum:          %u\n",sr,pclPackage.dataBlock[blockLoop][rollLoop].channelNum);
                printf("read %d bytes data from client timeOffSet:          %u\n",sr,pclPackage.dataBlock[blockLoop][rollLoop].timeOffSet);
                printf("read %d bytes data from client returnSn:            %u\n",sr,pclPackage.dataBlock[blockLoop][rollLoop].returnSn);
                for(uint8_t pointLoop = 0; pointLoop < pclPackage.dataBlock[blockLoop][rollLoop].channelNum; pointLoop++)
                {
                    printf("read %d bytes data from client distance:              %u\n",sr,pclPackage.dataBlock[blockLoop][rollLoop].pointT[pointLoop].distance);
                    printf("read %d bytes data from client azimuth:               %u\n",sr,pclPackage.dataBlock[blockLoop][rollLoop].pointT[pointLoop].azimuth);
                    printf("read %d bytes data from client elevation:             %u\n",sr,pclPackage.dataBlock[blockLoop][rollLoop].pointT[pointLoop].elevation);
                    printf("read %d bytes data from client reflectivity:          %u\n",sr,pclPackage.dataBlock[blockLoop][rollLoop].pointT[pointLoop].reflectivity);
                    printf("read %d bytes data from client rsv:                   %u\n",sr,pclPackage.dataBlock[blockLoop][rollLoop].pointT[pointLoop].rsv);
                }
            }
        }

        uint16_t* tmp_tail_buf = (uint16_t *)(bufOrigenAddr + 4);
        uint16_t len = *tmp_tail_buf;

        printf("write %d bytes data to server tail len:       %u\n",(int)sizeof(buf), len);

        printf("write %d bytes data to server tail rsv[0]:    %x\n",(int)sizeof(buf), *(uint8_t *)(bufOrigenAddr+40+len));
        printf("write %d bytes data to server tail rsv[1]:    %x\n",(int)sizeof(buf), *(uint8_t *)(bufOrigenAddr+40+len+1));
        printf("write %d bytes data to server tail rsv[2]:    %x\n",(int)sizeof(buf), *(uint8_t *)(bufOrigenAddr+40+len+2));
        printf("write %d bytes data to server tail rsv[3]:    %x\n",(int)sizeof(buf), *(uint8_t *)(bufOrigenAddr+40+len+3));

        printf("================================================================= end \n");

        // bzero(buf,sizeof(buf));
        // if((sr=read(cli_fd,buf,sizeof(buf)))<=0)
        // {
        //     printf("read error:%d:%s\n",errno,strerror(errno));
        //     close(cli_fd);
        //     return -1;
        // }
        // printf("read %d bytes data from server:%s\n",sr,buf);
        usleep(100000);//us
        if(while_num_for_client > WHILE_NUM)
        {
            break;
        }
    }

    close(cli_fd);
    printf("connection closed!\n");

    return 0;
}
