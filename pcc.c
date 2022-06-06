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
#include "pcc.h"

#define BUFFER_SIZE  1500
static uint8_t buffer[BUFFER_SIZE] = {0};
const char *hello = "Hello from server";

static LidarParamConfig lidarParamConfig = {0};
//static  uint8_t   bufOrigenAddr[MAXSIZE] = {0};
static  uint16_t  pkgSn = 0;

static void Set_1_Byte(uint8_t **buf, uint8_t value)
{
    *(uint8_t *)(*buf) = value;
    (*buf)++;
    return;
}

static void Set_2_Byte(uint8_t **buf, uint16_t value)
{
    // **buf = value;
    *(uint16_t *)(*buf) = value;
    *buf = *buf + 2;
    return;
}

static void Set_4_Byte(uint8_t **buf, uint32_t value)
{
    *(uint32_t *)(*buf) = value;
    *buf = *buf + 4;
    return;
}

static void PackPclPkgDataBlockTmp(PclPackage *pclPackage)
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
            for(uint8_t pointTLoop = 0; pointTLoop < LIDAR_NUM; pointTLoop++)
            {
                pointT_tmp[pointTLoop].distance       = 15;
                pointT_tmp[pointTLoop].azimuth        = 75;
                pointT_tmp[pointTLoop].elevation      = 13;
                pointT_tmp[pointTLoop].reflectivity   = 3;
            }
        }
    }
}

static void PackPclPkgTmp(PclPackage *pclPackage, uint16_t sn)
{
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ head
    //4byte code
    pclPackage->pclPackageHead.pkgLen = htons(1108);
    pclPackage->pclPackageHead.pkgSn = htons(sn);
    pclPackage->pclPackageHead.lidarType = htons(1);
    pclPackage->pclPackageHead.protocolVersion = htons(11);

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


static uint8_t* PackPclPkgHead(uint8_t *buf, PclPackage *pclPackage)
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

static uint8_t* PackPclPkgPointT(uint8_t *buf, PointT *pointT_tmp, uint8_t channelNum)
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

static uint8_t* PackPclPkgDataBlock(uint8_t *buf, DataBlock *dataBlock_tmp)
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

static uint8_t* PackPclPkgPayload(uint8_t *buf, PclPackage *pclPackage)
{
    for(uint8_t blockLoop = 0; blockLoop < MAX_BLOCK_NUM; blockLoop++)
    {
        DataBlock *dataBlock_tmp = &pclPackage->dataBlock[blockLoop][0];
        buf = PackPclPkgDataBlock(buf, dataBlock_tmp);
    }
    return buf;
}

static uint8_t* PackPclPkgTeil(uint8_t *buf, PclPackage *pclPackage)
{
    Set_1_Byte(&buf, pclPackage->pclPackageTail.rsv[0]);
    Set_1_Byte(&buf, pclPackage->pclPackageTail.rsv[1]);
    Set_1_Byte(&buf, pclPackage->pclPackageTail.rsv[2]);
    Set_1_Byte(&buf, pclPackage->pclPackageTail.rsv[3]);
    return buf;
}

static uint8_t* PackPclPkgToBuf(uint8_t *buf, PclPackage *pclPackage)
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
    Set_2_Byte(&tmp_buf_1, pkgSn);
    
    buf = PackPclPkgTeil(buf, pclPackage);
    return buf;
}


static uint32_t read_file(const char* filename, char** content)
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

static uint32_t LidarConfigInit()
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

    cJSON* item = cJSON_GetObjectItem(items, "lidar_type");
    lidarParamConfig.lidar_type = item->valueint;
    item = cJSON_GetObjectItem(items, "msg_source");
    lidarParamConfig.msg_source = item->valueint;
    item = cJSON_GetObjectItem(items, "lidar_num");
    lidarParamConfig.lidar_num = item->valueint;
    item = cJSON_GetObjectItem(items, "max_block_num");
    lidarParamConfig.max_block_num = item->valueint;
    item = cJSON_GetObjectItem(items, "roll_num");
    lidarParamConfig.roll_num = item->valueint;
    item = cJSON_GetObjectItem(items, "msop_port");
    lidarParamConfig.msop_port = item->valueint;
    item = cJSON_GetObjectItem(items, "difop_port");
    lidarParamConfig.difop_port = item->valueint;
    item = cJSON_GetObjectItem(items, "start_angle");
    lidarParamConfig.start_angle = item->valueint;
    item = cJSON_GetObjectItem(items, "end_angle");
    lidarParamConfig.end_angle = item->valueint;
    item = cJSON_GetObjectItem(items, "min_distance");
    lidarParamConfig.min_distance = item->valueint;
    item = cJSON_GetObjectItem(items, "max_distance");
    lidarParamConfig.max_distance = item->valueint;
    item = cJSON_GetObjectItem(items, "wait_for_difop");
    lidarParamConfig.wait_for_difop = item->valueint;
    item = cJSON_GetObjectItem(items, "use_lidar_clock");
    lidarParamConfig.use_lidar_clock = item->valueint;

    item = cJSON_GetObjectItem(items, "point_cloud_send_ip");
    char *msopIp = item->valuestring;
    strncpy(lidarParamConfig.point_cloud_send_ip, msopIp, sizeof(lidarParamConfig.point_cloud_send_ip));

    item = cJSON_GetObjectItem(items, "packet_send_ip");
    char *mispIp = item->valuestring;
    strncpy(lidarParamConfig.packet_send_ip, mispIp, sizeof(lidarParamConfig.packet_send_ip));

    item = cJSON_GetObjectItem(items, "pcap_path");
    char *pcapPath = item->valuestring;
    lidarParamConfig.pcap_path = pcapPath;

    cJSON_Delete(items);

    printf("point_cloud_send_ip: %s\n", lidarParamConfig.point_cloud_send_ip);
    printf("packet_send_ip: %s\n", lidarParamConfig.packet_send_ip);
    printf("pcap_path: %s\n", lidarParamConfig.pcap_path);

    return 0;
}

static void show_usage(const char *cmd)
{
    printf("Usage: %s [options] ... \n", cmd);
    printf("This is a point cloud client demo\n\n");
    printf("  -h, --help           display this help and exit\n");
    printf("  -v, --version        output version information and exit\n");
    printf("  -i, --ipaddr=ADDR    set target IP address\n");
    printf("  -p, --port=PORT      set target port\n\n");

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
    char *ipaddr = NULL;
    char *port = NULL;
    struct sockaddr_in saddr, caddr;
    size_t len = sizeof(struct sockaddr);

    const char * const short_options = "hvi:p:";
    const struct option long_options[] = {

        { "help",    0, NULL, 'h' },
        { "version", 0, NULL, 'v' },
        { "ipaddr",  1, NULL, 'i' },
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
            ipaddr = strdup(optarg);
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

    /* Server IP address and port */

    memset(&saddr, 0, sizeof(saddr));
    memset(&caddr, 0, sizeof(caddr));

    //bzero(&saddr, sizeof(struct sockaddr_in));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = INADDR_ANY;
    saddr.sin_port = htons(DEFAULT_MSOP_PORT);

    printf("Host IP: %s, Port: %d\n", inet_ntoa(saddr.sin_addr), ntohs(saddr.sin_port));

    /* Client IP address and port */
    //bzero(&caddr, sizeof(struct sockaddr_in));
    caddr.sin_family = AF_INET;

    if (NULL == port) {
        caddr.sin_port = htons(DEFAULT_MSOP_PORT);
    } else {
        caddr.sin_port = htons(atoi(port));
    }
    
    if (NULL == ipaddr) {
        caddr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else {
        caddr.sin_addr.s_addr = inet_addr(ipaddr);
    }

    printf("Target IP: %s, Port: %d\n", inet_ntoa(caddr.sin_addr), ntohs(caddr.sin_port));
    
    /* Create socket (UDP default) */
    int sockfd;

    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    /* Bind the socket with the server address */
    if ( bind(sockfd, (const struct sockaddr *)&saddr, sizeof(saddr)) < 0 )
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    int count = 10;

#if 0

    while (count--) {
        printf("[%d] sendto %s:%d\n", count, inet_ntoa(caddr.sin_addr), ntohs(caddr.sin_port));
        sendto(sockfd, (const char *)hello, strlen(hello), MSG_CONFIRM, 
           (const struct sockaddr *) &caddr, len);
        
        sleep(5);
    }

#else
    //LidarConfigInit();

    PclPackage pclPackage = {0};

    uint8_t sn = 0;
    uint32_t sr;
    int ret = 0;


    while (count--)
    {
        PackPclPkgTmp(&pclPackage, sn++);
        PackPclPkgToBuf(buffer, &pclPackage);

        printf("Send %lu bytes...\n", sizeof(buffer));

        ret = sendto(sockfd, (const char *)buffer, sizeof(buffer), 
                     MSG_CONFIRM, (const struct sockaddr *) &caddr, len);
        
        if (ret < 0) {
            printf("Send package error...\n");
            close(sockfd);
            return -1;
        }

        usleep(100000); // 10Hz
    }

#endif
    close(sockfd);
    printf("Exit!\n");

    return 0;
}
