/*
 * Copyright (c) 2014-2022, Asensing Group
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-10     luhuadong    The first version.
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
#include <endian.h>
#include <sys/time.h>
#include <time.h>

#include "pcc2.h"

#define FPS 10 /* 10Hz */
#define FOV_HORIZON 360
#define FOV_VERTICAL 40
#define RESOLUTION_HORIZON  1     /* 0.1, 0.2 */
#define RESOLUTION_VERTICAL 0.2

#define BUFFER_SIZE 1500
static uint8_t buffer[BUFFER_SIZE] = {0};
const char *hello = "Hello from server";

// static  uint8_t   bufOrigenAddr[MAXSIZE] = {0};
static uint16_t pkgSn = 0;
static uint16_t distance = 1;  /* for test */

static void fill_packet(Pandar128Packet *packet, uint16_t azimuth, uint16_t distance)
{
    /* Header */
    packet->header.Sob = htole16(0xFFEE); /* 0xEE, 0xFF */
    packet->header.VersionMajor = 0x01;
    packet->header.VersionMinor = 0x04;
    packet->header.DistUnit = 0x04; /* 4mm */
    packet->header.Flags = 0x0;
    packet->header.LaserNum = 0x80; /* 128 */
    packet->header.BlockNum = 0x02; /* 2 block */
    packet->header.EchoCount = 0x02;
    packet->header.EchoNum = 0x02;

    /* Block */
    for (int i = 0; i < PANDAR128_BLOCK_NUM; i++)
    {
        packet->blocks[i].Azimuth = htole16(azimuth); /* 低字节在前 */

        for (int channel = 0; channel < PANDAR128_LASER_NUM; channel++)
        {
            packet->blocks[i].units[channel].Distance = distance * 100;
            packet->blocks[i].units[channel].Intensity = 0xFF;
        }
    }

    /* CRC */
    packet->crc = 0x01020304;

    /* functionSafety */
    packet->functionSafety[0] = 0x00; /* FS Version */
    packet->functionSafety[1] = 0x28; /* 0x01 << 5 || 0x01 << 3 */

    /* Tail */
    struct timeval tv;
    struct tm *pt;

    gettimeofday(&tv, NULL);
    //printf("start : %ld.%ld\n", tv.tv_sec, tv.tv_usec);

    pt = gmtime(&tv.tv_sec);

    packet->tail.UTCTime0 = pt->tm_year;
    packet->tail.UTCTime1 = pt->tm_mon + 1;
    packet->tail.UTCTime2 = pt->tm_mday;
    packet->tail.UTCTime3 = pt->tm_hour;
    packet->tail.UTCTime4 = pt->tm_min;
    packet->tail.UTCTime5 = pt->tm_sec;

    packet->tail.Timestamp = tv.tv_usec;

    packet->tail.ShutdownFlag = 0x0; /* state + mode */
    packet->tail.ReturnMode = 0x33;  /* 第一回波 */
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

int main(int argc, char *argv[])
{
    int option;
    char *ipaddr = NULL;
    char *port = NULL;
    uint32_t count = 300;
    struct sockaddr_in saddr, caddr;
    size_t len = sizeof(struct sockaddr);

    const char *const short_options = "hvi:p:n:";
    const struct option long_options[] = {

        {"help",    0, NULL, 'h'},
        {"version", 0, NULL, 'v'},
        {"ipaddr",  1, NULL, 'i'},
        {"port",    1, NULL, 'p'},
        {"count",   1, NULL, 'n'},
        {NULL, 0, NULL, 0}};

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
        case 'n':
            count = atol(optarg);
            break;
        case '?':
        default:
            printf("Error: option invalid\n");
            exit(EXIT_FAILURE);
            break;
        }
    }

    /* Server IP address and port */

    memset(&saddr, 0, sizeof(saddr));
    memset(&caddr, 0, sizeof(caddr));

    // bzero(&saddr, sizeof(struct sockaddr_in));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = INADDR_ANY;
    saddr.sin_port = htons(DEFAULT_MSOP_PORT - 1);

    printf("Host IP: %s, Port: %d\n", inet_ntoa(saddr.sin_addr), ntohs(saddr.sin_port));

    /* Client IP address and port */
    // bzero(&caddr, sizeof(struct sockaddr_in));
    caddr.sin_family = AF_INET;

    if (NULL == port)
    {
        caddr.sin_port = htons(DEFAULT_MSOP_PORT);
    }
    else
    {
        caddr.sin_port = htons(atoi(port));
    }

    if (NULL == ipaddr)
    {
        caddr.sin_addr.s_addr = htonl(INADDR_ANY);
    }
    else
    {
        caddr.sin_addr.s_addr = inet_addr(ipaddr);
    }

    printf("Target IP: %s, Port: %d\n", inet_ntoa(caddr.sin_addr), ntohs(caddr.sin_port));

    /* Create socket (UDP default) */
    int sockfd;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    /* Bind the socket with the server address */
    if (bind(sockfd, (const struct sockaddr *)&saddr, sizeof(saddr)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

#if 0

    while (count--) {
        printf("[%d] sendto %s:%d\n", count, inet_ntoa(caddr.sin_addr), ntohs(caddr.sin_port));
        sendto(sockfd, (const char *)hello, strlen(hello), MSG_CONFIRM, 
           (const struct sockaddr *) &caddr, len);
        
        sleep(5);
    }

#else
    // LidarConfigInit();

    Pandar128Packet packet = {0};

    printf("Size of packet = %lu bytes\n", sizeof(packet));
    printf("Size of Pandar128Packet = %lu bytes\n", sizeof(Pandar128Packet));
    printf("Size of Pandar128Header = %lu bytes\n", sizeof(Pandar128Header));
    printf("Size of Pandar128Block = %lu bytes\n", sizeof(Pandar128Block));
    printf("Size of Pandar128Tail = %lu bytes\n", sizeof(Pandar128Tail));

    uint8_t sn = 0;
    uint32_t sr;
    int ret = 0;

    while (count--)
    {
        printf("------ %4u\n", count);
        if (distance > 10) {
            distance = 1;
        }

        for (int i = 0; i < FOV_HORIZON / RESOLUTION_HORIZON; i++)
        {
            memset(&packet, 0, sizeof(packet));
            fill_packet(&packet, i * RESOLUTION_HORIZON * 100, distance);

            uint16_t pack_len = 0;

            ret = sendto(sockfd, &packet, sizeof(packet), 0, (struct sockaddr *)&caddr, len);

            if (ret < 0)
            {
                printf("Send package error...\n");
                close(sockfd);
                return -1;
            }
        }

        //usleep(100000); // 10Hz
        sleep(1);
        distance++;
    }

#endif

__exit:
    close(sockfd);
    printf("Exit!\n");

    return 0;
}
