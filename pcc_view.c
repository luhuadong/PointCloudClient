#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <getopt.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "pcc.h"

int main(int argc, char *argv[])
{
    uint8_t    buffer[1500];
    PclPackage pclPackage = {0};

    //创建网络通信对象
    struct sockaddr_in saddr;
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(DEFAULT_MSOP_PORT);
    saddr.sin_addr.s_addr = INADDR_ANY;

    // 创建socket对象

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

    printf("Host IP: %s, Port: %d\n", inet_ntoa(saddr.sin_addr), ntohs(saddr.sin_port));

    socklen_t len = sizeof(saddr);

    while(1)
    {
        printf("Wait...\n");

        recvfrom(sockfd, &buffer, sizeof(buffer), 0, (struct sockaddr*)&saddr, &len);
        PrintpOrigineAddr(buffer);
        ParasePclPackage(buffer, &pclPackage);
    }
    close(sockfd);
}
