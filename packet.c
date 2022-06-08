#include <stdio.h>
#include <stdint.h>
#include "pcc.h"

void Get_1_Byte(uint8_t **buf, uint8_t *value)
{
    *value = *(uint8_t *)(*buf);
    *buf = *buf + 1;
    return;
}

void Get_2_Byte(uint8_t **buf, uint16_t *value)
{
    *value = *(uint16_t *)(*buf);
    *buf = *buf + 2;
    return;
}

void Get_4_Byte(uint8_t **buf, uint32_t *value)
{
    *value = *(uint32_t *)(*buf);
    *buf = *buf + 4;
    return;
}

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

int get_packet_length(uint8_t *buf, uint16_t *plen)
{
    uint8_t *addrTemp = buf + 4;
    Get_2_Byte(&addrTemp, plen);
    return 0;
}

void PackPclPkgDataBlockTmp(PclPackage *pclPackage, uint16_t sn)
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
                pointT_tmp[pointTLoop].distance       = (blockLoop*20 + rollLoop*30 + pointTLoop*40)%200;
                pointT_tmp[pointTLoop].azimuth        = ((sn%8)*15 + blockLoop);
                pointT_tmp[pointTLoop].elevation      = ((sn/8)%12*5 + pointTLoop);
                pointT_tmp[pointTLoop].reflectivity   = (blockLoop + rollLoop + pointTLoop)%255;
            }
        }
    }
}

void PackPclPkgTmp(PclPackage *pclPackage, uint16_t sn)
{
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ head
    //4byte code
    pclPackage->pclPackageHead.pkgLen = 1108;
    pclPackage->pclPackageHead.pkgSn = sn;
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

    PackPclPkgDataBlockTmp(pclPackage, sn);

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

uint8_t* PackPclPkgToBuf(uint8_t *buf, PclPackage *pclPackage, uint16_t pkgSn)
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
    tmp_buf_1 = tmp_buf_1 + 4;
    Set_2_Byte(&tmp_buf_1, pkgPayloadLen);
    Set_2_Byte(&tmp_buf_1, pkgSn);
    
    buf = PackPclPkgTeil(buf, pclPackage);
    return buf;
}

/*****************************************************/
void PrintPclPackageHead(PclPackageHead *pclPackageHead)
{
    if(NULL == pclPackageHead)
    {
        printf("pclPackageHead is empty!");
    }
    printf("headCode:                  %u\n", pclPackageHead->head_code);       //4   55aa5aa5
    printf("pkgLen:                    %u\n", pclPackageHead->pkgLen);         //2   1111
    printf("pkgSn:                     %u\n", pclPackageHead->pkgSn);          //2   0
    printf("lidarType:                 %u\n", pclPackageHead->lidarType);      //2   1
    printf("headCoprotocolVersionde:   %u\n", pclPackageHead->protocolVersion);//2   11
    // printf("timestamp:          %u\n", pclPackageHead->timestamp);          //10  0
    printf("measurenmentMode:          %u\n", pclPackageHead->measurenmentMode);//1  3
    printf("laserNum:                  %u\n", pclPackageHead->laserNum);       //1   8
    printf("blockNum:                  %u\n", pclPackageHead->blockNum);       //1   4
    printf("waveMode:                  %u\n", pclPackageHead->waveMode);       //1   0
    printf("timeSyncMode:              %u\n", pclPackageHead->timeSyncMode);   //1   1
    printf("timeSyncState:             %u\n", pclPackageHead->timeSyncState);  //1   0
    printf("memsTmp:                   %u\n", pclPackageHead->memsTmp);        //1   20
    printf("slotNum:                   %u\n", pclPackageHead->slotNum);        //1   1
    // printf("slotNum:                   %u\n", pclPackageHead->slotNum);     //10  0
    return;
}

void PrintPointT(PointT *pointT, uint16_t blockloop, uint16_t rollLoop, uint8_t loop)
{
    printf("distance:%u     ", pointT->distance);
    printf("azimuth:%u      ", pointT->azimuth);
    printf("elevation:%u    ", pointT->elevation);
    printf("reflectivity:%u \n", pointT->reflectivity);
    return;
}

void PrintDataBlock(DataBlock *dataBlock, uint16_t blockloop, uint16_t rollLoop, uint8_t loop)
{
    printf("***************************************************blockloop:%u, rollLoop:%u\n", blockloop, rollLoop);
    printf("channelNum:%u    ", dataBlock->channelNum);
    printf("timeOffSet:%u    ", dataBlock->timeOffSet);
    printf("returnSn:%u      \n", dataBlock->returnSn);
    for(uint16_t pointloop = 0; pointloop < MAX_POINT_NUM_IN_BLOCK; pointloop++)
    {
        PrintPointT(&dataBlock->pointT[pointloop], blockloop, rollLoop, loop);
    }
    return;
}

void PrintPclPackageTail(PclPackageTail *pclPackageTail)
{
    printf("rsv[0]:                %x\n", pclPackageTail->rsv[0]);
    printf("rsv[1]:                %x\n", pclPackageTail->rsv[1]);
    printf("rsv[2]:                %x\n", pclPackageTail->rsv[2]);
    printf("rsv[3]:                %x\n", pclPackageTail->rsv[3]);
    return;
}

static void PrintPclPackage(PclPackage *pclPackage, uint8_t loop)
{
    PrintPclPackageHead(&pclPackage->pclPackageHead);
    for(uint16_t blockLoop = 0; blockLoop < MAX_BLOCK_NUM; blockLoop++)
    {
       for(uint16_t rollLoop = 0; rollLoop < ROLL_NUM; rollLoop++)
       {
           PrintDataBlock(&pclPackage->dataBlock[blockLoop][rollLoop], blockLoop, rollLoop, loop);
       }
    }
    PrintPclPackageTail(&pclPackage->pclPackageTail);
    return;
}



StatusCode PclHeadCodeChech(uint8_t *buf)
{
    if((buf[0] != 0x55) || (buf[1] != 0xaa) || (buf[2] != 0x5a) || (buf[3] != 0xa5))
    {
        printf("PclHeadCodeChech fail! %x,%x,%x,%x\n", buf[0],buf[1],buf[2],buf[3]);
        return FALSE;
    }
    printf("PclHeadCodeChech OK!\n");
    return TRUE;
}

uint8_t* ParasePclPackageHead(uint8_t *buf, PclPackage *pclPackage)
{
    PclPackageHead *pclPackageHead = &pclPackage->pclPackageHead;
    
    StatusCode retCode = PclHeadCodeChech(buf);
    if(retCode != TRUE)
    {
        return NULL;//error code
    }
    printf("++++++++++++++++  0000 %p\n", buf);

    // Get_4_Byte(&buf, &pclPackageHead.head_code);//直接读4字节
    for(uint8_t i = 0; i < 4; i++)
    {
        printf("buf[%u]:%x\n",i, buf[i]);
    }
    buf = buf + 4;
    printf("++++++++++++++++  0000 %p\n", buf);
    Get_2_Byte(&buf, &pclPackageHead->pkgLen);
    Get_2_Byte(&buf, &pclPackageHead->pkgSn);
    Get_2_Byte(&buf, &pclPackageHead->lidarType);
    Get_2_Byte(&buf, &pclPackageHead->protocolVersion);
    printf("++++++++++++++++  0000 %p, %u, %u, %u, %u\n", buf, 
           pclPackageHead->pkgLen, pclPackageHead->pkgSn, 
           pclPackageHead->lidarType, pclPackageHead->protocolVersion);

    for(uint8_t loop = 0; loop<10; loop++)
    {
        Get_1_Byte(&buf, &pclPackageHead->timestamp[loop]);
    }

    Get_1_Byte(&buf, &pclPackageHead->measurenmentMode);
    Get_1_Byte(&buf, &pclPackageHead->laserNum);
    Get_1_Byte(&buf, &pclPackageHead->blockNum);
    Get_1_Byte(&buf, &pclPackageHead->waveMode);
    printf("++++++++++++++++  0000 %p, %u, %u, %u, %u\n", buf, pclPackageHead->measurenmentMode, pclPackageHead->laserNum, pclPackageHead->blockNum, pclPackageHead->waveMode);

    Get_1_Byte(&buf, &pclPackageHead->timeSyncMode);
    Get_1_Byte(&buf, &pclPackageHead->timeSyncState);
    Get_1_Byte(&buf, &pclPackageHead->memsTmp);
    Get_1_Byte(&buf, &pclPackageHead->slotNum);
    printf("++++++++++++++++  0000 %p, %u, %u, %u, %u\n", buf, pclPackageHead->timeSyncMode, pclPackageHead->timeSyncState, pclPackageHead->memsTmp, pclPackageHead->slotNum);

    for(uint8_t loop = 0; loop<10; loop++)
    {
        Get_1_Byte(&buf, &pclPackageHead->rsv[loop]);
    }
    return buf;
}

uint8_t* ParasePclPkgPointT(uint8_t *buf, PointT *pointT_tmp, uint8_t channelNum)
{
    for(uint8_t pointLoop = 0; pointLoop < channelNum; pointLoop++)
    {
        Get_2_Byte(&buf, &pointT_tmp[pointLoop].distance);
        Get_2_Byte(&buf, &pointT_tmp[pointLoop].azimuth);
        Get_2_Byte(&buf, &pointT_tmp[pointLoop].elevation);
        Get_1_Byte(&buf, &pointT_tmp[pointLoop].reflectivity);
        Get_2_Byte(&buf, &pointT_tmp[pointLoop].rsv);
    }
    return buf;
}

uint8_t* ParasePclPkgDataBlock(uint8_t *buf, DataBlock *dataBlock_tmp)
{
    uint8_t timeOffSetSetFlag = 0;

    for(uint8_t rollLoop = 0; rollLoop < ROLL_NUM; rollLoop++)
    {
        Get_1_Byte(&buf, &dataBlock_tmp[rollLoop].channelNum);
        if(dataBlock_tmp[rollLoop].channelNum == 0)
        {
            continue;
        }
        if(timeOffSetSetFlag == 0)//one time_Offset needed for one block
        {
            Get_1_Byte(&buf, &dataBlock_tmp[rollLoop].timeOffSet);
            timeOffSetSetFlag = 1;
        }
        Get_1_Byte(&buf, &dataBlock_tmp[rollLoop].returnSn);

        PointT *pointT_tmp = &dataBlock_tmp[rollLoop].pointT[0];
        uint8_t channelNum = dataBlock_tmp[rollLoop].channelNum;
        buf = ParasePclPkgPointT(buf, pointT_tmp, channelNum);
    }
    return buf;
}

uint8_t* ParasePclPackagePayLoad(uint8_t *buf, PclPackage *pclPackage)
{
    for(uint8_t blockLoop = 0; blockLoop < pclPackage->pclPackageHead.blockNum; blockLoop++)//支持动态block数
    {
        DataBlock *dataBlock_tmp = &pclPackage->dataBlock[blockLoop][0];
        buf = ParasePclPkgDataBlock(buf, dataBlock_tmp);
    }
    return buf;
}

uint8_t* ParasePclPackageTail(uint8_t *buf, PclPackage *pclPackage)
{
    Get_1_Byte(&buf, &pclPackage->pclPackageTail.rsv[0]);
    Get_1_Byte(&buf, &pclPackage->pclPackageTail.rsv[1]);
    Get_1_Byte(&buf, &pclPackage->pclPackageTail.rsv[2]);
    Get_1_Byte(&buf, &pclPackage->pclPackageTail.rsv[3]);
    return buf;
}

void ParasePclPackage(uint8_t *buf, PclPackage *pclPackage)
{
    printf("\n");

    printf("xxxxxxxxxxxxxxxxxxxxxxxxx headAddr 111 %p\n", buf);

    buf = ParasePclPackageHead(buf, pclPackage);
    printf("xxxxxxxxxxxxxxxxxxxxxxxxx payloadAddr 111 %p\n", buf);

    printf("+++++++++++++++++++++ 111 %u\n", pclPackage->pclPackageHead.pkgSn);
    buf = ParasePclPackagePayLoad(buf, pclPackage);
    printf("xxxxxxxxxxxxxxxxxxxxxxxxx tailAddr 111 %p\n", buf);

    buf = ParasePclPackageTail(buf, pclPackage);
    printf("xxxxxxxxxxxxxxxxxxxxxxxxx end 111 %p\n", buf);

    if(pclPackage->pclPackageHead.pkgSn%16 == 0)
    {
        PrintPclPackage(pclPackage, pclPackage->pclPackageHead.pkgSn);
    }

    return;
}

void PrintpOrigineAddr(uint8_t *bufAddr)
{
    uint8_t *curAddr =  bufAddr;
    uint16_t pkgPayloadLen = 0;
    uint8_t *lenAddr = curAddr + 4;
    Get_2_Byte(&lenAddr, &pkgPayloadLen);
    for(size_t loop = (size_t)curAddr; loop < (size_t)(curAddr+pkgPayloadLen+40+4); loop++)  //40byte head 4byte tail
    {
        if(loop == (size_t)(curAddr+40))
        {
            printf("\n");
            printf("***********************************************************above is head\n");
        }
        if(loop == (size_t)(curAddr+40+pkgPayloadLen))
        {
            printf("\n");
            printf("***********************************************************above is payload\n");
        }

        printf("%02x",*(uint8_t *)loop);
    }
    printf("***********************************************************above is tail\n");
}
