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

/* VTK library */
#include <vtkActor.h>
#include <vtkCleanPolyData.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkInteractorStyleTrackballCamera.h>

/* Custom */
#include "pcc.h"

static uint8_t    buffer[1500];
static PclPackage pclPackage = {0};

void PrintPointT(PointT *pointT, uint16_t blockloop, uint16_t rollLoop, uint8_t loop)
{
    printf("distance:%u     ", pointT->distance);
    printf("azimuth:%u      ", pointT->azimuth);
    printf("elevation:%u    ", pointT->elevation);
    printf("reflectivity:%u \n", pointT->reflectivity);
    return;
}

void insertPointFromPacket(vtkSmartPointer<vtkPoints> &m_Points, 
                           vtkSmartPointer<vtkCellArray> &vertices,
                           PclPackage *package)
{
    double x, y, z;
    double r, theta, phi;
    //uint16_t blockLoop, rollLoop, pointloop

    for(uint16_t blockLoop = 0; blockLoop < MAX_BLOCK_NUM; blockLoop++)
    {
        printf("======== Block %d\n", blockLoop+1);
        for(uint16_t rollLoop = 0; rollLoop < ROLL_NUM; rollLoop++)
        {
            DataBlock *dataBlock = &package->dataBlock[blockLoop][rollLoop];
            //PrintDataBlock(&pclPackage->dataBlock[blockLoop][rollLoop], blockLoop, rollLoop, loop);
            for(uint16_t pointloop = 0; pointloop < MAX_POINT_NUM_IN_BLOCK; pointloop++)
            {
                //PrintPointT(&dataBlock->pointT[pointloop], blockloop, rollLoop, loop);
                r = dataBlock->pointT[pointloop].distance;
                theta = dataBlock->pointT[pointloop].azimuth;
                phi = dataBlock->pointT[pointloop].elevation;

                /* convert polar to cartesian coordinates */
                x = r * std::sin(theta) * std::sin(phi);
                y = r * std::cos(theta);
                z = r * std::sin(theta) * std::cos(phi);

                int i = pointloop + rollLoop * MAX_POINT_NUM_IN_BLOCK + blockLoop * ROLL_NUM * MAX_POINT_NUM_IN_BLOCK;

                m_Points->InsertPoint(i, x, y, z);  //_加入点信息
                vertices->InsertNextCell(1);        //_加入细胞顶点信息----用于渲染点集
                vertices->InsertCellPoint(i);
            }
        }
    }
}

int main(int argc, char *argv[])
{
    /* VTK variables */
    vtkSmartPointer<vtkPoints> m_Points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();  //_存放细胞顶点，用于渲染（显示点云所必须的）
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyDataMapper> pointMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkActor> pointActor = vtkSmartPointer<vtkActor>::New();
    vtkSmartPointer<vtkRenderer> ren1 = vtkSmartPointer< vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
    vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> istyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();

    /* Init VTK window */
    //_创建待显示数据源
 
    polyData->SetPoints(m_Points);      //_设置点集
    polyData->SetVerts(vertices);       //_设置渲染顶点
    pointMapper->SetInputData(polyData);
 
    pointActor->SetMapper(pointMapper);
    pointActor->GetProperty()->SetColor(1, 0, 0);
    pointActor->GetProperty()->SetAmbient(0.5);
    pointActor->GetProperty()->SetPointSize(3);
    //pointActor->GetProperty()->SetRepresentationToWireframe();
    //pointActor->GetProperty()->SetRepresentationToSurface();
 
    ren1->AddActor(pointActor);
    ren1->SetBackground(0, 0, 0);
 
    renWin->AddRenderer(ren1);
    renWin->SetSize(800, 800);
 
    iren->SetInteractorStyle(istyle);
    iren->SetRenderWindow(renWin);  //交互
    

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
        insertPointFromPacket(m_Points, vertices, &pclPackage);

        renWin->Render();  /* 创建线程 disk 和 gdrv */
        iren->Start();
    }
    close(sockfd);
}
