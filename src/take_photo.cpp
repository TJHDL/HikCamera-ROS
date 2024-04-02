#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <sstream>
#include <ctime>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <pthread.h>
#include "MvCameraControl.h"

#define CAMERA_NUM  1

int8_t photo_flag = 0;
bool hasStartPhoto = false;
bool hasEndPhoto = false;
int idx = 0;
int image_idx[CAMERA_NUM] = {1};
time_t now = time(0);
tm* ltm = localtime(&now);
std::string today = "";

MVCC_INTVALUE stParam[CAMERA_NUM];

void* handle[CAMERA_NUM] = {NULL};
unsigned char * pData[CAMERA_NUM] = {NULL};        
unsigned char *pDataForRGB[CAMERA_NUM] = {NULL};
unsigned char *pDataForSaveImage[CAMERA_NUM] = {NULL};

void photoCallback(const std_msgs::Int8::ConstPtr& msg)
{
    photo_flag = msg->data;
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

void openCameras()
{
    int nRet = MV_OK;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
        return;
    }
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                break;
            } 
            PrintDeviceInfo(pDeviceInfo);            
        }  
    } 
    else
    {
        printf("Find No Devices!\n");
        return;
    }


    if(stDeviceList.nDeviceNum < CAMERA_NUM)
    {
        printf("only have %d camera\n", stDeviceList.nDeviceNum);
        return;
    }

    unsigned int nIndex = 0;
    for(int i = 0; i < CAMERA_NUM; i++)
    {
        nIndex = i;

        // 选择设备并创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle[i], stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: MV_CC_CreateHandle fail! nRet [%x]\n", i, nRet);
            return;
        }

        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(handle[i]);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: MV_CC_OpenDevice fail! nRet [%x]\n", i, nRet);
            MV_CC_DestroyHandle(handle[i]);
            return;
        }
        
        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle[i]);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle[i],"GevSCPSPacketSize",nPacketSize);
                if(nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
            }
        }
    }
    
    for(int i = 0; i < CAMERA_NUM; i++)
    {
        nRet = MV_CC_SetEnumValue(handle[i], "TriggerMode", 1);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: MV_CC_SetTriggerMode fail! nRet [%x]\n", i, nRet);
        }

        // ch:获取数据包大小 | en:Get payload size
        // MVCC_INTVALUE stParam;
        memset(&stParam[i], 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle[i], "PayloadSize", &stParam[i]);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: Get PayloadSize fail! nRet [0x%x]\n", i, nRet);
        }

        // 开始取流
        // start grab image
        nRet = MV_CC_StartGrabbing(handle[i]);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: MV_CC_StartGrabbing fail! nRet [%x]\n",i, nRet);
        }
    }
}

void closeCameras()
{
    int nRet = MV_OK;

    for(int i = 0; i < CAMERA_NUM; i++)
    {
        // 停止取流
        // end grab image
        nRet = MV_CC_StopGrabbing(handle[i]);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: MV_CC_StopGrabbing fail! nRet [%x]\n", i, nRet);
            return;
        }

        // 销毁句柄
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle[i]);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: MV_CC_DestroyHandle fail! nRet [%x]\n", i, nRet);
            return;
        }

        if (nRet != MV_OK)
        {
            if (handle[i] != NULL)
            {
                MV_CC_DestroyHandle(handle[i]);
                handle[i] = NULL;
            }
        }
    }

    for(int i = 0; i < CAMERA_NUM; i++)
    {
        if (pData[i])
        {
            free(pData[i]);	
            pData[i] = NULL;
        }
        if (pDataForRGB[i])
        {
            free(pDataForRGB[i]);
            pDataForRGB[i] = NULL;
        }
        if (pDataForSaveImage[i])
        {
            free(pDataForSaveImage[i]);
            pDataForSaveImage[i] = NULL;
        }
    }

    printf("exit.\n");
}

void timerCallback0(const ros::TimerEvent& time_e)
{
    int nRet = MV_OK;

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    pData[0] = (unsigned char *)malloc(sizeof(unsigned char) * stParam[0].nCurValue);
    if (NULL == pData)
    {
        return;
    }
    unsigned int nDataSize = stParam[0].nCurValue;

    nRet = MV_CC_GetOneFrameTimeout(handle[0], pData[0], nDataSize, &stImageInfo, 3000);
    if (nRet == MV_OK)
    {
        pDataForSaveImage[0] = (unsigned char*)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
        if (NULL == pDataForSaveImage[0])
        {
            return;
        }
        // 填充存图参数
        // fill in the parameters of save image
        MV_SAVE_IMAGE_PARAM_EX stSaveParam;
        memset(&stSaveParam, 0, sizeof(MV_SAVE_IMAGE_PARAM_EX));
        // 从上到下依次是：输出图片格式，输入数据的像素格式，提供的输出缓冲区大小，图像宽，
        // 图像高，输入数据缓存，输出图片缓存，JPG编码质量
        // stSaveParam.enImageType = MV_Image_Jpeg; 
        stSaveParam.enImageType = MV_Image_Bmp; 
        stSaveParam.enPixelType = stImageInfo.enPixelType; 
        stSaveParam.nBufferSize = stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048;
        stSaveParam.nWidth      = stImageInfo.nWidth; 
        stSaveParam.nHeight     = stImageInfo.nHeight; 
        stSaveParam.pData       = pData[0];
        stSaveParam.nDataLen    = stImageInfo.nFrameLen;
        stSaveParam.pImageBuffer = pDataForSaveImage[0];
        stSaveParam.nJpgQuality = 80;

        nRet = MV_CC_SaveImageEx2(handle[0], &stSaveParam);
        if(MV_OK != nRet)
        {
            printf("Camera0 failed in MV_CC_SaveImage,nRet[%x]\n", nRet);
            return;
        }

        double image_time = ros::Time::now().toSec();
        std::string image_time_str = std::to_string(image_time);
        size_t point_pos = image_time_str.find(".");
        image_time_str = image_time_str.substr(0, point_pos);
        std::string save_path = "";
        save_path.append("/home/nvidia/catkin_ws/images/");
        save_path.append(today);
        if(access(save_path.c_str(), 0))
            mkdir(save_path.c_str(), S_IRWXU);
        save_path.append("/camera0");
        save_path.append(std::to_string(exposure_time));
        if(access(save_path.c_str(), 0))
            mkdir(save_path.c_str(), S_IRWXU);
        
        save_path.append("/IMG_");
        save_path.append(std::to_string(image_idx[0]));
        // save_path.append(".jpg");
        save_path.append(".bmp");
        std::cout << save_path << std::endl;
        FILE* fp = fopen(save_path.c_str(), "wb");
        int errNum = 0;
        if (NULL == fp)
        {
            printf("Camera0 fopen failed\n");
            errNum = errno;
            printf("Camera0 Open fail errno = %d reason = %s \n", errNum, strerror(errNum));
            return;
        }
        fwrite(pDataForSaveImage[0], 1, stSaveParam.nImageLen, fp);
        fclose(fp);
        image_idx[0]++;
        printf("Camera0 save image succeed\n");
    }
    else
    {
        printf("Camera0 No data[%x]\n", nRet);
    }

    free(pData[0]);
    return;
}

int main(int argc, char **argv)
{
        ros::Subscriber subscriber;
        ros::init(argc, argv,"take_photo");
        ros::NodeHandle nd;
        ros::Subscriber photo_flag_sub = nd.subscribe("/take_photo_flag", 10, photoCallback);

        ros::Timer timer0 = nd.createTimer(ros::Duration(3), timerCallback0, false, false);

        today.append(std::to_string(1900 + ltm->tm_year));
        today.append("_");
        today.append(std::to_string(1 + ltm->tm_mon));
        today.append("_");
        today.append(std::to_string(ltm->tm_mday));
        today.append("_");
        today.append(std::to_string(ltm->tm_hour));
        today.append("_");
        today.append(std::to_string(ltm->tm_min));
        today.append("_");
        today.append(std::to_string(ltm->tm_sec));
        std::cout << "###################Opening cameras...###################" << std::endl;
        openCameras();

        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // std::cout << "#######Wait for command..#######" << std::endl;
            if((photo_flag == 1) && !hasStartPhoto)
            {
                std::cout << "======Start Camera Grab======" << std::endl;
                hasStartPhoto = true;
                timer0.start();
            }
            if((photo_flag == 0) && hasStartPhoto && !hasEndPhoto)
            {
                std::cout << "======Stop Camera Grab======" << std::endl;
                std::cout << "###################Closing cameras...###################" << std::endl;
                hasEndPhoto = true;
                timer0.stop();
                closeCameras();
            }

            loop_rate.sleep();
            ros::spinOnce();
        }

        return 0;
}
