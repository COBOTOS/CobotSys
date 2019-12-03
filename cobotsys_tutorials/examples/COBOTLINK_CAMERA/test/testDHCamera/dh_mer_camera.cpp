/*==========================================================
Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

===========================================================
File description:


===========================================================
Date            Name                 Description of Change
30-5-2018       eleven            Written
==========================================================*/

#include "dh_mer_camera.h"
#include "DxImageProc.h"
#include <opencv2/opencv.hpp>
#include <logger/Logger.h>
#define MEMORY_ALLOT_ERROR -1
const double dGammaParam = 2.82;
const  int nContrastParam = 25;
bool caid::CameraMer::m_initlib = false;

namespace caid {

    /// 错误提示函数宏定义
    void CameraMer::closeWithError(GX_STATUS status) {
        showErrorString(status);
        status = GXCloseDevice(m_device_handle);
        if (m_device_handle != NULL) {
            m_device_handle = NULL;
        }
        status = GXCloseLib();
    }


    void CameraMer::showErrorString(GX_STATUS error_status) {
        char *error_info = NULL;
        size_t size = 0;
        GX_STATUS status = GX_STATUS_ERROR;

        // 获取错误信息长度，并申请内存空间
        status = GXGetLastError(&error_status, NULL, &size);
        error_info = new char[size];
        if (NULL == error_info) {
            return;
        }

        // 获取错误信息，并显示
        status = GXGetLastError(&error_status, error_info, &size);
        if (status != GX_STATUS_SUCCESS) {
//        QMessageBox::about(NULL, "Error", " GXGetLastError接口调用失败 ! ");
            LOG_INFO << "GXGetLastError接口调用失败 !";
        } else {
//        QMessageBox::about(NULL, "Error", tr("%1").arg(QString(QLatin1String(error_info))));
            LOG_INFO << "GXGetLastError接口调用失败 !";
        }

        // 释放申请的内存空间
        if (NULL != error_info) {
            delete[] error_info;
            error_info = NULL;
        }
    }

    CameraMer::CameraMer() : m_i64ColorCorrection(0), m_pGammaLut(NULL), m_pContrastLut(NULL),m_isOpen(false){
    }

    CameraMer::~CameraMer() {
        close();
        LOG_INFO << "CameraMer::~CameraMer";
    }

    bool CameraMer::open(char* sign) {
        GX_STATUS status = GX_STATUS_SUCCESS;
        int ret = 0;

        //初始化库
        if(!m_initlib){
            status = GXInitLib();
            if (status != GX_STATUS_SUCCESS) {
                showErrorString(status);
                return false;
            }
            m_initlib = true;
        }

        //枚举设备个数
        uint32_t device_number = 0;
        status = GXUpdateDeviceList(&device_number, 1000);
        if (status != GX_STATUS_SUCCESS) {
            LOG_INFO << "未获取到设备";
            showErrorString(status);
            return false;
        }
        GX_OPEN_PARAM open_param;
        //初始化设备打开参数，默认打开序号为１的设备
        open_param.accessMode = GX_ACCESS_EXCLUSIVE;
        open_param.openMode = GX_OPEN_SN;
        open_param.pszContent = sign;

        if (device_number <= 0) {
            printf("<No device>\n");
            return false;
        } else {
            //默认打开第1个设备
            status = GXOpenDevice(&open_param, &m_device_handle);
            if (status == GX_STATUS_SUCCESS) {
                LOG_INFO << "打开成功";
            } else {
                LOG_INFO << "打开失败";
                return false;
            }
        }


        //设置采集模式为连续采集
        status = GXSetEnum(m_device_handle, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
        if (status != GX_STATUS_SUCCESS) {
            closeWithError(status);
            return false;
        }

        //设置触发开关为ON
        status = GXSetEnum(m_device_handle, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
        if (status != GX_STATUS_SUCCESS) {
            closeWithError(status);
            return false;
        }

        //设置触发源为软触发
        status = GXSetEnum(m_device_handle, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_SOFTWARE);
        if (status != GX_STATUS_SUCCESS) {
            closeWithError(status);
            return false;
        }


        //获取相机输出数据的颜色格式
        status = GXGetEnum(m_device_handle, GX_ENUM_PIXEL_FORMAT, &m_pixel_format);
        if (status != GX_STATUS_SUCCESS) {
            showErrorString(status);
        }

        //相机采集图像为彩色还是黑白
        status = GXGetEnum(m_device_handle, GX_ENUM_PIXEL_COLOR_FILTER, &m_color_filter);
        if (status != GX_STATUS_SUCCESS) {
            showErrorString(status);
        }

        //设置自动白平衡
        status = GXSetEnum(m_device_handle, GX_ENUM_AWB_LAMP_HOUSE,
                         GX_AWB_LAMP_HOUSE_ADAPTIVE);
        if (status != GX_STATUS_SUCCESS) {
            showErrorString(status);
        }
        //设置连续自动白平衡
        status = GXSetEnum(m_device_handle,GX_ENUM_BALANCE_WHITE_AUTO,
                         GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        if (status != GX_STATUS_SUCCESS) {
            showErrorString(status);
        }

        //为采集做准备
        ret = PreForImage();
        if (ret != 0) {
            closeWithError(0);
            printf("<Failed to prepare for acquire image>\n");
            return false;
        }
        status = GXSendCommand(m_device_handle, GX_COMMAND_ACQUISITION_START);
        if (status != GX_STATUS_SUCCESS) {
            closeWithError(status);
            return 0;
        }
        //发送开采命令
        status = GXSendCommand(m_device_handle, GX_COMMAND_ACQUISITION_START);
        if (status != GX_STATUS_SUCCESS) {
            closeWithError(status);
            return 0;
        }
        if (!PrepareForImageImprovement()) {
            LOG_INFO << "PrepareForImageImprovement failed!";
        }
        m_observerThread = std::thread(std::bind(&CameraMer::observerThread, this));
        m_observerThread.detach();
        m_isOpen = true;
        return true;
    }

    void CameraMer::close() {
        GX_STATUS status = GX_STATUS_SUCCESS;
        if(!m_isOpen)
        {
            LOG_INFO<<"未打开！！";
            return ;
        }
        int ret = 0;
        //为停止采集做准备

        ret = UnPreForImage();
        if (ret != 0) {
            closeWithError(0);
            LOG_INFO<<"closeWithError!";
            return;
        }
        //关闭设备
        status = GXCloseDevice(m_device_handle);
        if (status != GX_STATUS_SUCCESS) {

            return;
        }
        //释放库
//        status = GXCloseLib();
    }

    void CameraMer::grab(int exposureTime) {

        GX_STATUS status = GX_STATUS_SUCCESS;
        status = GXSetFloat(m_device_handle, GX_FLOAT_EXPOSURE_TIME, exposureTime);
        long long count = 0;
        //发送一次软触发命令
//        caid::Timer t1;
        status = GXSendCommand(m_device_handle, GX_COMMAND_TRIGGER_SOFTWARE);
        if (status != GX_STATUS_SUCCESS) {
            LOG_INFO << "CameraMer::grab failed";
//            return CAMERA_GRAB_TRIGGER_ERROR;
        }

//        if (m_io.get())
//            count = m_io->cameraIO();

        std::chrono::high_resolution_clock::time_point imageTime = std::chrono::high_resolution_clock::now();

        m_mask = count;
        m_time = imageTime;

        status = GXGetImage(m_device_handle, &m_frame_data, 1000);
        bool is_implemented = false;
        if (status == GX_STATUS_SUCCESS) {
            if (m_frame_data.nStatus == 0) {
//                printf("<Successful acquisition: Width: %d Height: %d>\n", m_frame_data.nWidth, m_frame_data.nHeight);
                status = GXIsImplemented(m_device_handle, GX_BUFFER_FRAME_INFORMATION, &is_implemented);
                if (status != GX_STATUS_SUCCESS) {
                    LOG_INFO << "fial!";
                }

                DxRaw8toRGB24((char *) m_frame_data.pImgBuf, m_rgb_frame_data, m_frame_data.nWidth,
                              m_frame_data.nHeight, RAW2RGB_NEIGHBOUR,
                              DX_PIXEL_COLOR_FILTER(m_color_filter), false);
                //图像增强
                if (m_i64ColorCorrection != 0 || m_pGammaLut != NULL || m_pContrastLut != NULL) {
                    status = DxImageImprovment(m_rgb_frame_data, m_rgb_frame_data, m_frame_data.nWidth,
                                               m_frame_data.nHeight, m_i64ColorCorrection, m_pContrastLut, m_pGammaLut);
                    if (status != DX_OK) {
                        LOG_INFO << "faild ImageImprovment! ";
                    }
                }

                cv::Mat temp = cv::Mat(m_frame_data.nHeight,
                                       m_frame_data.nWidth,
                                       CV_8UC3,
                                       (uint8_t *) m_rgb_frame_data).clone();
                cv::imwrite("test.bmp",temp);
            }
        }
//    qDebug()<<"出图耗时："<< t1.elapsed()<<"ms";
    }

    void CameraMer::observerThread() {

    }

    int CameraMer::PreForImage() {
        GX_STATUS status = GX_STATUS_SUCCESS;

        //为获取的图像分配存储空间
        int64_t payload_size = 0;
        status = GXGetInt(m_device_handle, GX_INT_PAYLOAD_SIZE, &payload_size);
        if (status != GX_STATUS_SUCCESS) {
            showErrorString(status);
            return status;
        }

        m_frame_data.pImgBuf = malloc(payload_size);
        if (m_frame_data.pImgBuf == NULL) {
            printf("<Failed to allocate memory>\n");
            return MEMORY_ALLOT_ERROR;
        }

        //将非8位raw数据转换成8位数据的时候的中转缓冲buffer
        m_raw8_buffer = malloc(payload_size);
        if (m_raw8_buffer == NULL) {
            printf("<Failed to allocate memory>\n");
            return MEMORY_ALLOT_ERROR;
        }

        //RGB数据是RAW数据的3倍大小
        m_rgb_frame_data = malloc(payload_size * 3);
        if (m_rgb_frame_data == NULL) {
            printf("<Failed to allocate memory>\n");
            return MEMORY_ALLOT_ERROR;
        }

        return 0;
    }

    int CameraMer::UnPreForImage() {
        GX_STATUS status = GX_STATUS_SUCCESS;
        //发送停采命令
        status = GXSendCommand(m_device_handle, GX_COMMAND_ACQUISITION_STOP);
        if (status != GX_STATUS_SUCCESS) {
            return status;
        }

        //释放buffer
        if (m_frame_data.pImgBuf != NULL) {
            free(m_frame_data.pImgBuf);
            m_frame_data.pImgBuf = NULL;
        }

        if (m_raw8_buffer != NULL) {
            free(m_raw8_buffer);
            m_raw8_buffer = NULL;
        }

        if (m_rgb_frame_data != NULL) {
            free(m_rgb_frame_data);
            m_rgb_frame_data = NULL;
        }

        return 0;
    }

    bool CameraMer::PrepareForImageImprovement() {
        VxInt32 emDxStatus = DX_OK;

        // Make sure Lut memory being released
        if(m_pGammaLut != NULL)
        {
            delete[] m_pGammaLut;
            m_pGammaLut = NULL;
        };
        if(m_pContrastLut != NULL)
        {
            delete[] m_pContrastLut;
            m_pContrastLut = NULL;
        };
        // This value only for get Lut Length, could be any value in range(0.1-10)

        // Get LUT length of Gamma LUT(LUT length not determine by dGammaParam)
        emDxStatus = DxGetGammatLut(dGammaParam, NULL, &m_nGammaLutLength);
        if (emDxStatus != DX_OK) {
            LOG_INFO << "DxGetGammatLUT Error", "Error : Get gamma LUT length failed!";
            return false;
        }
        try {
            m_pGammaLut = new unsigned char[m_nGammaLutLength];
        }
        catch (std::bad_alloc &e) {
            return false;
        }
        emDxStatus = DxGetGammatLut(dGammaParam, m_pGammaLut, &m_nGammaLutLength);
        if (emDxStatus != DX_OK) {
            LOG_INFO << "Gamma创建失败";
            return false;
        }
        // 创建对比度查找表, could be any value in range(-50-100)

        // Get LUT length of Contrast LUT(LUT length not determine by nContrastParam)
        emDxStatus = DxGetContrastLut(nContrastParam, NULL, &m_nContrastLutLength);
        if (emDxStatus != DX_OK) {
            LOG_INFO << "DxGetContrastLut Error", "Error : Get contrast LUT length failed";
            return false;
        }
        try {
            m_pContrastLut = new unsigned char[m_nContrastLutLength];
        }
        catch (std::bad_alloc &e) {
            return false;
        }
        emDxStatus = DxGetContrastLut(nContrastParam, m_pContrastLut, &m_nContrastLutLength);
        if (emDxStatus != DX_OK) {
            return false;
        }
        //色彩校正
        emDxStatus = GXGetInt(m_device_handle, GX_INT_COLOR_CORRECTION_PARAM, &m_i64ColorCorrection);
        if (emDxStatus != DX_OK) {
            LOG_INFO << "setDxColorCorre Error！";
            return false;
        }
        return true;
    }
}