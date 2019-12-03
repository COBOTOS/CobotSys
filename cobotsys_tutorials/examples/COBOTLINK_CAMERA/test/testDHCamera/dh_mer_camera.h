/*==========================================================
Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

===========================================================
File description:


===========================================================
Date            Name                 Description of Change
30-5-2018       eleven            Written
==========================================================*/

#ifndef CAID_DHMER_H
#define CAID_DHMER_H

#include "GxIAPI.h"
#include <memory>
#include <thread>

namespace caid {

    class CameraMer {
    public:
        CameraMer();

        ~CameraMer();

        virtual bool open( char sign[]);

        virtual void close();

        //一次探测操作
        void grab(int exposureTime = 500);

    private:
        void closeWithError(GX_STATUS status);

        void showErrorString(GX_STATUS error_status);

        int UnPreForImage();

        int PreForImage();

        bool PrepareForImageImprovement();

        void observerThread();

        GX_DEV_HANDLE m_device_handle; //< 设备句柄指针
        GX_FRAME_DATA m_frame_data;
        void *m_raw8_buffer;                                       ///< 将非8位raw数据转换成8位数据的时候的中转缓冲buffer
        void *m_rgb_frame_data;                                    ///< RAW数据转换成RGB数据后的存储空间，大小是相机输出数据大小的3倍
        int64_t m_pixel_format;               ///< 当前相机的pixelformat格式
        int64_t m_color_filter;                    ///< bayer插值的参数
        int64_t m_i64ColorCorrection;       ///< 颜色矫正参数
        unsigned char *m_pGammaLut;                ///< Gamma 查找表
        int m_nGammaLutLength;          ///< Gamma 查找表长度
        unsigned char *m_pContrastLut;             ///< 对比度查找表
        int m_nContrastLutLength;       ///< 对比度查找表长度
        //线程的生命周期与对象一致，对象析构时线程回收
        std::thread m_observerThread;
        long long m_mask;
        std::chrono::high_resolution_clock::time_point m_time;
        static bool m_initlib;
        bool m_isOpen;
    };
}

#endif //PROJECT_CAMERAPYLON5_H
