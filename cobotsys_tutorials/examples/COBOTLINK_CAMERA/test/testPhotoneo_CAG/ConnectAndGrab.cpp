// ConnectAndGrab.cpp : Defines the entry point for the console application.
//
#define PHOXI_OPENCV_SUPPORT
#define PHOXI_PCL_SUPPORT
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include  <opencv2/core.hpp>
#include  <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <PhoXiOpenCVSupport.h>
#include <PhoXiPCLSupport.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include "PhoXi.h"

#include <vector>
#include <string>
#include <iostream>
#include <boost/make_shared.hpp>
#if defined(_WIN32)
#include <windows.h>
#elif defined (__linux__)
#include <unistd.h>
#endif

int main(int argc, char *argv[]) {
    pho::api::PhoXiFactory Factory;
    //Check if the PhoXi Control Software is running
    if (!Factory.isPhoXiControlRunning()) return 0;
    std::cout << "PhoXi Control Software is running" << std::endl;
    //Get List of available devices on the network
    std::vector <pho::api::PhoXiDeviceInformation> DeviceList = Factory.GetDeviceList();
    std::cout << "PhoXi Factory found " << DeviceList.size() << " devices by GetDeviceList call." << std::endl
              << std::endl;
    for (std::size_t i = 0; i < DeviceList.size(); i++) {
        std::cout << "Device: " << i << std::endl;
        std::cout << "  Name:                    " << DeviceList[i].Name << std::endl;
        std::cout << "  Hardware Identification: " << DeviceList[i].HWIdentification << std::endl;
        std::cout << "  Type:                    " << (std::string) DeviceList[i].Type << std::endl;
        std::cout << "  Firmware version:        " << DeviceList[i].FirmwareVersion << std::endl;
        std::cout << "  Status:                  "
                  << (DeviceList[i].Status.Attached ? "Attached to PhoXi Control. " : "Not Attached to PhoXi Control. ")
                  << (DeviceList[i].Status.Ready ? "Ready to connect" : "Occupied") << std::endl << std::endl;
    }

    //Try to connect Device opened in PhoXi Control, if Any
//    pho::api::PPhoXi PhoXiDevice = Factory.CreateAndConnectFirstAttached();
//    if (PhoXiDevice) {
//        std::cout
//            << "You have already PhoXi device opened in PhoXi Control Software, the API Example is connected to device: "
//            << (std::string) PhoXiDevice->HardwareIdentification << std::endl;
//    } else {
//        std::cout
//            << "You have no PhoXi device opened in PhoXi Control Software, the API Example will try to connect to last device in device list"
//            << std::endl;
//        if (!DeviceList.empty()) {
//            PhoXiDevice = Factory.CreateAndConnect(DeviceList.back().HWIdentification);
//        }
//    }

    pho::api::PPhoXi PhoXiDevice = Factory.CreateAndConnect("InstalledExamples-PhoXi-example(File3DCamera)");
    if (!PhoXiDevice) {
        std::cout << "No device is connected!" << std::endl;
        return 0;
    }

    if (PhoXiDevice->isConnected()) {
        std::cout << "Your device is connected" << std::endl;
        if (PhoXiDevice->isAcquiring()) {
            PhoXiDevice->StopAcquisition();
        }
        std::cout << "Starting Software trigger mode" << std::endl;
        PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
        PhoXiDevice->ClearBuffer();
        PhoXiDevice->StartAcquisition();
        if (PhoXiDevice->isAcquiring()) {
            for (int i = 0; i < 5; i++) {
                std::cout << "Triggering the " << i << "-th frame" << std::endl;
                int FrameID = PhoXiDevice->TriggerFrame();
                if (FrameID < 0) {
                    //If negative number is returned trigger was unsuccessful
                    std::cout << "Trigger was unsuccessful!" << std::endl;
                    continue;
                } else {
                    std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
                }
                std::cout << "Waiting for frame " << i << std::endl;
                pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID, pho::api::PhoXiTimeout::Infinity);
                if (Frame) {
                    cv::Mat mat ;
                    Frame->PointCloud.ConvertTo(mat);
                    cv::imshow("PointCloud" ,mat);
                    cv::waitKey(-1);


                    cv::Mat mat1;
//
                    Frame->Texture.ConvertTo(mat1);
                    auto data = Frame->Texture[1][2];

                    cv::Mat Texture;
                    Texture = cv::Mat::zeros(Frame->Texture.Size.Width,
                                             Frame->Texture.Size.Height, CV_32FC1);
                    for (int i = 0; i < Texture.rows; ++i) {
                        for (int j = 0; j < Texture.cols; ++j) {
                            Texture.at<float>(i, j) = Frame->Texture.At(j, i);
                        }
                    }

//                    Texture = Texture.t() * 2.0;
                    Texture.convertTo(mat1, CV_8UC4);

                    cv::imshow("Texture" ,mat1);
                    cv::waitKey(-1);

                    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pointCloud  = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
                    Frame->ConvertTo(*pointCloud.get());
//                    pcl::io::savePCDFile("test.pcd",*(pointCloud.get()));
//                    pointCloud.clone
                    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
                    viewer.showCloud(pointCloud);
                    while (!viewer.wasStopped())
                    {
                        sleep(2);
                    }
                    std::cout << "Frame retrieved" << std::endl;
                    std::cout << "  Frame params: " << std::endl;
                    std::cout << "    Frame Index: " << Frame->Info.FrameIndex << std::endl;
                    std::cout << "    Frame Timestamp: " << Frame->Info.FrameTimestamp << std::endl;
                    std::cout << "    Frame Duration: " << Frame->Info.FrameDuration << std::endl;
                    std::cout << "    Frame Resolution: " << Frame->GetResolution().Width << " x "
                              << Frame->GetResolution().Height << std::endl;
                    std::cout << "    Sensor Position: " << Frame->Info.SensorPosition.x << "; "
                              << Frame->Info.SensorPosition.y << "; " << Frame->Info.SensorPosition.z << std::endl;
                    if (!Frame->Empty()) {
                        std::cout << "  Frame data: " << std::endl;
                        if (!Frame->PointCloud.Empty()) {
                            std::cout << "    PointCloud: " << Frame->PointCloud.Size.Width << " x "
                                      << Frame->PointCloud.Size.Height << " Type: "
                                      << Frame->PointCloud.GetElementName() << std::endl;
                        }
                        if (!Frame->NormalMap.Empty()) {
                            std::cout << "    NormalMap: " << Frame->NormalMap.Size.Width << " x "
                                      << Frame->NormalMap.Size.Height << " Type: " << Frame->NormalMap.GetElementName()
                                      << std::endl;
                        }
                        if (!Frame->DepthMap.Empty()) {
                            std::cout << "    DepthMap: " << Frame->DepthMap.Size.Width << " x "
                                      << Frame->DepthMap.Size.Height << " Type: " << Frame->DepthMap.GetElementName()
                                      << std::endl;
                        }
                        if (!Frame->ConfidenceMap.Empty()) {
                            std::cout << "    ConfidenceMap: " << Frame->ConfidenceMap.Size.Width << " x "
                                      << Frame->ConfidenceMap.Size.Height << " Type: "
                                      << Frame->ConfidenceMap.GetElementName() << std::endl;
                        }
                        if (!Frame->Texture.Empty()) {
                            std::cout << "    Texture: " << Frame->Texture.Size.Width << " x "
                                      << Frame->Texture.Size.Height << " Type: " << Frame->Texture.GetElementName()
                                      << std::endl;
                        }
                    } else {
                        std::cout << "Frame is empty.";
                    }
                } else {
                    std::cout << "Failed to retrieve the frame!";
                }
            }
        }

        PhoXiDevice->StopAcquisition();
        std::cout << "Starting Freerun mode" << std::endl;
        PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Freerun;
        PhoXiDevice->StartAcquisition();

        if (PhoXiDevice->isAcquiring()) {
            for (int i = 0; i < 5; i++) {
                std::cout << "Waiting for frame " << i << std::endl;
                pho::api::PFrame Frame = PhoXiDevice->GetFrame(pho::api::PhoXiTimeout::Infinity);
                if (Frame) {
                    std::cout << "Frame retrieved" << std::endl;
                    std::cout << "  Frame params: " << std::endl;
                    std::cout << "    Frame Index: " << Frame->Info.FrameIndex << std::endl;
                    std::cout << "    Frame Timestamp: " << Frame->Info.FrameTimestamp << std::endl;
                    std::cout << "    Frame Duration: " << Frame->Info.FrameDuration << std::endl;
                    std::cout << "    Frame Resolution: " << Frame->GetResolution().Width << " x "
                              << Frame->GetResolution().Height << std::endl;
                    std::cout << "    Sensor Position: " << Frame->Info.SensorPosition.x << "; "
                              << Frame->Info.SensorPosition.y << "; " << Frame->Info.SensorPosition.z << std::endl;
                    if (!Frame->Empty()) {
                        std::cout << "  Frame data: " << std::endl;
                        if (!Frame->PointCloud.Empty()) {
                            std::cout << "    PointCloud: " << Frame->PointCloud.Size.Width << " x "
                                      << Frame->PointCloud.Size.Height << " Type: "
                                      << Frame->PointCloud.GetElementName() << std::endl;
                        }
                        if (!Frame->NormalMap.Empty()) {
                            std::cout << "    NormalMap: " << Frame->NormalMap.Size.Width << " x "
                                      << Frame->NormalMap.Size.Height << " Type: " << Frame->NormalMap.GetElementName()
                                      << std::endl;
                        }
                        if (!Frame->DepthMap.Empty()) {
                            std::cout << "    DepthMap: " << Frame->DepthMap.Size.Width << " x "
                                      << Frame->DepthMap.Size.Height << " Type: " << Frame->DepthMap.GetElementName()
                                      << std::endl;
                        }
                        if (!Frame->ConfidenceMap.Empty()) {
                            std::cout << "    ConfidenceMap: " << Frame->ConfidenceMap.Size.Width << " x "
                                      << Frame->ConfidenceMap.Size.Height << " Type: "
                                      << Frame->ConfidenceMap.GetElementName() << std::endl;
                        }
                        if (!Frame->Texture.Empty()) {
                            std::cout << "    Texture: " << Frame->Texture.Size.Width << " x "
                                      << Frame->Texture.Size.Height << " Type: " << Frame->Texture.GetElementName()
                                      << std::endl;
                        }
                    } else {
                        std::cout << "Frame is empty.";
                    }
                } else {
                    std::cout << "Failed to retrieve the frame!";
                }
            }
        }
        PhoXiDevice->StopAcquisition();
    }
    PhoXiDevice->Disconnect();

    return 0;
}

