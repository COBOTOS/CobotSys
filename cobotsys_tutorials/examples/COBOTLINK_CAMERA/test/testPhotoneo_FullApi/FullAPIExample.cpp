// FullAPIExample.cpp : Defines the entry point for the console application.
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
#include <sstream>

#if defined(_WIN32)
#include <windows.h>
#elif defined (__linux__)

#include <unistd.h>

#endif

#if defined(_WIN32)
#define LOCAL_CROSS_SLEEP(Millis) Sleep(Millis)
#elif defined (__linux__) || defined(__APPLE__)
#define LOCAL_CROSS_SLEEP(Millis) usleep(Millis * 1000)
#endif

//The whole api is in namespace pho (Photoneo) :: api
class PhoXiExamples {
private:
    template<class T>
    bool ReadLine(T &Output) const {
        std::string Input;
        std::getline(std::cin, Input);
        std::stringstream InputSteam(Input);
        if (InputSteam >> Output) {
            return true;
        } else {
            return false;
        }
    }

    bool ReadLine(std::string &Output) const {
        std::getline(std::cin, Output);
        return true;
    }

public:
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList;
    pho::api::PPhoXi PhoXiDevice;
    pho::api::PhoXiFactory Factory;
    pho::api::PFrame SampleFrame;

    void GetAvailableDevicesExample() {
        //Wait for the PhoXiControl
        while (!Factory.isPhoXiControlRunning()) {
            LOCAL_CROSS_SLEEP(100);
        }
        std::cout << "PhoXi Control Version: " << Factory.GetPhoXiControlVersion() << std::endl;
        std::cout << "PhoXi API Version: " << Factory.GetAPIVersion() << std::endl;
        DeviceList = Factory.GetDeviceList();
        std::cout << "PhoXi Factory found " << DeviceList.size() << " devices by GetDeviceList call." << std::endl
                  << std::endl;
        for (std::size_t i = 0; i < DeviceList.size(); i++) {
            std::cout << "Device: " << i << std::endl;
            std::cout << "  Name:                    " << DeviceList[i].Name << std::endl;
            std::cout << "  Hardware Identification: " << DeviceList[i].HWIdentification << std::endl;
            std::cout << "  Type:                    " << (std::string) DeviceList[i].Type << std::endl;
            std::cout << "  Firmware version:        " << DeviceList[i].FirmwareVersion << std::endl;
            std::cout << "  Status:                  " << (DeviceList[i].Status.Attached ? "Attached to PhoXi Control. "
                                                                                         : "Not Attached to PhoXi Control. ")
                      << (DeviceList[i].Status.Ready ? "Ready to connect" : "Occupied") << std::endl << std::endl;
        }
    }

    void ConnectPhoXiDeviceExample() {
        //You can connect to any device connected to local network (with compatible ip4 settings)
        //The connection can be made in multiple ways
        while (true) {
            std::cout << "Please enter the number of the way to connect to your device from this possibilities:"
                      << std::endl;
            std::cout << "  1. Connect by Hardware Identification Number" << std::endl;
            std::cout << "  2. Connect by Index listed from GetDeviceList call" << std::endl;
            std::cout << "  3. Connect first device Attached to PhoXi Control - if Any" << std::endl << std::endl;
            std::cout << "  4. Refresh GetDeviceList" << std::endl << std::endl;
            std::cout << "Please enter the choice: ";
            std::size_t Index;
            if (!ReadLine(Index)) continue;
            switch (Index) {
                case 1:
                    ConnectPhoXiDeviceBySerialExample();
                    break;
                case 2:
                    ConnectPhoXiDeviceByPhoXiDeviceInformationEntryExample();
                    break;
                case 3:
                    ConnectFirstAttachedPhoXiDeviceExample();
                    break;
                case 4:
                    GetAvailableDevicesExample();
                    break;
                default:
                    continue;
            }
            if (PhoXiDevice && PhoXiDevice->isConnected()) break;
        }
    }

    void ConnectPhoXiDeviceBySerialExample() {
        std::cout << std::endl << "Please enter the Hardware Identification Number: ";
        std::string HardwareIdentification ="1712009";
        pho::api::PhoXiTimeout Timeout = pho::api::PhoXiTimeout::ZeroTimeout;
        PhoXiDevice = Factory.CreateAndConnect(HardwareIdentification, Timeout);
        if (PhoXiDevice) {
            std::cout << "Connection to the device " << HardwareIdentification << " was Successful!" << std::endl;
        } else {
            std::cout << "Connection to the device " << HardwareIdentification << " was Unsuccessful!" << std::endl;
        }
    }

    void ConnectPhoXiDeviceByPhoXiDeviceInformationEntryExample() {
        std::cout << std::endl << "Please enter the Index listed from GetDeviceList call: ";
        std::size_t Index;
        if (!ReadLine(Index)) return;
        if (Index >= DeviceList.size()) {
            std::cout << "Bad Index, or not number!" << std::endl;
            return;
        }
        PhoXiDevice = Factory.Create(DeviceList[Index]);
        if (PhoXiDevice) {
            if (PhoXiDevice->Connect()) {
                std::cout << "Connection to the device " << DeviceList[Index].HWIdentification << " was Successful!"
                          << std::endl;
            } else {
                std::cout << "Connection to the device " << DeviceList[Index].HWIdentification << " was Unsuccessful!"
                          << std::endl;
            }
        } else {
            std::cout << "Unspecified error" << std::endl;
        }
    }

    void ConnectFirstAttachedPhoXiDeviceExample() {
        PhoXiDevice = Factory.CreateAndConnectFirstAttached();
        if (PhoXiDevice) {
            std::cout << "Connection to the device " << (std::string) PhoXiDevice->HardwareIdentification
                      << " was Successful!" << std::endl;
        } else {
            std::cout << "There is no attached device, or the device is not ready!" << std::endl;
        }
    }

    void BasicDeviceStateExample() {
        //Check if the device is connected
        if (PhoXiDevice && PhoXiDevice->isConnected()) {
            std::cout << "You are connected to " << (std::string) PhoXiDevice->GetType()
                      << " with Hardware Identification " << (std::string) PhoXiDevice->HardwareIdentification
                      << std::endl;
            std::vector<std::string> SupportedFeatures = PhoXiDevice->Features.GetSupportedFeatures();
            std::cout << "  Status:" << std::endl;
            std::cout << "    "
                      << (PhoXiDevice->isConnected() ? "Device is connected" : "Device is not connected (Error)")
                      << std::endl;
            std::cout << "    " << (PhoXiDevice->isAcquiring() ? "Device is in acquisition mode"
                                                               : "Device is not in acquisition mode") << std::endl;
            std::cout << "  This device have these features supported:";
            for (std::size_t i = 0; i < SupportedFeatures.size(); i++) {
                std::cout << " " << SupportedFeatures[i] << ";";
            }
            std::cout << std::endl << std::endl;
            //We will go trough all current Device features
            //You can ask the feature if it is implemented and if it is possible to Get or Set the feature value
            if (PhoXiDevice->CapturingMode.isEnabled() && PhoXiDevice->CapturingMode.CanGet()) {
                pho::api::PhoXiCapturingMode CapturingMode = PhoXiDevice->CapturingMode;
                //You can ask the feature, if the last performed operation was successful
                if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());
                pho::api::PhoXiSize Resolution = CapturingMode.Resolution;
                //you can also access the resolution by PhoXiDevice->Resolution;
                std::cout << "  CapturingMode: " << std::endl;
                std::cout << "    Resolution:" << std::endl;
                std::cout << "      Width: " << Resolution.Width << std::endl;
                std::cout << "      Height: "
                          << PhoXiDevice->Resolution->Height /*You can also directly access the value inside*/
                          << std::endl;
            }
            if (PhoXiDevice->CapturingSettings.isEnabled() && PhoXiDevice->CapturingSettings.CanGet()) {
                pho::api::PhoXiCapturingSettings CapturingSettings = PhoXiDevice->CapturingSettings;
                if (!PhoXiDevice->CapturingSettings.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->CapturingSettings.GetLastErrorMessage().c_str());
                std::cout << "  CapturingSettings: " << std::endl;
                std::cout << "    ShutterMultiplier: " << CapturingSettings.ShutterMultiplier << std::endl;
                std::cout << "    ScanMultiplier: " << CapturingSettings.ScanMultiplier << std::endl;
            }
            if (PhoXiDevice->TriggerMode.isEnabled() && PhoXiDevice->TriggerMode.CanGet()) {
                pho::api::PhoXiTriggerMode TriggerMode = PhoXiDevice->TriggerMode;
                if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
                std::cout << "  TriggerMode: " << (std::string) TriggerMode << std::endl;
            }
            if (PhoXiDevice->Timeout.isEnabled() && PhoXiDevice->Timeout.CanGet()) {
                pho::api::PhoXiTimeout Timeout = PhoXiDevice->Timeout;
                if (!PhoXiDevice->Timeout.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->Timeout.GetLastErrorMessage().c_str());
                std::cout << "  Timeout: " << (std::string) Timeout << std::endl;
            }
            if (PhoXiDevice->ProcessingSettings.isEnabled() && PhoXiDevice->ProcessingSettings.CanGet()) {
                pho::api::PhoXiProcessingSettings ProcessingSettings = PhoXiDevice->ProcessingSettings;
                if (!PhoXiDevice->ProcessingSettings.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->ProcessingSettings.GetLastErrorMessage().c_str());
                std::cout << "  ProcessingSettings: " << std::endl;
                std::cout << "    Confidence: " << ProcessingSettings.Confidence << std::endl;
            }
            if (PhoXiDevice->OutputSettings.isEnabled() && PhoXiDevice->OutputSettings.CanGet()) {
                pho::api::FrameOutputSettings OutputSettings = PhoXiDevice->OutputSettings;
                if (!PhoXiDevice->OutputSettings.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->OutputSettings.GetLastErrorMessage().c_str());
                std::cout << "  OutputSettings: " << std::endl;
                std::cout << "    SendConfidenceMap: " << (OutputSettings.SendConfidenceMap ? "Yes" : "No")
                          << std::endl;
                std::cout << "    SendDepthMap: " << (OutputSettings.SendDepthMap ? "Yes" : "No") << std::endl;
                std::cout << "    SendNormalMap: " << (OutputSettings.SendNormalMap ? "Yes" : "No") << std::endl;
                std::cout << "    SendPointCloud: " << (OutputSettings.SendPointCloud ? "Yes" : "No") << std::endl;
                std::cout << "    SendTexture: " << (OutputSettings.SendTexture ? "Yes" : "No") << std::endl;
            }
            if (PhoXiDevice->SupportedCapturingModes.isEnabled() && PhoXiDevice->SupportedCapturingModes.CanGet()) {
                std::vector<pho::api::PhoXiCapturingMode>
                        SupportedCapturingModes = PhoXiDevice->SupportedCapturingModes;
                if (!PhoXiDevice->SupportedCapturingModes.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->SupportedCapturingModes.GetLastErrorMessage().c_str());
                std::cout << "  SupportedCapturingModes: " << std::endl;
                for (std::size_t i = 0; i < SupportedCapturingModes.size(); i++) {
                    std::cout << "    (" << std::to_string(SupportedCapturingModes[i].Resolution.Width) << "x"
                              << std::to_string(SupportedCapturingModes[i].Resolution.Height) << ")" << std::endl;
                }
            }
            if (PhoXiDevice->HardwareIdentification.isEnabled() && PhoXiDevice->HardwareIdentification.CanGet()) {
                std::string HardwareIdentification = PhoXiDevice->HardwareIdentification;
                if (!PhoXiDevice->HardwareIdentification.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->HardwareIdentification.GetLastErrorMessage().c_str());
                std::cout << "  HardwareIdentification: " << HardwareIdentification << std::endl;
            }
        }
    }

    void FreerunExample() {
        //Check if the device is connected
        if (PhoXiDevice && PhoXiDevice->isConnected()) {
            //If it is not in Freerun mode, we need to switch the modes
            if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Freerun) {
                std::cout << "Device is not in Freerun mode" << std::endl;
                if (PhoXiDevice->isAcquiring()) {
                    std::cout << "Stopping acquisition" << std::endl;
                    //If the device is in Acquisition mode, we need to stop the acquisition
                    if (!PhoXiDevice->StopAcquisition()) {
                        throw std::runtime_error("Error in StopAcquistion");
                    }
                }
                std::cout << "Switching to Freerun mode " << std::endl;
                //Switching the mode is as easy as assigning of a value, it will call the appropriate calls in the background
                PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Freerun;
                //Just check if did everything run smoothly
                if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
            }
            //Start the device acquisition, if necessary
            if (!PhoXiDevice->isAcquiring()) {
                if (!PhoXiDevice->StartAcquisition()) {
                    throw std::runtime_error("Error in StartAcquisition");
                }
            }
            //We can clear the current Acquisition buffer -- This will not clear Frames that arrives to the PC after the Clear command is performed
            int ClearedFrames = PhoXiDevice->ClearBuffer();
            std::cout << ClearedFrames << " were cleared from the cyclic buffer" << std::endl;

            //While we checked the state of the StartAcquisition call, this check is not necessary, but it is a good practice
            if (PhoXiDevice->isAcquiring()) {
                for (std::size_t i = 0; i < 5; i++) {
                    std::cout << "Waiting for frame " << i << std::endl;
                    //Get the frame
                    pho::api::PFrame Frame =
                            PhoXiDevice->GetFrame(/*You can specify Timeout here - default is the Timeout stored in Timeout Feature -> Infinity by default*/);
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
                                          << Frame->NormalMap.Size.Height << " Type: "
                                          << Frame->NormalMap.GetElementName() << std::endl;
                            }
                            if (!Frame->DepthMap.Empty()) {
                                std::cout << "    DepthMap: " << Frame->DepthMap.Size.Width << " x "
                                          << Frame->DepthMap.Size.Height << " Type: "
                                          << Frame->DepthMap.GetElementName() << std::endl;
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
        }
    }

    void SoftwareTriggerExample() {
        //Check if the device is connected
        if (PhoXiDevice && PhoXiDevice->isConnected()) {
            //If it is not in Software trigger mode, we need to switch the modes
            if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Software) {
                std::cout << "Device is not in Software trigger mode" << std::endl;
                if (PhoXiDevice->isAcquiring()) {
                    std::cout << "Stopping acquisition" << std::endl;
                    //If the device is in Acquisition mode, we need to stop the acquisition
                    if (!PhoXiDevice->StopAcquisition()) {
                        throw std::runtime_error("Error in StopAcquistion");
                    }
                }
                std::cout << "Switching to Software trigger mode " << std::endl;
                //Switching the mode is as easy as assigning of a value, it will call the appropriate calls in the background
                PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
                //Just check if did everything run smoothly
                if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
            }
            //Start the device acquisition, if necessary
            if (!PhoXiDevice->isAcquiring()) {
                if (!PhoXiDevice->StartAcquisition()) {
                    throw std::runtime_error("Error in StartAcquisition");
                }
            }
            //We can clear the current Acquisition buffer -- This will not clear Frames that arrives to the PC after the Clear command is performed
            int ClearedFrames = PhoXiDevice->ClearBuffer();
            std::cout << ClearedFrames << " frames were cleared from the cyclic buffer" << std::endl;

            //While we checked the state of the StartAcquisition call, this check is not necessary, but it is a good practice
            if (PhoXiDevice->isAcquiring()) {
                for (std::size_t i = 0; i < 5; i++) {
                    std::cout << "Triggering the " << i << "-th frame" << std::endl;
                    int FrameID =
                            PhoXiDevice->TriggerFrame(/*If false is passed here, the device will reject the frame if it is not ready to be triggered, if true us supplied, it will wait for the trigger*/);
                    if (FrameID < 0) {
                        //If negative number is returned trigger was unsuccessful
                        std::cout << "Trigger was unsuccessful!" << std::endl;
                        continue;
                    } else {
                        std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
                    }
                    std::cout << "Waiting for frame " << i << std::endl;
                    //Wait for a frame with specific FrameID. There is a possibility, that frame triggered before the trigger will arrive after the trigger call, and will be retrieved before requested frame
                    //  Because of this, the TriggerFrame call returns the requested frame ID, so it can than be retrieved from the Frame structure. This call is doing that internally in background
                    pho::api::PFrame Frame =
                            PhoXiDevice->GetSpecificFrame(
                                    FrameID/*, You can specify Timeout here - default is the Timeout stored in Timeout Feature -> Infinity by default*/);
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
                                          << Frame->NormalMap.Size.Height << " Type: "
                                          << Frame->NormalMap.GetElementName() << std::endl;
                            }
                            if (!Frame->DepthMap.Empty()) {
                                std::cout << "    DepthMap: " << Frame->DepthMap.Size.Width << " x "
                                          << Frame->DepthMap.Size.Height << " Type: "
                                          << Frame->DepthMap.GetElementName() << std::endl;
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
        }
    }

    void ChangeSettingsExample() {
        //Check if the device is connected
        if (PhoXiDevice && PhoXiDevice->isConnected()) {
            //Check if the feature is supported and if it we have required access permissions
            //  These checks are not necessary, these have in mind multiple different devices in the future
            if (!PhoXiDevice->CapturingSettings.isEnabled() || !PhoXiDevice->CapturingSettings.CanSet()
                || !PhoXiDevice->CapturingSettings.CanGet()) {
                std::cout
                        << "Settings used in example are not supported by the Device Hardware, or are Read only on the specific device"
                        << std::endl;
                return;
            }
            std::cout << "Settings change example" << std::endl;

            //For purpose of this example, we will change the trigger mode to Software Trigger, it is not necessary for the exhibition of desired functionality

            if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Software) {
                if (PhoXiDevice->isAcquiring()) {
                    if (!PhoXiDevice->StopAcquisition()) {
                        throw std::runtime_error("Error in StopAcquistion");
                    }
                }
                PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
                //Just check if did everything run smoothly
                if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
            }
            if (!PhoXiDevice->isAcquiring()) PhoXiDevice->StartAcquisition();

            int CurrentShutterMultiplier = PhoXiDevice->CapturingSettings->ShutterMultiplier;

            //To change the setting, just assign a new value
            PhoXiDevice->CapturingSettings->ShutterMultiplier = CurrentShutterMultiplier + 1;

            //You can check if the operation succeed
            if (!PhoXiDevice->CapturingSettings.isLastOperationSuccessful())
                throw std::runtime_error(PhoXiDevice->CapturingSettings.GetLastErrorMessage().c_str());

            //Get the current Output configuration
            pho::api::FrameOutputSettings CurrentOutputSettings = PhoXiDevice->OutputSettings;
            pho::api::FrameOutputSettings NewOutputSettings = CurrentOutputSettings;
            NewOutputSettings.SendPointCloud = true;
            NewOutputSettings.SendNormalMap = true;
            NewOutputSettings.SendDepthMap = true;
            NewOutputSettings.SendConfidenceMap = true;
            NewOutputSettings.SendTexture = true;
            //Send all outputs
            PhoXiDevice->OutputSettings = NewOutputSettings;

            //Trigger the frame
            int FrameID = PhoXiDevice->TriggerFrame();
            //Check if the frame was successfully triggered
            if (FrameID < 0) throw std::runtime_error("Software trigger failed!");
            //Retrieve the frame
            pho::api::PFrame Frame = PhoXiDevice->GetFrame(FrameID);
            if (Frame) {
                //Save the frame for next example
                SampleFrame = Frame;
            }

            //Change the setting back
            PhoXiDevice->OutputSettings = CurrentOutputSettings;
            PhoXiDevice->CapturingSettings->ShutterMultiplier = CurrentShutterMultiplier;

            if (!PhoXiDevice->CapturingSettings.isLastOperationSuccessful())
                throw std::runtime_error(PhoXiDevice->CapturingSettings.GetLastErrorMessage().c_str());

            //Try to change device resolution
            if (PhoXiDevice->SupportedCapturingModes.isEnabled() && PhoXiDevice->SupportedCapturingModes.CanGet()
                && PhoXiDevice->CapturingMode.isEnabled() && PhoXiDevice->CapturingMode.CanSet()
                && PhoXiDevice->CapturingMode.CanGet()) {
                //Retrieve current capturing mode
                pho::api::PhoXiCapturingMode CurrentCapturingMode = PhoXiDevice->CapturingMode;
                if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());

                //Get all supported modes
                std::vector<pho::api::PhoXiCapturingMode>
                        SupportedCapturingModes = PhoXiDevice->SupportedCapturingModes;
                if (!PhoXiDevice->SupportedCapturingModes.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->SupportedCapturingModes.GetLastErrorMessage().c_str());

                //Cycle trough all other Supported modes, change the settings and grab a frame
                for (std::size_t i = 0; i < SupportedCapturingModes.size(); i++) {
                    if (!(SupportedCapturingModes[i] == CurrentCapturingMode)) {
                        PhoXiDevice->CapturingMode = SupportedCapturingModes[i];
                        if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful())
                            throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());
                        //Trigger Frame
                        int FrameID = PhoXiDevice->TriggerFrame();
                        if (FrameID < 0) throw std::runtime_error("Software trigger failed!");
                        Frame = PhoXiDevice->GetSpecificFrame(FrameID);
                        if (Frame) {
                            std::cout << "Arrived Frame Resolution: " << Frame->GetResolution().Width << " x "
                                      << Frame->GetResolution().Height << std::endl;
                        }
                    }
                }
                //Change the mode back
                PhoXiDevice->CapturingMode = CurrentCapturingMode;
                if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful())
                    throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());

            }
        }
    }

    void DataHandlingExample() {
        //Check if we have SampleFrame Data
        if (SampleFrame && !SampleFrame->Empty()) {
            //We will count the number of measured points
            if (!SampleFrame->PointCloud.Empty()) {
                int MeasuredPoints = 0;
                pho::api::Point3_32f ZeroPoint(0.0f, 0.0f, 0.0f);
                for (int y = 0; y < SampleFrame->PointCloud.Size.Height; y++) {
                    for (int x = 0; x < SampleFrame->PointCloud.Size.Width; x++) {
                        if (SampleFrame->PointCloud[y][x] != ZeroPoint) {
                            MeasuredPoints++;
                        }
                    }
                }
                std::cout << "Your sample Point cloud has " << MeasuredPoints << " measured points." << std::endl;
                pho::api::Point3_32f *RawPointer = SampleFrame->PointCloud.GetDataPtr();
                float *MyLocalCopy = new float[SampleFrame->PointCloud.GetElementsCount() * 3];
                memcpy(MyLocalCopy, RawPointer, SampleFrame->PointCloud.GetDataSize());
                //Data are organized as a matrix of X, Y, Z floats, see the documentation for all other types
                delete[] MyLocalCopy;
                //Data from SampleFrame, or all other frames that are returned by the device are copied from the Cyclic buffer and will remain in the memory until the Frame will go out of scope
                //You can specifically call SampleFrame->PointCloud.Clear() to release some of the data
            }
            //You can store the Frame as a ply structure
            SampleFrame->SaveAsPly("SampleFrame.ply"/*, You have multiple storing options*/);
            //If you want OpenCV support, you need to link appropriate libraries and add OpenCV include directory
            //To add the support, add #define PHOXI_OPENCV_SUPPORT before include of PhoXi include files
#ifdef PHOXI_OPENCV_SUPPORT
            if (!SampleFrame->PointCloud.Empty()) {
                cv::Mat PointCloudMat;
                if (SampleFrame->PointCloud.ConvertTo(PointCloudMat)) {
                    cv::Point3f MiddlePoint = PointCloudMat.at<cv::Point3f>(PointCloudMat.rows, PointCloudMat.cols);
                    std::cout << "Middle point: " << MiddlePoint.x << "; " << MiddlePoint.y << "; " << MiddlePoint.z;
                }
            }
#endif
            //If you want PCL support, you need to link appropriate libraries and add PCL include directory
            //To add the support, add #define PHOXI_PCL_SUPPORT before include of PhoXi include files
#ifdef PHOXI_PCL_SUPPORT
            //The PCL convert will convert the appropriate data into the pcl PointCloud based on the Point Cloud type
            pcl::PointCloud<pcl::PointXYZRGBNormal> MyPCLCloud;
            SampleFrame->ConvertTo(MyPCLCloud);
#endif
        }
    }

    void CorrectDisconnectExample() {
        //The whole API is designed on C++ standards, using smart pointers and constructor/destructor logic
        //All resources will be closed automatically, but the device state will not be affected -> it will remain connected in PhoXi Control and if in freerun, it will remain Scanning
        //To Stop the device, just
        PhoXiDevice->StopAcquisition();
        //If you want to disconnect and logout the device from PhoXi Control, so it will then be available for other devices, call
        std::cout << "Do you want to logout the device? Enter 0 for no, enter 1 for yes: ";
        bool Entry;
        if (!ReadLine(Entry)) return;
        PhoXiDevice->Disconnect(Entry);
        //The call PhoXiDevice without Logout will be called automatically by destructor
    }

    PhoXiExamples() {
        try {
            GetAvailableDevicesExample();
            ConnectPhoXiDeviceExample();
            BasicDeviceStateExample();
            FreerunExample();
            SoftwareTriggerExample();
            ChangeSettingsExample();
            DataHandlingExample();
            CorrectDisconnectExample();
        } catch (std::runtime_error &InternalException) {
            std::cout << std::endl << "Exception was thrown: " << InternalException.what() << std::endl;
        }
    }
};

int main(int argc, char *argv[]) {
    PhoXiExamples Example;
    return 0;
}

