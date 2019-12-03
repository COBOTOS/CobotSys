//
// Created by cobot on 19-9-8.
//

#include <vtkAutoInit.h>
#include <vtkSmartPointer.h>
#include <vtkPNGReader.h>
#include <vtkImageViewer2.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkJPEGWriter.h>
#include <vtkRenderer.h> //定义了Camera

//VTK_MODULE_INIT(vtkRenderingOpenGL);

int readWriteImage(){
    //读取PNG图像
    vtkSmartPointer<vtkPNGReader> pngread =
            vtkSmartPointer<vtkPNGReader>::New();
    pngread->SetFileName("/home/cobot/图片/RegistorFactory.png");
    //显示该幅图像
    vtkSmartPointer<vtkImageViewer2> ImageViewer =
            vtkSmartPointer<vtkImageViewer2>::New();
    ImageViewer->SetInputConnection(pngread->GetOutputPort());

    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInter =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    ImageViewer->SetupInteractor(renderWindowInter);
    ImageViewer->Render();
    ImageViewer->GetRenderer()->ResetCamera();
    ImageViewer->Render();

    //写图像
    vtkSmartPointer<vtkJPEGWriter> writer =
            vtkSmartPointer<vtkJPEGWriter>::New();
    writer->SetFileName("VTK-logo.jpg");
    writer->SetInputConnection(pngread->GetOutputPort());
    writer->Write();

    renderWindowInter->Start();
}

int writeImage(){

}

int main(){
    readWriteImage();
}