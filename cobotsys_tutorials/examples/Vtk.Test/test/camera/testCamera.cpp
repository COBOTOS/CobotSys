//
// Created by cobot on 19-9-8.
//

#include <vtkAutoInit.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkLight.h>
#include <vtkCamera.h>
#include <vtkProperty.h>

//VTK_MODULE_INIT(vtkRenderingOpenGL);

int main(){
//    vtkSmartPointer<vtkCamera>myCamera = vtkSmartPointer<vtkCamera>::New();
//    myCamera->SetClippingRange(0.0475,2.3786); //这些值随便设置的，东灵提供的
//    myCamera->SetFocalPoint(0.0573,-0.2134, -0.0523);
//    myCamera->SetPosition(0.3245,-0.1139, -0.2932);//是否要保证相机位置-焦点向量 与 相机位置-向上方向 正交？？
//    myCamera->ComputeViewPlaneNormal();//重新计算视平面法向量  平行于 相机-焦点
//    myCamera->SetViewUp(-0.2234,0.9983, 0.0345);
//    renderer->SetActiveCamera(myCamera); //激活新生成的相机
}