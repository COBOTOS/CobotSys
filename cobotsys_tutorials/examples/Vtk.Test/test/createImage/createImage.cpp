//
// Created by cobot on 19-9-8.
//

#include <vtkAutoInit.h>
#include<vtkSmartPointer.h>
#include<vtkImageCanvasSource2D.h>
#include<vtkImageData.h>
#include<vtkImageActor.h>
#include<vtkRenderer.h>
#include<vtkRenderWindow.h>
#include<vtkRenderWindowInteractor.h>
#include<vtkInteractorStyleImage.h>

#include <vtkAutoInit.h>
#include "vtkCylinderSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkProperty.h"
#include "vtkCamera.h"
#include <vtkAxesActor.h>

#include <vtkAutoInit.h>
#include <vtkConeSource.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkActor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);

int create2dImage()
{
    vtkSmartPointer<vtkImageCanvasSource2D> canvas =
            vtkSmartPointer<vtkImageCanvasSource2D>::New();
    canvas->SetScalarTypeToUnsignedChar();
    canvas->SetNumberOfScalarComponents(1);
    canvas->SetExtent(0,100,0,100,0,0);
    canvas->SetDrawColor(0,0,0,0);
    canvas->FillBox(0,100,0,100);
    canvas->SetDrawColor(255,0,0,0);
    canvas->FillBox(20,40,20,40);
    canvas->Update();  //这一步还必不可少诶？？？

    //创建演员
    vtkSmartPointer<vtkImageActor> actor =
            vtkSmartPointer<vtkImageActor>::New();
    actor->SetInputData(canvas->GetOutput());
    //定义视窗
    double viewport[4] = {0,0,1,1};
    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    renderer->SetViewport(viewport);
    renderer->AddActor(actor);
    renderer->ResetCamera();
    renderer->SetBackground(1.0,1.0,1.0);

    //设置渲染窗口（搬上舞台）
    vtkSmartPointer<vtkRenderWindow> renderwindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    renderwindow->AddRenderer(renderer);
    renderwindow->SetSize(640,480);
    renderwindow->Render();
    renderwindow->SetWindowName("ImageCanvasSource2D");

    //设置窗口交互（演员-观众）
    vtkSmartPointer<vtkRenderWindowInteractor> rwi =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    vtkSmartPointer<vtkInteractorStyleImage> style =
            vtkSmartPointer<vtkInteractorStyleImage>::New();
    rwi->SetInteractorStyle(style);
    rwi->SetRenderWindow(renderwindow);
    rwi->Initialize();
    rwi->Start();

    return 0;
}

int create3dImage(){

int LINE_LEN = 2;

    // This creates a polygonal cylinder model with eight circumferential facets.
    vtkCylinderSource *cylinder = vtkCylinderSource::New();
    cylinder->SetResolution(8);

    // The mapper is responsible for pushing the geometry into the graphics
    // library. It may also do color mapping, if scalars or other attributes
    // are defined.
    vtkPolyDataMapper *cylinderMapper = vtkPolyDataMapper::New();
    cylinderMapper->SetInputConnection(cylinder->GetOutputPort());

    // The actor is a grouping mechanism: besides the geometry (mapper), it
    // also has a property, transformation matrix, and/or texture map.
    // Here we set its color and rotate it -22.5 degrees.
    vtkActor *cylinderActor = vtkActor::New();
    cylinderActor->SetMapper(cylinderMapper);
    cylinderActor->GetProperty()->SetColor(1.0000, 0.3882, 0.2784);
    cylinderActor->RotateX(30.0);
    cylinderActor->RotateY(-45.0);

    vtkSmartPointer<vtkAxesActor> actor2 = vtkSmartPointer<vtkAxesActor>::New();
    actor2->SetPosition(0, 0, 0);
    actor2->SetTotalLength(LINE_LEN, LINE_LEN, LINE_LEN);
    actor2->SetShaftType(0);
    actor2->SetAxisLabels(0);
    actor2->SetCylinderRadius(0.02);

    // Create the graphics structure. The renderer renders into the
    // render window. The render window interactor captures mouse events
    // and will perform appropriate camera or actor manipulation
    // depending on the nature of the events.
    vtkRenderer *ren1 = vtkRenderer::New();
    // Add the actors to the renderer, set the background and size
    ren1->AddActor(cylinderActor);
    ren1->AddActor(actor2);
    ren1->SetBackground(0.1, 0.2, 0.4);
    // We'll zoom in a little by accessing the camera and invoking a "Zoom"
    // method on it.
    ren1->ResetCamera();
    ren1->GetActiveCamera()->Zoom(1.5);

    vtkRenderWindow *renWin = vtkRenderWindow::New();
    renWin->AddRenderer(ren1);
    renWin->SetSize(200, 200);
    renWin->Render();

    vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
    iren->SetRenderWindow(renWin);
    // This starts the event loop and as a side effect causes an initial render.
    iren->Start();

    // Exiting from here, we have to delete all the instances that
    // have been created.
    cylinder->Delete();
    cylinderMapper->Delete();
    cylinderActor->Delete();
    ren1->Delete();
    renWin->Delete();
    iren->Delete();

    return 0;
}


int spliteWindow()
{
    vtkSmartPointer<vtkConeSource> cone = vtkSmartPointer<vtkConeSource>::New();
    vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
    vtkSmartPointer<vtkCylinderSource> cylinder = vtkSmartPointer<vtkCylinderSource>::New();
    vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();

    vtkSmartPointer<vtkPolyDataMapper> coneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    coneMapper->SetInputConnection(cone->GetOutputPort());
    vtkSmartPointer<vtkPolyDataMapper> cubeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    cubeMapper->SetInputConnection(cube->GetOutputPort());
    vtkSmartPointer<vtkPolyDataMapper> cylinderMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    cylinderMapper->SetInputConnection(cylinder->GetOutputPort());
    vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereMapper->SetInputConnection(sphere->GetOutputPort());

    vtkSmartPointer<vtkActor> coneActor = vtkSmartPointer<vtkActor>::New();
    coneActor->SetMapper(coneMapper);
    vtkSmartPointer<vtkActor> cubeActor = vtkSmartPointer<vtkActor>::New();
    cubeActor->SetMapper(cubeMapper);
    vtkSmartPointer<vtkActor> cylinderActor = vtkSmartPointer<vtkActor>::New();
    cylinderActor->SetMapper(cylinderMapper);
    vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
    sphereActor->SetMapper(sphereMapper);

    vtkSmartPointer<vtkRenderer> renderer1 = vtkSmartPointer<vtkRenderer>::New();
    renderer1->AddActor(coneActor);
    renderer1->SetBackground(1.0,0.0,0.0);
    renderer1->SetViewport(0.0,0.0,0.5,0.5);
    vtkSmartPointer<vtkRenderer> renderer2 = vtkSmartPointer<vtkRenderer>::New();
    renderer2->AddActor(cubeActor);
    renderer2->SetBackground(0.0,1.0,0.0);
    renderer2->SetViewport(0.5,0.0,1.0,0.5);
    vtkSmartPointer<vtkRenderer> renderer3 = vtkSmartPointer<vtkRenderer>::New();
    renderer3->AddActor(cylinderActor);
    renderer3->SetBackground(0.0,0.0,1.0);
    renderer3->SetViewport(0.0,0.5,0.5,1.0);
    vtkSmartPointer<vtkRenderer> renderer4 = vtkSmartPointer<vtkRenderer>::New();
    renderer4->AddActor(sphereActor);
    renderer4->SetBackground(1.0,1.0,0.0);
    renderer4->SetViewport(0.5,0.5,1.0,1.0);

    vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
    renWin->AddRenderer(renderer1);
    renWin->AddRenderer(renderer2);
    renWin->AddRenderer(renderer3);
    renWin->AddRenderer(renderer4);
    renWin->SetSize( 640, 480 );
    renWin->Render();
    renWin->SetWindowName("ViewPort");

    vtkSmartPointer<vtkRenderWindowInteractor> interactor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renWin);

    renWin->Render();
    interactor->Initialize();
    interactor->Start();

    return 0;
}

int main(){
//    create2dImage();
//    create3dImage();
    spliteWindow();
}
