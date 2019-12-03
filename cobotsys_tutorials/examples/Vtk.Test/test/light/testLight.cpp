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

int main()
{
    vtkSmartPointer<vtkCylinderSource> cylinder =
            vtkSmartPointer<vtkCylinderSource>::New();
    cylinder->SetHeight( 3.0 );
    cylinder->SetRadius( 1.0 );
    cylinder->SetResolution( 10 );

    vtkSmartPointer<vtkPolyDataMapper> cylinderMapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    cylinderMapper->SetInputConnection( cylinder->GetOutputPort() );

    vtkSmartPointer<vtkActor> cylinderActor =
            vtkSmartPointer<vtkActor>::New();
    cylinderActor->SetMapper( cylinderMapper );

    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor( cylinderActor );
    renderer->SetBackground( 1.0, 0, 0 );

    vtkSmartPointer<vtkRenderWindow> renWin =
            vtkSmartPointer<vtkRenderWindow>::New();
    renWin->AddRenderer( renderer );
    renWin->SetSize( 640, 480 );
    renWin->Render();
    renWin->SetWindowName("RenderCylinder-Lights");

    vtkSmartPointer<vtkRenderWindowInteractor> iren =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    iren->SetRenderWindow(renWin);

    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
            vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    iren->SetInteractorStyle(style);

    vtkSmartPointer<vtkLight> myLight =
            vtkSmartPointer<vtkLight>::New();
    myLight->SetColor(0,1,0);
    myLight->SetPosition(0,0,6);
    myLight->SetFocalPoint(
            renderer->GetActiveCamera()->GetFocalPoint());
    renderer->AddLight(myLight);

    vtkSmartPointer<vtkLight> myLight2 =
            vtkSmartPointer<vtkLight>::New();
    myLight2->SetColor(0,0,1);
    myLight2->SetPosition(0,0,-6);
    myLight2->SetFocalPoint(
            renderer->GetActiveCamera()->GetFocalPoint());
    renderer->AddLight(myLight2);

    iren->Initialize();
    iren->Start();

    return EXIT_SUCCESS;

}
