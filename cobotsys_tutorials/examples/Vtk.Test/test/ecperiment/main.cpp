#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkOBBTree.h>
#include <vtkProperty.h>
#include <vtkNamedColors.h>
//int main(int argc, char *argv[])
//{
//    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
//    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
//    if (argc > 1)
//    {
//        vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
//        reader->SetFileName(argv[1]);
//        reader->Update();
//        polydata = reader->GetOutput();
//    } else {
//        vtkSmartPointer<vtkSphereSource> modelSource = vtkSmartPointer<vtkSphereSource>::New();
//        modelSource->Update();
//        polydata = modelSource->GetOutput();
//    }
//    int maxLevel = 10;
//
//    vtkSmartPointer<vtkOBBTree> obbTree = vtkSmartPointer<vtkOBBTree>::New();
//    obbTree->SetDataSet(polydata);
//    obbTree->SetMaxLevel(maxLevel);
//    obbTree->BuildLocator();
//
//    vtkSmartPointer<vtkPolyData> obbPolydata = vtkSmartPointer<vtkPolyData>::New();
//    obbTree->GenerateRepresentation(0, obbPolydata);
//
//    vtkSmartPointer<vtkPolyDataMapper> obbTreeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//    obbTreeMapper->SetInputData(obbPolydata);
//
//    vtkSmartPointer<vtkActor> obbTreeActor = vtkSmartPointer<vtkActor>::New();
//    obbTreeActor->SetMapper(obbTreeMapper);
////    obbTreeActor->GetProperty()->SetInterpolationToFlat();
//    obbTreeActor->GetProperty()->SetOpacity(0.8);
//    obbTreeActor->GetProperty()->SetColor( colors->GetColor4d("SpringGreen").GetData());
//
//
//    vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//    modelMapper->SetInputData(polydata);
//
//    vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
//    modelActor->SetMapper(modelMapper);
//
//    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
//    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
//    renderWindow->SetSize(400, 400);
//    renderWindow->AddRenderer(renderer);
//    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
//    renderWindowInteractor->SetRenderWindow(renderWindow);
//    renderer->AddActor(modelActor);
//    renderer->AddActor(obbTreeActor);
//    renderWindow->Render();
//    renderWindowInteractor->Start();
//    return EXIT_SUCCESS;
//}
//
//

#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL2);
//VTK_MODULE_INIT(vtkInteractionStyle);
//VTK_MODULE_INIT(vtkRenderingFreeType);
//VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
#include<vtkRenderer.h>
#include<vtkRenderWindow.h>
#include<vtkRenderWindowInteractor.h>
#include<vtkInteractorStyleTrackballCamera.h>
#include<vtkVolume16Reader.h>
#include<vtkPolyDataMapper.h>
#include<vtkActor.h>
#include<vtkOutlineFilter.h>
#include<vtkCamera.h>
#include<vtkProperty.h>
#include<vtkPolyDataNormals.h>
#include<vtkContourFilter.h>
#include<vtkSmartPointer.h>
#include<vtkStripper.h>
#include <vtkSTLReader.h>
int main(int argc, char *argv[])
{
    vtkSmartPointer<vtkRenderer>render = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow>renWin = vtkSmartPointer<vtkRenderWindow>::New();
    renWin->AddRenderer(render);

    vtkSmartPointer<vtkRenderWindowInteractor>iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    iren->SetRenderWindow(renWin);

    vtkSmartPointer<vtkSTLReader> stlReader = vtkSmartPointer<vtkSTLReader>::New();
    stlReader->SetFileName("/home/cobot/ggg/bunny_0_011/mesh.ply");
    stlReader->Update();

    vtkSmartPointer<vtkContourFilter>skinExtractor = vtkSmartPointer<vtkContourFilter>::New();
//    skinExtractor->SetInputConnection(v16->GetOutputPort());
    skinExtractor->SetInputConnection(stlReader->GetOutputPort());
    skinExtractor->SetValue(0, 500);

    vtkSmartPointer<vtkPolyDataNormals>skinNorms = vtkSmartPointer<vtkPolyDataNormals>::New();
    skinNorms->SetInputConnection(skinExtractor->GetOutputPort());
    skinNorms->SetFeatureAngle(60.0);

    vtkSmartPointer<vtkStripper>skinStrip = vtkSmartPointer<vtkStripper>::New();
    skinStrip->SetInputConnection(skinNorms->GetOutputPort());

    vtkSmartPointer<vtkPolyDataMapper>skinMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    skinMapper->SetInputConnection(skinStrip->GetOutputPort());
    skinMapper->ScalarVisibilityOff();

    vtkSmartPointer<vtkActor>skinActor = vtkSmartPointer<vtkActor>::New();
    skinActor->SetMapper(skinMapper);
    skinActor->GetProperty()->SetDiffuseColor(1, 0.49, 0.25);
    skinActor->GetProperty()->SetSpecular(0.3);
    skinActor->GetProperty()->SetSpecularPower(20);
    skinActor->GetProperty()->SetOpacity(0.5);

    vtkSmartPointer<vtkContourFilter>boneExtractor = vtkSmartPointer<vtkContourFilter>::New();
    boneExtractor->SetInputConnection(stlReader->GetOutputPort());
    boneExtractor->SetValue(0, 1150);

    vtkSmartPointer<vtkPolyDataNormals>boneNormals = vtkSmartPointer<vtkPolyDataNormals>::New();
    boneNormals->SetInputConnection(boneExtractor->GetOutputPort());
    boneNormals->SetFeatureAngle(60.0);

    vtkSmartPointer<vtkStripper>boneStrip = vtkSmartPointer<vtkStripper>::New();
    boneStrip->SetInputConnection(boneNormals->GetOutputPort());

    vtkSmartPointer<vtkPolyDataMapper>boneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    boneMapper->SetInputConnection(boneStrip->GetOutputPort());
    boneMapper->ScalarVisibilityOff();

    vtkSmartPointer<vtkActor>bone = vtkSmartPointer<vtkActor>::New();
    bone->SetMapper(boneMapper);
    bone->GetProperty()->SetDiffuseColor(1, 1, 0.9412);

    vtkSmartPointer<vtkOutlineFilter>outLineData = vtkSmartPointer<vtkOutlineFilter>::New();
    outLineData->SetInputConnection(stlReader->GetOutputPort());

    vtkSmartPointer<vtkPolyDataMapper> mapOutline = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapOutline->SetInputConnection(outLineData->GetOutputPort());

    vtkSmartPointer<vtkActor>outline = vtkSmartPointer<vtkActor>::New();
    outline->SetMapper(mapOutline);
    outline->GetProperty()->SetColor(0, 0, 0);

    vtkSmartPointer<vtkCamera>camera = vtkSmartPointer<vtkCamera>::New();
    camera->SetViewUp(0, 0, -1);
    camera->SetPosition(0, 1, 0);
    camera->SetFocalPoint(0, 0, 0);
    camera->ComputeViewPlaneNormal();
    camera->Azimuth(30.0);
    camera->Elevation(30.0);

    render->AddActor(outline);
    render->AddActor(skinActor);
    render->AddActor(bone);
    render->SetActiveCamera(camera);
    render->ResetCamera();
    render->SetBackground(0.2, 0.3, 0.4);
    camera->Dolly(1.5);
    render->ResetCameraClippingRange();

    vtkSmartPointer<vtkInteractorStyleTrackballCamera>style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    iren->SetInteractorStyle(style);

    iren->Initialize();
    iren->Start();

    return 0;
}
