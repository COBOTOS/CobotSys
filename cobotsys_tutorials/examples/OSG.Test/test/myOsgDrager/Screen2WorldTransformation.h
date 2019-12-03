/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-9-3           qishimeng        Inital
===============================================================**/

#ifndef COBOTOS_SCREEN2WORLDTRANSFORMATION_H
#define COBOTOS_SCREEN2WORLDTRANSFORMATION_H
#include <osgUtil/Optimizer>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>

#include <osgGA/TerrainManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>

#include <osg/Material>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osg/Camera>
#include <osg/Vec3>
#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osgSim/OverlayNode>
#include <osgText/Text>
class Screen2WorldTransformation:public osgGA::GUIEventHandler {
public:
    Screen2WorldTransformation();

    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

    osg::Vec3 getPoseOfWorldCoordinate();

private:
    osg::Vec3 screenToWorld(osgViewer::View * view,const osgGA::GUIEventAdapter& eventAdapter);
    osg::Vec3 _poseOfWorldCoordinate;
};
#endif //COBOTOS_SCREEN2WORLDTRANSFORMATION_H
