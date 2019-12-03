/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-9-5           qishimeng        Inital
===============================================================**/

#include <osg/Quat>
#include <osg/PositionAttitudeTransform>
#include <osg/io_utils>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <iostream>

class RotateCallBack: public osg::NodeCallback{
    public:
    RotateCallBack():_rotateZ(0.0) {}

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv){
        osg::PositionAttitudeTransform* pat =
        dynamic_cast<osg::PositionAttitudeTransform*>(node);
        if(pat){
            osg::Vec3 vec(0, 0, 1);
            osg::Quat quat = osg::Quat(osg::DegreesToRadians(_rotateZ), osg::Z_AXIS);
            pat->setAttitude(quat);

            _rotateZ += 0.10;
            }

        traverse(node, nv);
        }

    private:
    double _rotateZ;
};


class InfoCallBack: public osg::NodeCallback{
    public:
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv){
        osg::PositionAttitudeTransform* pat =
        dynamic_cast<osg::PositionAttitudeTransform*>(node);

        if(pat){
            double angle = 0.0;
            osg::Vec3 axis;
            pat->getAttitude().getRotate(angle, axis);

            std::cout << "Node is rotate around the axis(" << axis << "), "
            <<osg::RadiansToDegrees(angle) << "degrees" << std::endl;
            }

            traverse(node, nv);
        }
};


int main(int argc, char** argv){
    osg::ArgumentParser argument(&argc, argv);
    osg::Node* model = osgDB::readNodeFiles(argument);
    if(!model)
    model = osgDB::readNodeFile("cow.osg") ;

    osg::ref_ptr<osg::PositionAttitudeTransform> pat =
    new osg::PositionAttitudeTransform();
    pat->addChild(model);

    pat->setUpdateCallback(new RotateCallBack() );
    pat->addUpdateCallback(new InfoCallBack() );

    osgViewer::Viewer viewer;
    viewer.setSceneData(pat.get() );
    return viewer.run();
}