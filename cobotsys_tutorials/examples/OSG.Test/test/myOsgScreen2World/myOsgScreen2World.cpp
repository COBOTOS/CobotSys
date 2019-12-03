/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-9-3           qishimeng        Inital
===============================================================**/
#include "Screen2WorldTransformation.h"
#include <osgDB/ReadFile>
osg::ref_ptr<osg::Geode> createShape()
{
    //创建一个叶节点
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();

    //设置半径和高度
    float radius = 0.8f;
    float height  = 1.0f;

    //创建精细度对象，精细度越高，细分就越多
    osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints();
    //设置精细度为0.5f
    hints->setDetailRatio(0.5f);

    //添加一个球体，第一个参数是预定义几何体对象，第二个参数是精细度，默认为0
    geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f),radius),hints.get()));

#if 0
    //添加一个正方体
    geode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(2.0f, 0.0f, 0.0f),2*radius),hints.get()));

    //圆锥
    geode->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(4.0f, 0.0f, 0.0f),radius,height),hints.get()));
    //圆柱体
    geode->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(6.0f, 0.0f, 0.0f),radius,height),hints.get()));
    //太空舱
    geode->addDrawable(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(8.0f, 0.0f, 0.0f),radius,height),hints.get()));
#endif
    return geode.get();
}



int main(int argc,char**argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    // read the scene from the list of file specified commandline args.
    osgViewer::Viewer viewer;

    auto  model = createShape();
    if (!model)
    {
        return 1;
    }

    // tilt the scene so the default eye position is looking down on the model.
    osg::ref_ptr<osg::MatrixTransform> rootnode = new osg::MatrixTransform;
    rootnode->setMatrix(osg::Matrix::rotate(osg::inDegrees(30.0f),1.0f,0.0f,0.0f));
    rootnode->addChild(model);

    // run optimization over the scene graph
    osgUtil::Optimizer optimzer;
    optimzer.optimize(rootnode);

    // set the scene to render
    viewer.setSceneData(rootnode);

    viewer.setCameraManipulator(new osgGA::TrackballManipulator());

    osg::ref_ptr<osgText::Text> updateText = new osgText::Text;
    // if not loaded assume no arguments passed in, try use default mode instead.

    {
        //osgViewer::Viewer viewer;


        // add all the camera manipulators
        {
            osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

            keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
            keyswitchManipulator->addMatrixManipulator( '2', "Flight", new osgGA::FlightManipulator() );
            keyswitchManipulator->addMatrixManipulator( '3', "Drive", new osgGA::DriveManipulator() );

            unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
            keyswitchManipulator->addMatrixManipulator( '4', "Terrain", new osgGA::TerrainManipulator() );

            std::string pathfile;
            char keyForAnimationPath = '5';
            while (arguments.read("-p",pathfile))
            {
                osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
                if (apm || !apm->valid())
                {
                    num = keyswitchManipulator->getNumMatrixManipulators();
                    keyswitchManipulator->addMatrixManipulator( keyForAnimationPath, "Path", apm );
                    ++keyForAnimationPath;
                }
            }

            keyswitchManipulator->selectMatrixManipulator(num);

            viewer.setCameraManipulator( keyswitchManipulator.get() );
        }

        // add the handler for doing the picking
        viewer.addEventHandler(new Screen2WorldTransformation());

        // set the scene to render
        viewer.setSceneData(model.get());

        return viewer.run();
    }
    return 0;
}