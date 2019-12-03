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
#include <osgUtil/SceneView>
#include <iostream>
Screen2WorldTransformation::Screen2WorldTransformation():_poseOfWorldCoordinate(0,0,0) {

}
bool Screen2WorldTransformation::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    switch(ea.getEventType())
    {
        default:
            osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
            if (view)
            {
                screenToWorld(view,ea);
            }
            return false;
    }
}
osg::Vec3 Screen2WorldTransformation::screenToWorld(osgViewer::View * view,const osgGA::GUIEventAdapter& eventAdapter)
{
#if 1
    osgUtil::LineSegmentIntersector::Intersections intersections;


        if (view->computeIntersections(eventAdapter,intersections))
        {
            for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
                hitr != intersections.end();
            ++hitr)
            {

                if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty()))
                {

                }
                else if (hitr->drawable.valid())
                {

                }

                _poseOfWorldCoordinate = hitr->getWorldIntersectPoint();
                const osgUtil::LineSegmentIntersector::Intersection::IndexList& vil = hitr->indexList;
                for(unsigned int i=0;i<vil.size();++i)
                {

                }

            }
        }
    return  _poseOfWorldCoordinate;
#endif
}
osg::Vec3 Screen2WorldTransformation::getPoseOfWorldCoordinate() {
    return _poseOfWorldCoordinate;
}
