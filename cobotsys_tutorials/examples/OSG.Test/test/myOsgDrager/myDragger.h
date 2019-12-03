/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-9-4           qishimeng        Inital
===============================================================**/

#ifndef COBOTOS_MYDRAGERCOBOTSTUDIO_H
#define COBOTOS_MYDRAGERCOBOTSTUDIO_H

#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/RotateCylinderDragger>
#include <osgManipulator/RotateSphereDragger>
#include <osgManipulator/Translate2DDragger>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/LineWidth>

namespace osgManipulator {

/**
 * Dragger for performing rotation in all axes.
 */
    class  myDragger : public CompositeDragger
{
    public:

    myDragger(bool useAutoTransform=false);

    META_OSGMANIPULATOR_Object(osgManipulator,myDragger)

    /** Setup default geometry for dragger. */
    void setupDefaultGeometry();

    /** Sets the width of the axis lines in pixels. */
    void setAxisLineWidth(float linePixelWidth);

    /** Retrieves the width of the axis lines in pixels. */
    float getAxisLineWidth() const { return _axisLineWidth; }

    /** Sets the height of the cylinders representing the axis lines for picking. */
    void setPickCylinderHeight(float pickCylinderHeight);

    /** Retrieves the height of the cylinders representing the axis lines for picking. */
    float getPickCylinderHeight() const { return _pickCylinderHeight; }

    virtual bool handle(const PointerInfo& pi, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

    protected:

    virtual ~myDragger();

    osg::ref_ptr<RotateCylinderDragger> _xDragger;
    osg::ref_ptr<RotateCylinderDragger> _yDragger;
    osg::ref_ptr<RotateCylinderDragger> _zDragger;

    osg::ref_ptr<Translate1DDragger> _xAxisDragger;
    osg::ref_ptr<Translate1DDragger> _yAxisDragger;
    osg::ref_ptr<Translate1DDragger> _zAxisDragger;

    osg::ref_ptr<Translate2DDragger> _xyAxisDragger;

    float _axisLineWidth;
    float _pickCylinderHeight;
    float _coneHeight;
    float _pickCylinderRadius;

    osg::ref_ptr<osg::Geode> _geode;
    osg::ref_ptr<osg::Geode> _lineGeode;
    osg::ref_ptr<osg::Cylinder> _cylinder;
    osg::ref_ptr<osg::LineWidth> _lineWidth;
    osg::ref_ptr<osg::Cone> _cone;


};


};
#endif //COBOTOS_MYDRAGERCOBOTSTUDIO_H
