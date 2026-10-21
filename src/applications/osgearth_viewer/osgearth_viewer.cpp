/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <iostream>

#include <osgEarth/Metrics>

#include <osgEarth/GeoData>
#include <osgEarth/Controls>
#include <osgEarth/Terrain>
#include <fstream>
#include <iomanip>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;

// On-screen distances, to evaluate LOD switch distances (key 'd' toggles): camera -> focal point (manipulator range),
// camera height above the terrain, and camera -> terrain point under the mouse (point at a model's base).
struct DistanceReadout : public osgGA::GUIEventHandler
{
    osg::observer_ptr<MapNode> _mapNode;
    osg::ref_ptr<Controls::LabelControl> _label;
    float _mx = -1.0f, _my = -1.0f;
    DistanceReadout(MapNode* mapNode, osgViewer::View* view) : _mapNode(mapNode)
    {
        _label = new Controls::LabelControl("", 16.0f);
        _label->setHorizAlign(Controls::Control::ALIGN_LEFT);
        _label->setVertAlign(Controls::Control::ALIGN_TOP);
        _label->setBackColor(osg::Vec4f(0, 0, 0, 0.6f));
        _label->setPadding(6.0f);
        Controls::ControlCanvas::getOrCreate(view)->addControl(_label.get());
    }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override
    {
        if (ea.getEventType() == ea.MOVE || ea.getEventType() == ea.DRAG)
        {
            _mx = ea.getX();
            _my = ea.getY();
        }
        if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == 'd')
        {
            _label->setVisible(!_label->visible());
            return true;
        }
        if (ea.getEventType() != ea.FRAME || !_label->visible())
            return false;
        osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
        osg::ref_ptr<MapNode> mapNode;
        if (!view || !_mapNode.lock(mapNode))
            return false;

        osg::Vec3d eye, center, up;
        view->getCamera()->getViewMatrixAsLookAt(eye, center, up);
        std::ostringstream s;
        s << std::fixed << std::setprecision(0);

        EarthManipulator* em = dynamic_cast<EarthManipulator*>(view->getCameraManipulator());
        if (em)
            s << "range (camera -> focal point): " << em->getViewpoint().range()->as(Units::METERS) << " m\n";

        GeoPoint eyeGeo;
        eyeGeo.fromWorld(mapNode->getMapSRS()->getGeographicSRS(), eye);
        double hMsl = 0.0, hEll = 0.0;
        if (mapNode->getTerrain()->getHeight(eyeGeo.getSRS(), eyeGeo.x(), eyeGeo.y(), &hMsl, &hEll))
            s << "camera height above ground: " << (eyeGeo.z() - hEll) << " m\n";

        osg::Vec3d world;
        if (_mx >= 0.0f && mapNode->getTerrain()->getWorldCoordsUnderMouse(view, _mx, _my, world))
            s << "camera -> point under mouse: " << (world - eye).length() << " m";
        else
            s << "camera -> point under mouse: -";
        _label->setText(s.str());
        return false;
    }
};

// Key 'k': writes the current camera as a Google Earth KML <Camera> (eye position, heading, tilt, horizontal field of
// view) to camera_<n>.kml in the working directory, and logs it with the manipulator's focal viewpoint. The KML Camera
// reproduces the exact eye, independently of how each application interprets a focal point and range.
struct CameraToKml : public osgGA::GUIEventHandler
{
    osg::observer_ptr<MapNode> _mapNode;
    int _count = 0;
    CameraToKml(MapNode* mapNode) : _mapNode(mapNode) { }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override
    {
        if (ea.getEventType() != ea.KEYDOWN || ea.getKey() != 'k')
            return false;
        osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
        osg::ref_ptr<MapNode> mapNode;
        if (!view || !_mapNode.lock(mapNode))
            return false;
        osg::Camera* cam = view->getCamera();

        osg::Vec3d eye, center, up;
        cam->getViewMatrixAsLookAt(eye, center, up);
        GeoPoint eyeGeo;
        eyeGeo.fromWorld(mapNode->getMapSRS()->getGeographicSRS(), eye);

        // look direction in the local east-north-up frame at the eye
        osg::Matrixd l2w;
        eyeGeo.createLocalToWorld(l2w);
        osg::Matrixd w2l = osg::Matrixd::inverse(l2w);
        osg::Vec3d dir = (center - eye);
        dir.normalize();
        osg::Vec3d d = (eye + dir) * w2l - eye * w2l;            // world -> local east-north-up (OSG row vectors)
        d.normalize();
        double heading = osg::RadiansToDegrees(atan2(d.x(), d.y()));
        if (heading < 0.0) heading += 360.0;
        double tilt = osg::RadiansToDegrees(acos(osg::clampBetween(-d.z(), -1.0, 1.0)));

        double fovy, aspect, zn, zf;
        cam->getProjectionMatrixAsPerspective(fovy, aspect, zn, zf);
        double hfov = osg::RadiansToDegrees(2.0 * atan(tan(osg::DegreesToRadians(fovy) * 0.5) * aspect));

        std::ostringstream vpText;
        EarthManipulator* em = dynamic_cast<EarthManipulator*>(view->getCameraManipulator());
        if (em)
        {
            Viewpoint vp = em->getViewpoint();
            vpText << "focal point lat " << vp.focalPoint()->y() << " lon " << vp.focalPoint()->x()
                   << " alt " << vp.focalPoint()->z() << ", heading " << vp.heading()->as(Units::DEGREES)
                   << ", pitch " << vp.pitch()->as(Units::DEGREES) << ", range " << vp.range()->as(Units::METERS) << " m";
        }

        std::ostringstream name;
        name << "camera_" << ++_count << ".kml";
        std::ofstream out(name.str().c_str());
        out << std::setprecision(10)
            << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            << "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\">\n"
            << "<Placemark>\n  <name>osgEarth camera " << _count << "</name>\n"
            << "  <description>Eye altitude is above the WGS84 ellipsoid. osgEarth viewpoint: " << vpText.str()
            << ". Vertical FOV " << fovy << " deg, aspect " << aspect << ".</description>\n"
            << "  <Camera>\n"
            << "    <longitude>" << eyeGeo.x() << "</longitude><latitude>" << eyeGeo.y() << "</latitude>"
            << "<altitude>" << eyeGeo.z() << "</altitude>\n"
            << "    <heading>" << heading << "</heading><tilt>" << tilt << "</tilt><roll>0</roll>\n"
            << "    <altitudeMode>absolute</altitudeMode>\n"
            << "    <gx:horizFov>" << hfov << "</gx:horizFov>\n"
            << "  </Camera>\n</Placemark>\n</kml>\n";
        out.close();

        OE_WARN << LC << name.str() << ": eye lat " << std::setprecision(9) << eyeGeo.y() << " lon " << eyeGeo.x()
                << " alt " << std::setprecision(6) << eyeGeo.z() << " m, heading " << heading << ", tilt " << tilt
                << ", hfov " << hfov << " (vfov " << fovy << ", aspect " << aspect << "); " << vpText.str() << std::endl;
        return true;
    }
};

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}


int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( true, false );

    // thread-safe initialization of the OSG wrapper manager. Calling this here
    // prevents the "unsupported wrapper" messages from OSG
    osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.0001);

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( node )
    {
        viewer.setSceneData( node );
        MapNode* mapNode = MapNode::get(node);
        if (mapNode)
        {
            viewer.addEventHandler(new CameraToKml(mapNode));
            viewer.addEventHandler(new DistanceReadout(mapNode, &viewer));
        }
        return Metrics::run(viewer);
    }
    else
    {
        return usage(argv[0]);
    }
}