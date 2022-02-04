#include "SamplePlugin.hpp"

#include <RobWorkStudio.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <boost/bind.hpp>

using rw::kinematics::State;

using rws::RobWorkStudioPlugin;

SamplePlugin::SamplePlugin() : RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    // now connect stuff from the ui component
    connect(_btn0, SIGNAL(pressed()), this, SLOT(btnPressed()));
    connect(_btn1, SIGNAL(pressed()), this, SLOT(btnPressed()));
    connect(_spinBox, SIGNAL(valueChanged(int)), this, SLOT(btnPressed()));
}

SamplePlugin::~SamplePlugin()
{
}

void SamplePlugin::initialize()
{
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
}

void SamplePlugin::open(rw::models::WorkCell *workcell)
{
}

void SamplePlugin::close()
{
}

void SamplePlugin::btnPressed()
{
    QObject *obj = sender();
    if (obj == _btn0)
    {
        log().info() << "Button 0\n";
        this->addTreeDevice();
    }
    else if (obj == _btn1)
    {
        log().info() << "Button 1\n";
    }
    else if (obj == _spinBox)
    {
        log().info() << "spin value:" << _spinBox->value() << "\n";
    }
}

//
void SamplePlugin::stateChangedListener(const State &state)
{
    log().info() << "State changed\n";
}

void SamplePlugin::addTreeDevice()
{

    rw::models::WorkCell::Ptr sptr_workcell = getRobWorkStudio()->getWorkCell();

    // Adding the device base
    this->addFixedFrame("MachineWCBase", sptr_workcell);

    const rw::math::Transform3D<double> offset = rw::math::Transform3D<double>::Transform3D(rw::math::Vector3D<double>::Vector3D(0.0, 0.0, 0.5));
    this->addPrisJoint("Endeffector", offset, sptr_workcell, sptr_workcell->findFrame("MachineWCBase"));
    
    getRobWorkStudio()->getWorkCellScene()->setFrameAxisVisible(true, sptr_workcell->findFrame("MachineWCBase"));
    getRobWorkStudio()->getWorkCellScene()->setFrameAxisVisible(true, sptr_workcell->findFrame("Endeffector"));
    rw::models::TreeDevice::Ptr dev = rw::common::ownedPtr(new 
    rw::models::TreeDevice(
        sptr_workcell->findFrame("MachineWCBase"), 
        std::vector<rw::kinematics::Frame*>(1, sptr_workcell->findFrame("Endeffector")), 
        "Device", 
        sptr_workcell->getDefaultState()));
    dev->setBounds(std::make_pair(rw::math::Q(1,0.0), rw::math::Q(1, 0.5)));
    sptr_workcell->addDevice(dev);

    // Adding Drawables to the workcell
    // Testing whether the render type auto detection feature works well
    rw::graphics::WorkCellScene::Ptr sptr_wcscene =  getRobWorkStudio()->getWorkCellScene();
    std::string geo_file = "C:\\Dev\\Projects\\RWStudioPlugin\\data\\Geometry\\base.stl";
    sptr_wcscene->addDrawable(geo_file, sptr_workcell->findFrame("MachineWCBase"), 1); //maks=1 := phisycal element
    geo_file = "C:\\Dev\\Projects\\RWStudioPlugin\\data\\Geometry\\joint1.stl";
    sptr_wcscene->addDrawable(geo_file, sptr_workcell->findFrame("Endeffector"), 1);
    //----Debug check if added successfully
    if (sptr_workcell->getDevices().front())
    {
        log().info() << "Print device name:\n" << sptr_workcell->getDevices().front()->getName() << "\n";
        log().info() << "Print DOF:\n" << sptr_workcell->getDevices().front()->getDOF() << "\n";
    }
}

void SamplePlugin::addFixedFrame(const std::string &frame, rw::models::WorkCell::Ptr workcell, rw::kinematics::Frame *parent)
{

    rw::kinematics::State defState = workcell->getDefaultState();

    rw::kinematics::Frame::Ptr fixed_frame = rw::common::ownedPtr(new rw::kinematics::FixedFrame(frame, rw::math::Transform3D<>::identity()));
    if (parent)
    {
        workcell->addFrame(fixed_frame, rw::common::ownedPtr(parent));
    }
    else
    {
        workcell->addFrame(fixed_frame);
    }

    defState = workcell->getStateStructure()->upgradeState(defState);
    getRobWorkStudio()->setState(defState);
}

void SamplePlugin::addPrisJoint(const std::string& frame, const rw::math::Transform3D<double>& transform, rw::models::WorkCell::Ptr workcell, rw::kinematics::Frame* parent)
{
    rw::kinematics::State defState = workcell->getDefaultState();
    rw::models::PrismaticJoint::Ptr pris_jnt = rw::common::ownedPtr(new
    rw::models::PrismaticJoint(frame, transform));
    workcell->addFrame(pris_jnt, parent);
    defState = workcell->getStateStructure()->upgradeState(defState);
    getRobWorkStudio()->setState(defState);
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(SamplePlugin);
#endif
