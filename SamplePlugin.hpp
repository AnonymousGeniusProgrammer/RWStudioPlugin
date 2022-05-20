#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition

#include <rws/RobWorkStudioPlugin.hpp>

#include "ui_SamplePlugin.h"
#include <rw/models/WorkCell.hpp>

class SamplePlugin : public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
    Q_OBJECT
    Q_INTERFACES(rws::RobWorkStudioPlugin)
#if RWS_USE_QT5
    Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "Sampleplugin.json")
#endif
public:
    SamplePlugin();
    virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell *workcell);

    virtual void close();

    virtual void initialize();

private slots:
    void btnPressed();

    void stateChangedListener(const rw::kinematics::State &state);

    void addTreeDevice();

    void addFixedFrame(const std::string &frame, rw::models::WorkCell::Ptr workcell, rw::kinematics::Frame *parent = NULL);

    void addPrisJoint(const std::string &frame, const rw::math::Transform3D<double> &transform, rw::models::WorkCell::Ptr workcell, rw::kinematics::Frame *parent = NULL);

private:
    void jogDeviceManualy();
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
