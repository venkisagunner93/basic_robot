#include "planner/planner.h"

Planner::Planner(ros::NodeHandle& nh) : nh_(nh), planner_class_loader_("planner", "planner_interface::PlannerBase")
{
    loadParameters();
    initializePlannerPlugin();
}

Planner::~Planner()
{
    planner_plugin_.reset();
    planner_class_loader_.unloadLibraryForClass(planner_plugin_type_);
}

void Planner::loadParameters()
{
    if(nh_.hasParam("planner_plugin/type"))
    {
        nh_.getParam("planner_plugin/type", planner_plugin_type_);
    }
    else
    {
        ROS_WARN("No plugin provided. Loading default plugin: planner_plugins/ManualPlanner plugin");
        planner_plugin_type_ = "planner_plugins::ManualPlanner";
    }
    if(nh_.hasParam("limits/max_velocity"))
    {
        nh_.getParam("limits/max_velocity", limits_.max_velocity);
    }
    else
    {
        limits_.max_velocity = 0.5;
        ROS_WARN("No max_velocity provided. Loading default value: 0.5 m/s");
    }
    if(nh_.hasParam("limits/max_steering_angle"))
    {
        nh_.getParam("limits/max_steering_angle", limits_.max_steering_angle);
    }
    else
    {
        limits_.max_steering_angle = 25.0;
        ROS_WARN("No max_steering_angle provided. Loading default value: 25.0 degrees");
    }
}

void Planner::initializePlannerPlugin()
{
    planner_plugin_.reset();
    if(planner_class_loader_.isClassLoaded(planner_plugin_type_))
    {
        planner_class_loader_.unloadLibraryForClass(planner_plugin_type_);
    }

    planner_plugin_ = planner_class_loader_.createInstance(planner_plugin_type_);

    if(planner_plugin_->initialize(nh_))
    {
        ROS_INFO_STREAM("Created " << planner_plugin_type_ << " plugin successfully");
        planner_plugin_->setLimits(limits_);
    }
}

bool Planner::modeServiceCallback(basic_robot::SetModeRequest& request, basic_robot::SetModeResponse& response)
{
    if(request.mode.mode == basic_robot::Mode::MANUAL)
    {
        planner_plugin_type_ = "planner_plugins::ManualPlanner";
        initializePlannerPlugin();
    }
    else if(request.mode.mode == basic_robot::Mode::AUTONOMOUS)
    {
        planner_plugin_type_ = "planner_plugins::AutonomousPlanner";
        initializePlannerPlugin();
    }
    else
    {
        ROS_ERROR_STREAM("Unidentified plugin. Try again please");
    }
}

void Planner::publishSetpoints()
{
    planner_plugin_->sendSetpoints();
}