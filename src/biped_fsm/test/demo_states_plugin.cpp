#include "biped_fsm/demo_states.hpp"
#include "pluginlib/class_list_macros.hpp"

// PLUGINLIB_EXPORT_CLASS(Class_Name, Base_Class)
PLUGINLIB_EXPORT_CLASS(biped_fsm::DemoInitState, yasmin::State)
PLUGINLIB_EXPORT_CLASS(biped_fsm::DemoPassiveState, yasmin::State)
PLUGINLIB_EXPORT_CLASS(biped_fsm::DemoStandbyState, yasmin::State)