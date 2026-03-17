#include "biped_fsm/fsm_classes.hpp"
#include "pluginlib/class_list_macros.hpp"

// PLUGINLIB_EXPORT_CLASS(Class_Name, Base_Class)
PLUGINLIB_EXPORT_CLASS(biped_fsm::InitState, yasmin::State)
PLUGINLIB_EXPORT_CLASS(biped_fsm::PassiveState, yasmin::State)
PLUGINLIB_EXPORT_CLASS(biped_fsm::StandbyState, yasmin::State)
PLUGINLIB_EXPORT_CLASS(biped_fsm::ActiveState, yasmin::State)
PLUGINLIB_EXPORT_CLASS(biped_fsm::FallenState, yasmin::State)
PLUGINLIB_EXPORT_CLASS(biped_fsm::ErrorState, yasmin::State)
PLUGINLIB_EXPORT_CLASS(biped_fsm::StopState, yasmin::State)