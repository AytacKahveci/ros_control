//Author Aytaç Kahveci
#ifndef HARDWARE_INTERFACE_CURRENT_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_CURRENT_COMMAND_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/current_sensor_interface.h>

namespace hardware_interface
{
    class CurrentCommandHandle : public CurrentSensorHandle
    {
    public:
        CurrentCommandHandle() : CurrentSensorHandle(), cmd_current_(0), cmd_direction_(0){}

        CurrentCommandHandle(const CurrentSensorHandle& cs, double* cmd_current, double* cmd_direction)
            : CurrentSensorHandle(cs), cmd_current_(cmd_current), cmd_direction_(cmd_direction)
        {
            if (!cmd_current_)
            {
                throw HardwareInterfaceException("Cannot create handle '" + cs.getName() + "' . Command current pointer is null.");
            }
            if(!cmd_direction_)
            {
                throw HardwareInterfaceException("Cannot create handle '" + cs.getName() + "' . Command direction pointer is null.");
            }
        }

        void setCommand(double cmd_current, double cmd_direction)
        {
            setCommandCurrent(cmd_current);
            setCommandDirection(cmd_direction);
        }

        void setCommandCurrent(double cmd_current){assert(cmd_current_); *cmd_current_ = cmd_current;}
        void setCommandDirection(double cmd_direction){assert(cmd_direction_); *cmd_direction_ = cmd_direction;}

        double getCommandCurrent() const {assert(cmd_current_); return *cmd_current_;}
        double getCommandDirection() const {assert(cmd_direction_); return *cmd_direction_;}

    private:
        double* cmd_current_;
        double* cmd_direction_;
    };

    class CurrentCommandInterface : public HardwareResourceManager<CurrentCommandHandle, ClaimResources>{};
}

#endif
