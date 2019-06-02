//Author Aytaç Kahveci

#ifndef HARDWARE_INTERFACE_CURRENT_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_CURRENT_STATE_INTERFACE_H

#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{
    /// A handle used to read the state of a current sensor
    class CurrentSensorHandle
    {
    public:
        CurrentSensorHandle() : name_(""), current_(0), direction_(0){}
        /*
            \param name The name of the sensor
            \param current A pointer to the storage of the current value
        */
        CurrentSensorHandle(const std::string& name,
                            double* current, double* direction)
        : name_(name),
          current_(current),
          direction_(direction)
        {}

        std::string getName() const {return name_;}
        double getCurrent() const {return *current_;}
        double getDirection() const {return *direction_;}
    private:
        std::string name_;
        const double* current_;
        const double* direction_;
    };
    // Hardware interface to support reading the state of a force-torque sensor
    class CurrentSensorInterface : public HardwareResourceManager<CurrentSensorHandle>{};
}

#endif
