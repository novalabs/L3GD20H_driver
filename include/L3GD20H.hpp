/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <ModuleConfiguration.hpp>

#include <core/hw/SPI.hpp>
#include <core/hw/EXT.hpp>
#include <core/utils/BasicSensor.hpp>
#include <core/os/Thread.hpp>

namespace sensors {
class L3GD20H
{
public:
    L3GD20H(
        core::hw::SPIDevice&  spi,
        core::hw::EXTChannel& ext
    );

    virtual
    ~L3GD20H();

public:
    bool
    probe();

    uint8_t
    readRegister(
        uint8_t reg
    );

    void
    writeRegister(
        uint8_t reg,
        uint8_t value
    );


public:
    core::hw::SPIDevice&  _spi;
    core::hw::EXTChannel& _ext;
};


class L3GD20H_Gyro:
    public core::utils::BasicSensor<ModuleConfiguration::L3GD20H_GYRO_DATATYPE>
{
public:
    L3GD20H_Gyro(
        L3GD20H& device
    );

    virtual
    ~L3GD20H_Gyro();

public:
    bool
    init();

    bool
    configure();

    bool
    start();

    bool
    stop();

    bool
    waitUntilReady();

    bool
    update();

    void
    get(
        DataType& data
    );


protected:
    core::os::Thread* _runner;
    core::os::Time    _timestamp;
    ModuleConfiguration::L3GD20H_GYRO_DATATYPE _data;

private:
    L3GD20H& _device;
};
}
