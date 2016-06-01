/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <Configuration.hpp>

#include <Core/HW/SPI.hpp>
#include <Core/HW/EXT.hpp>
#include <Core/MW/CoreSensor.hpp>
#include <Core/MW/Thread.hpp>

namespace sensors {
   class L3GD20H
   {
public:
      L3GD20H(
         Core::HW::SPIDevice&  spi,
         Core::HW::EXTChannel& ext
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
      Core::HW::SPIDevice&  _spi;
      Core::HW::EXTChannel& _ext;
   };


   class L3GD20H_Gyro:
      public Core::MW::CoreSensor<Configuration::L3GD20H_GYRO_DATATYPE>
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
      Core::MW::Thread* _runner;
      Core::MW::Time    _timestamp;
      Configuration::L3GD20H_GYRO_DATATYPE _data;

private:
      L3GD20H& _device;
   };
}
