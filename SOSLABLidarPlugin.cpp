/////////////////////////////////////////////////////////////////////////////////////////
// MIT License
//
// Copyright (c) 2025 SOSLAB Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
///////////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <dw/core/base/Types.h>
#include <dw/core/context/Context.h>
#include <dw/sensors/common/Sensors.h>
#include <dw/sensors/lidar/Lidar.h>
#include <dw/sensors/legacy/plugins/SensorCommonPlugin.h>
#include <dw/sensors/legacy/plugins/lidar/LidarPlugin.h>

#include "SOSLABLidar.hpp"

std::vector<std::unique_ptr<dw::plugin::lidar::SOSLABLidar>> dw::plugin::lidar::SOSLABLidar::g_sensorContext;

//################################################################################
//############################### Helper Functions ###############################
//################################################################################
static dwStatus IsValidSensor(dw::plugin::lidar::SOSLABLidar* sensor)
{
    for (auto& i : dw::plugin::lidar::SOSLABLidar::g_sensorContext)
    {
        if (i.get() == sensor)
        {
            return DW_SUCCESS;
        }
    }

    return DW_INVALID_HANDLE;
}

//################################################################################
//###################### Common Sensor Plugin Functions ##########################
//################################################################################

// exported functions
extern "C" {

dwStatus _dwSensorPlugin_createHandle(dwSensorPluginSensorHandle_t* sensor,
                                      dwSensorPluginProperties*,
                                      char const*,
                                      dwContextHandle_t ctx)
{
    if (!sensor)
    {
        return DW_INVALID_ARGUMENT;
    }

    auto sensorContext = std::unique_ptr<dw::plugin::lidar::SOSLABLidar>(new dw::plugin::lidar::SOSLABLidar(ctx));
    dw::plugin::lidar::SOSLABLidar::g_sensorContext.push_back(move(sensorContext));
    *sensor = static_cast<dwSensorPluginSensorHandle_t>(dw::plugin::lidar::SOSLABLidar::g_sensorContext.back().get());

    return DW_SUCCESS;
}
//################################################################################

dwStatus _dwSensorPlugin_release(dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::SOSLABLidar* sensor = static_cast<dw::plugin::lidar::SOSLABLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    //Check if sensor in sensor list
    auto iter = std::find_if(dw::plugin::lidar::SOSLABLidar::g_sensorContext.begin(),
                             dw::plugin::lidar::SOSLABLidar::g_sensorContext.end(),
                             [&sensor](std::unique_ptr<dw::plugin::lidar::SOSLABLidar>& lidarSensor) {
                                 return (lidarSensor.get() == sensor);
                             });

    //If sensor in list remove it
    if (iter != dw::plugin::lidar::SOSLABLidar::g_sensorContext.end())
    {
        // Stop decoding process
        ret = sensor->stopSensor();
        if (ret != DW_SUCCESS)
        {
            return ret;
        }

        // Release resources claimed
        ret = sensor->releaseSensor();
        if (ret != DW_SUCCESS)
        {
            return ret;
        }

        // Remove sensor instance from context vector
        dw::plugin::lidar::SOSLABLidar::g_sensorContext.erase(iter);
        return DW_SUCCESS;
    }

    //If sensor was not found in sensor list
    return DW_FAILURE;
}
//################################################################################

dwStatus _dwSensorPlugin_createSensor(char const* params,
                                      dwSALHandle_t sal,
                                      dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::SOSLABLidar* sensor = static_cast<dw::plugin::lidar::SOSLABLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->createSensor(sal, params);
}
//################################################################################

dwStatus _dwSensorPlugin_start(dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::SOSLABLidar* sensor = static_cast<dw::plugin::lidar::SOSLABLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    if (!sensor->isVirtualSensor())
    {
        return sensor->startSensor();
    }

    return DW_SUCCESS;
}
//################################################################################

dwStatus _dwSensorPlugin_stop(dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::SOSLABLidar* sensor = static_cast<dw::plugin::lidar::SOSLABLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->stopSensor();
}
//################################################################################

dwStatus _dwSensorPlugin_reset(dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::SOSLABLidar* sensor = static_cast<dw::plugin::lidar::SOSLABLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->resetSensor();
}
//################################################################################

dwStatus _dwSensorPlugin_readRawData(uint8_t const** data,
                                     size_t* size,
                                     dwTime_t* timestamp,
                                     dwTime_t timeout_us,
                                     dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::SOSLABLidar* sensor = static_cast<dw::plugin::lidar::SOSLABLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->readRawData(data, size, timestamp, timeout_us);
}
//################################################################################

dwStatus _dwSensorPlugin_returnRawData(uint8_t const* data,
                                       dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::SOSLABLidar* sensor = static_cast<dw::plugin::lidar::SOSLABLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->returnRawData(data);
}
//################################################################################

dwStatus _dwSensorPlugin_pushData(size_t* lenPushed,
                                  uint8_t const* data,
                                  size_t const size,
                                  dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::SOSLABLidar* sensor = static_cast<dw::plugin::lidar::SOSLABLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->pushData(lenPushed, data, size);
}

//################################################################################
//###################### Lidar Specific Plugin Functions #########################
//################################################################################
dwStatus _dwSensorLidarPlugin_parseDataBuffer(dwLidarDecodedPacket* output,
                                              const dwTime_t hostTimestamp,
                                              dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::SOSLABLidar* sensor = static_cast<dw::plugin::lidar::SOSLABLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->parseDataBuffer(output, hostTimestamp);
}
//################################################################################

dwStatus _dwSensorLidarPlugin_getConstants(_dwSensorLidarDecoder_constants* constants,
                                           dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::SOSLABLidar* sensor = static_cast<dw::plugin::lidar::SOSLABLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->getConstants(constants);
}
//################################################################################

//################################################################################
//################# Sensor Class <-> Plugin Function Mapping #####################
//################################################################################
dwStatus dwSensorLidarPlugin_getFunctionTable(dwSensorLidarPluginFunctionTable* functions)
{
    if (functions == nullptr)
    {
        return DW_INVALID_ARGUMENT;
    }

    //Map common functions
    functions->common               = {};
    functions->common.createHandle  = _dwSensorPlugin_createHandle;
    functions->common.createSensor  = _dwSensorPlugin_createSensor;
    functions->common.release       = _dwSensorPlugin_release;
    functions->common.start         = _dwSensorPlugin_start;
    functions->common.stop          = _dwSensorPlugin_stop;
    functions->common.reset         = _dwSensorPlugin_reset;
    functions->common.readRawData   = _dwSensorPlugin_readRawData;
    functions->common.returnRawData = _dwSensorPlugin_returnRawData;
    functions->common.pushData      = _dwSensorPlugin_pushData;

    //Map lidar specific functions
    functions->parseDataBuffer     = _dwSensorLidarPlugin_parseDataBuffer;
    functions->getDecoderConstants = _dwSensorLidarPlugin_getConstants;

    return DW_SUCCESS;
}

} // extern "C"
