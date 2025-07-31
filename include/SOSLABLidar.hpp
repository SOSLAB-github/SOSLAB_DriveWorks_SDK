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
#ifndef SOSLABLidar_HPP
#define SOSLABLidar_HPP

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include <dw/core/base/Types.h>
#include <dw/core/context/Context.h>
#include <dw/sensors/common/Sensors.h>
#include <dw/sensors/lidar/Lidar.h>
#include <dw/sensors/legacy/plugins/lidar/LidarPlugin.h>

#include "BufferPool.hpp"
#include "SOSLABLidar_Properties.hpp"

namespace dw
{
namespace plugin
{
namespace lidar
{

class SOSLABLidar
{
public:
    //Data members
    static std::vector<std::unique_ptr<SOSLABLidar>> g_sensorContext;

    //Member functions
    //Constructors
    explicit SOSLABLidar(dwContextHandle_t ctx);
    ~SOSLABLidar() = default;

    //Common Sensor Plugin Functions
    dwStatus createSensor(dwSALHandle_t sal, char const* params);
    dwStatus startSensor();
    dwStatus stopSensor();
    dwStatus resetSensor();
    dwStatus releaseSensor();

    dwStatus readRawData(uint8_t const** data,
                         size_t* size,
                         dwTime_t* timestamp,
                         dwTime_t timeout_us);

    dwStatus returnRawData(uint8_t const* data);
    dwStatus pushData(size_t* lenPushed, uint8_t const* data, size_t const size);

    //Lidar Specific Plugin Functions
    dwStatus parseDataBuffer(dwLidarDecodedPacket* output, const uint64_t hostTimestamp);
    dwStatus getConstants(_dwSensorLidarDecoder_constants* constants);

    //Member Functions
    /**
         * Get type of lidar sensor.
         *
         * @return false if virtual snesor used for replay.
         *         true if live sensor stream used.
         */
    bool isVirtualSensor();

    /**
         * Get copy of file descriptor used for socket connection.
         *
         * @return uint32_t File descriptor value.
         */
    int32_t& getFileDescriptor();

    /**
         * Closes file descriptor used for socket connection.
         *
         * @return dwStatus DW_SUCCESS Successfully closed file descriptor.
         *                  DW_FAILURE Failed to close file descriptor.
         */
    dwStatus closeFileDescriptor();

    /**
         * Get parameters provided via command line.
         *
         * @param[out] string Reference to string to be populated with parameter value requested.
         * @param[in]  string Name of paramters to be searched for.
         * @param[in]  char*  Pointer to char array holding paramters values provided via command line.
         *
         * @return dwStatus DW_SUCCESS Successfully retrieved parameter value.
         *                  DW_FAILURE Failed to retrieve parameter value.
         */
    dwStatus getParameter(std::string& val, const std::string& param, const char* params);

private:
    //Framework
    dwContextHandle_t m_ctx;
    dwSALHandle_t m_sal = nullptr;

    //Socket connection
    bool m_isVirtual = true;
    int32_t m_fd     = -1;
    std::string m_ip;
    uint16_t m_port = 0;
    struct sockaddr_in m_socketAddress;

    //Raw data assembly
    dwTime_t m_currScanTimestamp;
    uint32_t m_lastPacketSize;
    bool m_scanComplete = false;
    dw::plugins::common::BufferPool<rawPacket> m_dataBuffer{BUFFER_SIZE};

    //Decoding
    std::unordered_map<uint8_t*, rawPacket*> m_map;
    std::queue<rawPacket> m_packetQueue;
    LidarPacket m_lidarOutput;

    //Decoder Constants
    _dwSensorLidarDecoder_constants m_constants = {};

    //Other
    bool m_init     = false;
    int m_frequency = 20;

    /**
         * Check if running packet collection is enough to assemble a full scan
         *
         * @return bool true  Received data worth a full scan.
         *              false Not enough data received.
         */
    bool isScanComplete();

    /**
         * Setup and connect socket to provided address.
         *
         * @return dwStatus DW_SUCCESS Successfully setup socket connection.
         *                  DW_FAILURE Failed to setup socket connection.
         */
    dwStatus establishConnection();
};

} // namespace lidar
} // namespace plugin
} // namespace dw
#endif
