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

#include "SOSLABLidar.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <iostream>

#include "BufferPool.hpp"

namespace dw
{
namespace plugin
{
namespace lidar
{

SOSLABLidar::SOSLABLidar(dwContextHandle_t ctx)
    : m_ctx(ctx)
{
}

//################################################################################
//###################### Common Sensor Plugin Functions ##########################
//################################################################################
dwStatus SOSLABLidar::createSensor(dwSALHandle_t sal, char const* params)
{
    //Set flag that real sensor is being used
    m_isVirtual = false;

    //Pass SAL handle
    m_sal = sal;

    //Get connections details
    if (getParameter(m_ip, "ip", params) != DW_SUCCESS)
    {
        std::cout << "SOSLABLidar::createSensor: No ip specified in parameter list!" << std::endl;
        return DW_FAILURE;
    }

    std::string portNumber;
    if (getParameter(portNumber, "port", params) != DW_SUCCESS)
    {
        std::cout << "SOSLABLidar::createSensor: No port specified in parameter list!" << std::endl;
        return DW_FAILURE;
    }
    else
    {
        m_port = static_cast<uint16_t>(atoi(portNumber.c_str()));
    }

    for (uint8_t i = 0; i < BUFFER_SIZE; ++i)
    {
        rawPacket* p_rawPacket = nullptr;
        m_dataBuffer.get(p_rawPacket);
        uint8_t* p_rawData = &(p_rawPacket->rawData[0]);
        m_map[p_rawData]   = p_rawPacket;
        m_dataBuffer.put(p_rawPacket);
    }

    return DW_SUCCESS;
}

dwStatus SOSLABLidar::startSensor()
{
    if (!isVirtualSensor())
    {
        return establishConnection();
    }
    return DW_SUCCESS;
}

dwStatus SOSLABLidar::stopSensor()
{
    if (!isVirtualSensor())
    {
        return closeFileDescriptor();
    }
    return DW_SUCCESS;
}

dwStatus SOSLABLidar::resetSensor()
{
    dwStatus status = stopSensor();
    if (status != DW_SUCCESS)
    {
        return status;
    }

    // Clear buffer pool
    for (int i = 0; i < BUFFER_SIZE; ++i)
    {
        // Get pointer to element in buffer
        rawPacket* p_rawPacket = nullptr;
        m_dataBuffer.get(p_rawPacket, 1000);

        // Clear memory
        std::memset(p_rawPacket, 0, sizeof(rawPacket));

        //Return pointer to element
        m_dataBuffer.put(p_rawPacket);
    }

    // Empty packet buffer
    while (!m_packetQueue.empty())
    {
        m_packetQueue.pop();
    }

    return startSensor();
}

dwStatus SOSLABLidar::releaseSensor()
{
    if (!isVirtualSensor())
    {
        return closeFileDescriptor();
    }

    return DW_SUCCESS;
}

dwStatus SOSLABLidar::readRawData(uint8_t const** data,
                              size_t* size,
                              dwTime_t* timestamp,
                              dwTime_t timeout_us)
{
    //Get pointer to object in buffer pool to be used for raw data storage
    rawPacket* p_rawPacket = nullptr;
    bool result            = m_dataBuffer.get(p_rawPacket, timeout_us);
    if (!result)
    {
        std::cerr << "SOSLABLidar::readRawData: Failed to get slot from buffer pool" << std::endl;
        return DW_BUFFER_FULL;
    }

    //Get next packet
    uint32_t bytesReceived = recvfrom(m_fd, &(p_rawPacket->rawData[PAYLOAD_OFFSET]), UDP_PAYLOAD_SIZE, 0, nullptr, nullptr);

    //Get host timestamp for received packet
    dwTime_t packetTimestamp;
    dwContext_getCurrentTime(&packetTimestamp, m_ctx);

    //Copy meta data to buffer and assign to pointers passed to function call
    //NOTE: Raw data is always prefixed with size of received data (uint32_t) and timestamp (dwTime_t)
    std::memcpy(&(p_rawPacket->rawData[0]), &bytesReceived, sizeof(uint32_t));
    std::memcpy(&(p_rawPacket->rawData[sizeof(uint32_t)]), &packetTimestamp, sizeof(dwTime_t));
    *timestamp = packetTimestamp;
    *size      = bytesReceived;
    *data      = &(p_rawPacket->rawData[0]);

    return DW_SUCCESS;
}

dwStatus SOSLABLidar::returnRawData(uint8_t const* data)
{
    if (data == nullptr)
    {
        return DW_INVALID_HANDLE;
    }

    //Return object to buffer pool
    bool result = m_dataBuffer.put(const_cast<rawPacket*>(m_map[const_cast<uint8_t*>(data)]));
    if (!result)
    {
        std::cerr << "SOSLABLidar::returnRawData: Failed to return object to buffer pool!" << std::endl;
        return DW_INVALID_ARGUMENT;
    }

    data = nullptr;

    return DW_SUCCESS;
}

dwStatus SOSLABLidar::pushData(size_t* lenPushed,
                           uint8_t const* data,
                           size_t const size)
{
    /*
     * NOTE: The parameter "size" equals the amount of raw data recevied in bytes plus the
     *       offset of 12 Bytes introduced by prepending the timestamp and amount of
     *       received data in readRawData => size = payload offset + bytesReceived
     *
     *       ===============================================================================
     *       | uint32_t | dwTime_t |                    rawData                            |
     *       ===============================================================================
     *          4 Bytes   8 Bytes                       bytesReceived
    */

    //Copy raw data collected in readRawData to rawPacket object
    rawPacket packet;

    //Copy data into packet struct
    std::memcpy(&(packet.rawData[PAYLOAD_OFFSET]), data, size);

    //Add scan to queue of received scans
    m_packetQueue.push(packet);

    //Update size of data "pushed" to queue
    *lenPushed = size;

    //Update size for last packet received
    m_lastPacketSize = size;

    return DW_SUCCESS;
}

//################################################################################
//###################### Lidar Specific Plugin Functions #########################
//################################################################################
dwStatus SOSLABLidar::parseDataBuffer(dwLidarDecodedPacket* output, const uint64_t hostTimestamp)
{
    //Buffer for rawData assembly
    uint8_t assemblyBuffer[MAX_BUFFER_SIZE];

    //Get pointer to next packet in queue
    rawPacket* p_rawPacket = &(m_packetQueue.front());

    //Copy data from packet into assembly buffer
    std::memcpy(&(assemblyBuffer[0]), &(p_rawPacket->rawData[PAYLOAD_OFFSET]), UDP_PAYLOAD_SIZE);

    //Release pointer to packet
    p_rawPacket = nullptr;

    //Remove packet from queue
    m_packetQueue.pop();

    //ML-A
    ml_lidar_packet_t lidar_pkt = *reinterpret_cast<ml_lidar_packet_t*>(&assemblyBuffer[0]);
    
    uint32_t received_size = sizeof(_ML_LIDAR_PACKET_);
    std::vector<LidarPointXYZI> pcdi_data(MAX_POINTS_PER_PACKET);
    received_size += 576 * 4;
    // lidar data
    for (int c = 0; c < MAX_POINTS_PER_PACKET; c++) {
            received_size += 4;
            uint16_t intensity_value = *reinterpret_cast<uint16_t*>(&assemblyBuffer[received_size]);
            received_size += 4;
            point_cloud_t point_cloud = *reinterpret_cast<point_cloud_t*>(&assemblyBuffer[received_size]);
            pcdi_data[c].x = (float)point_cloud.x / 1000.0f;
            pcdi_data[c].y = (float)point_cloud.y / 1000.0f;
            pcdi_data[c].z = (float)point_cloud.z / 1000.0f;
            pcdi_data[c].intensity = (float)intensity_value / 2286.0f;
            received_size += sizeof(point_cloud_t);
    }
    
    output->hostTimestamp   = hostTimestamp;
    output->sensorTimestamp = ((lidar_pkt.timestamp >> 32) * 1000 + ((lidar_pkt.timestamp & 0xFFFFFFFF)/ 1000000)) ; //ns -> ms
    output->maxPoints       =  MAX_POINTS_PER_FRAME;
    output->nPoints         =  MAX_POINTS_PER_PACKET;
    output->scanComplete    = (lidar_pkt.row_number == 55) ? 1 : 0; 

    dwLidarPointXYZI* xyzi = const_cast<dwLidarPointXYZI*>(output->pointsXYZI);
    dwLidarPointRTHI* rthi = const_cast<dwLidarPointRTHI*>(output->pointsRTHI);

    for (size_t i = 0; i < output->nPoints; ++i)
    {
        xyzi[i].x         = pcdi_data[i].x;
        xyzi[i].y         = pcdi_data[i].y;
        xyzi[i].z         = pcdi_data[i].z;
        xyzi[i].intensity = pcdi_data[i].intensity;

        rthi[i].radius    = pcdi_data[i].x;
        rthi[i].theta     = pcdi_data[i].y;
        rthi[i].phi       = pcdi_data[i].z;
        rthi[i].intensity = pcdi_data[i].intensity;
    }

    //Clear assembly buffer
    std::memset(assemblyBuffer, 0, sizeof(assemblyBuffer));

    return DW_SUCCESS;
}

dwStatus SOSLABLidar::getConstants(_dwSensorLidarDecoder_constants* constants)
{
    if (!m_init)
    {
        m_constants.maxPayloadSize = UDP_PAYLOAD_SIZE;

        if (isVirtualSensor())
        {
            m_frequency = MAX_FRAMES_PER_SECOND;
        }

        //Populate lidar properties
        dwLidarProperties* properties = &(m_constants.properties);
        properties->pointsPerSpin     = MAX_POINTS_PER_FRAME;
        properties->pointsPerSecond   = MAX_POINTS_PER_FRAME * m_frequency;
        properties->packetsPerSpin    = PACKETS_PER_SPIN;
        properties->spinFrequency     = m_frequency;
        properties->packetsPerSecond  = m_frequency * PACKETS_PER_SPIN;
        properties->pointsPerPacket   = MAX_POINTS_PER_PACKET;
        properties->pointStride       = POINT_STRIDE;
        properties->availableReturns  = DW_LIDAR_RETURN_TYPE_ANY;
        strcpy(properties->deviceString, "SOSLAB_LIDAR");

        //Assign populatatet constants object to function parameter
        *constants = m_constants;

        //Avoid repeated query for constants
        m_init = true;

        return DW_SUCCESS;
    }

    *constants = m_constants;

    return DW_SUCCESS;
}

//################################################################################
//############################## Helper Functions ################################
//################################################################################
bool SOSLABLidar::isVirtualSensor()
{
    return m_isVirtual;
}

dwStatus SOSLABLidar::closeFileDescriptor()
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return DW_FAILURE;
    }

    struct sockaddr_in local_addr;
    local_addr.sin_addr.s_addr = inet_addr("192.168.1.15");
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(2000);

    if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        close(sock);
        perror("Bind failed");
        return DW_FAILURE;
    }

    // Set up the server address
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port        = htons(m_port);
    server_addr.sin_addr.s_addr = inet_addr(m_ip.c_str());

    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        close(sock);
        perror("Connection failed");
        return DW_FAILURE;
    }

    // Get the local endpoint information
    socklen_t addr_len = sizeof(local_addr);
    int local_port = 2000;
    if (getsockname(sock, (struct sockaddr*)&local_addr, &addr_len) == 0) {
        char local_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &local_addr.sin_addr, local_ip, INET_ADDRSTRLEN);
        local_port = ntohs(local_addr.sin_port);
    } else {
        perror("getsockname failed");
    }

    std::string msg = "{\"command\":\"stop\"}";
    send(sock , msg.c_str() , strlen(msg.c_str()) , 0 );

    close(sock);

    int8_t result = close(m_fd);
    if (result == 0)
    {
        return DW_SUCCESS;
    }
    else
    {
        std::cerr << "SOSLABLidar::closeFileDescriptor: Failed to close file descriptor!" << std::endl;
        return DW_FAILURE;
    }
}

int32_t& SOSLABLidar::getFileDescriptor()
{
    return m_fd;
}

dwStatus SOSLABLidar::getParameter(std::string& val, const std::string& param, const char* params)
{
    std::string paramsString(params);
    std::string searchString(param + "=");
    size_t pos = paramsString.find(searchString);
    if (pos == std::string::npos)
    {
        return DW_FAILURE;
    }

    val = paramsString.substr(pos + searchString.length());
    pos = val.find_first_of(',');
    val = val.substr(0, pos);

    return DW_SUCCESS;
}

bool SOSLABLidar::isScanComplete()
{
    //NOTE: Size of last packet of raw data of a scan is used as separator between scans.
    if (m_lastPacketSize == UDP_PAYLOAD_SIZE && ((m_packetQueue.size()) == 0))
    {
        //If most recent packet received is last packet of scan and queue holds the expected number of packets.
        return true;
    }
    else if (m_lastPacketSize == UDP_PAYLOAD_SIZE && ((m_packetQueue.size()) != 0))
    {
        //If most recent packet received is last packet of scan and queue holds not the expected number of packets.
        //Empty queue to start with new scan
        while (!m_packetQueue.empty())
        {
            m_packetQueue.pop();
        }
        return false;
    }
    else if (m_packetQueue.size() >= 1)
    {
        //If amount of buffered packets exceeds the number of packets needed to assemble a full scan
        while (!m_packetQueue.empty())
        {
            m_packetQueue.pop();
        }
        return false;
    }
    else
    {
        //If packets are still needed for complete scan and most recent packet received is not the last packet of a scan.
        return false;
    }
}

dwStatus SOSLABLidar::establishConnection()
{
    //Depending on communication protocol (UDP and TCP) setup socket accordingly
    {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            perror("Socket creation failed");
            return DW_FAILURE;
        }

        struct sockaddr_in local_addr;
        local_addr.sin_addr.s_addr = inet_addr("192.168.1.15");
        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(2000);

        if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            close(sock);
            perror("Bind failed");
            return DW_FAILURE;
        }

        // Set up the server address
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port        = htons(m_port);
        server_addr.sin_addr.s_addr = inet_addr(m_ip.c_str());

        if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            close(sock);
            perror("Connection failed");
            return DW_FAILURE;
        }

        // Get the local endpoint information
        socklen_t addr_len = sizeof(local_addr);
        int local_port = 2000;
        if (getsockname(sock, (struct sockaddr*)&local_addr, &addr_len) == 0) {
            char local_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &local_addr.sin_addr, local_ip, INET_ADDRSTRLEN);
            local_port = ntohs(local_addr.sin_port);
        } else {
            perror("getsockname failed");
        }

        std::string msg = "{\"command\":\"run\"}";
        send(sock , msg.c_str() , strlen(msg.c_str()) , 0 );
        close(sock);
    }

    m_fd = socket(AF_INET, SOCK_DGRAM, 0);

    if (m_fd < 0)
    {
        std::cerr << "SOSLABLidar::establishConnection: Failed to create socket!" << std::endl;
        return DW_FAILURE;
    }

    uint32_t reuse = 1;
    if (setsockopt(m_fd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) < 0)
    {
        std::cerr << "SOSLABLidar::establishConnection: Failed to set SO_REUSEPORT socket option" << std::endl;
        return DW_FAILURE;
    }

    if (setsockopt(m_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
    {
        std::cerr << "SOSLABLidar::establishConnection: Failed to set SO_REUSEADDR socket option" << std::endl;
        return DW_FAILURE;
    }

    int flags = 1;
    if (setsockopt(m_fd, SOL_SOCKET, SO_TIMESTAMP, &flags, sizeof(flags)) < 0)
    {
        std::cerr << "SOSLABLidar::establishConnection: Failed to enable kernel timestamping socket options" << std::endl;
        return DW_FAILURE;
    }

    //Socket address
    m_socketAddress.sin_family      = AF_INET;
    m_socketAddress.sin_port        = htons(m_port);
    m_socketAddress.sin_addr.s_addr = inet_addr(m_ip.c_str());

    int8_t result = 0;

    //Establish connection
    struct sockaddr_in local_addr;
    local_addr.sin_addr.s_addr = inet_addr("192.168.1.15");
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(2000);

    result = bind(m_fd, (struct sockaddr*)&local_addr, sizeof(local_addr));


    if (result < 0)
    {
        std::cerr << "SOSLABLidar::establishConnection: Failed to bind socket" << std::endl;
        return DW_FAILURE;
    }

    return DW_SUCCESS;
}

} // namespace lidar
} // namespace plugin
} // namespace dw
