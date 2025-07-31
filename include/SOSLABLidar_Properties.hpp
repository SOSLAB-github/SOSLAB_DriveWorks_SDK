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
#ifndef NVLIDAR_PROPERTIES_H_
#define NVLIDAR_PROPERTIES_H_

#include <cmath>
#include <cstdint>

#include <dw/core/base/Types.h>

namespace dw
{
namespace plugin
{
namespace lidar
{

//################################################################################
//################ Sensor specific parameters and data structures ################
//################################################################################

//SOSLAB ML Series Decoder Parameters
static const uint32_t MAX_POINTS_PER_FRAME  = 10752;    // 10752 points per one frame
static const uint32_t POINT_STRIDE         = 4U;        // x, y, z and intensity
static const uint16_t UDP_PAYLOAD_SIZE = 5408;      // 5408 bytes per one Packet
static const uint16_t MAX_FRAMES_PER_SECOND = 20;         // 20 frames per one second
static const uint32_t MAX_POINTS_PER_PACKET = 192;
static const uint32_t MAX_BUFFER_SIZE      = UDP_PAYLOAD_SIZE;
static const uint16_t BUFFER_SIZE            = 10;
static const uint32_t PAYLOAD_OFFSET       = sizeof(uint32_t) + sizeof(dwTime_t);
static const uint32_t PACKETS_PER_SPIN     = static_cast<int>(std::ceil((float)MAX_POINTS_PER_FRAME / (float)MAX_POINTS_PER_PACKET));

//DATA STRUCTURES
#pragma pack(1)
typedef struct
{
    uint8_t rawData[UDP_PAYLOAD_SIZE + PAYLOAD_OFFSET];
} rawPacket;

typedef struct
{
    //Cartesian coordinates
    float x;         // 4 Bytes [m]
    float y;         // 4       [m]
    float z;         // 4       [m]
    float intensity; // 4       [0.0-1.0]
} LidarPointXYZI;    // 16

typedef struct
{
    //Polar coordinates
    float theta;     // 4 [m]
    float phi;       // 4 [m]
    float radius;    // 4 [m]
    float intensity; // 4 [0.0-1.0]
} LidarPointRTHI;    // 16

typedef struct _ML_LIDAR_PACKET_ {
    char header[8];
    uint64_t timestamp;
    uint64_t status;
    uint8_t packet_type;
    uint8_t frame_id;
    uint8_t row_number;
    uint8_t rsvd[5];
} ml_lidar_packet_t;

typedef struct _PCD_ {
		int64_t x : 21;
		int64_t y : 21;
		int64_t z : 21;
		int64_t rsvd : 1;
}point_cloud_t;

typedef struct
{
    bool scan_complete;                        // 1
    dwTime_t sensor_timestamp;                 // 8
    uint32_t max_points_scan;                  // 4
    uint32_t n_points;                         // 4
    LidarPointRTHI rthi[MAX_POINTS_PER_PACKET]; // MAX_POINTS_PER_PACKET * sizeof(LidarPointRTHI)
    LidarPointXYZI xyzi[MAX_POINTS_PER_PACKET]; // MAX_POINTS_PER_PACKET * sizeof(LidarPointXYZI)
} LidarPacket;                                 // 1 + 8 + 4 + 4 + MAX_POINTS_PER_PACKET * 16 + MAX_POINTS_PER_PACKET * 16 = 57617
#pragma pack()

} // namespace lidar
} // namespace plugin
} // namespace dw

#endif
