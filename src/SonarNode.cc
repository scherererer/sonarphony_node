///////////////////////////////////////////////////////////////////////////
//  SonarPhony Node
//  Copyright (C) 2025 Michael P. Scherer
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
///////////////////////////////////////////////////////////////////////////

#include "SonarNode.hh"
using namespace sonarphony_node;

#include <QMetaObject>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <memory>
using namespace std::chrono_literals;


SonarNode::~SonarNode()
{
}

SonarNode::SonarNode(std::shared_ptr<sonarphony::sonarConnection_t> aConnection)
    : Node("sonar_node")
    , mConnection(std::move(aConnection))
    , mTimer(create_wall_timer(500ms, std::bind(&SonarNode::rotateFrequency, this)))
    , mPubImage(create_publisher<marine_acoustic_msgs::msg::RawSonarImage>("sonarphony/image", 1))
    , mPubRanges(create_publisher<marine_acoustic_msgs::msg::SonarRanges>("sonarphony/ranges", 1))
    , mPubNative(create_publisher<sonarphony_msgs::msg::SonarPhonyNative>("sonarphony/native", 1))
    , mPubSerial(create_publisher<std_msgs::msg::String>("sonarphony/serial", 1))
    , mSetRange(create_service<sonarphony_msgs::srv::SetSonarPhonyRange>(
        "sonarphony/set_range", std::bind(&SonarNode::handleSetRange, this,
            std::placeholders::_1, std::placeholders::_2)))
    , mSetFreq(create_service<sonarphony_msgs::srv::SetSonarPhonyFrequency>(
        "sonarphony/set_frequency", std::bind(&SonarNode::handleSetFrequency, this,
            std::placeholders::_1, std::placeholders::_2)))
    , mSelectedFrequencies(1U | 4U)
    , mPeriod(500ms)
    , mCurrentFrequency(0)
{
}

void SonarNode::rotateFrequency()
{
    assert(mSelectedFrequencies > 0);

    while(true)
    {
        ++mCurrentFrequency;
        if(mCurrentFrequency > 3)
            mCurrentFrequency = 0;

        if((mSelectedFrequencies >> mCurrentFrequency) & 0x01)
            break;
    }

    using namespace sonarphony;

    sonarConnection_t::frequency_t f = sonarConnection_t::F_125;

    switch(mCurrentFrequency)
    {
    case 0: f = sonarConnection_t::F_80; break;
    case 1: f = sonarConnection_t::F_125; break;
    case 2: f = sonarConnection_t::F_200; break;
    }

    QMetaObject::invokeMethod(mConnection.get(), "setFrequency", Qt::QueuedConnection,
                              Q_ARG(sonarConnection_t::frequency_t, f));
}

void SonarNode::publishPing(quint64 aTstamp, sonarphony::pingMsg_t const &aPing)
{
    int32_t seconds = aTstamp / 1000;
    int32_t nanoseconds = (aTstamp - (seconds * 1000)) * 1000000;
    rclcpp::Time ros_timestamp(seconds, nanoseconds);

    // Feet to meters
    double constexpr const FT2M = 0.3048;
    double const range = aPing.depth() * FT2M;

    {
        sonarphony_msgs::msg::SonarPhonyNative msg;

        msg.header.stamp = ros_timestamp;
        msg.header.frame_id = "sonarphony_frame";

        // Minimum range of watercolumn data (meters)
        msg.min_range = aPing.minRange() * FT2M;
        // Maximum range of watercolumn data (meters)
        msg.max_range = aPing.maxRange() * FT2M;

        // Waterdepth / range to detected hard bottom (meters)
        msg.range = range;

        // Temperature in degrees C
        msg.temperature = aPing.temperature();

        // Battery voltage in volts
        msg.battery_voltage = aPing.batteryVoltage();

        // Battery level (0-100%)
        msg.battery_level = aPing.batteryLevel();

        msg.frequency = aPing.frequency();
        msg.beam_width = aPing.beamWidth() * M_PI/180.0;

        // Watercolumn data
        auto rawData = reinterpret_cast<const unsigned char *>(aPing.pingData());
        msg.data = std::vector<uint8_t>(rawData, rawData + aPing.pingSize());

        mPubNative->publish(std::move(msg));
    }
    {
        marine_acoustic_msgs::msg::RawSonarImage msg;

        msg.header.stamp = ros_timestamp;
        msg.header.frame_id = "sonarphony_frame";

        // Center frequency of sonar in Hz
        // Set to 0 if unavailable
        msg.ping_info.frequency = aPing.frequency();

        // Speed of sound (m/s) used to calculate ranges;
        // Set to 0 if unavailable
        //msg.ping_info.sound_speed = 0;
        // NOTE: Using an assumed speed of sound value
        msg.ping_info.sound_speed = 1500;

        // Sonar reported -3db beamwidths
        // May be empty if not reported
        // reported in radians
        float const beamwidth = aPing.beamWidth() * M_PI/180.0;
        msg.ping_info.tx_beamwidths = std::vector<float> {beamwidth};
        msg.ping_info.rx_beamwidths = std::vector<float> {beamwidth};

        // hz
        //float32 sample_rate
        //msg.sample_rate = 0;
        // NOTE: This is a fake sample rate
        double sample_rate = double(aPing.pingSize()) *
            msg.ping_info.sound_speed / (2.0 * aPing.maxRange() * 0.3048);
        msg.sample_rate = sample_rate;

        // the number of samples in each beam
        msg.samples_per_beam = aPing.pingSize();

        // Many sonars have some kind of upper gate for water column data
        // this represents the sample number of the first non empty sample
        // for beam n
        msg.sample0 = 0;

        // Multi-sector multibeams can transmit different sectors at different times
        // The Reson doesn't but we include a TX delay for each beam anyway.
        //
        // Overall, therefore, our overall time is:
        //
        // header.stamp: TX cycle starts
        // Each beam's TX time: header.stamp + transmit_delay[i]
        // Each beam's RX time: header.stamp + transmit_delay[i] + sample_rate*image_row
        msg.tx_delays = std::vector<float> {0};

        // Steering angle applied to tx beam
        // reported in radians
        msg.tx_angles = std::vector<float> {0};

        // Steering angle applied to rx beam
        // reported in radians
        msg.rx_angles = std::vector<float> {0};

        msg.image.is_bigendian = true;
        msg.image.dtype = marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8;

        // the number of beams associated with the image
        msg.image.beam_count = 1;

        // The actually pixel data in row-major (beam_index major) format
        auto rawData = reinterpret_cast<const unsigned char *>(aPing.pingData());
        msg.image.data = std::vector<uint8_t>(rawData, rawData + aPing.pingSize());

        mPubImage->publish(std::move(msg));
    }
    {
        marine_acoustic_msgs::msg::SonarRanges msg;

        msg.header.stamp = ros_timestamp;
        msg.header.frame_id = "sonarphony_frame";

        // Center frequency of sonar in Hz
        // Set to 0 if unavailable
        msg.ping_info.frequency = aPing.frequency();

        // Speed of sound (m/s) used to calculate ranges;
        // Set to 0 if unavailable
        msg.ping_info.sound_speed = 0;

        // Sonar reported -3db beamwidths
        // May be empty if not reported
        // reported in radians
        float const beamwidth = aPing.beamWidth() * M_PI/180.0;
        msg.ping_info.tx_beamwidths = std::vector<float> {beamwidth};
        msg.ping_info.rx_beamwidths = std::vector<float> {beamwidth};

        // Constants declaring whether each beam is good or bad.
        // This is implemented as an 8-bit masked value; it is possible for more
        // than one thing to flag beams as bad. To check if a beam is good, simply
        // check if it is zero.
        msg.flags = std::vector<marine_acoustic_msgs::msg::DetectionFlag>
            {marine_acoustic_msgs::msg::DetectionFlag::DETECT_OK};

        // Multi-sector multibeams can transmit different sectors at different times
        // The Reson doesn't but we include a TX delay for each beam anyway.
        //
        // Overall, therefore, our overall time is:
        //
        // header.stamp: TX cycle starts
        // Each beam's TX time: header.stamp + txDelay[i]
        // Each beam's RX time: header.stamp + txDelay[i] + twowayTravelTime[i]
        msg.transmit_delays = std::vector<float> {0};

        // Sonar-reported intensity.  Usually uncalibrated and crude
        double const scale = (aPing.depth() - aPing.minRange()) /
                             (aPing.maxRange() - aPing.minRange());
        unsigned const idx = std::floor(scale * aPing.pingSize());
        msg.intensities = std::vector<float> {idx < aPing.pingSize() ?
                                              aPing.pingData()[idx] : 0.0f};

        geometry_msgs::msg::Vector3 v;
        v.x = 0; v.y = 0; v.z = 0;

        msg.beam_unit_vec = std::vector<geometry_msgs::msg::Vector3> {v};

        msg.ranges = std::vector<float> {static_cast<float> (range)};

        mPubRanges->publish(std::move(msg));
    }
}

void SonarNode::publishSerialNumber(std::string const &aSerialNumber)
{
    std_msgs::msg::String msg;

    msg.data = aSerialNumber;

    mPubSerial->publish(std::move(msg));
}

void SonarNode::handleSetRange(
    std::shared_ptr<sonarphony_msgs::srv::SetSonarPhonyRange::Request> const aReq,
    std::shared_ptr<sonarphony_msgs::srv::SetSonarPhonyRange::Response> const aRes)
{
    if(aReq->min_range < 0 || aReq->max_range < aReq->min_range)
    {
        aRes->success = false;
        aRes->message = "Invalid range, must be positive, and max>min";
        return;
    }

    QMetaObject::invokeMethod(mConnection.get(), "setRange", Qt::QueuedConnection,
                              Q_ARG(double, aReq->min_range),
                              Q_ARG(double, aReq->max_range));

    aRes->success = true;
}

void SonarNode::handleSetFrequency(
    std::shared_ptr<sonarphony_msgs::srv::SetSonarPhonyFrequency::Request> const aReq,
    std::shared_ptr<sonarphony_msgs::srv::SetSonarPhonyFrequency::Response> const aRes)
{
    if(aReq->period < 0)
    {
        aRes->success = false;
        aRes->message = "Period must be >= 0";
        return;
    }
    else if(aReq->frequency > 7)
    {
        aRes->success = false;
        aRes->message = "Invalid frequency selection";
        return;
    }

    mPeriod = std::chrono::milliseconds(int(aReq->period * 1000));
    mSelectedFrequencies = aReq->frequency;

    mTimer->cancel();
    mTimer = create_wall_timer(mPeriod, std::bind(&SonarNode::rotateFrequency, this));

    aRes->success = true;
}

