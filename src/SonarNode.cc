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

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
using namespace std::chrono_literals;


SonarNode::~SonarNode()
{
}

SonarNode::SonarNode()
    : Node("sonar_node")
    , mPub(create_publisher<marine_acoustic_msgs::msg::RawSonarImage>("sonar", 1))
    , mTimer()
{
    //mPub = create_publisher<std_msgs::msg::String>("sonar", 10);
    // Use a timer to schedule periodic message publishing.
    mTimer = create_wall_timer(1s, [this]() {return this->on_timer();});
}

void SonarNode::publishPing(quint64 aTstamp, sonarphony::pingMsg_t const &aPing)
{
    int32_t seconds = aTstamp / 1000;
    int32_t nanoseconds = (aTstamp - (seconds * 1000)) * 1000000;
    rclcpp::Time ros_timestamp(seconds, nanoseconds);

    (void) aPing;

    marine_acoustic_msgs::msg::RawSonarImage msg;

    msg.header.stamp = ros_timestamp;
    msg.header.frame_id = "sonarphony_frame";

    // Center frequency of sonar in Hz
    // Set to 0 if unavailable
    // TODO: This varies based on model, this is the default for the T-POD
    msg.ping_info.frequency = 125000;

    // Speed of sound (m/s) used to calculate ranges;
    // Set to 0 if unavailable
    msg.ping_info.sound_speed = 0;

    // Sonar reported -3db beamwidths
    // May be empty if not reported
    // reported in radians
    // TODO: Default for T-POD, but not the same for the other models
    msg.ping_info.tx_beamwidths = std::vector<float> {30.0 * M_PI / 180.0};
    msg.ping_info.rx_beamwidths = std::vector<float> {30.0 * M_PI / 180.0};

    // hz
    //float32 sample_rate
    // TODO: Not sure what this value is
    msg.sample_rate = 0;

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

    mPub->publish(std::move(msg));
}

void SonarNode::on_timer()
{
    //auto msg = std::make_unique<std_msgs::msg::String>();
    //msg->data = "Hello, World";
    //mPub->publish(std::move(msg));
}

