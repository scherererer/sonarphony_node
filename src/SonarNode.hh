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

#pragma once

#include "sonarphony/pingMsg.hh"
#include "sonarphony/sonarConnection.hh"

#include "sonarphony_msgs/msg/sonar_phony_native.hpp"
#include "sonarphony_msgs/srv/set_sonar_phony_range.hpp"
#include "sonarphony_msgs/srv/set_sonar_phony_frequency.hpp"

#include <marine_acoustic_msgs/msg/raw_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/sonar_ranges.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>

namespace sonarphony_node
{

class SonarNode : public rclcpp::Node
{
public:
    virtual ~SonarNode();
    SonarNode(std::shared_ptr<sonarphony::sonarConnection_t> aConnection);

    void rotateFrequency();

    void publishPing(quint64 aTstamp, sonarphony::pingMsg_t const &aPing);

    void publishSerialNumber(std::string const &aSerialNumber);

    void handleSetRange(
        std::shared_ptr<sonarphony_msgs::srv::SetSonarPhonyRange::Request> const aReq,
        std::shared_ptr<sonarphony_msgs::srv::SetSonarPhonyRange::Response> const aRes);
    void handleSetFrequency(
        std::shared_ptr<sonarphony_msgs::srv::SetSonarPhonyFrequency::Request> const aReq,
        std::shared_ptr<sonarphony_msgs::srv::SetSonarPhonyFrequency::Response> const aRes);

private:
    std::shared_ptr<sonarphony::sonarConnection_t> mConnection;

    rclcpp::TimerBase::SharedPtr mTimer;

    rclcpp::Publisher<marine_acoustic_msgs::msg::RawSonarImage>::SharedPtr mPubImage;
    rclcpp::Publisher<marine_acoustic_msgs::msg::SonarRanges>::SharedPtr mPubRanges;
    rclcpp::Publisher<sonarphony_msgs::msg::SonarPhonyNative>::SharedPtr mPubNative;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mPubSerial;

    rclcpp::Service<sonarphony_msgs::srv::SetSonarPhonyRange>::SharedPtr mSetRange;
    rclcpp::Service<sonarphony_msgs::srv::SetSonarPhonyFrequency>::SharedPtr mSetFreq;

    unsigned char mSelectedFrequencies;
    std::chrono::milliseconds mPeriod;
    unsigned char mCurrentFrequency;
};

}
