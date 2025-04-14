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

#include <marine_acoustic_msgs/msg/raw_sonar_image.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sonarphony_node
{

class SonarNode : public rclcpp::Node
{
public:
    virtual ~SonarNode();
    SonarNode();

    void publishPing(quint64 aTstamp, sonarphony::pingMsg_t const &aPing);

protected:
    void on_timer();

private:
    rclcpp::Publisher<marine_acoustic_msgs::msg::RawSonarImage>::SharedPtr mPubImage;
    rclcpp::Publisher<marine_acoustic_msgs::msg::RawSonarImage>::SharedPtr mPubDet;
    rclcpp::TimerBase::SharedPtr mTimer;
};

}
