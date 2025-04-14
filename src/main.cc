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

#include "SonarApplication.hh"
using namespace sonarphony_node;

#include "rclcpp/rclcpp.hpp"

#include <QCoreApplication>

#include <chrono>
#include <memory>
using namespace std::chrono_literals;


int main(int aArgc, char **aArgv)
{
    rclcpp::init(aArgc, aArgv);
	QCoreApplication app(aArgc, aArgv);

    SonarApplication sa;

	int code = app.exec();

    rclcpp::shutdown();

    return code;
}

