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

#include <QCoreApplication>


SonarApplication::~SonarApplication()
{
}

SonarApplication::SonarApplication()
    : QObject()
    , mTimer()
    , mConnection(std::make_shared<sonarphony::sonarConnection_t>(this))
    , mNode(std::make_shared<SonarNode>(mConnection))
{
    mTimer.setInterval(50);
    mTimer.setSingleShot(false);

    connect(&mTimer, &QTimer::timeout,
            this, &SonarApplication::runSome);

    mTimer.start();

	connect (mConnection.get(), &sonarphony::sonarConnection_t::ping,
	         this, &SonarApplication::handlePing);
	mConnection->start ();
}

void SonarApplication::runSome()
{
    if (! rclcpp::ok())
        QCoreApplication::quit();

    try
    {
        rclcpp::spin_some(mNode);
    }
    catch (rclcpp::exceptions::RCLError const &e)
    {
        RCLCPP_ERROR(mNode->get_logger(), "unexpectedly failed with %s", e.what());
    }
}

void SonarApplication::handlePing(quint64 aTstamp,
                                  sonarphony::pingMsg_t const &aPing)
{
    mNode->publishPing(aTstamp, aPing);
}

void SonarApplication::serialNumberChanged ()
{
    mNode->publishSerialNumber(mConnection->serialNumber());
}

