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

#include "SonarNode.hh"

#include "sonarphony/sonarConnection.hh"

#include <QObject>
#include <QTimer>

#include <memory>


namespace sonarphony_node
{

class SonarApplication : public QObject
{
public:
    virtual ~SonarApplication();
    SonarApplication();

private slots:
    void runSome();
	void handlePing(quint64 aTstamp, sonarphony::pingMsg_t const &aPing);
	void serialNumberChanged ();

private:
    QTimer mTimer;

    /// \brief Connection to the sonar unit
    std::shared_ptr<sonarphony::sonarConnection_t> mConnection;

    std::shared_ptr<SonarNode> mNode;
};

}
