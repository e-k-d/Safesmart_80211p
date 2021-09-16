//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#pragma once

#include "veins/veins.h"
#include <iostream>
#include <vector>

#include "veins/modules/application/ieee80211p/SafeSmartApplLayer.h"

using namespace omnetpp;

namespace veins {

/**
 * @brief
 * This is a stub for a typical Veins application layer.
 * Most common functions are overloaded.
 * See MyVeinsApp.cc for hints
 *
 * @author David Eckhoff
 *
 */

class VEINS_API VehicleControlApp2 : public SafeSmartApplLayer {
public:
    void initialize(int stage) override;
    void finish() override;
    bool hasStopped;
    int numBSM;
    int tripTime;
    cOutVector numBSMVector;
    /*cOutVector timeToArriveRSU0;
    cOutVector timeToArriveRSU1;
    cOutVector timeToArriveRSU2;
    cOutVector timeToArriveRSU3;
    cOutVector timeToArriveRSU4;
    cOutVector timeToArriveRSU140;
    cOutVector timeToArriveRSU1100;
    cOutVector timeToArriveRSU1200;
    cOutVector timeToArriveRSU1300;
    cOutVector timeToArriveRSU1400;
    cOutVector distanceRSU0;
    cOutVector distanceRSU1;
    cOutVector distanceRSU2;
    cOutVector distanceRSU3;
    cOutVector distanceRSU4;*/

    cOutVector averageSpeedVector;
    /*cOutVector averageSpeedVector40;
    cOutVector averageSpeedVector100;
    cOutVector averageSpeedVector200;
    cOutVector averageSpeedVector300;
    cOutVector averageSpeedVector400;
    cOutVector KalmanSpeedVector;
    cOutVector distanceVector;*/
    cOutVector ttcVector;

    cLongHistogram numBSMStats;

    std::list<std::string> routeEdges;
    std::list<std::string> remainingEdges;
    std::list<std::string> srmEdges;
    std::string routeId;
    float averageSpeed;
    float averageSpeed40;
    float averageSpeed100;
    float averageSpeed200;
    float averageSpeed300;
    float averageSpeed400;
    float kalmanSpeed;
    int averageSpeedTerms;
    int relevantEdges;
    int TET_1s;
    int TET_2s;
    int TET_3s;
    int TET_4s;
    int TET_5s;

    double TIT;


    bool subscribed;
    int hasRecordedStats;
    simtime_t timerInterval;
    cMessage *timer;

    int safetyMetricTerms;
    double safetyMetricAverage;
    Coord safetyPrevPos;

    const int averageSpeedMaxTerms = 400;

    struct RSU{
        Coord pos;
        int id;
        std::vector<std::string> incomingEdges;
        std::vector<std::string> outgoingEdges;
        double prevDist;
        RSU(Coord pos, int id)
        {
            this->pos = pos;
            this->id = id;
            this->prevDist = 0;
        }
    };

    std::vector<RSU> knownRSU;


    //struct RSU knownRSU[300];

    //Coord knownRSU[300];


protected:
    int currentSubscribedServiceId;

protected:
    void onBSM(VehicleBSM* bsm) override;
    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
    void populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial) override;
    void checkAndAddRSU(Coord pos, int id);
    void addRSUMap (int id, RSUMap *map);
    void prepareSRM (int rsuId);
};

} // namespace veins
