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

#include "veins/modules/application/ieee80211p/SafeSmartApplLayer.h"

namespace veins {

/**
 * Small RSU Demo using 11p
 */
class VEINS_API RSUControl : public SafeSmartApplLayer {

public:
    void initialize(int stage) override;
    void finish() override;

    struct Vehicle{
        Coord pos;
        int id;
        Vehicle(Coord pos, int id)
        {
            this->pos = pos;
            this->id = id;
        }
    };

    struct Request{
        int receiverId;
        std::vector<std::string> route;
        simtime_t estimatedArrival;
        int scenario;
        int senderId;
        float curDist;
        float prevDist;
        int gettingAway;
        bool beenActivated;
        Request(int receiverId, std::vector<std::string> route, simtime_t estimatedArrival, int scenario, int senderId, int curDist)
        {
            this->route = route;
            this->receiverId = receiverId;
            this->estimatedArrival = estimatedArrival;
            this->scenario = scenario;
            this->senderId = senderId;
            this->curDist = this->prevDist = curDist;
            this->beenActivated = false;
        }
    };

    struct Schedule{
        simtime_t begin;
        simtime_t end;
        int scenario;
        Schedule(simtime_t begin, simtime_t end, int scenario)
        {
            this->begin = begin;
            this->end = end;
            this->scenario = scenario;
        }
    };

    std::vector<Vehicle> knownVehicles;
    std::vector<std::string> controlledTLS;
    std::vector<std::string> incomingEdges;
    std::vector<std::string> outgoingEdges;
    std::vector<Request> requests;
    std::vector<Schedule> schedule;
    simtime_t remainingPhaseTime;

    const simtime_t timeThreshold = 400.0;

    bool once;
    bool beenPreempted;

protected:
    simtime_t timerInterval;
    cMessage *timer;
    cMessage *scenarioSchedule;

protected:
    void onWSM(BaseFrame1609_4* wsm) override;
    void onBSM(VehicleBSM* bsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;
    void handleSelfMsg(cMessage* msg) override;
    void populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial) override;
    void checkAndAddVehicle(Coord pos, int id);
    void lanesToEdges();
    void checkAndAddRequest(int rsuId, std::vector<std::string> route, simtime_t estimatedArrival, int senderId);
    void updateVehiclePosition(int senderId, double distance);
    int translateRequest(std::vector<std::string> route);
    void attendRequest();
    void translateScenario(int rsuId, int scenario);
    void setTLSMinTime(std::string tls);
    void revertToNormal();
    //void checkAndSchedule(int scenario, simtime_t estimatedArrival);
    //void handleLowerMsg(cMessage* msg) override;
    //void handleUpperMsg(cMessage* msg) override;
    //void handleMessage(cMessage* msg) override;
};

} // namespace veins
