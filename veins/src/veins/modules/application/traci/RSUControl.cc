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

#define RSU0ID 14
#define RSU1ID 19
#define RSU2ID 24
#define RSU3ID 29
#define RSU4ID 34


//#define DEBUG

#include "veins/modules/application/traci/RSUControl.h"

#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"

#include "veins/modules/mobility/traci/TraCIConnection.h"

using namespace veins;

Define_Module(veins::RSUControl);

void RSUControl::initialize(int stage)
{
    SafeSmartApplLayer::initialize(stage);

    TraCIScenarioManager *manager = TraCIScenarioManagerAccess().get();
    TraCICommandInterface *traci = manager->getCommandInterface();

    if (stage == 0)
    {
        timer = new cMessage("timer");
        scenarioSchedule = new cMessage("scenarioSchedule");
        scenarioSchedule->setKind(SEND_SELF_EVT);
        wsaInterval = 5;
        beaconInterval = 0.1;
        mapInterval = 1;
        spatInterval = 0.1;
        timerInterval = 0.5;
        once = false;
        beenPreempted = false;
    }
    else
    if (stage == 1)
    {
        //13,18,23,28,33
        EV << "myid=" << myId << std::endl;

        if (myId == RSU0ID)
        {
            controlledTLS.emplace_back("655189915");
            controlledTLS.emplace_back("655189922");
            controlledTLS.emplace_back("604969891");
            controlledTLS.emplace_back("655189917");
            controlledTLS.emplace_back("655189920");
        }
        else
        if (myId == RSU1ID)
        {
            controlledTLS.emplace_back("605008045");
            controlledTLS.emplace_back("604969890");
            controlledTLS.emplace_back("1666024484");
            controlledTLS.emplace_back("1666024479");
            controlledTLS.emplace_back("606181027");
            controlledTLS.emplace_back("605008047");
        }
        else
        if (myId == RSU2ID)
        {
            controlledTLS.emplace_back("1111343446");
            controlledTLS.emplace_back("606181023");
            controlledTLS.emplace_back("1111343417");
            controlledTLS.emplace_back("1111343376");
            controlledTLS.emplace_back("266205952");
            controlledTLS.emplace_back("1111343438");
        }
        else
        if (myId == RSU3ID)
        {
            controlledTLS.emplace_back("1949937054");
            controlledTLS.emplace_back("1662318224");
            controlledTLS.emplace_back("606181005");
            controlledTLS.emplace_back("1949937055");
            controlledTLS.emplace_back("609788279");
            controlledTLS.emplace_back("1662318221");
        }
        else
        if (myId == RSU4ID)
        {
            controlledTLS.emplace_back("3788629975");
            controlledTLS.emplace_back("1111343412");
        }


        EV << "TSU ID " << myId << std::endl;

        scheduleAt(computeAsynchronousSendingTime(spatInterval, ChannelType::control), sendSpatEvt);
        scheduleAt(computeAsynchronousSendingTime(mapInterval, ChannelType::control), sendMapEvt);
        scheduleAt(simTime() + timerInterval, scenarioSchedule); // timer for SRM processing
        scheduleAt(computeAsynchronousSendingTime(beaconInterval, ChannelType::control), sendBeaconEvt);
    }

}


void RSUControl::handleSelfMsg(cMessage* msg)
{
    Coord pos;
    TraCIScenarioManager *manager = TraCIScenarioManagerAccess().get();
    TraCICommandInterface *traci = manager->getCommandInterface();

    if (once == false)
    {
        lanesToEdges();
        once = true;
    }

    switch (msg->getKind()) {
        case SEND_BEACON_EVT: {
#ifdef DEBUG
            EV << "RSUControl::handleSelfMsg SEND_BEACON_EVT" << std::endl;
#endif
            VehicleBSM* bsm = new VehicleBSM();
            populateWSM(bsm, LAddress::L2BROADCAST(), 0);
            sendDown(bsm);
            scheduleAt(simTime() + beaconInterval, sendBeaconEvt);

            //RSUSpat* spat = new RSUSpat();
            //populateWSM(spat, LAddress::L2BROADCAST(), 0);
            //sendDown(spat);
            //scheduleAt(simTime() + spatInterval, sendSpatEvt);
            break;
        }
        case SEND_WSA_EVT: {
#ifdef DEBUG
            EV << "RSUControl::handleSelfMsg SEND_WSA_EVT" << std::endl;
#endif
            DemoServiceAdvertisment* wsa = new DemoServiceAdvertisment();
            populateWSM(wsa, LAddress::L2BROADCAST(), 0);
            sendDown(wsa);
            scheduleAt(simTime() + wsaInterval, sendWSAEvt);
            break;
        }
        case SEND_MAP_EVT: {
#ifdef DEBUG
            EV << "RSUControl::handleSelfMsg SEND_MAP_EVT" << std::endl;
#endif
            RSUMap* map = new RSUMap();
            populateWSM(map, LAddress::L2BROADCAST(), 0);
            sendDown(map);
            scheduleAt(simTime() + mapInterval, sendMapEvt);
            break;
        }
        case SEND_SPAT_EVT: {
#ifdef DEBUG
            EV << "RSUControl::handleSelfMsg SEND_SPAT_EVT" << std::endl;
#endif
            RSUSpat* spat = new RSUSpat();
            populateWSM(spat, LAddress::L2BROADCAST(), 0);
            sendDown(spat);
            scheduleAt(simTime() + spatInterval, sendSpatEvt);
            break;
        }

        case SEND_SELF_EVT:{
#ifdef DEBUG
            EV << "RSUControl::handleSelfMsg Timer (SRM) RSUid " << this->getId() << std::endl;
#endif
            attendRequest();
            scheduleAt(simTime() + timerInterval, scenarioSchedule);  // rescheduling
            break;
        }
        default: {
            if (msg) EV_WARN << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
            break;
        }
        }
}

void RSUControl::onWSA(DemoServiceAdvertisment* wsa)
{
    // if this RSU receives a WSA for service 42, it will tune to the chan
    if (wsa->getPsid() == 42) {
        mac->changeServiceChannel(static_cast<Channel>(wsa->getTargetChannel()));
    }
#ifdef DEBUG
    EV << "RSUControl::onWSA " << myId << std::endl;
#endif
}

void RSUControl::onWSM(BaseFrame1609_4* frame)
{
    std::vector<std::string> tempRoute;
    int k;

#ifdef DEBUG
    EV << "RSUControl::onWSM " << myId << std::endl;
#endif

    if (RSUMap* wsm = dynamic_cast<RSUMap*>(frame))
    {
        // this rsu repeats the received traffic update in 2 seconds plus some random delay
        //sendDelayedDown(wsm->dup(), 2 + uniform(0.01, 0.2));
#ifdef DEBUG
        EV << "RSUControl::onWSM MAPMsg " << myId << std::endl;
#endif
    }
    else
    if (VehicleSRM* srm = dynamic_cast<VehicleSRM*>(frame)) //received SRM from vehicle
    {
#ifdef DEBUG
        EV << "RSUControl::onWSM VehicleSRM " << myId << std::endl;
#endif
        //requests.emplace_back(srm);
        if (myId == srm->getReceiverId()) //if this RSU is the intended receiver
        {
#ifdef DEBUG
            EV << "RSU " << myId << " received SRM from " << srm->getSenderId() << std::endl;
            EV << "Current time: " << simTime() << " Arrives: " << srm->getTimeToArrive() << std::endl;
#endif

            // fills the request with vehicle route
#ifdef DEBUG
            EV << "Received vehicle route: ";
#endif
            for (k = 0; k < srm->getRouteArraySize(); k++)
            {
                tempRoute.emplace_back(srm->getRoute(k));
#ifdef DEBUG
                EV << srm->getRoute(k) << " ";
#endif
            }
            checkAndAddRequest(srm->getReceiverId(), tempRoute, srm->getTimeToArrive(), srm->getSenderId());
            //requests.emplace_back(srm->getReceiverId(), tempRoute, srm->getTimeToArrive());
            attendRequest();
        }
    }
}


void RSUControl::onBSM(VehicleBSM* bsm)
{
    Coord senderPos = bsm->getSenderPos();
    Coord currentPos = check_and_cast<BaseMobility*>(getSimulation()->getModuleByPath("rsu[0].mobility"))->getPositionAt(simTime());
    //double distance = (senderPos - currentPos).length();
    double distanceNaive = (curPosition - bsm->getSenderPos()).length();

#ifdef DEBUG
    EV << "RSUControl::onBSM " << myId << " (x,y,z)=" << senderPos.x << ", " << senderPos.y << ", "  << senderPos.z << ")";
    EV << "Curdist: " << distanceNaive << std::endl;
#endif
    updateVehiclePosition(bsm->getSenderId(), distanceNaive);
    //checkAndAddVehicle(senderPos, bsm->getSenderId());
}

void RSUControl::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial)
{
    wsm->setRecipientAddress(rcvId);
    wsm->setBitLength(headerLength);

    int k = 0;

#ifdef DEBUG
    EV << "RSUControl::populateWSM evt=" << wsm->getKind() << std::endl;
#endif

    if (VehicleBSM* bsm = dynamic_cast<VehicleBSM*>(wsm)) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(-1);
        bsm->setChannelNumber(static_cast<int>(Channel::cch));
        bsm->addBitLength(beaconLengthBits);
        bsm->setSenderId(this->myId);
        bsm->setCreationTime(simTime());
#ifdef DEBUG
        EV << "RSUControl::populateWSM BSM " << this->myId << std::endl;
#endif
        wsm->setUserPriority(beaconUserPriority);
    }
    else
    if (RSUMap* map = dynamic_cast<RSUMap*>(wsm)) {
        map->setSenderId(this->myId);
        map->setChannelNumber(static_cast<int>(Channel::cch));
#ifdef DEBUG
        EV << "RSUControl::populateWSM MAP " << this->myId << std::endl;
#endif
        wsm->setUserPriority(beaconUserPriority);
        k = 0;
        for (auto it = incomingEdges.cbegin(); it != incomingEdges.cend(); it++)
        {
            map->setIncomingEdges(k, it->c_str());
            k++;
        }
        k = 0;
        for (auto it = outgoingEdges.cbegin(); it != outgoingEdges.cend(); it++)
        {
            map->setOutgoingEdges(k, it->c_str());
            k++;
        }
    }
    else
    if (RSUSpat* spat = dynamic_cast<RSUSpat*>(wsm)) {
        //spat->setMapData(0, 42);
        spat->setChannelNumber(static_cast<int>(Channel::cch));
        spat->setSenderId(this->myId);
        spat->setSenderPos(curPosition);
        spat->setTransactionId(spat->getId()); // Uses the msg's unique id as transaction id
#ifdef DEBUG
        EV << "RSUControl::populateWSM SPAT " << this->myId << std::endl;
#endif
        wsm->setUserPriority(beaconUserPriority);
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(static_cast<int>(Channel::cch));
        wsa->setTargetChannel(static_cast<int>(currentServiceChannel));
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
#ifdef DEBUG
        EV << "RSUControl::populateWSM WSA " << this->myId << std::endl;
#endif
    }
    else {
        if (dataOnSch)
            wsm->setChannelNumber(static_cast<int>(Channel::sch1)); // will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        else
            wsm->setChannelNumber(static_cast<int>(Channel::cch));
        wsm->addBitLength(dataLengthBits);
        wsm->setUserPriority(dataUserPriority);
#ifdef DEBUG
        EV << "RSUControl::populateWSM ELSE " << this->myId << std::endl;
#endif
    }
}

void RSUControl::checkAndAddVehicle(Coord pos, int id)
{
    int found = 0;
#ifdef DEBUG
    EV << "RSUControl::checkAndAddVehicle checking for " << id << std::endl;
#endif

    if (!knownVehicles.empty())
        for (std::vector<Vehicle>::iterator it = knownVehicles.begin() ; it != knownVehicles.end(); ++it)
        {
            if (it->id == id)
            {
#ifdef DEBUG
                EV << id << " is on the list." << std::endl;
#endif
                it->pos = pos;
                found = 1;
            }
        }

    if (found == 0)
    {
        knownVehicles.emplace_back(pos, id);
#ifdef DEBUG
        EV << "Added " << id << std::endl;
#endif
    }
}

void RSUControl::lanesToEdges()
{
    TraCIScenarioManager *manager = TraCIScenarioManagerAccess().get();
    TraCICommandInterface *traci = manager->getCommandInterface();

    std::list<std::list<TraCITrafficLightLink>> tls;
    std::string tempLane;
    std::string delimiter = "_";
    EV << "RSUControl::lanesToEdges" << std::endl;
    for (auto tls_it = controlledTLS.cbegin(); tls_it != controlledTLS.cend(); tls_it++)
        {
            tls = traci->trafficlight(*tls_it).getControlledLinks();
            for (std::list<std::list<TraCITrafficLightLink>>::iterator it = tls.begin() ; it != tls.end(); ++it)
            {
                for (std::list<TraCITrafficLightLink>::iterator it2 = it->begin() ; it2 != it->end(); ++it2)
                {
                    EV << it2->incoming << " " << it2->internal << " " << it2->outgoing << std::endl;
                    tempLane = it2->incoming.substr(0, it2->incoming.find(delimiter));
                    //search list for the edge
                    auto it = std::find(incomingEdges.begin(), incomingEdges.end(), tempLane);
                    if ( it == incomingEdges.end())
                    {
                        incomingEdges.emplace_back(tempLane);
                    }

                    tempLane = it2->outgoing.substr(0, it2->outgoing.find(delimiter));
                    //search list for the edge
                    it = std::find(outgoingEdges.begin(), outgoingEdges.end(), tempLane);
                    if ( it == outgoingEdges.end())
                    {
                        outgoingEdges.emplace_back(tempLane);
                    }
                }
            }
        }
#ifdef DEBUG
       EV << "RSUControl::lanesToEdges incoming ";

       for (auto it2 = incomingEdges.cbegin(); it2 != incomingEdges.cend(); it2++)
       {
           EV << *it2 << " ";
       }
       EV << std::endl;

       EV << "RSUControl::lanesToEdges outgoing ";

       for (auto it3 = outgoingEdges.cbegin(); it3 != outgoingEdges.cend(); it3++)
       {
           EV << *it3 << " ";
       }
       EV << std::endl;
#endif
}

void RSUControl::checkAndAddRequest(int rsuId, std::vector<std::string> route, simtime_t estimatedArrival, int senderId)
{
    int scenario;

#ifdef DEBUG
    EV << "RSUControl::checkAndAddRequest checking for RSU " << rsuId << "at time " << estimatedArrival << std::endl;
#endif

    if (myId != rsuId)
    {
        EV << "Myid: " << myId << " rsuid:" << rsuId << std::endl;
        return;
    }

#ifdef DEBUG
    EV << "received route: ";
    for (auto it4 = route.begin() ; it4 != route.end(); ++it4)
    {
        EV << *it4 << " ";
    }
    EV << std::endl;
#endif


#ifdef DEBUG
    EV << "Printing requests: " << std::endl;
    if (!requests.empty())
        for (auto it = requests.begin() ; it != requests.end(); ++it)
        {
            if (it->receiverId == rsuId)
            {
                EV << "Scenario "  << it->scenario << "at " << it->estimatedArrival << " on RSU " << myId << "from " << senderId << std::endl;
            }
        }
#endif


    if (!requests.empty())
        for (auto it = requests.begin() ; it != requests.end(); ++it)
        {
            if (it->receiverId == rsuId && it->senderId == senderId)
            {
                scenario = translateRequest(route);
                if (scenario != -1)
                {
                    it->estimatedArrival = estimatedArrival;
                    it->route = route;
                    it->scenario = scenario;
                    it->senderId = senderId;
#ifdef DEBUG
                    EV << "Updated scenario " << it->scenario << "at " << it->estimatedArrival << " on RSU " << myId << std::endl;
#endif
                }
                else
                {
                    requests.erase(it);
#ifdef DEBUG
                    EV << "RSUControl::checkAndAddRequest Removed scenario " << it->scenario << "from vehicle " << senderId << std::endl;
#endif
                    if (it->beenActivated)
                       revertToNormal();
                }

                return;
            }
        }

    scenario = translateRequest(route);
    if (scenario != -1)
    {
        requests.emplace_back(rsuId, route, estimatedArrival, scenario, senderId, 0);
#ifdef DEBUG
        EV << "Added scenario " << scenario << "at " << estimatedArrival << " on RSU " << myId << "from " << senderId << std::endl;
#endif
    }

    else
    {
#ifdef DEBUG
        EV << "First request scenario -1" << std::endl;
#endif
    }


}

void RSUControl::updateVehiclePosition(int senderId, double distance)
{
    int scenario;

#ifdef DEBUG
    EV << "RSUControl::checkVehiclePosition checking for vehicle " << senderId << std::endl;
#endif

    if (!requests.empty())
        for (auto it = requests.begin() ; it != requests.end(); ++it)
        {
            if (it->senderId == senderId)
            {
                //if (it->prevDist < it->curDist && it->curDist < distance) //vehicle is getting away
                if ((it->curDist - it->prevDist) > 5.0 && (distance - it->curDist) > 5.0) //vehicle is getting away
                //if (distance > it->curDist) //vehicle is getting away
                {
                    it->gettingAway++;
                    if (it->gettingAway >= 3)
                    {
                        requests.erase(it);
#ifdef DEBUG
                        EV << "RSUControl::updateVehiclePosition Removed scenario " << it->scenario << "from vehicle " << senderId << std::endl;
#endif
                        if (it->beenActivated)
                            revertToNormal();
                    }
                }
                it->prevDist = it->curDist;
                it->curDist = distance;
#ifdef DEBUG
                    EV << "RSUControl::updateVehiclePosition Update distance (" << it->prevDist << "->" << it->curDist << ") from vehicle " << senderId << std::endl;
#endif

                return;
            }
        }

}

int RSUControl::translateRequest(std::vector<std::string> route)
{
    std::list<std::string> tempRoute;
    int k;

    std::vector<std::string>::iterator it_edge1, it_edge2, it_edge3, it_edge4;
    simtime_t estimatedArrival;
#ifdef DEBUG
    EV << "RSUControl::translateRequest() id " << myId << std::endl;
#endif

    //for (auto it = requests.rbegin(); it != requests.rend(); it++)
       {

            //for (auto it2 = it->route.rbegin(); (it2 != it->route.rend()); it2++)
            {
                if (myId == RSU0ID)
                {
                    it_edge1 = std::find (route.begin(), route.end(), "372687453#1");
                    it_edge2 = std::find (route.begin(), route.end(), "372687453#3");
                    it_edge3 = std::find (route.begin(), route.end(), "372687453#4");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) ) //baixo para cima
                        return 1;

                    it_edge1 = std::find (route.begin(), route.end(), "184497382#0");
                    it_edge2 = std::find (route.begin(), route.end(), "184497382#3");
                    it_edge3 = std::find (route.begin(), route.end(), "184497382#2");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) ) //cima para baixo
                        return 2;

                    it_edge1 = std::find (route.begin(), route.end(), "-8244134#2"); //direita para cima
                    it_edge2 = std::find (route.begin(), route.end(), "372687453#3");

                    if (it_edge1 != route.end() && it_edge2 != route.end())
                        return 3;

                    it_edge1 = std::find (route.begin(), route.end(), "372687453#1"); //baixo para direita
                    it_edge2 = std::find (route.begin(), route.end(), "8244134#2");
                    it_edge3 = std::find (route.begin(), route.end(), "8244134#1");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 1; //merged with scenario 1

                    it_edge1 = std::find (route.begin(), route.end(), "372687453#1"); //baixo U
                    it_edge2 = std::find (route.begin(), route.end(), "184497382#2");
                    it_edge3 = std::find (route.begin(), route.end(), "184497382#3");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 4;
                }
                else
                if (myId == RSU1ID)
                {
                    it_edge1 = std::find (route.begin(), route.end(), "184497381#0"); //baixo para cima
                    it_edge2 = std::find (route.begin(), route.end(), "450080910#1");
                    it_edge3 = std::find (route.begin(), route.end(), "450080910#0");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 1;

                    it_edge1 = std::find (route.begin(), route.end(), "184497381#0"); //baixo para esq
                    it_edge2 = std::find (route.begin(), route.end(), "38961111#1");
                    it_edge3 = std::find (route.begin(), route.end(), "38961111#0");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 2;

                    it_edge1 = std::find (route.begin(), route.end(), "184497381#0"); //baixo para dir
                    it_edge2 = std::find (route.begin(), route.end(), "450080909#0");
                    it_edge3 = std::find (route.begin(), route.end(), "450080909#1");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 1;

                    it_edge1 = std::find (route.begin(), route.end(), "184497381#0"); //baixo para baixo U
                    it_edge2 = std::find (route.begin(), route.end(), "95914916#1");
                    it_edge3 = std::find (route.begin(), route.end(), "95914916#0");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 8;

                    it_edge1 = std::find (route.begin(), route.end(), "-450080909#1"); //direita para baixo
                    it_edge2 = std::find (route.begin(), route.end(), "95914916#0");
                    if (it_edge1 != route.end() && it_edge2 != route.end())
                        return 3;

                    it_edge1 = std::find (route.begin(), route.end(), "-450080909#1"); //direita para cima
                    it_edge2 = std::find (route.begin(), route.end(), "450080910#0");
                    if (it_edge1 != route.end() && it_edge2 != route.end())
                        return 9;

                    it_edge1 = std::find (route.begin(), route.end(), "-450080909#1"); //direita para esq
                    it_edge2 = std::find (route.begin(), route.end(), "38961111#0");
                    it_edge3 = std::find (route.begin(), route.end(), "38961111#1");
                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 9;

                    it_edge1 = std::find (route.begin(), route.end(), "95914921#7"); //cima para direita
                    it_edge2 = std::find (route.begin(), route.end(), "450080909#1");
                    it_edge3 = std::find (route.begin(), route.end(), "450080909#0");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 4;

                    it_edge1 = std::find (route.begin(), route.end(), "95914921#7");
                    it_edge2 = std::find (route.begin(), route.end(), "450080910#1"); // u de cima pra cima

                    if (it_edge1 != route.end() && it_edge2 != route.end())
                        return 5;

                    it_edge1 = std::find (route.begin(), route.end(), "95914921#7"); //cima para baixo
                    it_edge2 = std::find (route.begin(), route.end(), "95914916#1");
                    it_edge3 = std::find (route.begin(), route.end(), "95914916#0");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 6;

                    it_edge1 = std::find (route.begin(), route.end(), "95914921#7"); //cima para esq
                    it_edge2 = std::find (route.begin(), route.end(), "38961111#0");
                    it_edge3 = std::find (route.begin(), route.end(), "38961111#1");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 10;

                    it_edge1 = std::find (route.begin(), route.end(), "-38961111#1"); //esquerda para baixo
                    it_edge2 = std::find (route.begin(), route.end(), "95914916#0");

                    if (it_edge1 != route.end() && it_edge2 != route.end())
                        return 7;

                    it_edge1 = std::find (route.begin(), route.end(), "-38961111#1"); //esquerda para cima
                    it_edge2 = std::find (route.begin(), route.end(), "450080910#0");

                    if (it_edge1 != route.end() && it_edge2 != route.end())
                        return 7;

                    it_edge1 = std::find (route.begin(), route.end(), "-38961111#1"); //esquerda para dir
                    it_edge2 = std::find (route.begin(), route.end(), "450080909#0");
                    it_edge3 = std::find (route.begin(), route.end(), "450080909#1");

                    if (it_edge1 != route.end() && it_edge2 != route.end())
                        return 7;
                }
                else
                if (myId == RSU2ID)
                {
                    it_edge1 = std::find (route.begin(), route.end(), "450080910#1"); //baixo para cima
                    it_edge2 = std::find (route.begin(), route.end(), "450080910#4");
                    it_edge3 = std::find (route.begin(), route.end(), "450080910#3");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 1;

                    it_edge1 = std::find (route.begin(), route.end(), "450080910#1"); //de baixo para direita
                    it_edge2 = std::find (route.begin(), route.end(), "45103222#1");
                    it_edge3 = std::find (route.begin(), route.end(), "45103222#0");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 1; //merged with scenario 1

                    it_edge1 = std::find (route.begin(), route.end(), "450080910#1"); //de baixo para esq
                    it_edge2 = std::find (route.begin(), route.end(), "-95914920#5");
                    it_edge3 = std::find (route.begin(), route.end(), "-95914920#6");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 3;

                    it_edge1 = std::find (route.begin(), route.end(), "95914921#4"); //cima para baixo
                    it_edge2 = std::find (route.begin(), route.end(), "95914921#7");
                    it_edge3 = std::find (route.begin(), route.end(), "95914921#6");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 4;

                    it_edge1 = std::find (route.begin(), route.end(), "95914921#4"); //cima para esq
                    it_edge2 = std::find (route.begin(), route.end(), "-95914920#5");
                    it_edge3 = std::find (route.begin(), route.end(), "-95914920#6");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 5;

                    it_edge1 = std::find (route.begin(), route.end(), "95914921#4"); //cima para dir
                    it_edge2 = std::find (route.begin(), route.end(), "45103222#1");
                    it_edge3 = std::find (route.begin(), route.end(), "45103222#0");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 6;

                    it_edge1 = std::find (route.begin(), route.end(), "95914920#5"); //esq para cima
                    it_edge2 = std::find (route.begin(), route.end(), "450080910#4");
                    it_edge3 = std::find (route.begin(), route.end(), "450080910#3");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 7;

                    it_edge1 = std::find (route.begin(), route.end(), "-45103222#1"); //dir para baixo
                    it_edge2 = std::find (route.begin(), route.end(), "95914921#7");
                    it_edge3 = std::find (route.begin(), route.end(), "95914921#6");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 8;

                    it_edge1 = std::find (route.begin(), route.end(), "450080910#1"); //baixo U
                    it_edge2 = std::find (route.begin(), route.end(), "95914921#6");
                    it_edge3 = std::find (route.begin(), route.end(), "95914921#7");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 9;

                    it_edge1 = std::find (route.begin(), route.end(), "95914921#4"); //cima U
                    it_edge2 = std::find (route.begin(), route.end(), "450080910#3");
                    it_edge3 = std::find (route.begin(), route.end(), "450080910#4");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 10;

                    it_edge1 = std::find (route.begin(), route.end(), "95914920#5"); //esq para baixo
                    it_edge2 = std::find (route.begin(), route.end(), "95914921#6");
                    it_edge3 = std::find (route.begin(), route.end(), "95914921#7");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 11;

                    it_edge1 = std::find (route.begin(), route.end(), "95914920#5"); //esq para dir
                    it_edge2 = std::find (route.begin(), route.end(), "45103222#0");
                    it_edge3 = std::find (route.begin(), route.end(), "45103222#1");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 12;

                    it_edge1 = std::find (route.begin(), route.end(), "-45103222#1"); //dir para cima
                    it_edge2 = std::find (route.begin(), route.end(), "450080910#3");
                    it_edge3 = std::find (route.begin(), route.end(), "450080910#4");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 13;

                    it_edge1 = std::find (route.begin(), route.end(), "-45103222#1"); //dir para esq
                    it_edge2 = std::find (route.begin(), route.end(), "-95914920#6");
                    it_edge3 = std::find (route.begin(), route.end(), "-95914920#5");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 14;
                }
                else
                if (myId == RSU3ID)
                {
                    it_edge1 = std::find (route.begin(), route.end(), "450080910#4"); //baixo para cima
                    it_edge2 = std::find (route.begin(), route.end(), "450080910#6");
                    it_edge3 = std::find (route.begin(), route.end(), "450080910#7");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 1;

                    it_edge1 = std::find (route.begin(), route.end(), "95914921#1"); //cima para baixo
                    it_edge2 = std::find (route.begin(), route.end(), "95914921#4");
                    it_edge3 = std::find (route.begin(), route.end(), "95914921#3");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 2;

                    it_edge1 = std::find (route.begin(), route.end(), "450080910#4"); //baixo para direita
                    it_edge2 = std::find (route.begin(), route.end(), "-45103225#13");
                    it_edge3 = std::find (route.begin(), route.end(), "-45103225#14");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 3;

                    it_edge1 = std::find (route.begin(), route.end(), "450080910#4"); //baixo para esq
                    it_edge2 = std::find (route.begin(), route.end(), "43835122#2");
                    it_edge3 = std::find (route.begin(), route.end(), "43835122#1");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 4;

                    it_edge1 = std::find (route.begin(), route.end(), "450080910#4"); //baixo u
                    it_edge2 = std::find (route.begin(), route.end(), "95914921#3");
                    it_edge3 = std::find (route.begin(), route.end(), "95914921#4");

                    if (it_edge1 != route.end() && (it_edge2 != route.end() || it_edge3 != route.end()) )
                        return 5;
                }


            }




            /*if (attendedScenario != 0)
            {
                checkAndSchedule(attendedScenario, estimatedArrival);
            }*/
       }
    return -1;
}

void RSUControl::attendRequest()
{
    std::list<std::string> tempRoute;
    int k;
    bool preemptionToOccur = false;
    std::vector<std::string>::iterator it_edge1, it_edge2, it_edge3, it_edge4;
    simtime_t estimatedArrival;

#ifdef DEBUG
    EV << "RSUControl::attendRequest() id " << myId << std::endl;
#endif

    for (auto it = requests.rbegin(); it != requests.rend(); it++)
       {
#ifdef DEBUG
            EV << "RSU " << myId << " scheduled scenario " << it->scenario << " from " << it->estimatedArrival << std::endl;
#endif
            if (simTime() > (0) && simTime() < (it->estimatedArrival+timeThreshold) && it->scenario > 0)
            {
#ifdef DEBUG
                EV << "RSU " << myId << " activating scenario " << it->scenario << " from " << it->estimatedArrival << "at " << simTime() << std::endl;
#endif
                it->beenActivated = true;
                translateScenario(myId, it->scenario);
                preemptionToOccur = true;
                beenPreempted = true;
            }
       }

    if (beenPreempted == true && preemptionToOccur == false) //returns RSU to normal operation
    {
        beenPreempted = false;
        revertToNormal();
    }
}

void RSUControl::revertToNormal()
{
    TraCIScenarioManager *manager = TraCIScenarioManagerAccess().get();
    TraCICommandInterface *traci = manager->getCommandInterface();

#ifdef DEBUG
    EV << "Reverting RSU " << myId << " back to normal operation." << std::endl;
#endif
    if (myId == RSU0ID)
    {
        traci->trafficlight("655189922").setPhaseIndex(0);
        traci->trafficlight("604969891").setPhaseIndex(0);
        traci->trafficlight("655189917").setPhaseIndex(0);
        traci->trafficlight("655189915").setPhaseIndex(0);
        traci->trafficlight("604969891").setPhaseIndex(0);
        traci->trafficlight("655189920").setPhaseIndex(0);
    }
    else
    if (myId == RSU1ID)
    {
        traci->trafficlight("1666024484").setPhaseIndex(0);
        traci->trafficlight("606181027").setPhaseIndex(0);
        traci->trafficlight("605008047").setPhaseIndex(0);
        traci->trafficlight("1666024479").setPhaseIndex(0);
        traci->trafficlight("604969890").setPhaseIndex(0);
        traci->trafficlight("605008045").setPhaseIndex(0);
    }
    else
    if (myId == RSU2ID)
    {
        traci->trafficlight("1111343446").setPhaseIndex(0);
        traci->trafficlight("1111343417").setPhaseIndex(0);
        traci->trafficlight("606181023").setPhaseIndex(0);
        traci->trafficlight("1111343376").setPhaseIndex(0);
        traci->trafficlight("1111343438").setPhaseIndex(0);
        traci->trafficlight("266205952").setPhaseIndex(0);
    }
    else
    if (myId == RSU3ID)
    {
        traci->trafficlight("1949937054").setPhaseIndex(0);
        traci->trafficlight("1662318224").setPhaseIndex(0);
        traci->trafficlight("1949937055").setPhaseIndex(0);
        traci->trafficlight("1662318221").setPhaseIndex(0);
        traci->trafficlight("609788279").setPhaseIndex(0);
        traci->trafficlight("606181005").setPhaseIndex(0);
    }
}

void RSUControl::translateScenario(int rsuId, int scenario)
{
    TraCIScenarioManager *manager = TraCIScenarioManagerAccess().get();
    TraCICommandInterface *traci = manager->getCommandInterface();



    if (rsuId == RSU0ID)
    {
        switch(scenario)
        {
            case 1:
                if (traci->trafficlight("655189922").getCurrentState() != "GG")
                    traci->trafficlight("655189922").setPhaseIndex(0);
                else
                    setTLSMinTime("655189922");

                if (traci->trafficlight("604969891").getCurrentState() != "rrrrrrGgg")
                    traci->trafficlight("604969891").setPhaseIndex(4);
                else
                    setTLSMinTime("604969891");

                if (traci->trafficlight("655189917").getCurrentState() != "GG")
                    traci->trafficlight("655189917").setPhaseIndex(0);
                else
                    setTLSMinTime("655189917");
                //traci->trafficlight("655189922").setPhaseIndex(2);
                //traci->trafficlight("655189922").setState("uu");
            break;

            case 2:
                if (traci->trafficlight("655189915").getCurrentState() != "GG")
                    traci->trafficlight("655189915").setPhaseIndex(0);
                else
                    setTLSMinTime("655189915");

                if (traci->trafficlight("604969891").getCurrentState() != "GGgrrrGgg")
                    traci->trafficlight("604969891").setPhaseIndex(0);
                else
                    setTLSMinTime("604969891");

                if (traci->trafficlight("655189920").getCurrentState() != "GG")
                    traci->trafficlight("655189920").setPhaseIndex(0);
                else
                    setTLSMinTime("655189920");
                //traci->trafficlight("655189922").setState("uu");
            break;

            case 3:
                if (traci->trafficlight("604969891").getCurrentState() != "GGgrrrGgg")
                    traci->trafficlight("604969891").setPhaseIndex(0);
                else
                    setTLSMinTime("604969891");

                if (traci->trafficlight("655189922").getCurrentState() != "rr")
                    traci->trafficlight("655189922").setPhaseIndex(2);
                else
                    setTLSMinTime("655189922");

                if (traci->trafficlight("655189917").getCurrentState() != "GG")
                    traci->trafficlight("655189917").setPhaseIndex(0);
                else
                    setTLSMinTime("655189917");
            break;

            case 4:
                if (traci->trafficlight("655189922").getCurrentState() != "GG")
                    traci->trafficlight("655189922").setPhaseIndex(0);

                else
                    setTLSMinTime("655189922");

                if (traci->trafficlight("604969891").getCurrentState() != "GGgrrrGgg")
                    traci->trafficlight("604969891").setPhaseIndex(0);
                else
                    setTLSMinTime("604969891");

                if (traci->trafficlight("655189917").getCurrentState() != "GG")
                    traci->trafficlight("655189917").setPhaseIndex(0);
                else
                    setTLSMinTime("655189917");

                if (traci->trafficlight("655189915").getCurrentState() != "rr")
                    traci->trafficlight("655189915").setPhaseIndex(2);
                else
                    setTLSMinTime("655189915");

                if (traci->trafficlight("655189920").getCurrentState() != "GG")
                    traci->trafficlight("655189920").setPhaseIndex(0);
                else
                    setTLSMinTime("655189920");
            break;

            case 20000: //test scenario - doesnt happen in real simulations
                 traci->trafficlight("655189922").setState("OO");
                 //traci->trafficlight("655189920").setState("GG");
            break;

            case 20001: //test scenario - doesnt happen in real simulations
                 //traci->trafficlight("655189922").setState("GG");
                 traci->trafficlight("655189920").setState("OO");
            break;
        }
    }
    else
    if (rsuId == RSU1ID)
        {
            switch(scenario)
            {
               case 1: // baixo para cima
                    if (traci->trafficlight("605008045").getCurrentState() != "rGG")
                        traci->trafficlight("605008045").setPhaseIndex(2);
                    else
                        setTLSMinTime("605008045");

                    if (traci->trafficlight("1666024484").getCurrentState() != "GGr")
                        traci->trafficlight("1666024484").setPhaseIndex(0);
                    else
                        setTLSMinTime("1666024484");

                    if (traci->trafficlight("604969890").getCurrentState() != "rrrrrrGgg")
                        traci->trafficlight("604969890").setPhaseIndex(4);
                    else
                        setTLSMinTime("604969890");

                    if (traci->trafficlight("606181027").getCurrentState() != "Gggrrrrrr")
                        traci->trafficlight("606181027").setPhaseIndex(4);
                    else
                        setTLSMinTime("606181027");

                    if (traci->trafficlight("605008047").getCurrentState() != "rGG")
                        traci->trafficlight("605008047").setPhaseIndex(2);
                    else
                        setTLSMinTime("605008047");
                break;

               case 2: //baixo para esq
                    if (traci->trafficlight("605008045").getCurrentState() != "rGG")
                        traci->trafficlight("605008045").setPhaseIndex(2);
                    else
                        setTLSMinTime("605008045");

                    if (traci->trafficlight("606181027").getCurrentState() != "Gggrrrrrr")
                        traci->trafficlight("606181027").setPhaseIndex(4);
                    else
                        setTLSMinTime("606181027");

                    if (traci->trafficlight("1666024484").getCurrentState() != "GGr")
                        traci->trafficlight("1666024484").setPhaseIndex(0);
                    else
                        setTLSMinTime("1666024484");

                    if (traci->trafficlight("604969890").getCurrentState() != "rrrrrrGgg")
                        traci->trafficlight("604969890").setPhaseIndex(4);
                    else
                        setTLSMinTime("604969890");

                    if (traci->trafficlight("1666024479").getCurrentState() != "Grr")
                      traci->trafficlight("1666024479").setPhaseIndex(0);
                  else
                      setTLSMinTime("1666024479");
                break;

               case 3:
                    if (traci->trafficlight("604969890").getCurrentState() != "GGgrrrGgg")
                        traci->trafficlight("604969890").setPhaseIndex(0);
                    else
                        setTLSMinTime("604969890");

                    if (traci->trafficlight("605008047").getCurrentState() != "rGG")
                        traci->trafficlight("605008047").setPhaseIndex(2);
                    else
                        setTLSMinTime("605008047");

                    if (traci->trafficlight("1666024484").getCurrentState() != "GGr")
                        traci->trafficlight("1666024484").setPhaseIndex(0);
                    else
                        setTLSMinTime("1666024484");

                    if (traci->trafficlight("1666024479").getCurrentState() != "Grr")
                      traci->trafficlight("1666024479").setPhaseIndex(0);
                    else
                      setTLSMinTime("1666024479");

                    if (traci->trafficlight("606181027").getCurrentState() != "GggGGgrrr")
                        traci->trafficlight("606181027").setPhaseIndex(0);
                    else
                        setTLSMinTime("606181027");
                break;

                case 4: //cima para direita
                    if (traci->trafficlight("1666024479").getCurrentState() != "rGG")
                        traci->trafficlight("1666024479").setPhaseIndex(2);
                    else
                        setTLSMinTime("1666024479");

                    if (traci->trafficlight("604969890").getCurrentState() != "rrrrrrGgg")
                        traci->trafficlight("604969890").setPhaseIndex(4);
                    else
                        setTLSMinTime("604969890");

                    if (traci->trafficlight("605008045").getCurrentState() != "Grr")
                        traci->trafficlight("605008045").setPhaseIndex(0);
                    else
                        setTLSMinTime("605008045");

                    if (traci->trafficlight("1666024484").getCurrentState() != "GGr")
                        traci->trafficlight("1666024484").setPhaseIndex(0);
                    else
                        setTLSMinTime("1666024484");

                    if (traci->trafficlight("606181027").getCurrentState() != "Gggrrrrrr")
                        traci->trafficlight("606181027").setPhaseIndex(4);
                    else
                        setTLSMinTime("606181027");
                break;

               case 5: // u de cima pra cima
                    if (traci->trafficlight("1666024479").getCurrentState() != "rGG")
                        traci->trafficlight("1666024479").setPhaseIndex(2);
                    else
                        setTLSMinTime("1666024479");

                    if (traci->trafficlight("1666024484").getCurrentState() != "GGr")
                        traci->trafficlight("1666024484").setPhaseIndex(0);
                    else
                        setTLSMinTime("1666024484");

                    if (traci->trafficlight("605008045").getCurrentState() != "Grr")
                        traci->trafficlight("605008045").setPhaseIndex(0);
                    else
                        setTLSMinTime("605008045");

                    if (traci->trafficlight("604969890").getCurrentState() != "rrrrrrGgg")
                        traci->trafficlight("604969890").setPhaseIndex(4);
                    else
                        setTLSMinTime("604969890");

                    if (traci->trafficlight("606181027").getCurrentState() != "Gggrrrrrr")
                        traci->trafficlight("606181027").setPhaseIndex(4);
                    else
                        setTLSMinTime("606181027");
                break;

               case 6:
                   if (traci->trafficlight("1666024479").getCurrentState() != "rGG")
                       traci->trafficlight("1666024479").setPhaseIndex(2);
                   else
                       setTLSMinTime("1666024479");

                   if (traci->trafficlight("605008047").getCurrentState() != "rGG")
                       traci->trafficlight("605008047").setPhaseIndex(2);
                   else
                       setTLSMinTime("605008047");

                   if (traci->trafficlight("604969890").getCurrentState() != "GGgrrrGgg")
                       traci->trafficlight("604969890").setPhaseIndex(0);
                   else
                       setTLSMinTime("604969890");

                   if (traci->trafficlight("606181027").getCurrentState() != "Gggrrrrrr")
                       traci->trafficlight("606181027").setPhaseIndex(4);
                   else
                       setTLSMinTime("606181027");
               break;

               case 7: //esq para cima, esq para baixo, esq para dir
                  if (traci->trafficlight("605008045").getCurrentState() != "Grr")
                      traci->trafficlight("605008045").setPhaseIndex(0);
                  else
                      setTLSMinTime("605008045");

                  if (traci->trafficlight("1666024484").getCurrentState() != "GGr")
                      traci->trafficlight("1666024484").setPhaseIndex(0);
                  else
                      setTLSMinTime("1666024484");

                  if (traci->trafficlight("606181027").getCurrentState() != "GggGGgrrr")
                      traci->trafficlight("606181027").setPhaseIndex(0);
                  else
                      setTLSMinTime("606181027");

                  if (traci->trafficlight("1666024479").getCurrentState() != "Grr")
                      traci->trafficlight("1666024479").setPhaseIndex(0);
                  else
                      setTLSMinTime("1666024479");

                  if (traci->trafficlight("605008047").getCurrentState() != "rGG")
                     traci->trafficlight("605008047").setPhaseIndex(2);
                 else
                     setTLSMinTime("605008047");

                  if (traci->trafficlight("604969890").getCurrentState() != "rrrrrrGgg")
                      traci->trafficlight("604969890").setPhaseIndex(4);
                  else
                      setTLSMinTime("604969890");
              break;

               case 8: //u baixo para baixo
                   if (traci->trafficlight("605008045").getCurrentState() != "rGG")
                       traci->trafficlight("605008045").setPhaseIndex(2);
                   else
                       setTLSMinTime("605008045");

                   if (traci->trafficlight("606181027").getCurrentState() != "Gggrrrrrr")
                       traci->trafficlight("606181027").setPhaseIndex(4);
                   else
                       setTLSMinTime("606181027");

                   if (traci->trafficlight("1666024484").getCurrentState() != "GGr")
                       traci->trafficlight("1666024484").setPhaseIndex(0);
                   else
                       setTLSMinTime("1666024484");

                   if (traci->trafficlight("604969890").getCurrentState() != "rrrrrrGgg")
                       traci->trafficlight("604969890").setPhaseIndex(4);
                   else
                       setTLSMinTime("604969890");

                   if (traci->trafficlight("1666024479").getCurrentState() != "Grr")
                    traci->trafficlight("1666024479").setPhaseIndex(0);
                   else
                    setTLSMinTime("1666024479");

                   if (traci->trafficlight("605008047").getCurrentState() != "rGG")
                    traci->trafficlight("605008047").setPhaseIndex(2);
                   else
                    setTLSMinTime("605008047");

               break;

              case 9: //direita para cima, direita para esq
                  if (traci->trafficlight("605008045").getCurrentState() != "Grr")
                      traci->trafficlight("605008045").setPhaseIndex(0);
                  else
                      setTLSMinTime("605008045");

                  if (traci->trafficlight("606181027").getCurrentState() != "Gggrrrrrr")
                      traci->trafficlight("606181027").setPhaseIndex(4);
                  else
                      setTLSMinTime("606181027");

                  if (traci->trafficlight("1666024484").getCurrentState() != "GGr")
                      traci->trafficlight("1666024484").setPhaseIndex(0);
                  else
                      setTLSMinTime("1666024484");

                  if (traci->trafficlight("604969890").getCurrentState() != "GGgrrrGgg")
                      traci->trafficlight("604969890").setPhaseIndex(0);
                  else
                      setTLSMinTime("604969890");

                  if (traci->trafficlight("1666024479").getCurrentState() != "Grr")
                   traci->trafficlight("1666024479").setPhaseIndex(0);
                  else
                   setTLSMinTime("1666024479");

                  if (traci->trafficlight("605008047").getCurrentState() != "rGG")
                   traci->trafficlight("605008047").setPhaseIndex(2);
                  else
                   setTLSMinTime("605008047");
              break;

              case 10: //cima para esq
                 if (traci->trafficlight("1666024479").getCurrentState() != "rGG")
                     traci->trafficlight("1666024479").setPhaseIndex(2);
                 else
                     setTLSMinTime("1666024479");

                 if (traci->trafficlight("605008047").getCurrentState() != "rGG")
                     traci->trafficlight("605008047").setPhaseIndex(2);
                 else
                     setTLSMinTime("605008047");

                 if (traci->trafficlight("606181027").getCurrentState() != "Gggrrrrrr")
                     traci->trafficlight("606181027").setPhaseIndex(4);
                 else
                     setTLSMinTime("606181027");
             break;

              case 11: //esq para dir
                    if (traci->trafficlight("605008045").getCurrentState() != "Grr")
                        traci->trafficlight("605008045").setPhaseIndex(0);
                    else
                        setTLSMinTime("605008045");

                    if (traci->trafficlight("606181027").getCurrentState() != "GggGGgrrr")
                        traci->trafficlight("606181027").setPhaseIndex(0);
                    else
                        setTLSMinTime("606181027");

                    if (traci->trafficlight("1666024484").getCurrentState() != "GGr")
                        traci->trafficlight("1666024484").setPhaseIndex(0);
                    else
                        setTLSMinTime("1666024484");

                    if (traci->trafficlight("604969890").getCurrentState() != "GGgrrrGgg")
                        traci->trafficlight("604969890").setPhaseIndex(0);
                    else
                        setTLSMinTime("604969890");

                    if (traci->trafficlight("1666024479").getCurrentState() != "Grr")
                     traci->trafficlight("1666024479").setPhaseIndex(0);
                    else
                     setTLSMinTime("1666024479");

                    if (traci->trafficlight("605008047").getCurrentState() != "rGG")
                     traci->trafficlight("605008047").setPhaseIndex(2);
                    else
                     setTLSMinTime("605008047");
                break;

            }
        }
    else
    if (rsuId == RSU2ID)
        {
#ifdef DEBUG
        EV << "1111343446 state:" << traci->trafficlight("1111343446").getCurrentState() << "phase=" << traci->trafficlight("1111343446").getCurrentPhaseIndex() << std::endl;
        EV << "1111343417 state:" << traci->trafficlight("1111343417").getCurrentState() << "phase=" << traci->trafficlight("1111343417").getCurrentPhaseIndex() << std::endl;
        EV << "606181023 state:" << traci->trafficlight("606181023").getCurrentState() << "phase=" << traci->trafficlight("606181023").getCurrentPhaseIndex() << std::endl;
        EV << "1111343376 state:" << traci->trafficlight("1111343376").getCurrentState() << "phase=" << traci->trafficlight("1111343376").getCurrentPhaseIndex() << std::endl;
        EV << "1111343438 state:" << traci->trafficlight("1111343438").getCurrentState() << "phase=" << traci->trafficlight("1111343438").getCurrentPhaseIndex() << std::endl;
        EV << "266205952 state:" << traci->trafficlight("266205952").getCurrentState() << "phase=" << traci->trafficlight("266205952").getCurrentPhaseIndex() << std::endl;
#endif
            switch(scenario)
            {
                case 1: //baixo para cima
                case 3: //de baixo para esq
                    if (traci->trafficlight("1111343446").getCurrentState() != "GGr")
                        traci->trafficlight("1111343446").setPhaseIndex(0);
                    else
                        setTLSMinTime("1111343446");

                    if (traci->trafficlight("1111343376").getCurrentState() != "Grr")
                        traci->trafficlight("1111343376").setPhaseIndex(0);
                    else
                        setTLSMinTime("1111343376");

                    if (traci->trafficlight("1111343417").getCurrentState() != "rGG")
                        traci->trafficlight("1111343417").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343417");

                    if (traci->trafficlight("606181023").getCurrentState() != "rrrrrrGgg")
                        traci->trafficlight("606181023").setPhaseIndex(4);
                    else
                        setTLSMinTime("606181023");

                    if (traci->trafficlight("1111343438").getCurrentState() != "rGG")
                        traci->trafficlight("1111343438").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343438");

                    if (traci->trafficlight("266205952").getCurrentState() != "Gggrrrrrr")
                        traci->trafficlight("266205952").setPhaseIndex(4);
                    else
                        setTLSMinTime("266205952");
                break;

                case 4: //cima para baixo
                    if (traci->trafficlight("1111343376").getCurrentState() != "rGG")
                        traci->trafficlight("1111343376").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343376");

                    if (traci->trafficlight("1111343438").getCurrentState() != "rGG")
                        traci->trafficlight("1111343438").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343438");

                    if (traci->trafficlight("266205952").getCurrentState() != "Gggrrrrrr")
                        traci->trafficlight("266205952").setPhaseIndex(4);
                    else
                        setTLSMinTime("266205952");
                break;

                case 5: //cima para esq
                    if (traci->trafficlight("1111343376").getCurrentState() != "rGG")
                        traci->trafficlight("1111343376").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343376");

                    if (traci->trafficlight("266205952").getCurrentState() != "GggGGgrrr")
                        traci->trafficlight("266205952").setPhaseIndex(0);
                    else
                        setTLSMinTime("266205952");

                    if (traci->trafficlight("1111343438").getCurrentState() != "rGG")
                        traci->trafficlight("1111343438").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343438");
                break;

                case 6: //cima para dir
                    if (traci->trafficlight("1111343376").getCurrentState() != "rGG")
                        traci->trafficlight("1111343376").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343376");

                    if (traci->trafficlight("1111343446").getCurrentState() != "rrG")
                        traci->trafficlight("1111343446").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343446");

                    if (traci->trafficlight("606181023").getCurrentState() != "rrrrrrGgg")
                        traci->trafficlight("606181023").setPhaseIndex(4);
                    else
                        setTLSMinTime("606181023");

                    if (traci->trafficlight("1111343417").getCurrentState() != "rGG")
                        traci->trafficlight("1111343417").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343417");

                    if (traci->trafficlight("266205952").getCurrentState() != "Gggrrrrrr")
                        traci->trafficlight("266205952").setPhaseIndex(4);
                    else
                        setTLSMinTime("266205952");

                    if (traci->trafficlight("1111343438").getCurrentState() != "rGG")
                        traci->trafficlight("1111343438").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343438");
                break;

                case 7: //esq para cima
                case 11: //esq para baixo
                case 12: //esq para dir
                    if (traci->trafficlight("266205952").getCurrentState() != "GggGGgrrr")
                        traci->trafficlight("266205952").setPhaseIndex(0);
                    else
                        setTLSMinTime("266205952");

                    if (traci->trafficlight("1111343376").getCurrentState() != "Grr")
                        traci->trafficlight("1111343376").setPhaseIndex(0);
                    else
                        setTLSMinTime("1111343376");

                    if (traci->trafficlight("1111343417").getCurrentState() != "rGG")
                        traci->trafficlight("1111343417").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343417");

                    if (traci->trafficlight("1111343446").getCurrentState() != "rrG")
                        traci->trafficlight("1111343446").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343446");

                    if (traci->trafficlight("1111343438").getCurrentState() != "rGG")
                        traci->trafficlight("1111343438").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343438");

                    if (traci->trafficlight("606181023").getCurrentState() != "rrrrrrGgg")
                        traci->trafficlight("606181023").setPhaseIndex(4);
                    else
                        setTLSMinTime("606181023");
                break;

                case 8: //dir para baixo
                case 13: //dir para cima
                case 14: //dir para esq
                    if (traci->trafficlight("606181023").getCurrentState() != "GGgrrrGgg")
                        traci->trafficlight("606181023").setPhaseIndex(0);
                    else
                        setTLSMinTime("606181023");

                    if (traci->trafficlight("1111343446").getCurrentState() != "rrG")
                        traci->trafficlight("1111343446").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343446");

                    if (traci->trafficlight("1111343376").getCurrentState() != "Grr")
                        traci->trafficlight("1111343376").setPhaseIndex(0);
                    else
                        setTLSMinTime("1111343376");

                    if (traci->trafficlight("1111343438").getCurrentState() != "rGG")
                        traci->trafficlight("1111343438").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343438");

                    if (traci->trafficlight("266205952").getCurrentState() != "Gggrrrrrr")
                        traci->trafficlight("266205952").setPhaseIndex(4);
                    else
                        setTLSMinTime("266205952");

                    if (traci->trafficlight("1111343417").getCurrentState() != "rGG")
                        traci->trafficlight("1111343417").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343417");
                break;

                case 9: //baixo u
                    if (traci->trafficlight("266205952").getCurrentState() != "Gggrrrrrr")
                        traci->trafficlight("266205952").setPhaseIndex(4);
                    else
                        setTLSMinTime("266205952");

                    if (traci->trafficlight("1111343376").getCurrentState() != "Grr")
                        traci->trafficlight("1111343376").setPhaseIndex(0);
                    else
                        setTLSMinTime("1111343376");

                    if (traci->trafficlight("1111343417").getCurrentState() != "rGG")
                        traci->trafficlight("1111343417").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343417");

                    if (traci->trafficlight("1111343446").getCurrentState() != "GGr")
                        traci->trafficlight("1111343446").setPhaseIndex(0);
                    else
                        setTLSMinTime("1111343446");

                    if (traci->trafficlight("1111343438").getCurrentState() != "rGG")
                        traci->trafficlight("1111343438").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343438");

                    if (traci->trafficlight("606181023").getCurrentState() != "rrrrrrGgg")
                        traci->trafficlight("606181023").setPhaseIndex(4);
                    else
                        setTLSMinTime("606181023");
                break;

                case 10: //cima u
                    if (traci->trafficlight("1111343376").getCurrentState() != "rGG")
                        traci->trafficlight("1111343376").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343376");

                    if (traci->trafficlight("1111343446").getCurrentState() != "rrG")
                        traci->trafficlight("1111343446").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343446");

                    if (traci->trafficlight("606181023").getCurrentState() != "rrrrrrGgg")
                        traci->trafficlight("606181023").setPhaseIndex(4);
                    else
                        setTLSMinTime("606181023");

                    if (traci->trafficlight("1111343417").getCurrentState() != "rGG")
                        traci->trafficlight("1111343417").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343417");

                    if (traci->trafficlight("266205952").getCurrentState() != "Gggrrrrrr")
                        traci->trafficlight("266205952").setPhaseIndex(4);
                    else
                        setTLSMinTime("266205952");

                    if (traci->trafficlight("1111343438").getCurrentState() != "rGG")
                        traci->trafficlight("1111343438").setPhaseIndex(2);
                    else
                        setTLSMinTime("1111343438");
                break;

            }
        }
    else
    if (rsuId == RSU3ID)
        {
            switch(scenario)
            {
                case 1: //baixo para cima
                case 3: //baixo para dir
                case 4: //baixo para esq
                case 5: //baixo u
                    if (traci->trafficlight("1949937054").getCurrentState() != "GG")
                        traci->trafficlight("1949937054").setPhaseIndex(0);
                    else
                        setTLSMinTime("1949937054");

                    if (traci->trafficlight("1662318224").getCurrentState() != "GGr")
                        traci->trafficlight("1662318224").setPhaseIndex(0);
                    else
                        setTLSMinTime("1662318224");

                    if (traci->trafficlight("606181005").getCurrentState() != "rrrrrrGgg")
                        traci->trafficlight("606181005").setPhaseIndex(4);
                    else
                        setTLSMinTime("606181005");

                    if (traci->trafficlight("1662318221").getCurrentState() != "rGG")
                        traci->trafficlight("1662318221").setPhaseIndex(2);
                    else
                        setTLSMinTime("1662318221");

                    if (traci->trafficlight("609788279").getCurrentState() != "Gggrrrrrr")
                        traci->trafficlight("609788279").setPhaseIndex(4);
                    else
                        setTLSMinTime("609788279");

                    if (traci->trafficlight("1949937055").getCurrentState() != "rr")
                        traci->trafficlight("1949937055").setPhaseIndex(2);
                    else
                        setTLSMinTime("1949937055");
                break;

                case 2: //cima para baixo
                    if (traci->trafficlight("1949937055").getCurrentState() != "GG")
                        traci->trafficlight("1949937055").setPhaseIndex(0);
                    else
                        setTLSMinTime("1949937055");

                    if (traci->trafficlight("1662318221").getCurrentState() != "rGG")
                        traci->trafficlight("1662318221").setPhaseIndex(2);
                    else
                        setTLSMinTime("1662318221");

                    if (traci->trafficlight("609788279").getCurrentState() != "Gggrrrrrr")
                        traci->trafficlight("609788279").setPhaseIndex(4);
                    else
                        setTLSMinTime("609788279");
                break;



            }
        }
}

void RSUControl::setTLSMinTime(std::string tls)
{
    TraCIScenarioManager *manager = TraCIScenarioManagerAccess().get();
    TraCICommandInterface *traci = manager->getCommandInterface();
#ifdef DEBUG
    EV << "simtime:" << simTime() << " NextSwitch:" << traci->trafficlight(tls).getAssumedNextSwitchTime() << " Result:" << (traci->trafficlight(tls).getAssumedNextSwitchTime() - simTime()) << std::endl;
#endif
    if ((traci->trafficlight(tls).getAssumedNextSwitchTime() - simTime()) < timeThreshold)
    {
       traci->trafficlight(tls).setPhaseDuration(timeThreshold);
    }
}

/*
void RSUControl::checkAndSchedule(int scenario, simtime_t estimatedArrival)
{
    bool timeSlotFree = true;
    simtime_t begin = estimatedArrival - timeThreshold;
    simtime_t end = estimatedArrival + timeThreshold;

    EV << "RSUControl::checkAndSchedule on RSU: " << myId << " Scenario: " << scenario << " Time: " << estimatedArrival << std::endl;

    for (auto it = schedule.rbegin(); it != schedule.rend() && timeSlotFree == true; it++)
    {
        if (it->begin < begin && it->end > begin && it->end < end) //event overlap 1
            timeSlotFree = false;

        if (it->begin > begin && it->begin < end && it->end > end) //event overlap 1
            timeSlotFree = false;
    }

    if (timeSlotFree == true)
    {
        schedule.emplace_back(begin, end, scenario);
    }
}*/

void RSUControl::finish()
{
    cancelAndDelete(timer);
}
