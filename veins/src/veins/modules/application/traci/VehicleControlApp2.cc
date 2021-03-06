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

#include "veins/modules/application/traci/VehicleControlApp2.h"
//#define DEBUG
#define HOWMANYEVS 4

using namespace veins;

Define_Module(veins::VehicleControlApp2);

void VehicleControlApp2::initialize(int stage)
{
    int idDebug;

    SafeSmartApplLayer::initialize(stage);
    if (stage == 0) {
        // Initializing members and pointers of your application goes here
        numBSMVector.setName("Packet Delay");
        /*timeToArriveRSU0.setName("Time to Arrive RSU 0");
        timeToArriveRSU1.setName("Time to Arrive RSU 1");
        timeToArriveRSU2.setName("Time to Arrive RSU 2");
        timeToArriveRSU3.setName("Time to Arrive RSU 3");
        timeToArriveRSU4.setName("Time to Arrive RSU 4");*/

        /*timeToArriveRSU140.setName("Time to Arrive RSU 1-40");
        timeToArriveRSU1100.setName("Time to Arrive RSU 1-100");
        timeToArriveRSU1200.setName("Time to Arrive RSU 1-200");
        timeToArriveRSU1300.setName("Time to Arrive RSU 1-300");
        timeToArriveRSU1400.setName("Time to Arrive RSU 1-400");*/
        /*distanceRSU0.setName("Dist to RSU 0");
        distanceRSU1.setName("Dist to RSU 1");
        distanceRSU2.setName("Dist to RSU 2");
        distanceRSU3.setName("Dist to RSU 3");
        distanceRSU4.setName("Dist to RSU 4");*/
        /*averageSpeedVector40.setName("Average Speed 40");
        averageSpeedVector100.setName("Average Speed 100");
        averageSpeedVector200.setName("Average Speed 200");
        averageSpeedVector300.setName("Average Speed 300");
        averageSpeedVector400.setName("Average Speed 400");*/
        ttcVector.setName("TTC");

        //KalmanSpeedVector.setName("Kalman Average Speed");
        //distanceVector.setName("Distance Vector");

        mobility = TraCIMobilityAccess().get(getParentModule());
        currentSubscribedServiceId = -1;
        currentOfferedServiceId = 7;

        //wsaInterval = 5;
        beaconInterval = 0.1;
        averageSpeed = 0.0;
        averageSpeed40 = 0.0;
        averageSpeed100 = 0.0;
        averageSpeed200 = 0.0;
        averageSpeed300 = 0.0;
        averageSpeed400 = 0.0;
        kalmanSpeed = 0.0;
        averageSpeedTerms = 0;
        safetyMetricTerms = 0;
        safetyMetricAverage = 0.0;

        TET_1s = TET_2s = TET_3s = TET_4s = TET_5s = TIT = 0.0;

        relevantEdges = 0;
        subscribed = false;
        timer = new cMessage("timer");
        timerInterval = 0.1;
        hasRecordedStats = false;
    }
    else if (stage == 1) {
        // Initializing members that require initialized other modules goes here

        if (traci->vehicle(mobility->getExternalId()).getTypeId() == "emergency")
        {
            routeId = traci->vehicle(mobility->getExternalId()).getRouteId();
            routeEdges = traci->route(routeId).getRoadIds();
            remainingEdges = routeEdges;
            mobility->setIsEV(1);

            SafeSmartApplLayer::isEV = 1;

            EV << "Vehicle ID" << myId << std::endl;

            // started service and server advertising, schedule message to self to send later
            scheduleAt(computeAsynchronousSendingTime(beaconInterval, ChannelType::control), sendBeaconEvt);
        }
        else
        if (traci->vehicle(mobility->getExternalId()).getTypeId() == "emergencyo")
        {
            scheduleAt(simTime() + timerInterval, timer); // timer for SRM processing
        }
    }
    numBSM = 0;
}

void VehicleControlApp2::finish()
{
    SafeSmartApplLayer::finish();

    //EV << "finishstart" << std::endl;
    // statistics recording goes here
    if (numBSM != 0 && hasRecordedStats < HOWMANYEVS)
    {
       hasRecordedStats++;
       recordScalar("numBSM", numBSM);
       recordScalar("TET_1s", TET_1s);
       recordScalar("TET_2s", TET_2s);
       recordScalar("TET_3s", TET_3s);
       recordScalar("TET_4s", TET_4s);
       recordScalar("TET_5s", TET_5s);
       recordScalar("TIT", TIT);
       mobility->finish();
       //EV << "hasrecorded=" << hasRecordedStats << std::endl;
       if (hasRecordedStats >= HOWMANYEVS)
           endSimulation();
    }
    /*else
        recordScalar("totalTime", -1);*/

}

void VehicleControlApp2::onBSM(VehicleBSM* bsm)
{
    //endSimulation();
    if (traci->vehicle(mobility->getExternalId()).getTypeId() != "emergency")
        {
            //EV << "NOT EMERGENCY!!!" << std::endl;
            return;
        }
#ifdef DEBUG
    EV << "VehicleControlApp2::onBSM " << myId << " from " << bsm->getSenderId() << std::endl;
#endif

    /*if (bsm->getSenderId() == 15)
        numBSM++;*/

    //numBSMVector.record(numBSM);
    numBSMStats.collect(numBSM);

    //numBSMVector.record((simTime() - bsm->getCreationTime())*1000000);

    //Coord dist(this->mobility->getPositionAt(SimTime()) - bsm->getSenderPos());


    //distanceVector.record(dist.length());

    /*Coord& leadVehicleSpeed = bsm->getSenderSpeed();
    traciVehicle->setSpeedMode(0x1f);
    //traciVehicle->setSpeed(leadVehicleSpeed.length());
    EV << "SENDERIDIOTA " << bsm->getSenderId() << std::endl;
    EV << "SENDERIDIOTAvel " << leadVehicleSpeed.length() << std::endl;
    EV << "SIMTIME " << simTime();
    EV << " SIMTIMEMILI " << simTime().dbl();
    EV << " CREATION TIME " << bsm->getCreationTime();*/

}


void VehicleControlApp2::checkAndAddRSU(Coord pos, int id)
{
    int found = 0;

    if (traci->vehicle(mobility->getExternalId()).getTypeId() != "emergency")
        {
            //EV << "NOT EMERGENCY!!!" << std::endl;
            return;
        }

#ifdef DEBUG
    EV << "VehicleControlApp2::checkAndAddRSU checking for " << id << std::endl;
#endif

    if (!knownRSU.empty())
        for (std::vector<RSU>::iterator it = knownRSU.begin() ; it != knownRSU.end(); ++it)
        {
            if (it->id == id)
            {

#ifdef DEBUG
                EV << id << " is on the list." << std::endl;
#endif
                it->pos = pos;
                found = 1;
#ifdef DEBUG
                EV << "VehicleControlApp2::checkAndAddRSU Incoming: ";
                for (auto it2 = it->incomingEdges.rbegin(); it2 != it->incomingEdges.rend(); it2++)
                {
                    EV << *it2 << " ";
                }
                EV << std::endl;
                EV << std::endl << "VehicleControlApp2::checkAndAddRSU Outgoing: ";
                for (auto it2 = it->outgoingEdges.rbegin(); it2 != it->outgoingEdges.rend(); it2++)
                {
                    EV << *it2 << " ";
                }
#endif
            }
        }

    if (found == 0)
    {
        knownRSU.emplace_back(pos, id);
#ifdef DEBUG
        EV << "Added " << id << std::endl;
#endif
    }
}

void VehicleControlApp2::addRSUMap (int id, RSUMap *map)
{
    if (traci->vehicle(mobility->getExternalId()).getTypeId() != "emergency")
        {
            //EV << "NOT EMERGENCY!!!" << std::endl;
            return;
        }

    int nIncoming = map->getIncomingEdgesArraySize();
    int nOutgoing = map->getOutgoingEdgesArraySize();
    int k;

#ifdef DEBUG
    EV << "VehicleControlApp2::addRSUMap" << std::endl;
#endif

    if (knownRSU.empty())
    {
        EV << "SOMETHING SERIOUSLY WRONG HAPPENED!" << std::endl;
        return;
    }

    for (auto it = knownRSU.rbegin(); it != knownRSU.rend(); it++)
    {
#ifdef DEBUG
        EV << "VehicleControlApp2::addRSUMap check " << it->id << "vs " << id;
#endif
        if (it->id == id)
        {
            //Clear and update edges in case there are changes
            it->incomingEdges.clear();
            it->outgoingEdges.clear();
            for (k = 0; k < nIncoming; k++)
                it->incomingEdges.emplace_back(map->getIncomingEdges(k));
            for (k = 0; k < nOutgoing; k++)
                it->outgoingEdges.emplace_back(map->getOutgoingEdges(k));
#ifdef DEBUG
            EV << "VehicleControlApp2::addRSUMap Updated Routes of RSU " << map->getSenderId() << std::endl;
#endif
            // To stop searching
            return;
        }
    }

}


void VehicleControlApp2::prepareSRM(int rsuId)
{
    if (traci->vehicle(mobility->getExternalId()).getTypeId() != "emergency")
        {
            //EV << "NOT EMERGENCY!!!" << std::endl;
            return;
        }

    //EV << "VehicleControlApp2::prepareSRM" << map->getSenderId() << std::endl;
    std::vector<std::string>::iterator iterIncoming;
    std::vector<std::string>::iterator iterOutgoing;
#ifdef DEBUG
    EV << "VehicleControlApp2::prepareSRM looking for " << rsuId << std::endl;
#endif
    srmEdges.clear();

    std::string currentEdge = traci->vehicle(mobility->getExternalId()).getRoadId();

    for (auto it = knownRSU.rbegin(); it != knownRSU.rend(); it++)
    {
        if (it->id == rsuId) //the RSU this vehicle is going to send the SRM to
        {
            iterIncoming = std::find (it->incomingEdges.begin(), it->incomingEdges.end(), currentEdge);
            iterOutgoing = std::find (it->outgoingEdges.begin(), it->outgoingEdges.end(), currentEdge);
            if (iterIncoming == it->incomingEdges.end() && iterOutgoing == it->outgoingEdges.end())
            {
#ifdef DEBUG
                EV << "VehicleControlApp2::prepareSRM Current Edge (" << currentEdge << ") NOT FOUND" << std::endl;
#endif
            }
            else
            {
                srmEdges.emplace_back(currentEdge);
                relevantEdges++;
#ifdef DEBUG
                EV << "VehicleControlApp2::prepareSRM Current Edge (" << currentEdge << ") FOUNDED" << std::endl;
#endif
            }

            for (auto it2 = remainingEdges.rbegin(); it2 != remainingEdges.rend(); it2++)
            {
#ifdef DEBUG
                EV << "search for " << *it2 << std::endl;
#endif
                iterIncoming = std::find (it->incomingEdges.begin(), it->incomingEdges.end(), *it2);
                iterOutgoing = std::find (it->outgoingEdges.begin(), it->outgoingEdges.end(), *it2);
                if (iterIncoming == it->incomingEdges.end() && iterOutgoing == it->outgoingEdges.end())
                {
#ifdef DEBUG
                    EV << "VehicleControlApp2::prepareSRM Edge (" << *it2 << ") NOT FOUND" << std::endl;
#endif
                }
                else
                {
                    srmEdges.emplace_back(*it2);
                    relevantEdges++;
#ifdef DEBUG
                    EV << "VehicleControlApp2::prepareSRM Edge (" << *it2 << ") FOUNDED" << std::endl;
#endif
                }
            }
        }
    }
}

void VehicleControlApp2::onWSM(BaseFrame1609_4* wsm)
{
    // Your application has received a data message from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
    int senderId;

    if (traci->vehicle(mobility->getExternalId()).getTypeId() != "emergency")
        {
            //EV << "NOT EMERGENCY!!!" << std::endl;
            return;
        }



    /*if (myId != 39)
        return;*/

    //13, 18, 23, 28, 33
#ifdef DEBUG
    EV << "VehicleControlApp2::onWSM " << myId << std::endl;
#endif

    if (RSUMap* mapMsg = dynamic_cast<RSUMap*>(wsm))
    {
        // this rsu repeats the received traffic update in 2 seconds plus some random delay
        //sendDelayedDown(wsm->dup(), 2 + uniform(0.01, 0.2));
#ifdef DEBUG
        EV << "VehicleControlApp2::onWSM MAPMsg " << myId << std::endl;
#endif
        senderId = mapMsg->getSenderId();
        this->addRSUMap(senderId, mapMsg);
    }
    else
    if (RSUSpat* spatMsg = dynamic_cast<RSUSpat*>(wsm))
    {
#ifdef DEBUG
        EV << "VehicleControlApp2::onWSM SPaTMsg " << myId << std::endl;
#endif
        senderId = spatMsg->getSenderId(); //Gets the RSU id from the SPaT message
        this->checkAndAddRSU(spatMsg->getSenderPos(), senderId);
        VehicleSRM* srm = new VehicleSRM();


        //double distance = (mobility->getPositionAt(simTime()) - spatMsg->getSenderPos()).length(); (OLD NAIVE FORMULA)


        double distance = traci->getDistance(mobility->getPositionAt(simTime()) , spatMsg->getSenderPos(), true);
        double distanceNaive = (mobility->getPositionAt(simTime()) - spatMsg->getSenderPos()).length();

        if (distance > 2*distanceNaive)
            distance = distanceNaive; //workaround because the traci function doesn't work too well within close distances*/
#ifdef DEBUG
        EV << "DIstance= " << distance << " DistNaive=" << distanceNaive << std::endl;
#endif
        //workaround because the traci function doesn't work too well within close distances
        for (std::vector<RSU>::iterator it = knownRSU.begin() ; it != knownRSU.end(); ++it)
        {
            if (it->id == senderId)
            {
                if (it->prevDist <= 0.01)
                    it->prevDist = distance;
                else
                if (fabs(it->prevDist - distance) > 40.0)
                {
                    delete srm;
                    return;
                }
                else
                    it->prevDist = distance;
            }
        }

        if (distance > 10000.0 || distance <= -10000.0) //unreachable or has already passed
        {
            EV << "BUG IN DISTANCE CALCULATION" << std::endl;
            delete srm;
            return;
        }

        double extraTime;

        double extraTime40,extraTime100,extraTime200,extraTime300,extraTime400;


        if (averageSpeed > 1.0)
            extraTime = distance/averageSpeed;
        else
        if (averageSpeed < 1.0 && averageSpeedTerms < averageSpeedMaxTerms/2)
            extraTime = distance/6.0;
        else
            extraTime = distance/1.0;

        /*if (averageSpeed40 > 1.0)
            extraTime40 = distance/averageSpeed40;
        else
        if (averageSpeed40 < 1.0 && averageSpeedTerms < averageSpeedMaxTerms/2)
            extraTime40 = distance/6.0;
        else
            extraTime40 = distance/1.0;

        if (averageSpeed100 > 1.0)
            extraTime100 = distance/averageSpeed100;
        else
        if (averageSpeed100 < 1.0 && averageSpeedTerms < averageSpeedMaxTerms/2)
            extraTime100 = distance/6.0;
        else
            extraTime100 = distance/1.0;

        if (averageSpeed200 > 1.0)
            extraTime200 = distance/averageSpeed200;
        else
        if (averageSpeed200 < 1.0 && averageSpeedTerms < averageSpeedMaxTerms/2)
            extraTime200 = distance/6.0;
        else
            extraTime200 = distance/1.0;

        if (averageSpeed300 > 1.0)
            extraTime300 = distance/averageSpeed300;
        else
        if (averageSpeed300 < 1.0 && averageSpeedTerms < averageSpeedMaxTerms/2)
            extraTime300 = distance/6.0;
        else
            extraTime300 = distance/1.0;

        if (averageSpeed400 > 1.0)
            extraTime400 = distance/averageSpeed400;
        else
        if (averageSpeed400 < 1.0 && averageSpeedTerms < averageSpeedMaxTerms/2)
            extraTime400 = distance/6.0;
        else
            extraTime400 = distance/1.0;*/


#ifdef DEBUG
        EV << "Vehicle Pos: " << mobility->getPositionAt(simTime()) << " RSU Pos: " << spatMsg->getSenderPos() << std::endl;
        EV << "Vehicle Speed: " << mobility->getSpeed() << " Distance " << distance << "Arrival " << (simTime() + extraTime) <<  std::endl;
        EV << "Average Speed: " << averageSpeed << " Terms: " << averageSpeedTerms << std::endl;
#endif

        srm->setReceiverId(senderId);
        srm->setTimeToArrive(simTime() + extraTime);

        relevantEdges = 0;
        populateWSM(srm,LAddress::L2BROADCAST(), 0);
#ifdef DEBUG
        EV << "Relevant Edges to RSU" << senderId << " = " << relevantEdges << std::endl;
#endif
        if (relevantEdges > 0)
        {
            /*switch (senderId)
            {
                case 13:
                    timeToArriveRSU0.record(simTime() + extraTime);
                    distanceRSU0.record(distance);
                break;
                case 18:
                    timeToArriveRSU1.record(simTime() + extraTime);
                    timeToArriveRSU140.record(simTime() + extraTime40);
                    timeToArriveRSU1100.record(simTime() + extraTime100);
                    timeToArriveRSU1200.record(simTime() + extraTime200);
                    timeToArriveRSU1300.record(simTime() + extraTime300);
                    timeToArriveRSU1400.record(simTime() + extraTime400);
                    distanceRSU1.record(distance);
                break;
                case 23:
                    timeToArriveRSU2.record(simTime() + extraTime);
                    distanceRSU2.record(distance);
                break;
                case 28:
                    timeToArriveRSU3.record(simTime() + extraTime);
                    distanceRSU3.record(distance);
                break;
                case 33:
                    timeToArriveRSU4.record(simTime() + extraTime);
                    distanceRSU4.record(distance);
                break;
            }*/
            sendDown(srm);
        }
        else
            delete(srm);
    }
}

void VehicleControlApp2::onWSA(DemoServiceAdvertisment* wsa)
{
    if (traci->vehicle(mobility->getExternalId()).getTypeId() != "emergency")
        {
            //EV << "NOT EMERGENCY!!!" << std::endl;
            return;
        }
    // Your application has received a service advertisement from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
#ifdef DEBUG
    EV << "VehicleControlApp2::onWSA " << myId << std::endl;
#endif
}

void VehicleControlApp2::handleSelfMsg(cMessage* msg)
{
    //SafeSmartApplLayer::handleSelfMsg(msg);
    // this method is for self messages (mostly timers)
    // it is important to call the SafeSmartApplLayer function for BSM and WSM transmission

    double safety;
    if (traci->vehicle(mobility->getExternalId()).getTypeId() == "emergencyo")
    {
        scheduleAt(simTime() + timerInterval, timer);  // rescheduling
    }
    mobility->setIsEV(1);
    if (subscribed == false)
    {
       //EV << "SafeSmart Subscribe" << std::endl;
       TraCIScenarioManager* manager = TraCIScenarioManagerAccess().get();
       //EV << "mobility->getExternalId() " << mobility->getExternalId() << std::endl;
       //EV << "emergencyo " << simTime() << std::endl ;
       safety = manager->safesmartSubscribe(mobility->getExternalId());


       if (safety < 99999.0)
       {
           if (safetyPrevPos != mobility->getPositionAt(simTime())) //disconsider if EV is stopped
           {
               safetyPrevPos = mobility->getPositionAt(simTime());
               safetyMetricAverage = (safetyMetricAverage*safetyMetricTerms + safety)/(safetyMetricTerms+1);
               safetyMetricTerms++; //adds one sample
               EV << "Safety Metric: " << safety << std::endl;
               EV << "Safety AVG=" << safetyMetricAverage << " terms=" << safetyMetricTerms << std::endl;
               ttcVector.record(safety);
               if (safety < 1.0)
                  TET_1s++;
               else
               if (safety < 2.0)
                  TET_2s++;
               else
               if (safety < 3.0)
                  TET_3s++;
               else
               if (safety < 4.0)
                  TET_4s++;
               else
               if (safety < 5.0)
                  TET_5s++;

               if (safety < 5.0)
                   TIT += 5.0 - safety;
           }

       }

      // subscribed = true;

    }

    if (traci->vehicle(mobility->getExternalId()).getTypeId() != "emergency")
    {
        //EV << "handleSelfMsg NOT EMERGENCY!!!" << std::endl;
        return;
    }

    if (averageSpeedTerms < averageSpeedMaxTerms) //first speed sample
    {
        averageSpeed = (averageSpeed*averageSpeedTerms + mobility->getSpeed())/(averageSpeedTerms+1);
        averageSpeedTerms++; //adds one sample
    }

    if (averageSpeedTerms >= averageSpeedMaxTerms) //discards older samples
    {
        averageSpeed = (averageSpeed*(averageSpeedMaxTerms-1) + mobility->getSpeed())/(averageSpeedMaxTerms);
    }


    /*averageSpeed40 = (averageSpeed40*(40-1) + mobility->getSpeed())/(40.0);
    averageSpeed100 = (averageSpeed100*(100-1) + mobility->getSpeed())/(100.0);
    averageSpeed200 = (averageSpeed200*(200-1) + mobility->getSpeed())/(200.0);
    averageSpeed300 = (averageSpeed300*(300-1) + mobility->getSpeed())/(300.0);
    averageSpeed400 = (averageSpeed400*(400-1) + mobility->getSpeed())/(400.0);

    averageSpeedVector.record(averageSpeed);*/

    /*averageSpeedVector40.record(averageSpeed40);
    averageSpeedVector100.record(averageSpeed100);
    averageSpeedVector200.record(averageSpeed200);
    averageSpeedVector300.record(averageSpeed300);
    averageSpeedVector400.record(averageSpeed400);*/

    std::list<std::string> tempRemainingEdges;

    std::list<std::string>::iterator itCurEdge = remainingEdges.begin();
//Remove current edge from remaining edges list
//Work around for ~really small edges that go undetected
    do
    {
        itCurEdge = std::find (remainingEdges.begin(), remainingEdges.end(), traci->vehicle(mobility->getExternalId()).getRoadId());

        if (itCurEdge == remainingEdges.end())
        {
            //Current edge is not on the remaining edges list
        }
        else
        {
#ifdef DEBUG
            EV << "Removed edge " << remainingEdges.front() << " from remaining edges list." << std::endl;
#endif
            remainingEdges.pop_front();
        }

    } while(itCurEdge != remainingEdges.end());




#ifdef DEBUG
    EV << "Road: " << traci->vehicle(mobility->getExternalId()).getRoadId() << std::endl;
    EV << "Route: " << routeId << std::endl;
    EV << "Route: ";



    for (auto it = routeEdges.cbegin(); it != routeEdges.cend(); it++)
    {
        EV << *it << " ";
    }
    EV << std::endl;


    EV << "Remaining Edges: ";

    for (auto it = remainingEdges.cbegin(); it != remainingEdges.cend(); it++)
    {
        EV << *it << " ";
    }
    EV << std::endl;

    EV << "Node id " << this->myId << std::endl;
#endif


#ifdef DEBUG
    EV << "Vehicle ExtId+type " << mobility->getExternalId() << " type: " << traci->vehicle(mobility->getExternalId()).getTypeId() << std::endl;
#endif
    /*if (myId != 39)
        return;*/

    switch (msg->getKind()) {
        case SEND_BEACON_EVT: {
            VehicleBSM* bsm = new VehicleBSM();
            populateWSM(bsm,LAddress::L2BROADCAST(), 0);
            sendDown(bsm);
            scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
            break;
        }
        case SEND_WSA_EVT: {
            DemoServiceAdvertisment* wsa = new DemoServiceAdvertisment();
            populateWSM(wsa,LAddress::L2BROADCAST(), 0);
            sendDown(wsa);
            scheduleAt(simTime() + wsaInterval, sendWSAEvt);
            break;
        }
        default: {
            if (msg) EV_WARN << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
            break;
        }
        }

}

void VehicleControlApp2::handlePositionUpdate(cObject* obj)
{
    if (traci->vehicle(mobility->getExternalId()).getTypeId() != "emergency")
    {
        //EV << "NOT EMERGENCY!!!" << std::endl;
        if (traci->vehicle(mobility->getExternalId()).getTypeId() == "emergencyo")
        {
            if (mobility->getExternalId() == "carflow1.R.0")
               numBSM = 20;
            if (mobility->getExternalId() == "carflow1.S.0")
               numBSM = 21;
            if (mobility->getExternalId() == "carflow1.U.0")
               numBSM = 22;
            if (mobility->getExternalId() == "carflow2.L.0")
               numBSM = 23;
            if (mobility->getExternalId() == "carflow2.R.0")
               numBSM = 24;
            if (mobility->getExternalId() == "carflow2.S.0")
               numBSM = 25;
            if (mobility->getExternalId() == "carflow2.U.0")
               numBSM = 26;
            if (mobility->getExternalId() == "carflow3.L.0")
               numBSM = 27;
            if (mobility->getExternalId() == "carflow3.R.0")
               numBSM = 28;
            if (mobility->getExternalId() == "carflow3.S.0")
               numBSM = 29;
            if (mobility->getExternalId() == "carflow3.U.0")
               numBSM = 30;

            if (mobility->getExternalId() == "carflow4.L.0")
               numBSM = 40;
            if (mobility->getExternalId() == "carflow4.R.0")
               numBSM = 41;
            if (mobility->getExternalId() == "carflow4.S.0")
               numBSM = 42;
            if (mobility->getExternalId() == "carflow4.U.0")
               numBSM = 43;
            if (mobility->getExternalId() == "carflowC.0.0")
               numBSM = 44;
            if (mobility->getExternalId() == "carflowC.1.0")
               numBSM = 45;
            if (mobility->getExternalId() == "carflow6.1")
               numBSM = 46;
            if (mobility->getExternalId() == "carflow7.1")
               numBSM = 47;
            if (mobility->getExternalId() == "carflow8.1")
               numBSM = 48;
            if (mobility->getExternalId() == "carflow9.1")
               numBSM = 49;

            numBSMStats.collect(numBSM);
            return;
        }

    }

    if (mobility->getExternalId() == "carflow1.R.0")
       numBSM = 100;
    if (mobility->getExternalId() == "carflow1.S.0")
       numBSM = 101;
    if (mobility->getExternalId() == "carflow1.U.0")
       numBSM = 102;
    if (mobility->getExternalId() == "carflow2.L.0")
       numBSM = 103;
    if (mobility->getExternalId() == "carflow2.R.0")
       numBSM = 104;
    if (mobility->getExternalId() == "carflow2.S.0")
       numBSM = 105;
    if (mobility->getExternalId() == "carflow2.U.0")
       numBSM = 106;
    if (mobility->getExternalId() == "carflow3.L.0")
       numBSM = 107;
    if (mobility->getExternalId() == "carflow3.R.0")
       numBSM = 108;
    if (mobility->getExternalId() == "carflow3.S.0")
       numBSM = 109;
    if (mobility->getExternalId() == "carflow3.U.0")
       numBSM = 110;
    if (mobility->getExternalId() == "carflow4.L.0")
       numBSM = 111;
    if (mobility->getExternalId() == "carflow4.R.0")
       numBSM = 112;
    if (mobility->getExternalId() == "carflow4.S.0")
       numBSM = 113;
    if (mobility->getExternalId() == "carflow4.U.0")
       numBSM = 114;
    if (mobility->getExternalId() == "carflowC.0.0")
       numBSM = 115;
    if (mobility->getExternalId() == "carflowC.1.0")
       numBSM = 116;

    if (mobility->getExternalId() == "carflow0.1")
       numBSM = 120;
    if (mobility->getExternalId() == "carflow1.1")
       numBSM = 121;
    if (mobility->getExternalId() == "carflow2.1")
       numBSM = 122;
    if (mobility->getExternalId() == "carflow3.1")
       numBSM = 123;
    if (mobility->getExternalId() == "carflow4.1")
       numBSM = 124;
    if (mobility->getExternalId() == "carflow5.1")
       numBSM = 125;
    if (mobility->getExternalId() == "carflow6.1")
       numBSM = 126;
    if (mobility->getExternalId() == "carflow7.1")
       numBSM = 127;
    if (mobility->getExternalId() == "carflow8.1")
       numBSM = 128;
    if (mobility->getExternalId() == "carflow9.1")
       numBSM = 129;

    numBSMStats.collect(numBSM);

    SafeSmartApplLayer::handlePositionUpdate(obj);

    /*if (myId != 39)
        return;*/


    /*test = traci->getVehicleTypeIds();

    for (auto it = test.cbegin(); it != test.cend(); it++)
    {
        EV << *it << " ";
    }*/

    /*if (this->getId() == 14)
    {
        const simtime_t t = simTime();
        if (t == 10)
        {
            traciVehicle->setSpeedMode(0x1f);
            traciVehicle->setSpeed(0);
        }
        else
        if (t == 40)
        {
            traciVehicle->setSpeedMode(0x1f);
            traciVehicle->setSpeed(20);
        }
    }*/

    // the vehicle has moved. Code that reacts to new positions goes here.
    // member variables such as currentPosition and currentSpeed are updated in the parent class
}

void VehicleControlApp2::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial)
{
    int k;

    if (traci->vehicle(mobility->getExternalId()).getTypeId() != "emergency")
        {
            //EV << "NOT EMERGENCY!!!" << std::endl;
            return;
        }

    /*if (myId != 39)
        return;*/

    wsm->setRecipientAddress(rcvId);
    wsm->setBitLength(headerLength);

    if (VehicleBSM* bsm = dynamic_cast<VehicleBSM*>(wsm)) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(-1);
        bsm->setChannelNumber(static_cast<int>(Channel::cch));
        bsm->addBitLength(beaconLengthBits);
        bsm->setSenderId(numBSM);
        bsm->setCreationTime(simTime());
        wsm->setUserPriority(beaconUserPriority);
#ifdef DEBUG
        EV << "VehicleControlApp2::populateWSM (BSM) messageid " << bsm->getId() << std::endl;
#endif
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(static_cast<int>(Channel::cch));
        wsa->setTargetChannel(static_cast<int>(currentServiceChannel));
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
#ifdef DEBUG
        EV << "VehicleControlApp2::populateWSM (WSA) messageid " << wsa->getId() << std::endl;
#endif
    }
    else if (VehicleSRM* srm = dynamic_cast<VehicleSRM*>(wsm)) {

        srm->setMessageType(0);
        srm->setSenderPos(mobility->getPositionAt(simTime()));
        srm->setSenderId(numBSM);

        //srm->setReceiverId(0);
        srm->setVehicleType("n/a");
        srm->setPriority(1);
        //srm->setTimeToArrive(0);
        srm->setCancelTime(0);
        srm->setCreationTime(simTime());
        srm->setTransactionId(wsm->getId());
        srm->setChannelNumber(static_cast<int>(Channel::cch));

        k = 0;


        //EV << "HALLELUJAH" << traci->vehicle(mobility->getExternalId()).getRoadId();

        /*k = 0;
        for (auto it = remainingEdges.cbegin(); it != remainingEdges.cend(); it++)
        {
            srm->setRemainingEdges(k, it->c_str());
            k++;
        }*/

        //EV << std::endl;

        prepareSRM(srm->getReceiverId());

        for (auto it = srmEdges.cbegin(); it != srmEdges.cend(); it++)
        {
            srm->setRoute(k, it->c_str()); //Sets the edges related to that RSU in the SRM msg
#ifdef DEBUG
            EV << "SETTING " << *it << std::endl;
#endif
            k++;
        }

#ifdef DEBUG
        EV << "VehicleControlApp2::populateWSM (SRM) messageid " << srm->getId() << std::endl;
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
        EV << "VehicleControlApp2::populateWSM (else)" << std::endl;
#endif
    }
}
