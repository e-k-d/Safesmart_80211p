//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

cplusplus {{
#include "veins/base/utils/Coord.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/base/utils/SimpleAddress.h"
#include "veins/veins.h"
#include <iostream>
#include <vector>
}}

namespace veins;

class BaseFrame1609_4;
class noncobject Coord;
class LAddress::L2Type extends void;

//
// TODO generated message class
//
packet VehicleSRM extends BaseFrame1609_4 {
    int messageType;
    Coord senderPos;
    int senderId;
    int receiverId;
    string vehicleType;
    //int scenario;
    int priority;
    simtime_t timeToArrive;
    simtime_t cancelTime;
    simtime_t creationTime;
    int transactionId;
    string route[100];
    string remainingEdges[100];
    
    int testField;
}
