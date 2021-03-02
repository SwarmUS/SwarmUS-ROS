#ifndef HIVEBOARD_BRIDGE_HIVEBOARDBRIDGEFACTORY_H
#define HIVEBOARD_BRIDGE_HIVEBOARDBRIDGEFACTORY_H

#include "hiveboard_bridge/HiveBoardBridge.h"

namespace HiveBoardBridgeFactory {
    HiveBoardBridge createHiveBoardBridge(int port);
}

#endif //HIVEBOARD_BRIDGE_HIVEBOARDBRIDGEFACTORY_H
