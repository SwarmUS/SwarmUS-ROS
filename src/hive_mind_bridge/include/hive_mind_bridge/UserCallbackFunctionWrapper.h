#ifndef HIVE_MIND_BRIDGE_USERCALLBACKFUNCTIONWRAPPER_H
#define HIVE_MIND_BRIDGE_USERCALLBACKFUNCTIONWRAPPER_H

#include "hive_mind_bridge/Callback.h"
#include <functional>
#include <pheromones/FunctionCallArgumentDTO.h>
#include <pheromones/FunctionCallRequestDTO.h>
#include <pheromones/FunctionDescriptionArgumentTypeDTO.h>
#include <unordered_map>

class UserCallbackFunctionWrapper {
  public:
    UserCallbackFunctionWrapper() = default;

    UserCallbackFunctionWrapper(CallbackFunction function, CallbackArgsManifest manifest);

    CallbackFunction getFunction();

    CallbackArgsManifest getManifest();

  private:
    CallbackFunction m_function;
    CallbackArgsManifest m_manifest;
};

#endif // HIVE_MIND_BRIDGE_USERCALLBACKFUNCTIONWRAPPER_H
