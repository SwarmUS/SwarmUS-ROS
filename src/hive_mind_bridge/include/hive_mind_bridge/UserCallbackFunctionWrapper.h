#ifndef HIVE_MIND_BRIDGE_USERCALLBACKFUNCTIONWRAPPER_H
#define HIVE_MIND_BRIDGE_USERCALLBACKFUNCTIONWRAPPER_H

#include "hive_mind_bridge/Callback.h"
#include <functional>
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <hivemind-host/FunctionCallRequestDTO.h>
#include <hivemind-host/FunctionDescriptionArgumentTypeDTO.h>
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
