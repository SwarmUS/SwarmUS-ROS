#ifndef HIVE_MIND_BRIDGE_USERCALLBACKFUNCTIONWRAPPER_H
#define HIVE_MIND_BRIDGE_USERCALLBACKFUNCTIONWRAPPER_H

#include "hive_mind_bridge/UserCallbackArgumentDescription.h"
#include <functional>
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <hivemind-host/FunctionCallRequestDTO.h>
#include <hivemind-host/FunctionDescriptionArgumentTypeDTO.h>
#include <unordered_map>

typedef std::array<FunctionCallArgumentDTO,
                   FunctionCallRequestDTO::FUNCTION_CALL_ARGUMENTS_MAX_LENGTH>
    CallbackArgs;

typedef std::function<std::optional<CallbackArgs>(CallbackArgs, int)> CallbackFunction;

typedef std::vector<UserCallbackArgumentDescription> CallbackArgsManifest;

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
