#ifndef HIVE_MIND_BRIDGE_USERCALLBACKFUNCTIONWRAPPER_H
#define HIVE_MIND_BRIDGE_USERCALLBACKFUNCTIONWRAPPER_H

#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <hivemind-host/FunctionCallRequestDTO.h>
#include <hivemind-host/FunctionDescriptionArgumentTypeDTO.h>
#include "hive_mind_bridge/UserCallbackArgumentWrapper.h"
#include <functional>
#include <unordered_map>

typedef std::array<FunctionCallArgumentDTO,
                   FunctionCallRequestDTO::FUNCTION_CALL_ARGUMENTS_MAX_LENGTH>
    CallbackArgs;

typedef std::function<void(CallbackArgs, int)> CallbackFunction;

//typedef std::unordered_map<std::string, FunctionDescriptionArgumentTypeDTO> CallbackArgsManifest;
typedef std::vector<UserCallbackArgumentWrapper> CallbackArgsManifest;

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
