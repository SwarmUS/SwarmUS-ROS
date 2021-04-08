#ifndef HIVE_MIND_BRIDGE_CALLBACK_H
#define HIVE_MIND_BRIDGE_CALLBACK_H

#include "hive_mind_bridge/UserCallbackArgumentDescription.h"
#include <functional>
#include <pheromones/FunctionCallArgumentDTO.h>
#include <pheromones/FunctionCallRequestDTO.h>

typedef std::array<FunctionCallArgumentDTO,
                   FunctionCallRequestDTO::FUNCTION_CALL_ARGUMENTS_MAX_LENGTH>
    CallbackArgs;

typedef std::vector<UserCallbackArgumentDescription> CallbackArgsManifest;

/**
 * The return type of a user callback.
 */
class CallbackReturn {
  public:
    CallbackReturn(std::string returnFunctionName, CallbackArgs args) :
        m_args(args), m_returnFunctionName(returnFunctionName) {}

    CallbackArgs getReturnArgs() { return m_args; }

    std::string getReturnFunctionName() { return m_returnFunctionName; }

  private:
    CallbackArgs m_args; // The return values
    std::string
        m_returnFunctionName; // The name of the function call request in which to wrap the response
};

typedef std::function<std::optional<CallbackReturn>(CallbackArgs, int)> CallbackFunction;

#endif // HIVE_MIND_BRIDGE_CALLBACK_H
