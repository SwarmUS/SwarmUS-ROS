#ifndef HIVEMIND_BRIDGE_IMESSAGEHANDLER_H
#define HIVEMIND_BRIDGE_IMESSAGEHANDLER_H

#include "InboundRequestHandle.h"
#include "hive_mind_bridge/InboundResponseHandle.h"
#include "hive_mind_bridge/UserCallbackFunctionWrapper.h"
#include <optional>
#include <pheromones/FunctionCallArgumentDTO.h>
#include <pheromones/FunctionCallRequestDTO.h>
#include <pheromones/FunctionCallResponseDTO.h>
#include <pheromones/MessageDTO.h>
#include <pheromones/RequestDTO.h>
#include <variant>

typedef std::unordered_map<std::string, UserCallbackFunctionWrapper> CallbackMap;

class IMessageHandler {
  public:
    virtual ~IMessageHandler() = default;

    /**
     * Parse a message and execute the appropriate callback or action.
     * @param message the message to parse.
     * @return A message containing the appropriate acknowlege (with appropriate errors if
     * necessary)
     */
    virtual std::variant<std::monostate, InboundRequestHandle, InboundResponseHandle> handleMessage(
        MessageDTO message) = 0;

    /**
     * Parse a greet message and return the contained swarmAgentId.
     * @param greetMessage The message to parse.
     * @return The contained swarmAgentId if the operation succeded.
     */
    virtual std::optional<uint32_t> handleGreet(MessageDTO greetMessage) = 0;

    /**
     * Register a callback
     * @param name Key of the callback
     * @param callback Callback function
     * @returns True if an existing callback function was overwritten, false otherwise
     */
    virtual bool registerCallback(std::string name, CallbackFunction callback) = 0;

    /**
     * Register a callback
     * @param name Key of the callback
     * @param callback Callback function
     * @param manifest A list describing the callback's expected arguments name and type
     * @returns True if an existing callback function was overwritten, false otherwise
     */
    virtual bool registerCallback(std::string name,
                                  CallbackFunction callback,
                                  CallbackArgsManifest manifest) = 0;

    /**
     * Get an instance of a callback, if it exists.
     * @param name Key under which the callback was registered
     * @return The callback function if it exists.
     */
    virtual std::optional<CallbackFunction> getCallback(const std::string& name) = 0;
};

#endif // HIVEMIND_BRIDGE_IMESSAGEHANDLER_H
