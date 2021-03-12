#include "hive_mind_bridge/UserCallbackFunctionWrapper.h"

UserCallbackFunctionWrapper::UserCallbackFunctionWrapper(CallbackFunction function,
                                                         CallbackArgsManifest manifest) :
    m_function(function), m_manifest(manifest) {}

CallbackFunction UserCallbackFunctionWrapper::getFunction() {
    return m_function;
}

CallbackArgsManifest UserCallbackFunctionWrapper::getManifest() {
    return m_manifest;
}