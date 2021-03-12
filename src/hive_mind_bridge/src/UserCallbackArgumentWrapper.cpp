#include <string>
#include <hivemind-host/FunctionDescriptionArgumentTypeDTO.h>
#include "hive_mind_bridge/UserCallbackArgumentWrapper.h"

UserCallbackArgumentWrapper::UserCallbackArgumentWrapper(std::string name, FunctionDescriptionArgumentTypeDTO type) :
m_name(name), m_type(type) {

}

std::string UserCallbackArgumentWrapper::getName() {
    return m_name;
}

FunctionDescriptionArgumentTypeDTO UserCallbackArgumentWrapper::getType() {
    return m_type;
}