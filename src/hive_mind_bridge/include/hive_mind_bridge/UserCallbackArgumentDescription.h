#ifndef HIVE_MIND_BRIDGE_USERCALLBACKARGUMENTDESCRIPTION_H
#define HIVE_MIND_BRIDGE_USERCALLBACKARGUMENTDESCRIPTION_H

#include <hivemind-host/FunctionDescriptionArgumentTypeDTO.h>
#include <string>

class UserCallbackArgumentDescription {
  public:
    UserCallbackArgumentDescription() = default;

    UserCallbackArgumentDescription(std::string name, FunctionDescriptionArgumentTypeDTO type);

    std::string getName();

    FunctionDescriptionArgumentTypeDTO getType();

  private:
    std::string m_name;
    FunctionDescriptionArgumentTypeDTO m_type = FunctionDescriptionArgumentTypeDTO::Unknown;
};

#endif // HIVE_MIND_BRIDGE_USERCALLBACKARGUMENTDESCRIPTION_H
