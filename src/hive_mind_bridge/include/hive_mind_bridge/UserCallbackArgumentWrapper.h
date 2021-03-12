#ifndef HIVE_MIND_BRIDGE_USERCALLBACKARGUMENTWRAPPER_H
#define HIVE_MIND_BRIDGE_USERCALLBACKARGUMENTWRAPPER_H

class UserCallbackArgumentWrapper {
  public:
    UserCallbackArgumentWrapper() = default;

    UserCallbackArgumentWrapper(std::string name, FunctionDescriptionArgumentTypeDTO type);

    std::string getName();

    FunctionDescriptionArgumentTypeDTO getType();

  private:
    std::string m_name;
    FunctionDescriptionArgumentTypeDTO m_type = FunctionDescriptionArgumentTypeDTO::Unknown;
};

#endif // HIVE_MIND_BRIDGE_USERCALLBACKARGUMENTWRAPPER_H
