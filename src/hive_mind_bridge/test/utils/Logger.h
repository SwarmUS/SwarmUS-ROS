#ifndef HIVE_MIND_BRIDGE_LOGGER_H
#define HIVE_MIND_BRIDGE_LOGGER_H

#include <cpp-common/ILogger.h>
#include <cstdarg>
#include <cstdio>

class Logger : public ILogger {
public:
    Logger() {}

    LogRet log(LogLevel level, const char* format, ...) override {
        va_list args;
        va_start(args, format);
        int retVal = formatAndAppend(format, args);
        va_end(args);

        flush(level);

        if (retVal >= 0) {
            return LogRet::Ok;
        } else {
            return LogRet::Error;
        }
    }

    int formatAndAppend(const char* format, va_list args) {
        // Copy varargs
        va_list vaCopy;
        va_copy(vaCopy, args);
        const int requiredLength = std::vsnprintf(NULL, 0, format, vaCopy);
        va_end(vaCopy);

        // Create a string with adequate length
        std::string tmpStr;
        tmpStr.resize((size_t)requiredLength);

        // Build a new string
        int retValue = vsnprintf(tmpStr.data(), tmpStr.size() + 1, format, args);
        m_accumulatedString = m_accumulatedString + tmpStr;

        return retValue;

    }

    void flush(LogLevel level) {
        switch (level) {
            case LogLevel::Debug :
                printf("[DEBUG] %s", m_accumulatedString.c_str());
                break;
            case LogLevel::Info :
                printf("[INFO] %s", m_accumulatedString.c_str());
                break;
            case LogLevel::Warn :
                printf("[WARN] %s", m_accumulatedString.c_str());
                break;
            case LogLevel::Error :
                printf("[ERROR] %s", m_accumulatedString.c_str());
                break;
        }

        m_accumulatedString = "";
    }

private:
    std::string m_accumulatedString;
};

#endif //HIVE_MIND_BRIDGE_LOGGER_H
