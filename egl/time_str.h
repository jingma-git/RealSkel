#pragma once

#include <ctime>
#include <string>

namespace egl
{
    inline std::string current_time_str()
    {
        // https://stackoverflow.com/questions/16357999/current-date-and-time-as-string/16358264
        char buffer[80];

        auto rawtime = std::time(nullptr);
        auto timeinfo = std::localtime(&rawtime);
        std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", timeinfo);
        return buffer;
    }
}
