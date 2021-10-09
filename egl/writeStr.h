#pragma once

#include <fstream>

namespace egl
{
    inline void writeStr(std::string filename, const std::string &str)
    {
        std::ofstream file;
        file.open(filename);
        file << str;
        file.close();
    }
}
