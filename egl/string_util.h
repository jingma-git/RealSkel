#pragma once
#include <string>
namespace egl
{
    inline void ltrim(std::string &s)
    {
        s.erase(s.begin(), s.begin() + s.find_first_not_of("\n\r\t"));
    }

    inline void rtrim(std::string &s)
    {
        s.erase(s.find_last_not_of("\n\r\t") + 1);
    }

    inline void trim(std::string &s)
    {
        ltrim(s);
        rtrim(s);
    }
}