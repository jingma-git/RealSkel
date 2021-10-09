#pragma once
#include <vector>
#include <string>
#include <iostream>

typedef std::vector<std::vector<double>> MatD;
typedef std::vector<std::vector<int>> MatI;

class MapsData
{
public:
    static bool visualizeMask(std::string fileName, const MatI &mask);
    static bool visualizeDepth(std::string fileName, const MatD &depth);

    template <class T>
    static bool flipVertical(const std::vector<T> &inImg, std::vector<T> &outImg, int width, int height, int channel)
    {
        using namespace std;
        if (width * height * channel != (int)inImg.size())
        {
            std::cerr << "Error: incorrect image size" << endl;
            return false;
        }

        vector<T> buffer(inImg.size());
        int lineSize = width * channel;
        for (int h = 0; h < height; h++)
        {
            auto first = inImg.begin() + h * lineSize;
            auto last = first + lineSize;
            auto dest = buffer.begin() + (height - h - 1) * lineSize;
            copy(first, last, dest);
        }
        outImg.swap(buffer);
        return true;
    }
};