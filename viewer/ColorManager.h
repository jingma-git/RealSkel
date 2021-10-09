#pragma once
#include <QColor>

class ColorManager
{
public:
    static QVector<QColor> rndColors(int count)
    {
        QVector<QColor> colors;
        float currentHue = 0.0;
        for (int i = 0; i < count; i++)
        {
            colors.push_back(QColor::fromHslF(currentHue, 1.0, 0.5));
            currentHue += 0.618033988749895f;
            currentHue = std::fmod(currentHue, 1.0f);
        }
        return colors;
    }

    static QVector<QColor> colorPattern1()
    {
        QVector<QColor> colors = {
            QColor(0, 0, 0),     //black
            QColor(0, 0, 255),   //b
            QColor(0, 255, 0),   //g
            QColor(255, 0, 0),   //r
            QColor(0, 255, 255), //cyan
            QColor(255, 255, 0), //yellow
            QColor(255, 0, 255), //pink
            QColor(125, 0, 255), //purple
            QColor(255, 125, 0), //orange

            QColor(25, 25, 25),              //gray
            QColor(0, 0, 255 * 0.5),         //r
            QColor(0, 255 * 0.5, 0),         //g
            QColor(255 * 0.5, 0, 0),         //b
            QColor(0, 255 * 0.5, 255 * 0.5), //yellow
            QColor(255 * 0.5, 255 * 0.5, 0), // cyan
            QColor(255 * 0.5, 0, 255 * 0.5), //pink
            QColor(125 * 0.5, 0, 255 * 0.5), //purple
            QColor(0, 125 * 0.5, 255 * 0.5)  //orange
        };

        return colors;
    }

    // static std::vector<QColor> colorPattern2()
    // {
    //     std::vector<QColor> colors[10] = {QColor("cyan"), QColor("magenta"), QColor("red"),
    //                                       QColor("darkRed"), QColor("darkCyan"), QColor("darkMagenta"),
    //                                       QColor("green"), QColor("darkGreen"), QColor("yellow"),
    //                                       QColor("blue")};
    //     return colors;
    // }
};