#pragma once

struct BrushPainter
{
    int m_type;
    int m_intSize;
    float m_refSize;
    float m_size;

    float m_alpha;
    float m_pressure; //brush pressure

    float m_spacing;
    float m_innerRatio;
    float m_accumDistance; // accumulated distance of brush since last paint op

    BrushPainter()
    {
        m_type = 0;
        m_intSize = 6;
        m_refSize = -1.0;
        m_size = 0.0;

        m_alpha = 1.0;
        m_pressure = 1.0;

        m_spacing = 0.03;
        m_innerRatio = 0.5;
        m_accumDistance = 0.0;
    };
};
