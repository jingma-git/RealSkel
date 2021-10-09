#pragma once
#include "common.h"
class ColorI
{
public:
    BYTE b, g, r, a; // do not change the sequence of the variables, it may cause disastrous errors

    ColorI() {}

    ColorI(BYTE _r, BYTE _g, BYTE _b, BYTE _a = 255)
    {
        r = _r;
        g = _g;
        b = _b;
        a = _a;
    }

    ColorI(DWORD x)
    {
        *(DWORD *)this = x;
    }

    void Set(BYTE _r, BYTE _g, BYTE _b, BYTE _a = 255)
    {
        r = _r;
        g = _g;
        b = _b;
        a = _a;
    }

    bool IsBlack() const { return (*(DWORD *)this & 0x00FFFFFF) == 0; };
    bool IsWhite() const { return (*(DWORD *)this & 0x00FFFFFF) == 0x00FFFFFF; };
    bool IsTrans() const { return (*(DWORD *)this & 0xFF000000) == 0; };
    ColorI GetInvertRGB() const { return (*(DWORD *)this ^ 0x00FFFFFF); };
    void InvertRGB() { *(DWORD *)this ^= 0x00FFFFFF; };

    operator DWORD() { return *(DWORD *)this; }

    BYTE &operator[](int i) { return *(((BYTE *)this) + i); }

    const BYTE &operator[](int i) const { return *(((BYTE *)this) + i); }

    void HSV(Float &h, Float &s, Float &v)
    {
        Float fr, fg, fb;
        Float maxv, minv, delta = 1;

        fr = r;
        fg = g;
        fb = b;

        if (fr < fg)
        {
            maxv = fg < fb ? fb : fg;
            minv = fb < fr ? fb : fr;
        }
        else
        {
            maxv = fr < fb ? fb : fr;
            minv = fb < fg ? fb : fg;
        }
        v = maxv * (1 / 255.0f);
        s = (maxv - minv) / maxv;
        if (s == 0.0)
            h = -1;
        else
            delta = maxv - minv;

        if (fr == maxv)
            h = (Float)(fg - fb) / delta;
        if (fg == maxv)
            h = 2 + (Float)(fb - fr) / delta;
        if (fb == maxv)
            h = 4 + (Float)(fr - fg) / delta;

        h *= 60.0f;
        if (h < 0)
            h += 360.0f;
    }

    void YUV(Float &y, Float &u, Float &v)
    {
        y = 0.299f * r + 0.587f * g + 0.114f * b;
        u = 0.492f * (b - y) + 128.0f;
        v = 0.877f * (r - y) + 128.0f;
    }
    void FromYUV(Float y, Float u, Float v)
    {
        Float fr, fg, fb;

        u = u - 128;
        v = v - 128;

        fb = y + u * (1.0f / 0.492f);
        fr = y + v * (1.0f / 0.877f);
        fg = (y - 0.299f * fr - 0.114f * fb) * (1.0f / 0.587f);

        if (fr > 255.0)
            r = 255;
        else if (fr < 0.0)
            r = 0;
        else
            r = (unsigned char)fr;

        if (fg > 255.0)
            g = 255;
        else if (fg < 0.0)
            g = 0;
        else
            g = (unsigned char)fg;

        if (fb > 255.0)
            b = 255;
        else if (fb < 0.0)
            b = 0;
        else
            b = (unsigned char)fb;

        a = 255;
    }
};

class ColorF
{
public:
    float r, g, b, a;

    ColorF() {}

    ColorF(float _r, float _g, float _b, float _a = 1.0f)
    {
        r = _r;
        g = _g;
        b = _b;
        a = _a;
    }

    void Set(float _r, float _g, float _b, float _a = 1.0f)
    {
        r = _r;
        g = _g;
        b = _b;
        a = _a;
    }

    float &operator[](int i) { return *(((float *)this) + i); }

    const float &operator[](int i) const { return *(((float *)this) + i); }

    void ConvertToColorI(ColorI &clr)
    {
        float total = r + g + b;
        clr[0] = (BYTE)(r / total * 255);
        clr[1] = (BYTE)(g / total * 255);
        clr[2] = (BYTE)(b / total * 255); /*clr[3] = (BYTE)(a*255)*/
    };
};