#pragma once
#include <algorithm>
#include "common.h"
#include "Color.h"

using namespace std;
class PixelImage;

////////////////////////////////////////////////////
//Blender
class PixelBlender
{
public:
    virtual void operator()(ColorI &, const ColorI &) = 0;
};

////////////////////////////////////////////////////
//Scaler
class ImageScaler
{
public:
    virtual bool operator()(PixelImage &, const PixelImage &, int x, int y, int w, int h) = 0;
};

////////////////////////////////////////////////////
//Blender Functions
class CopyBlendFunction : public PixelBlender
{
public:
    void operator()(ColorI &d, const ColorI &s) { d = s; }
};

class AddBlendFunction : public PixelBlender
{
public:
    void operator()(ColorI &d, const ColorI &s)
    {
        d.b = (BYTE)std::min(int(d.b) + int(s.b), 255);
        d.g = (BYTE)std::min(int(d.g) + int(s.g), 255);
        d.r = (BYTE)std::min(int(d.r) + int(s.r), 255);
        d.a = (BYTE)std::min(int(d.a) + int(s.a), 255);
    }
};

class DecBlendFunction : public PixelBlender
{
public:
    void operator()(ColorI &d, const ColorI &s)
    {
        d.b = (BYTE)max(int(d.b) - int(s.b), 0);
        d.g = (BYTE)max(int(d.g) - int(s.g), 0);
        d.r = (BYTE)max(int(d.r) - int(s.r), 0);
        d.a = (BYTE)max(int(d.a) - int(s.a), 0);
    }
};

class AlphaBlendFunction : public PixelBlender
{
public:
    void operator()(ColorI &d, const ColorI &s)
    {
        d.b = min((d.b * (255 - s.a) + s.b * s.a) >> 8, 255);
        d.g = min((d.g * (255 - s.a) + s.g * s.a) >> 8, 255);
        d.r = min((d.r * (255 - s.a) + s.r * s.a) >> 8, 255);
        d.a = min((d.a + s.a), 255);
    }
};

// https://www.cnblogs.com/yc_sunniwell/archive/2010/07/14/1777431.html
// 'extern' indicates a global declaration that should be defined once in .cpp,
//  and can be used multiple times in other .cpp files
// 'static' also indicates a global scope, but only restricted on the file that defined it
// so, it is usually defined on .cpp file
// extern const T var; // a global scope const variable
extern CopyBlendFunction CopyBlender;
extern AddBlendFunction AddBlender;
extern DecBlendFunction DecBlender;
extern AlphaBlendFunction AlphaBlender;

////////////////////////////////////////////////////
//Scaler Functions
class NormalScaleFunction : public ImageScaler
{
public:
    bool operator()(PixelImage &, const PixelImage &, int x, int y, int w, int h);
};

class BiLinearScaleFunction : public ImageScaler
{
public:
    bool operator()(PixelImage &, const PixelImage &, int x, int y, int w, int h);
};

class BiCubicScaleFunction : public ImageScaler
{
public:
    bool operator()(PixelImage &, const PixelImage &, int x, int y, int w, int h);
};

class BiCubicSplineScaleFunction : public ImageScaler
{
public:
    bool operator()(PixelImage &, const PixelImage &, int x, int y, int w, int h);
};

extern NormalScaleFunction NormalScaler;
extern BiLinearScaleFunction BiLinearScaler;
extern BiCubicScaleFunction BiCubicScaler;
extern BiCubicSplineScaleFunction BiCubicSplineScaler;

////////////////////////////////////////////////////
//PixelImage
class PixelImage
{
public:
    PixelImage();
    PixelImage(UINT w, UINT h);
    ~PixelImage();

    void Clear();

    UINT Width() const { return m_width; }
    UINT Height() const { return m_height; }
    size_t BytesPerLine() const { return m_width * sizeof(ColorI); };

    // ToDO: use cvector
    // ColorI *GetBuffer() { return m_pixel.begin(); }
    // const ColorI *GetBuffer() const { return m_pixel.begin(); }
    // const ColorI *GetBufferEnd() const { return m_pixel.end(); };

    // https://www.geeksforgeeks.org/difference-between-iterators-and-pointers-in-c-c-with-examples/
    ColorI *GetBuffer() { return m_pixel.data(); }
    const ColorI *GetBuffer() const { return m_pixel.data(); }
    const ColorI *GetBufferEnd() const { return m_pixel.data() + m_pixel.size(); }; //&(*m_pixel.end())

    ColorI &operator()(UINT x, UINT y) { return m_pixel[x + y * m_width]; }
    const ColorI &operator()(UINT x, UINT y) const { return m_pixel[x + y * m_width]; }
    bool IsAroundExistBlack(UINT x, UINT y) const
    {
        assert(y * m_width + x < static_cast<UINT>(m_pixel.size()));
        int i = (int)(y * m_width + x);
        if (x > 0)
            if (m_pixel.at(i - 1).IsBlack())
                return true;
        if (x < m_width - 1)
            if (m_pixel.at(i + 1).IsBlack())
                return true;
        if (y > 0)
            if (m_pixel.at(i - m_width).IsBlack())
                return true;
        if (y < m_height - 1)
            if (m_pixel.at(i + m_width).IsBlack())
                return true;
        return false;
    };

    bool IsAroundExistWhite(UINT x, UINT y) const
    {
        assert(y * m_width + x < static_cast<UINT>(m_pixel.size()));
        int i = (int)(y * m_width + x);
        if (x > 0)
            if (m_pixel.at(i - 1).IsWhite())
                return true;
        if (x < m_width - 1)
            if (m_pixel.at(i + 1).IsWhite())
                return true;
        if (y > 0)
            if (m_pixel.at(i - m_width).IsWhite())
                return true;
        if (y < m_height - 1)
            if (m_pixel.at(i + m_width).IsWhite())
                return true;
        return false;
    };

    bool IsAroundExistTrans(UINT x, UINT y) const
    {
        assert(y * m_width + x < static_cast<UINT>(m_pixel.size()));
        int i = (int)(y * m_width + x);
        if (x > 0)
            if (m_pixel.at(i - 1).IsTrans())
                return true;
        if (x < m_width - 1)
            if (m_pixel.at(i + 1).IsTrans())
                return true;
        if (y > 0)
            if (m_pixel.at(i - m_width).IsTrans())
                return true;
        if (y < m_height - 1)
            if (m_pixel.at(i + m_width).IsTrans())
                return true;
        return false;
    };

    ColorI GetBestColor(Float x, Float y) const
    {
        assert(x >= 0 && x < m_width && y >= 0 && y < m_height);
        UINT a = (int)(x);
        UINT b = (int)(y);
        Float dx = x - a;
        if (a == m_width - 1)
            dx = 0;
        Float dy = y - b;
        if (b == m_height - 1)
            dy = 0;
        Float invdx = 1 - dx;
        Float invdy = 1 - dy;
        ColorI clr;
        const ColorI *baseC = &(m_pixel.at(a + b * m_width));
        assert(baseC < &m_pixel.back()); //ToDO: is this similar to m_pixel.data() + m_pixel.size()
        clr.r = (BYTE)(invdx * invdy * baseC->r);
        clr.g = (BYTE)(invdx * invdy * baseC->g);
        clr.b = (BYTE)(invdx * invdy * baseC->b);
        //clr.a = 0xFF;
        clr.a = (BYTE)(invdx * invdy * baseC->a);

        if (a >= m_width - 1)
        {
            clr.r = clr.r + (BYTE)(dx * invdy * baseC->r);
            clr.g = clr.g + (BYTE)(dx * invdy * baseC->g);
            clr.b = clr.b + (BYTE)(dx * invdy * baseC->b);
            clr.a = clr.a + (BYTE)(dx * invdy * baseC->a);
        }
        else
        {
            baseC++;
            assert(baseC < &m_pixel.back()); //ToDO: is this similar to m_pixel.data() + m_pixel.size()
            clr.r = clr.r + (BYTE)(dx * invdy * baseC->r);
            clr.g = clr.g + (BYTE)(dx * invdy * baseC->g);
            clr.b = clr.b + (BYTE)(dx * invdy * baseC->b);
            clr.a = clr.a + (BYTE)(dx * invdy * baseC->a);
            baseC--;
        }
        if (b >= m_height - 1)
        {
            clr.r = clr.r + (BYTE)(invdx * dy * baseC->r);
            clr.g = clr.g + (BYTE)(invdx * dy * baseC->g);
            clr.b = clr.b + (BYTE)(invdx * dy * baseC->b);
            clr.a = clr.a + (BYTE)(invdx * dy * baseC->a);
        }
        else
        {
            baseC += m_width;
            assert(baseC < &m_pixel.back());
            clr.r = clr.r + (BYTE)(invdx * dy * baseC->r);
            clr.g = clr.g + (BYTE)(invdx * dy * baseC->g);
            clr.b = clr.b + (BYTE)(invdx * dy * baseC->b);
            clr.a = clr.a + (BYTE)(invdx * dy * baseC->a);
        }
        if (a >= m_width - 1)
        {
            clr.r = clr.r + (BYTE)(dx * dy * baseC->r);
            clr.g = clr.g + (BYTE)(dx * dy * baseC->g);
            clr.b = clr.b + (BYTE)(dx * dy * baseC->b);
            clr.a = clr.a + (BYTE)(dx * dy * baseC->a);
        }
        else
        {
            baseC++;
            assert(baseC < &m_pixel.back());
            clr.r = clr.r + (BYTE)(dx * dy * baseC->r);
            clr.g = clr.g + (BYTE)(dx * dy * baseC->g);
            clr.b = clr.b + (BYTE)(dx * dy * baseC->b);
            clr.a = clr.a + (BYTE)(dx * dy * baseC->a);
        }
        return clr;
    };

    void SetSize(UINT width, UINT height);
    void FlipX();
    void FlipY();

    //////////////////////////////////////////////////////////////////////

    bool FitImage(const PixelImage &imgSrc, int xSrc, int ySrc, int wSrc, int hSrc, ImageScaler &scaler = BiCubicScaler);

    void BitBlt(int xDest, int yDest, const PixelImage &imgSrc, PixelBlender &blender);
    void BitBlt(int xDest, int yDest, const PixelImage &imgSrc);

    bool BitBlt(int xDest, int yDest, int wDest, int hDest,
                const PixelImage &imgSrc, int xSrc, int ySrc, int wSrc, int hSrc,
                PixelBlender &blender = CopyBlender, ImageScaler &scaler = BiCubicScaler);

    void Fill(const ColorI &col) { std::fill(m_pixel.begin(), m_pixel.end(), col); }

    Float m_fWidth;  // the real width of the image in millimeter
    Float m_fHeight; // the real height of the image in millimeter
protected:
    vector<ColorI> m_pixel;
    UINT m_width;
    UINT m_height;
};
