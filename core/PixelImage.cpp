#include "PixelImage.h"

////////////////////////////////////////////////////
CopyBlendFunction CopyBlender;
AddBlendFunction AddBlender;
DecBlendFunction DecBlender;
AlphaBlendFunction AlphaBlender;
NormalScaleFunction NormalScaler;
BiLinearScaleFunction BiLinearScaler;
BiCubicScaleFunction BiCubicScaler;
BiCubicSplineScaleFunction BiCubicSplineScaler;

////////////////////////////////////////////////////

class ImageExtender
{
public:
    UINT p0, p1;
    UINT pp, pn;
    int f, g, h;
    UINT b0, b1, b2;
    int t0, t1, t2, t3;
    int tt; //Liner & Cubic
    UINT lp;

    inline void set_t(Float t)
    {
        tt = static_cast<int>(t * 255.0f);
        Float ff, fg, fh;

        t0 = static_cast<int>(((t - 1) * (t - 1) * (2 * t + 1)) * 255.0f);
        t1 = static_cast<int>((t * t * (3 - 2 * t)) * 255.0f);
        t2 = static_cast<int>(((1 - t) * (1 - t) * t) * 255.0f);
        t3 = static_cast<int>(((t - 1) * t * t) * 255.0f);

        if (t < 0.5f)
        {
            t = t + 0.5f;
            ff = 0.5f * (1.0f - t) * (1.0f - t);
            fg = (1.0f - t) * t + 0.5f;
            fh = 0.5f * t * t;
            b0 = pp;
            b1 = p0;
            b2 = p1;
        }
        else
        {
            t = t - 0.5f;
            ff = 0.5f * (1.0f - t) * (1.0f - t);
            fg = (1.0f - t) * t + 0.5f;
            fh = 0.5f * t * t;
            b0 = p0;
            b1 = p1;
            b2 = pn;
        }
        f = static_cast<int>(ff * 255.0f);
        g = static_cast<int>(fg * 255.0f);
        h = static_cast<int>(fh * 255.0f);
    }

    inline void Cubic(ColorI &ret, int u, int v,
                      const ColorI &u0, const ColorI &u1,
                      const ColorI &v0, const ColorI &v1,
                      const ColorI &c00, const ColorI &c01,
                      const ColorI &c10, const ColorI &c11)
    {
        int a = ((255 - u) * (255 - v)) >> 8;
        int b = (u * (255 - v)) >> 8;
        int c = ((255 - u) * v) >> 8;
        int d = (u * v) >> 8;

        ret.r = (BYTE)min(max((int(u0.r) * (255 - v) + int(u1.r) * v + int(v0.r) * (255 - u) + int(v1.r) * u - int(c00.r) * a - int(c01.r) * b - int(c10.r) * c - int(c11.r) * d) >> 8, 0), 255);
        ret.g = (BYTE)min(max((int(u0.g) * (255 - v) + int(u1.g) * v + int(v0.g) * (255 - u) + int(v1.g) * u - int(c00.g) * a - int(c01.g) * b - int(c10.g) * c - int(c11.g) * d) >> 8, 0), 255);
        ret.b = (BYTE)min(max((int(u0.b) * (255 - v) + int(u1.b) * v + int(v0.b) * (255 - u) + int(v1.b) * u - int(c00.b) * a - int(c01.b) * b - int(c10.b) * c - int(c11.b) * d) >> 8, 0), 255);
        ret.a = (BYTE)min(max((int(u0.a) * (255 - v) + int(u1.a) * v + int(v0.a) * (255 - u) + int(v1.a) * u - int(c00.a) * a - int(c01.a) * b - int(c10.a) * c - int(c11.a) * d) >> 8, 0), 255);
    }

    void CSpline(ColorI &ret, const ColorI &a, const ColorI &b, const ColorI &c, const ColorI &d)
    {
        if (p0 == p1)
        {
            ret = b;
            return;
        }
        int d0[4], d1[4];
        if (pp == p0)
        {
            d0[0] = int(b.b) - int(a.b);
            d0[1] = int(b.g) - int(a.g);
            d0[2] = int(b.r) - int(a.r);
            d0[3] = int(b.a) - int(a.a);
        }
        else
        {
            d0[0] = (-int(a.b) + int(c.b)) >> 1;
            d0[1] = (-int(a.g) + int(c.g)) >> 1;
            d0[2] = (-int(a.r) + int(c.r)) >> 1;
            d0[3] = (-int(a.a) + int(c.a)) >> 1;
        }
        if (p1 == pn)
        {
            d1[0] = int(b.b) - int(a.b);
            d1[1] = int(b.g) - int(a.g);
            d1[2] = int(b.r) - int(a.r);
            d1[3] = int(b.a) - int(a.a);
        }
        else
        {
            d1[0] = (int(d.b) - int(b.b)) >> 1;
            d1[1] = (int(d.g) - int(b.g)) >> 1;
            d1[2] = (int(d.r) - int(b.r)) >> 1;
            d1[3] = (int(d.a) - int(b.a)) >> 1;
        }
        ret.b = (BYTE)min(max(int(b.b) * t0 + int(c.b) * t1 + d0[0] * t2 + d1[0] * t3, 0) >> 8, 255);
        ret.g = (BYTE)min(max(int(b.g) * t0 + int(c.g) * t1 + d0[1] * t2 + d1[1] * t3, 0) >> 8, 255);
        ret.r = (BYTE)min(max(int(b.r) * t0 + int(c.r) * t1 + d0[2] * t2 + d1[2] * t3, 0) >> 8, 255);
        ret.a = (BYTE)min(max(int(b.a) * t0 + int(c.a) * t1 + d0[3] * t2 + d1[3] * t3, 0) >> 8, 255);
    }

    inline void BSpline(ColorI &ret, const ColorI &a, const ColorI &b, const ColorI &c)
    {
        ret.r = (BYTE)((WORD(a.r) * f + WORD(b.r) * g + WORD(c.r) * h) >> 8);
        ret.g = (BYTE)((WORD(a.g) * f + WORD(b.g) * g + WORD(c.g) * h) >> 8);
        ret.b = (BYTE)((WORD(a.b) * f + WORD(b.b) * g + WORD(c.b) * h) >> 8);
        ret.a = (BYTE)((WORD(a.a) * f + WORD(b.a) * g + WORD(c.a) * h) >> 8);
    }
    inline void Linear(ColorI &ret, const ColorI &a, const ColorI &b)
    {
        UINT t0 = (255 - tt);
        ret.r = (BYTE)((WORD(a.r) * t0 + WORD(b.r) * tt) >> 8);
        ret.g = (BYTE)((WORD(a.g) * t0 + WORD(b.g) * tt) >> 8);
        ret.b = (BYTE)((WORD(a.b) * t0 + WORD(b.b) * tt) >> 8);
        ret.a = (BYTE)((WORD(a.a) * t0 + WORD(b.a) * tt) >> 8);
    }
};

class ImageScaleTable
{
public:
    inline ImageScaleTable(PixelImage &_imgDest, const PixelImage &_imgSrc, int _x, int _y, int w, int h)
    {
        int x, y;
        imgDest = &_imgDest;
        imgSrc = &_imgSrc;
        xSrc = _x;
        ySrc = _y;
        wSrc = w;
        hSrc = h;
        wDst = _imgDest.Width();
        hDst = _imgDest.Height();

        x_table.resize(wDst);
        y_table.resize(hDst);
        x_rtable.resize(wSrc);
        y_rtable.resize(hSrc);

        dx = Float(wSrc) / Float(wDst);
        dy = Float(hSrc) / Float(hDst);

        for (x = 0; x < wDst; x++)
        {
            Float fpos = Float(x) * dx;
            x_table[x].lp = static_cast<UINT>(fpos + xSrc);
            fpos -= 0.5f;
            int ipos = (int)(fpos + xSrc);
            x_table[x].pp = max(ipos - 1, 0);
            x_table[x].p0 = max(min(ipos, xSrc + wSrc - 1), 0);
            x_table[x].p1 = max(min(ipos + 1, xSrc + wSrc - 1), 0);
            x_table[x].pn = max(min(ipos + 2, xSrc + wSrc - 1), 0);
            x_table[x].set_t(fpos - ipos);
        }
        for (y = 0; y < hDst; y++)
        {
            Float fpos = Float(y) * dy;
            y_table[y].lp = static_cast<UINT>(fpos);
            fpos -= 0.5f;
            int ipos = (int)fpos;
            y_table[y].pp = max(ipos - 1, 0);
            y_table[y].p0 = max(min(ipos, hSrc - 1), 0);
            y_table[y].p1 = max(min(ipos + 1, hSrc - 1), 0);
            y_table[y].pn = max(min(ipos + 2, hSrc - 1), 0);
            y_table[y].set_t(fpos - ipos);
        }
    }

    inline void MakeReduceTable()
    {
        int x, y;

        for (x = 0; x < wSrc; x++)
            x_rtable[x] = min(UINT(Float(x) / Float(wSrc - 1) * wDst), (UINT)wDst - 1);
        for (y = 0; y < hSrc; y++)
            y_rtable[y] = min(UINT(Float(y) / Float(hSrc - 1) * hDst), (UINT)hDst - 1);
    }

    inline void ScaleBiCubic(bool spline)
    {
        int x, y;

        if (wDst > wSrc && hDst > hSrc)
        {
            const PixelImage &s = *imgSrc;
            ColorI u0, u1, v0, v1;

            if (spline)
            {
                for (y = 0; y < hDst; y++)
                {
                    ImageExtender &ey = y_table[y];
                    ColorI *dest = &(*imgDest)(0, y);
                    const ColorI *s0 = &(*imgSrc)(0, ey.p0);
                    const ColorI *s1 = &(*imgSrc)(0, ey.p1);

                    for (x = 0; x < wDst; x++)
                    {
                        ImageExtender &ex = x_table[x];
                        ex.CSpline(u0, *(s0 + ex.pp), *(s0 + ex.p0), *(s0 + ex.p1), *(s0 + ex.pn));
                        ex.CSpline(u1, *(s1 + ex.pp), *(s1 + ex.p0), *(s1 + ex.p1), *(s1 + ex.pn));
                        ey.CSpline(v0, s(ex.p0, ey.pp), s(ex.p0, ey.p0), s(ex.p0, ey.p1), s(ex.p0, ey.pn));
                        ey.CSpline(v1, s(ex.p1, ey.pp), s(ex.p1, ey.p0), s(ex.p1, ey.p1), s(ex.p1, ey.pn));

                        ex.Cubic(*dest++,
                                 ex.tt, ey.tt,
                                 u0, u1, v0, v1,
                                 s(ex.p0, ey.p0), s(ex.p1, ey.p0), s(ex.p0, ey.p1), s(ex.p1, ey.p1));
                    }
                }
            }
            else
            {
                for (y = 0; y < hDst; y++)
                {
                    ImageExtender &ey = y_table[y];
                    ColorI *dest = &(*imgDest)(0, y);
                    const ColorI *s0 = &(*imgSrc)(0, ey.p0);
                    const ColorI *s1 = &(*imgSrc)(0, ey.p1);

                    for (x = 0; x < wDst; x++)
                    {
                        ImageExtender &ex = x_table[x];
                        ex.BSpline(u0, *(s0 + ex.b0), *(s0 + ex.b1), *(s0 + ex.b2));
                        ex.BSpline(u1, *(s1 + ex.b0), *(s1 + ex.b1), *(s1 + ex.b2));
                        ey.BSpline(v0, s(ex.p0, ey.b0), s(ex.p0, ey.b1), s(ex.p0, ey.b2));
                        ey.BSpline(v1, s(ex.p1, ey.b0), s(ex.p1, ey.b1), s(ex.p1, ey.b2));

                        ex.Cubic(*dest++,
                                 ex.tt, ey.tt,
                                 u0, u1, v0, v1,
                                 s(ex.p0, ey.p0), s(ex.p1, ey.p0), s(ex.p0, ey.p1), s(ex.p1, ey.p1));
                    }
                }
            }
        }
        else
        {
            ScaleBiLinear();
        }
    }

    inline void ScaleBiLinear()
    {
        int x, y;

        PixelImage tbuf(wDst, hSrc);

        MakeReduceTable();

        if (wDst > wSrc)
        {
            for (y = 0; y < hSrc; y++)
            {
                ColorI *destptr = &tbuf(0, y);
                for (x = 0; x < wDst; x++)
                {
                    const ColorI &s0 = (*imgSrc)(x_table[x].p0, y + ySrc);
                    const ColorI &s1 = (*imgSrc)(x_table[x].p1, y + ySrc);
                    x_table[x].Linear(*destptr++, s0, s1);
                }
            }
        }
        else
        {

            for (y = 0; y < hSrc; y++)
            {
                UINT last = x_rtable[0];
                UINT count = 0;
                WORD r = 0, g = 0, b = 0, a = 0;

                for (x = 0; x < wSrc; x++)
                {
                    if (x_rtable[x] != last)
                    {
                        tbuf(last, y).Set((BYTE)(r / count), (BYTE)(g / count), (BYTE)(b / count), (BYTE)(a / count));
                        r = 0, g = 0, b = 0, a = 0;
                        count = 0;
                        last = x_rtable[x];
                    }
                    const ColorI &c = (*imgSrc)(x + xSrc, y + ySrc);
                    r = (WORD)(r + c.r);
                    g = (WORD)(g + c.g);
                    b = (WORD)(b + c.b);
                    a = (WORD)(a + c.a);
                    count++;
                }
                if (count)
                    tbuf(last, y).Set((BYTE)(r / count), (BYTE)(g / count), (BYTE)(b / count), (BYTE)(a / count));
            }
        }

        if (hDst > hSrc)
        {
            for (x = 0; x < wDst; x++)
            {
                for (y = 0; y < hDst; y++)
                {
                    const ColorI &s0 = tbuf(x, y_table[y].p0);
                    const ColorI &s1 = tbuf(x, y_table[y].p1);
                    y_table[y].Linear((*imgDest)(x, y), s0, s1);
                }
            }
        }
        else
        {
            for (x = 0; x < wDst; x++)
            {
                UINT last = y_rtable[0];
                UINT count = 0;
                WORD r = 0, g = 0, b = 0, a = 0;

                for (y = 0; y < hSrc; y++)
                {
                    if (y_rtable[y] != last)
                    {
                        (*imgDest)(x, last).Set((BYTE)(r / count), (BYTE)(g / count), (BYTE)(b / count), (BYTE)(a / count));
                        r = 0, g = 0, b = 0, a = 0;
                        count = 0;
                        last = y_rtable[y];
                    }
                    const ColorI &c = tbuf(x, y);
                    r = (WORD)(r + c.r);
                    g = (WORD)(g + c.g);
                    b = (WORD)(b + c.b);
                    a = (WORD)(a + c.a);
                    count++;
                }
                if (count)
                    (*imgDest)(x, last).Set((BYTE)(r / count), (BYTE)(g / count), (BYTE)(b / count), (BYTE)(a / count));
            }
        }
    }

    inline void ScaleNormal()
    {
        int x, y;

        for (y = 0; y < hDst; y++)
        {
            ColorI *dstptr = &(*imgDest)(0, y);
            const ColorI *srcptr = &(*imgSrc)(0, y_table[y].lp + ySrc);
            for (x = 0; x < wDst; x++)
            {
                *dstptr++ = *(srcptr + x_table[x].lp);
            }
        }
    }

private:
    PixelImage *imgDest;
    const PixelImage *imgSrc;
    vector<ImageExtender> x_table;
    vector<ImageExtender> y_table;
    vector<UINT> x_rtable;
    vector<UINT> y_rtable;
    int xSrc, ySrc, wSrc, hSrc;
    int wDst, hDst;
    Float dx, dy;
};

////////////////////////////////////////////////////
//ImageScaler
bool NormalScaleFunction::operator()(PixelImage &imgDest, const PixelImage &imgSrc,
                                     int xSrc, int ySrc, int wSrc, int hSrc)
{

    if (xSrc + wSrc > static_cast<int>(imgSrc.Width()))
        return false;
    if (ySrc + hSrc > static_cast<int>(imgSrc.Height()))
        return false;

    ImageScaleTable table(imgDest, imgSrc, xSrc, ySrc, wSrc, hSrc);

    table.ScaleNormal();

    return true;
}

bool BiLinearScaleFunction::operator()(PixelImage &imgDest, const PixelImage &imgSrc,
                                       int xSrc, int ySrc, int wSrc, int hSrc)
{
    if (xSrc + wSrc > static_cast<int>(imgSrc.Width()))
        return false;
    if (ySrc + hSrc > static_cast<int>(imgSrc.Height()))
        return false;

    ImageScaleTable table(imgDest, imgSrc, xSrc, ySrc, wSrc, hSrc);

    table.ScaleBiLinear();

    return true;
}

bool BiCubicScaleFunction::operator()(PixelImage &imgDest, const PixelImage &imgSrc,
                                      int xSrc, int ySrc, int wSrc, int hSrc)
{
    if (xSrc + wSrc > static_cast<int>(imgSrc.Width()))
        return false;
    if (ySrc + hSrc > static_cast<int>(imgSrc.Height()))
        return false;

    ImageScaleTable table(imgDest, imgSrc, xSrc, ySrc, wSrc, hSrc);
    table.ScaleBiCubic(false);

    return true;
}

bool BiCubicSplineScaleFunction::operator()(PixelImage &imgDest, const PixelImage &imgSrc,
                                            int xSrc, int ySrc, int wSrc, int hSrc)
{
    if (xSrc + wSrc > static_cast<int>(imgSrc.Width()))
        return false;
    if (ySrc + hSrc > static_cast<int>(imgSrc.Height()))
        return false;

    ImageScaleTable table(imgDest, imgSrc, xSrc, ySrc, wSrc, hSrc);
    table.ScaleBiCubic(true);

    return true;
}

////////////////////////////////////////////////////
//PixelImage
PixelImage::PixelImage()
{
    m_fWidth = 100;
    m_fHeight = 100;
    m_width = 0;
    m_height = 0;
}

PixelImage::PixelImage(UINT w, UINT h)
{
    SetSize(w, h);
    m_fWidth = 100;
    m_fHeight = 100;
}

PixelImage::~PixelImage()
{
    Clear();
}

void PixelImage::Clear()
{
    m_pixel.clear();
    m_width = 0;
    m_height = 0;
}

void PixelImage::SetSize(UINT width, UINT height)
{
    Clear();

    m_width = width;
    m_height = height;
    m_pixel.resize(m_width * m_height);
}

//////////////////////////////////////////////////////////////////////////////////////
void PixelImage::BitBlt(int xDest, int yDest, const PixelImage &imgSrc, PixelBlender &blender)
{
    int x, y;

    int start_x = max(0, -xDest);
    int start_y = max(0, -yDest);
    int end_x = min(imgSrc.Width(), Width() - xDest);
    int end_y = min(imgSrc.Height(), Height() - yDest);

    for (y = start_y; y < end_y; y++)
    {
        const ColorI *src = &imgSrc(start_x, y);
        ColorI *dest = &(*this)(start_x + xDest, y + yDest);
        for (x = start_x; x < end_x; x++)
        {
            blender(*dest++, *src++);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////
void PixelImage::BitBlt(int xDest, int yDest, const PixelImage &imgSrc)
{
    int y, sy;

    int start_x = max(0, -xDest);
    int start_y = max(0, -yDest);
    int end_x = min(imgSrc.Width(), Width() - xDest);
    int end_y = min(imgSrc.Height(), Height() - yDest);

    for (sy = 0, y = start_y; y < end_y; y++, sy++)
    {
        ::memcpy(&(*this)(xDest, y), &imgSrc(0, sy), sizeof(ColorI) * (end_x - start_x));
    }
}

///////////////////////////////////////////////////////////////////////
bool PixelImage::FitImage(const PixelImage &imgSrc, int xSrc, int ySrc, int wSrc, int hSrc, ImageScaler &scaler)
{
    return scaler(*this, imgSrc, xSrc, ySrc, wSrc, hSrc);
}

///////////////////////////////////////////////////////////////////////
bool PixelImage::BitBlt(int xDest, int yDest, int wDest, int hDest,
                        const PixelImage &imgSrc, int xSrc, int ySrc, int wSrc, int hSrc,
                        PixelBlender &blender, ImageScaler &scaler)
{
    PixelImage tbuf(wDest, hDest);

    if (!tbuf.FitImage(imgSrc, xSrc, ySrc, wSrc, hSrc, scaler))
        return false;
    BitBlt(xDest, yDest, tbuf, blender);

    return true;
}

void PixelImage::FlipX()
{
    int x, y;
    PixelImage tmp;
    tmp = *this;
    for (y = 0; y < static_cast<int>(Height()); y++)
    {
        for (x = 0; x < static_cast<int>(Width()); x++)
        {
            (*this)(x, y) = tmp(Width() - x, y);
        }
    }
}

void PixelImage::FlipY()
{
    int x, y;
    PixelImage tmp;
    tmp = *this;
    for (y = 0; y < static_cast<int>(Height()); y++)
    {
        for (x = 0; x < static_cast<int>(Width()); x++)
        {
            (*this)(x, y) = tmp(x, Height() - y - 1);
        }
    }
}
