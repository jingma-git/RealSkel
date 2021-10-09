#pragma once

#include "PixelImage.h"
#include "common.h"

class Texture
{
    int GetPaintColorIdx(ColorI color);

public:
    enum TEXTURE_TYPE
    {
        DIFFUSE,
        BUMP,
        ALPHA,
        LIGHT
    };

public:
    Texture()
    {
        m_Type = DIFFUSE;
        m_texID = m_TextureHandle = 0xffffffff;
        m_bUseMipMap = true;
    }

    bool Reload(bool flag = true);

    void SetType(DWORD type) { m_Type = type; }
    DWORD GetType() { return m_Type; }

    PixelImage &GetImage() { return m_PixelImage; }
    const PixelImage &GetImage() const { return m_PixelImage; }

    bool &UseMipMap() { return m_bUseMipMap; }
    bool UseMipMap() const { return m_bUseMipMap; }

    const UINT &GetTextureID() const { return m_texID; }
    void SetTextureID(UINT id) { m_texID = id; }

    void Save(const char *fname);

public:
    DWORD m_TextureHandle;

protected:
    PixelImage m_PixelImage;
    DWORD m_Type;
    bool m_bUseMipMap;
    UINT m_texID;
};