#include "Texture.h"
#include <opencv2/opencv.hpp>

bool Texture::Reload(bool flag /*= true*/)
{
    bool ret = true;
    PixelImage &img = GetImage();
    PixelImage limg;

    img.SetSize(512, 512);
    try
    {
        if (flag) //reload by load image
        {
            // String ext = GetName().right(4).lower();
            // String ext5 = GetName().right(5).lower();

            // if (ext == _T(".jpg"))
            // {
            // }
            // else if (ext == _T(".png"))
            // {
            // }
        }
        else //reload blank image
        {
            limg.SetSize(512, 512);
            limg.Fill(ColorI(255, 255, 255));
            img.BitBlt(0, 0, img.Width(), img.Height(), limg, 0, 0, limg.Width(), limg.Height());
        }
    }
    catch (std::runtime_error err)
    {
        img.Clear();
        return false;
    }

    return ret;
}

void Texture::Save(const char *fname)
{
    PixelImage &img = GetImage();
    cv::Mat mat(img.Height(), img.Width(), CV_8UC4, img.GetBuffer(), img.BytesPerLine());
    cv::imwrite(fname, mat);
    cout << __FILE__ << " " << __LINE__ << " Texture::Save img.BytesPerLine()=" << img.BytesPerLine() << endl;
}