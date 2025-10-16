#include "core/Bitmap.h"
#include "ImfArray.h"
#include "ImfRgbaFile.h"
#include "stb_image.h"

void loadBitmap(const std::string &filename, bool raw, Bitmap &bitmap) {
    std::string extension = filename.substr(filename.find_last_of('.') + 1);
    if (extension == "exr") {
        Imf::RgbaInputFile file{filename.c_str()};
        Imath::Box2i dw = file.dataWindow();
        int width = dw.max.x - dw.min.x + 1;
        int height = dw.max.y - dw.min.y + 1;
        Imf::Array2D<Imf::Rgba> temp{height, width};
        file.setFrameBuffer(&temp[0][0] - dw.min.x - dw.min.y * width, 1, width);
        file.readPixels(dw.min.y, dw.max.y);
        bitmap = Bitmap{width, height};
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x) {
                const Imf::Rgba &p = temp[y][x];
                bitmap(x, y) = Vec3f{static_cast<Float>(p.r), static_cast<Float>(p.g), static_cast<Float>(p.b)};
            }
    } else if (extension == "png" || extension == "jpg" || extension == "jpeg") {
        int channels;
        int width, height;
        unsigned char *data = stbi_load(filename.c_str(), &width, &height, &channels, 0);
        if (!data)
            throw std::runtime_error("Failed to load image: " + filename);
        bitmap = Bitmap{width, height};
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x) {
                int idx = (y * width + x) * channels;
                Float r, g, b;
                if (channels == 1)
                    r = g = b = data[idx] / 255.0;
                else if (channels == 3 || channels == 4)
                    r = data[idx] / 255.0, g = data[idx + 1] / 255.0, b = data[idx + 2] / 255.0;
                else {
                    stbi_image_free(data);
                    throw std::runtime_error("Unsupported number of channels in image: " + filename);
                }
                bitmap(x, y) = Vec3f{r, g, b};
            }
        stbi_image_free(data);
        // convert to linear space if not raw
        if (!raw) {
            for (auto &c : bitmap.pixels)
                for (int i = 0; i < 3; ++i)
                    c[i] = (c[i] <= 0.04045) ? (c[i] / 12.92) : std::pow((c[i] + 0.055) / 1.055, 2.4);
        }
    } else {
        throw std::runtime_error("Unsupported image format for Bitmap Texture: " + filename);
    }
}
