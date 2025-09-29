#pragma once
#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "core/MathUtils.h"
#include "stb_image_write.h"


// TODO: right now, no reconstruction filter, just store raw pixel values
class Film {
private:
    uint32_t width = 0, height = 0;
    std::vector<Vec3f> pixels;  // in RGB format, [0, 1], row-major order

public:
    Film(uint32_t width, uint32_t height) : width(width), height(height) {
        pixels.resize(width * height);
    }

    /// y: 0 is bottom, height-1 is top
    /// x: 0 is left, width-1 is right
    void set_pixel(uint32_t x, uint32_t y, const Vec3f &color) {
        if (x < width && y < height) {
            pixels[y * width + x] = color;
        }
    }

    /// Output the image to a file (PNG format)
    void output_image(const std::string &filename) {
        std::vector<uint8_t> img_data(width * height * 3);
        for (uint32_t y = 0; y < height; y++) {
            for (uint32_t x = 0; x < width; x++) {
                // Flip y-coordinate: bottom-up -> top-down
                Vec3f color = pixels[(height - 1 - y) * width + x];
                // clamp and convert to [0, 255]
                img_data[(y * width + x) * 3 + 0] = static_cast<uint8_t>(std::clamp(color.r, 0.0f, 1.0f) * 255.0f);
                img_data[(y * width + x) * 3 + 1] = static_cast<uint8_t>(std::clamp(color.g, 0.0f, 1.0f) * 255.0f);
                img_data[(y * width + x) * 3 + 2] = static_cast<uint8_t>(std::clamp(color.b, 0.0f, 1.0f) * 255.0f);
            }
        }
        stbi_write_png(filename.c_str(), width, height, 3, img_data.data(), width * 3);
    }

    std::string to_string() const {
        std::ostringstream oss;
        oss << "Film: " << "[ resolution=" << width << "x" << height << " ]";
        return oss.str();
    }
};
