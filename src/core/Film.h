#pragma once
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "core/MathUtils.h"
#include "stb_image_write.h"

// TODO: right now, no reconstruction filter, just store raw pixel values
class Film {
public:
    std::vector<Vec3f> pixels;  // in RGB format, [0, 1], row-major order
    uint32_t width, height;

    Film(uint32_t width, uint32_t height) : width(width), height(height), pixels(width * height) {}

    /// row: 0 is bottom, height-1 is top
    /// col: 0 is left, width-1 is right
    void set_pixel(uint32_t row, uint32_t col, const Vec3f& color) {
        if (col >= width || row >= height)
            throw std::out_of_range("Pixel coordinates out of range");
        pixels[row * width + col] = color;
    }

    /// Output the image to a file (PNG format)
    void output_image(const std::string& filename, bool tone_mapping = true) const {
        std::vector<Vec3f> mapped_pixels = pixels;
        if (tone_mapping) {
            for (auto& color : mapped_pixels) {
                color = color / (color + Vec3f{1.0});
                color = glm::pow(color, Vec3f{1.0 / 2.2});  // gamma correction
            }
        }
        
        if (filename.size() >= 4 && filename.substr(filename.size() - 4) == ".png")
            save_png(filename, mapped_pixels);
        else if (filename.size() >= 4 && filename.substr(filename.size() - 4) == ".ppm")
            save_ppm(filename, mapped_pixels);
        else
            throw std::invalid_argument("Unsupported file format. Use .png or .ppm");
    }

    void save_png(const std::string& filename, const std::vector<Vec3f>& pixels) const {
        std::vector<uint8_t> img_data(width * height * 3);
        for (uint32_t row = 0; row < height; row++) {
            for (uint32_t col = 0; col < width; col++) {
                // Flip y-coordinate: bottom-up -> top-down
                Vec3f color = pixels[(height - 1 - row) * width + col];
                // clamp and convert to [0, 255]
                img_data[(row * width + col) * 3 + 0] = static_cast<uint8_t>(std::clamp(color.r, Float(0.0), Float(1.0)) * 255.0 + 0.5);
                img_data[(row * width + col) * 3 + 1] = static_cast<uint8_t>(std::clamp(color.g, Float(0.0), Float(1.0)) * 255.0 + 0.5);
                img_data[(row * width + col) * 3 + 2] = static_cast<uint8_t>(std::clamp(color.b, Float(0.0), Float(1.0)) * 255.0 + 0.5);
            }
        }
        stbi_write_png(filename.c_str(), width, height, 3, img_data.data(), width * 3);
    }

    void save_ppm(const std::string& filename, const std::vector<Vec3f>& pixels) const {
        std::ofstream file(filename, std::ios::binary);

        // PPM header
        file << "P6\n";
        file << width << " " << height << "\n";
        file << "255\n";

        // Write pixels with y-flip: start from top row (height-1) down to bottom row (0)
        for (uint32_t y = 0; y < height; ++y) {
            for (uint32_t x = 0; x < width; ++x) {
                // Flip y-coordinate: map PPM row y to your storage row (height-1-y)
                int index = (height - 1 - y) * width + x;
                const Vec3f& pixel = pixels[index];

                // Clamp and convert to 0-255 range
                uint8_t r = static_cast<uint8_t>(std::clamp(pixel.r, Float(0.0), Float(1.0)) * 255.0 + 0.5);
                uint8_t g = static_cast<uint8_t>(std::clamp(pixel.g, Float(0.0), Float(1.0)) * 255.0 + 0.5);
                uint8_t b = static_cast<uint8_t>(std::clamp(pixel.b, Float(0.0), Float(1.0)) * 255.0 + 0.5);

                file.write(reinterpret_cast<const char*>(&r), 1);
                file.write(reinterpret_cast<const char*>(&g), 1);
                file.write(reinterpret_cast<const char*>(&b), 1);
            }
        }
        file.close();
    }

    std::string to_string() const {
        std::ostringstream oss;
        oss << "Film: " << "[ resolution=" << width << "x" << height << " ]";
        return oss.str();
    }
};
