#pragma once
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include "core/MathUtils.h"
#include "core/RFilter.h"
#include "stb_image_write.h"

class Film {
private:
    std::vector<Float> pixels_weights_sum;
    std::vector<std::mutex> pixels_mutex;
    const RFilter* rfilter;

public:
    std::vector<Vec3f> pixels;  // in RGB format, [0, 1], row-major order
    uint32_t width, height;

    Film(uint32_t width, uint32_t height, const RFilter* rfilter) : width(width), height(height), pixels(width * height), pixels_weights_sum(width * height), pixels_mutex(width * height), rfilter(rfilter) {}

    /// row: 0 is bottom, height-1 is top
    /// col: 0 is left, width-1 is right
    /// px, py are the sample position in sensor space (in range [0, 1])
    void commit_sample(const Vec3f& value, uint32_t row, uint32_t col, Float px, Float py) {
        Float x = px * width - (col + 0.5);
        Float y = py * height - (row + 0.5);
        for (int r = row - rfilter->bound; r <= row + rfilter->bound; r++) {
            if (r < 0 || r >= height)
                continue;
            for (int c = col - rfilter->bound; c <= col + rfilter->bound; c++) {
                if (c < 0 || c >= width)
                    continue;
                Float filter_weight = rfilter->eval(x - (c - col), y - (r - row));
                std::lock_guard<std::mutex> lock(pixels_mutex[r * width + c]);
                pixels[r * width + c] += filter_weight * value;
                pixels_weights_sum[r * width + c] += filter_weight;
            }
        }
    }

    /// called after the rendering is complete. divides each pixel by its weight_sum
    void normalize_pixels() {
        for (uint32_t row = 0; row < height; row++)
            for (uint32_t col = 0; col < width; col++)
                if (pixels_weights_sum[row * width + col] <= 0)
                    pixels[row * width + col] = Vec3f{0.0};
                else
                    pixels[row * width + col] /= pixels_weights_sum[row * width + col];
    }

    /// Output the image to a file (PNG format)
    void output_image(const std::string& filename, bool raw = false) const {
        std::vector<Vec3f> mapped_pixels = pixels;
        if (!raw) {
            for (auto& color : mapped_pixels) {
                // tonemapping
                color = color / (color + Vec3f{1.0});

                // gamma correction
                for (int i = 0; i < 3; i++) {
                    if (color[i] <= 0.0031308)
                        color[i] = Float(12.92) * color[i];
                    else
                        color[i] = 1.055 * std::pow(color[i], 1.0 / 2.4) - 0.055;
                }
            }
        }

        if (filename.size() >= 4 && filename.substr(filename.size() - 4) == ".ppm")
            save_ppm(filename, mapped_pixels);
        else if (filename.size() >= 4 && filename.substr(filename.size() - 4) == ".hdr")
            save_hdr(filename, pixels);
        else
            save_image(filename, mapped_pixels);
    }

    void save_hdr(const std::string& filename, const std::vector<Vec3f>& pixels) const {
        std::vector<float> img_data(width * height * 3);
        for (uint32_t row = 0; row < height; row++) {
            for (uint32_t col = 0; col < width; col++) {
                // Flip y-coordinate: bottom-up -> top-down
                Vec3f color = pixels[(height - 1 - row) * width + col];
                img_data[(row * width + col) * 3 + 0] = color.r;
                img_data[(row * width + col) * 3 + 1] = color.g;
                img_data[(row * width + col) * 3 + 2] = color.b;
            }
        }
        stbi_write_hdr(filename.c_str(), width, height, 3, img_data.data());
    }
    
    void save_image(const std::string& filename, const std::vector<Vec3f>& pixels) const {
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
        if (filename.ends_with(".png")) {
            stbi_write_png(filename.c_str(), width, height, 3, img_data.data(), width * 3);
        } else if (filename.ends_with(".jpg") || filename.ends_with(".jpeg")) {
            stbi_write_jpg(filename.c_str(), width, height, 3, img_data.data(), 95);  // quality = 95
        } else if (filename.ends_with(".bmp")) {
            stbi_write_bmp(filename.c_str(), width, height, 3, img_data.data());
        } else if (filename.ends_with(".tga")) {
            stbi_write_tga(filename.c_str(), width, height, 3, img_data.data());
        } else {
            throw std::invalid_argument("Unsupported file format.");
        }
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
