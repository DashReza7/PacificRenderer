#include <vector>
#include "core/MathUtils.h"

/// @brief RGB bitmap image
class Bitmap {
public:
    int width;
    int height;
    std::vector<Vec3f> pixels; // Stored in row-major order
    
    Bitmap() : width(0), height(0) {}
    Bitmap(int width, int height) : width(width), height(height) {
        pixels.resize(width * height);
    }

    Vec3f& operator()(int x, int y) {
        return pixels[y * width + x];
    }

    const Vec3f& operator()(int x, int y) const {
        return pixels[y * width + x];
    }
};


void loadBitmap(const std::string &filename, bool raw, Bitmap &bitmap);
