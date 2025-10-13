#include "core/Pacific.h"
#include <unordered_map>
#include <string>

inline const std::unordered_map<std::string, Float> IOR_TABLE = {
    {"vacuum", 1.0},
    {"bromine", 1.661},
    {"helium", 1.00004},
    {"water ice", 1.31},
    {"hydrogen", 1.00013},
    {"fused quartz", 1.458},
    {"air", 1.00028},
    {"pyrex", 1.47},
    {"carbon dioxide", 1.00045},
    {"acrylic glass", 1.49},
    {"water", 1.333},
    {"polypropylene", 1.49},
    {"acetone", 1.36},
    {"bk7", 1.5046},
    {"ethanol", 1.361},
    {"sodium chloride", 1.544},
    {"carbon tetrachloride", 1.461},
    {"amber", 1.55},
    {"glycerol", 1.4729},
    {"pet", 1.575},
    {"benzene", 1.501},
    {"diamond", 2.419},
    {"silicone oil", 1.52045},
};
