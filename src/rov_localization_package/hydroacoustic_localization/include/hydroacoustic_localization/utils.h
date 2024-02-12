#pragma once

#include <experimental/filesystem>

#include "types.h"

namespace fs = std::experimental::filesystem;

namespace rov_localization {

std::shared_ptr<PointCloud> load_map(const fs::path& path);
ManagerParams load_config(const fs::path& path);
} // end namespace rov_localization
