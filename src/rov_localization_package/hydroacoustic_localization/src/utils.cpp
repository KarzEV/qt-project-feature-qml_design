#include <hydroacoustic_localization/utils.h>

#include <pcl/io/pcd_io.h>

#include <hydroacoustic_localization/yaml-convert.h>

namespace rov_localization {

namespace {

inline bool is_path_correct(const fs::path& path) {
    return fs::is_regular_file(path) && fs::exists(path);
}
} // end namespace

std::shared_ptr<PointCloud> load_map(const fs::path& path)
{
    if(!is_path_correct(path)) {
        throw std::invalid_argument("load_map: Path for map not exist");
    }

    const auto map = std::make_shared<PointCloud>();
    if (pcl::io::loadPCDFile<Point>(path, *map) == -1) {
        throw std::invalid_argument("load_map: Map isn't correct: " + path.string());
    }

    return map;
}

ManagerParams load_config(const fs::path& path) {
    if(!is_path_correct(path)) {
        throw std::invalid_argument("load_map: Path for map not exist");
    }

    return fromYaml(YAML::LoadFile(path));
}

} // end namespace rov_localization
