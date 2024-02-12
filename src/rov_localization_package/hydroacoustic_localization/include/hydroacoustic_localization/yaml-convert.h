#pragma once

#include <yaml-cpp/yaml.h>

#include "types.h"

#if defined(LOAD_FIELD)
#error LOAD_FIELD either already defined
#else
#define LOAD_FIELD(structure, field, node) structure.field = (node)[#field].as<decltype((structure).field)>()

namespace YAML {

template <>
struct convert<rov_localization::MatherParams> {
    static bool decode(const Node& mather_map,
                       rov_localization::MatherParams& mather_params)
    {
        if (!mather_map.IsMap()) {
            return false;
        }

        LOAD_FIELD(mather_params, optimization_step, mather_map);
        LOAD_FIELD(mather_params, grid_cell_size, mather_map);
        LOAD_FIELD(mather_params, optimization_treshold, mather_map);
        LOAD_FIELD(mather_params, optimization_iterations, mather_map);

        return true;
    }
};

template <>
struct convert<rov_localization::ManagerParams> {
    static bool decode(const Node& manager_map,
                       rov_localization::ManagerParams& manager_params)
    {
        if (!manager_map.IsMap()) {
            return false;
        }

        LOAD_FIELD(manager_params, filter_cell_size, manager_map);
        LOAD_FIELD(manager_params, filter_enabled, manager_map);
        LOAD_FIELD(manager_params, mather_params, manager_map);

        return true;
    }
};
}  // namespace YAML

namespace rov_localization {

inline ManagerParams fromYaml(const YAML::Node& yaml_node) { return yaml_node.as<ManagerParams>(); }
} // end namespace rov_localization

#undef LOAD_FIELD
#endif