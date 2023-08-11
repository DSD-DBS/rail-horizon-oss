/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_CORE_PRETTY_PRINT_H
#define DSD_RAIL_HORIZON_CORE_PRETTY_PRINT_H

#include "dsd_rail_horizon_core/MapDBReader.h"

#include <boost/format.hpp>

#include <optional>
#include <ostream>
#include <string>

namespace std
{
template <typename T>
std::ostream& operator<<(std::ostream& os, std::optional<T> const& opt)
{
    return opt ? os << opt.value() : os << "std::nullopt";
}
} // namespace std

inline std::ostream& operator<<(std::ostream& os, const MapDBReaderConfig& config)
{
    return os << (boost::format(
                      "{persistent_storage_root_dir: %1%, map_catalog: %2%, map_endpoint: %3%, map_version: %4%}") %
                  config.db_map_persistent_storage_root_dir % config.db_map_catalog % config.db_map_endpoint %
                  config.db_requested_map_version)
                     .str();
}

#endif // DSD_RAIL_HORIZON_CORE_PRETTY_PRINT_H
