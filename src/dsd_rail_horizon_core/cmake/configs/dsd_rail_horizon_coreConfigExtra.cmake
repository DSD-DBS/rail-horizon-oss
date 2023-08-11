# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

# The purpose of this file is to help with dependency handling of this package because of limitations regarding
# ament_export_dependencies (see https://github.com/ament/ament_cmake/issues/282)
include(CMakeFindDependencyMacro)
find_dependency(Boost 1.65.0 COMPONENTS filesystem)
