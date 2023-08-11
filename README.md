<!--
 ~ SPDX-FileCopyrightText: Copyright DB Netz AG
 ~ SPDX-License-Identifier: CC0-1.0
 -->

# Rail Horizon

Rail Horizon is a ROS2 Node which provides train map foresight (an excerpt of digital map data) by processing mission profile, map regions and coupled localization data. The train map foresight is provided in a form of a ROS Message called RailHorizonStamped, containing the map data in a local sensor coordinate system and global coordinate system.

![Rviz](docs/images/rviz_RH.png)

## Build

### Cloning and Submodules

First clone the project (ssh recommended) and in case of ssh add some replacement patterns, so submodules will be converted to ssh:
```bash
git config --global url."git@github.com:".insteadOf "https://github.com/"
```

Then initialize all submodules recursively:
```bash
git submodule update --init --recursive
```

### Dependencies

Dependencies (Recommended: Ubuntu 22.04):
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* Python: `sudo apt install python`
* Boost: `sudo apt install libboost-all-dev`
* Proj: `sudo apt install libproj-dev`
* Protobuf (Map Service): `sudo apt install libprotobuf-dev protobuf-compiler`
* Curl (Map Service): `sudo apt install libcurl4-openssl-dev`

You should also create the underlay, which is the underlying workspace with ROS2 Tools available. It behaves similarly to a virtual environment and allows having multiple ROS2 versions installed on the system and use a specific one for each project. The underlay can be created automatically for every terminal session or manually at the beginning of a new session like this:
```bash
source /opt/ros/humble/setup.bash
```

Check if all dependencies are available with:
```bash
rosdep install -i --from-path src --rosdistro humble -y
```

### Preparation

Some environment variables need to be set for the map service:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PERSISTENT_STORAGE_ROOT_DIR=~/documents/maps
export DB_MAP_VERSION=12
export DB_MAP_ENDPOINT=DBMC
export DB_MAP_CATALOG=validate.s4r2.oss.4
```
* They can be set for a terminal session, loaded automatically in `~/.bashrc` (Don't forget to source the file: `source ~/.bashrc`) or globally for all sessions and users in `etc/environment` (Syntax is different)

### Build

Use colcon build to build the project. Some variables are specified, which are needed for the projects. As in CMake, they only have to specified once for each build directory as they are cached:

```bash
export CMAKE_BUILD_PARALLEL_LEVEL=10
colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --event-handlers console_cohesion+ --packages-up-to dsd_rail_horizon
```
- `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` is optional. It generates a compilation database for tooling. For instance, clang-tidy needs it, which is run as a pre-commit.
- `-G Ninja` can optionally be added to the `--cmake-args`. If the `ninja` build system is installed on the system, it can be used instead of make. Builds usually are a lot faster then.

After the build, you should source the install folder:
```bash
source ./install/local_setup.bash
```

### Run

Currently, Rail Horizon supports one map region of interest (roi) which is given in the parameters.yaml configuration file. Rail Horizon can be run with the config file specified on the command line.

```bash
ros2 run dsd_rail_horizon dsd_rail_horizon --ros-args --params-file ./install/dsd_rail_horizon/config/dsd_rail_horizon/parameters.yaml
```

### Test

Run all tests:
```bash
ctest --test-dir build/dsd_rail_horizon --output-on-failure
```

Run all C++ unit tests:
```bash
ctest --test-dir build/dsd_rail_horizon -L "UnitTests" --output-on-failure
```

Run all Python integration tests:
```bash
ctest --test-dir build/dsd_rail_horizon -L "IntegrationTests" --output-on-failure
```

#### Coverage

Generate coverage report with:
```bash
gcovr --exclude-unreachable-branches --exclude-throw-branches -r . --filter src/ --exclude '.*tests/' --html --html-details -o build/coverage.html
```

The generated file `build/coverage.html` contains the report.

### Development Tools

Dependencies:
* [pre-commit ^3.2](https://pre-commit.com/): `sudo apt install pre-commit`
* [clang-format ^14.0](https://clang.llvm.org/docs/ClangFormat.html): `sudo apt install clang-format` (or: `pipx install clang-format`)
* [clang-tidy ^14.0](https://clang.llvm.org/extra/clang-tidy/):  `sudo apt install clang-tidy`
* [include-what-you-use ^0.17](https://include-what-you-use.org/) (**optional**, see below): `sudo apt install iwyu`

#### Pre-Commit

We use [pre-commit](https://pre-commit.com/) to enforce usage of tools such as clang-format on commit. This ensures that tools such as clang-format are run on a commit, and thus keeps our repository consistent and makes review easier.

Install all the hooks with:
```bash
pre-commit install
```

Pre-commit will be then automatically run on changed files in a commit. Additionally, it can also be run manually against all files with:
```bash
pre-commit run --all-files
```

#### include-what-you-use (optional, experimental)

The tool [include-what-you-use](https://include-what-you-use.org/) can help to find the right includes. This makes dependencies obvious and avoids transitive includes, which are bad practice.
The setup of the tool is more involved and additionally the output of the tool is not yet perfect and requires tweaking. For these reasons, the tool should be run only manually after other changes have been committed, and the output should be carefully reviewed.

As include-what-you-use is basically built on top of some specific clang version, the development header should be installed:
```bash
sudo apt install libclang-common-13-dev
```
* The clang version can be found out with `strings /usr/bin/iwyu | grep LLVM`. On Ubuntu 22.04 this gives `LLVM_13`.

The tool can be run as a manual pre-commit job:
```bash
pre-commit run --hook-stage manual include-what-you-use --all-files
```
* Note: This will also automatically apply clang format to fix the order of includes

The configuration of the command line arguments can be found in `tools/iwyu_precommit.py`. Additionally, we use mostly automatically generated [mapping](https://github.com/include-what-you-use/include-what-you-use/blob/master/docs/IWYUMappings.md) files. If `iwyu` suggest using the wrong header (for instance some internal header file, which is not supposed to be used) then likely adjustments need to be made to the mapping file. In this case, make the appropriate changes to `tools/generate_mappings.py`. The mappings will be automatically created by the precommit in `build/iwyu_mappings`.
Additionally, some [pragmas](https://github.com/include-what-you-use/include-what-you-use/blob/master/docs/IWYUPragmas.md) can be used as well to fix the output. Or changes can be manually overwritten.

## Licenses

This project is compliant with the [REUSE Specification Version 3.0](https://git.fsfe.org/reuse/docs/src/commit/d173a27231a36e1a2a3af07421f5e557ae0fec46/spec.md)

Copyright DB Netz AG, licensed under Apache 2.0 (see full text in [LICENSES/Apache-2.0.txt](./LICENSES/Apache-2.0.txt))

Dot-files, cmake-files and config-files are licensed under CC0-1.0 (see full text in
[LICENSES/CC0-1.0.txt](./LICENSES/CC0-1.0.txt))