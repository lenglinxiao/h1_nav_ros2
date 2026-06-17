#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY_DIR="${WORKSPACE_DIR}/third_party"
INSTALL_DIR="${WORKSPACE_DIR}/install"
JOBS="${JOBS:-$(nproc --all 2>/dev/null || echo 4)}"

KISS_MATCHER_REF="669fe866b1c612daa3702292571c25dc1bc61c86"
SMALL_GICP_REF="57c1106daf83c2c79ee0c58a9c7ed0032298ff4e"

KISS_MATCHER_SRC="${THIRD_PARTY_DIR}/KISS-Matcher"
SMALL_GICP_SRC="${THIRD_PARTY_DIR}/small_gicp"
ROBIN_SRC="${THIRD_PARTY_DIR}/ROBIN"
PMC_SRC="${THIRD_PARTY_DIR}/pmc"
XENIUM_SRC="${THIRD_PARTY_DIR}/xenium"
KISS_MATCHER_PREFIX="${INSTALL_DIR}/kiss_matcher"
SMALL_GICP_PREFIX="${INSTALL_DIR}/small_gicp"
OFFLINE_DEPS_DIR="${WORKSPACE_DIR}/.codex_tmp/relocalization_deps"

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 2
  fi
}

require_clean_git_tree() {
  local path="$1"
  if [[ -d "${path}/.git" ]]; then
    if ! git -C "${path}" diff --quiet || ! git -C "${path}" diff --cached --quiet; then
      echo "Refusing to change ${path}: it has local git modifications." >&2
      exit 2
    fi
  fi
}

require_existing_checkout() {
  local ref="$2"
  local path="$1"

  if [[ ! -d "${path}/.git" ]]; then
    echo "Missing third-party source checkout: ${path}" >&2
    echo "Copy the prepared third_party source tree into this workspace, without build/ or install/ artifacts." >&2
    exit 2
  fi

  require_clean_git_tree "${path}"
  if ! git -C "${path}" cat-file -e "${ref}^{commit}" 2>/dev/null; then
    echo "Required commit is not available in ${path}: ${ref}" >&2
    echo "Copy a third_party checkout that contains this commit, then rerun this script." >&2
    exit 2
  fi
  git -C "${path}" checkout --detach "${ref}"
}

require_existing_source_dir() {
  local path="$1"
  local marker="$2"

  if [[ ! -e "${path}/${marker}" ]]; then
    echo "Missing third-party source: ${path}/${marker}" >&2
    echo "Copy the prepared third_party source tree into this workspace, without build/ or install/ artifacts." >&2
    exit 2
  fi
}

prepare_offline_robin_source() {
  local offline_robin_src="${OFFLINE_DEPS_DIR}/ROBIN"
  rm -rf "${offline_robin_src}"
  mkdir -p "${OFFLINE_DEPS_DIR}"
  cmake -E copy_directory "${ROBIN_SRC}" "${offline_robin_src}"

  cat >"${offline_robin_src}/cmake/DownloadExternal.cmake" <<EOF
include(GNUInstallDirs)

function(robin_download_pmc)
  set(pmc_SOURCE_DIR "${PMC_SRC}" PARENT_SCOPE)
  set(pmc_BINARY_DIR "\${CMAKE_BINARY_DIR}/pmc-build" PARENT_SCOPE)
endfunction()

function(robin_download_xenium)
  add_library(xenium INTERFACE)
  target_include_directories(xenium
    INTERFACE
      \$<BUILD_INTERFACE:${XENIUM_SRC}>
      \$<INSTALL_INTERFACE:\${CMAKE_INSTALL_INCLUDEDIR}>)

  install(
    TARGETS xenium
    EXPORT xeniumTargets
    ARCHIVE DESTINATION \${CMAKE_INSTALL_LIBDIR}
  )

  export(
    EXPORT xeniumTargets
    FILE \${CMAKE_CURRENT_BINARY_DIR}/xeniumTargets.cmake
    NAMESPACE xenium::
  )

  install(
    EXPORT xeniumTargets
    FILE xeniumTargets.cmake
    DESTINATION \${CMAKE_INSTALL_LIBDIR}/cmake/xenium/
    NAMESPACE xenium::
  )
  install(DIRECTORY ${XENIUM_SRC}/xenium DESTINATION \${CMAKE_INSTALL_INCLUDEDIR})
endfunction()

function(find_external_dependency PACKAGE_NAME TARGET_NAME INCLUDED_CMAKE_PATH)
  string(TOUPPER \${PACKAGE_NAME} PACKAGE_NAME_UP)
  set(USE_FROM_SYSTEM_OPTION "USE_SYSTEM_\${PACKAGE_NAME_UP}")
  if(\${\${USE_FROM_SYSTEM_OPTION}})
    find_package(\${PACKAGE_NAME} QUIET NO_MODULE)
  endif()
  if(NOT \${\${USE_FROM_SYSTEM_OPTION}} OR NOT TARGET \${TARGET_NAME})
    set(\${USE_FROM_SYSTEM_OPTION} OFF PARENT_SCOPE)
    include(\${INCLUDED_CMAKE_PATH})
  endif()
endfunction()
EOF

  echo "${offline_robin_src}"
}

build_kiss_matcher() {
  local offline_robin_src="$1"
  local build_dir="${KISS_MATCHER_SRC}/cpp/kiss_matcher/build_h1_offline"

  cmake -S "${KISS_MATCHER_SRC}/cpp/kiss_matcher" \
    -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${KISS_MATCHER_PREFIX}" \
    -DFETCHCONTENT_SOURCE_DIR_ROBIN="${offline_robin_src}" \
    -DFETCHCONTENT_UPDATES_DISCONNECTED=ON \
    -DUSE_SYSTEM_ROBIN=OFF \
    -DUSE_SYSTEM_TBB=ON \
    -DUSE_SYSTEM_EIGEN3=ON
  cmake --build "${build_dir}" -j"${JOBS}"
  cmake --install "${build_dir}"
}

build_small_gicp() {
  local build_dir="${SMALL_GICP_SRC}/build_h1_offline"

  cmake -S "${SMALL_GICP_SRC}" \
    -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${SMALL_GICP_PREFIX}" \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_BENCHMARKS=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DBUILD_WITH_TBB=OFF
  cmake --build "${build_dir}" -j"${JOBS}"
  cmake --install "${build_dir}"
}

verify_installs() {
  local verify_dir="${WORKSPACE_DIR}/.codex_tmp/relocalization_deps_find"
  mkdir -p "${verify_dir}"
  cat >"${verify_dir}/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.16)
project(relocalization_deps_find LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(robin REQUIRED)
find_package(TBB REQUIRED)
find_package(kiss_matcher REQUIRED)
find_package(small_gicp REQUIRED)
add_executable(relocalization_deps_find main.cpp)
target_link_libraries(relocalization_deps_find
  PRIVATE
    kiss_matcher::kiss_matcher_core
    small_gicp::small_gicp
)
EOF
  cat >"${verify_dir}/main.cpp" <<'EOF'
#include <kiss_matcher/KISSMatcher.hpp>
#include <small_gicp/registration/registration.hpp>

int main() {
  return 0;
}
EOF
  cmake -S "${verify_dir}" \
    -B "${verify_dir}/build" \
    -DCMAKE_PREFIX_PATH="${KISS_MATCHER_PREFIX};${SMALL_GICP_PREFIX}"
  cmake --build "${verify_dir}/build" -j"${JOBS}"
}

main() {
  require_command git
  require_command cmake
  require_command c++
  mkdir -p "${INSTALL_DIR}"

  require_existing_checkout "${KISS_MATCHER_SRC}" "${KISS_MATCHER_REF}"
  require_existing_source_dir "${ROBIN_SRC}" "CMakeLists.txt"
  require_existing_source_dir "${PMC_SRC}" "CMakeLists.txt"
  require_existing_source_dir "${XENIUM_SRC}" "xenium"
  offline_robin_src="$(prepare_offline_robin_source)"
  build_kiss_matcher "${offline_robin_src}"

  require_existing_checkout "${SMALL_GICP_SRC}" "${SMALL_GICP_REF}"
  build_small_gicp

  verify_installs

  echo "Installed KISS-Matcher to ${KISS_MATCHER_PREFIX}"
  echo "Installed small_gicp to ${SMALL_GICP_PREFIX}"
}

main "$@"
