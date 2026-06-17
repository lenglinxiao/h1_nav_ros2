from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_repo_file(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_relocalization_dependency_installer_is_reproducible_and_local():
    script = read_repo_file("scripts/install_relocalization_deps.sh")

    assert "KISS_MATCHER_REF=\"669fe866b1c612daa3702292571c25dc1bc61c86\"" in script
    assert "SMALL_GICP_REF=\"57c1106daf83c2c79ee0c58a9c7ed0032298ff4e\"" in script
    assert 'THIRD_PARTY_DIR="${WORKSPACE_DIR}/third_party"' in script
    assert 'INSTALL_DIR="${WORKSPACE_DIR}/install"' in script
    assert 'KISS_MATCHER_SRC="${THIRD_PARTY_DIR}/KISS-Matcher"' in script
    assert 'SMALL_GICP_SRC="${THIRD_PARTY_DIR}/small_gicp"' in script
    assert 'ROBIN_SRC="${THIRD_PARTY_DIR}/ROBIN"' in script
    assert 'PMC_SRC="${THIRD_PARTY_DIR}/pmc"' in script
    assert 'XENIUM_SRC="${THIRD_PARTY_DIR}/xenium"' in script
    assert 'KISS_MATCHER_PREFIX="${INSTALL_DIR}/kiss_matcher"' in script
    assert 'SMALL_GICP_PREFIX="${INSTALL_DIR}/small_gicp"' in script
    assert "prepare_offline_robin_source" in script
    assert "FETCHCONTENT_SOURCE_DIR_ROBIN" in script
    assert "-DUSE_SYSTEM_ROBIN=OFF" in script
    assert "-DUSE_SYSTEM_TBB=ON" in script
    assert "-DBUILD_TESTS=OFF" in script
    assert "-DBUILD_EXAMPLES=OFF" in script
    assert "-DBUILD_PYTHON_BINDINGS=OFF" in script
    assert "find_package(kiss_matcher REQUIRED)" in script
    assert "find_package(small_gicp REQUIRED)" in script
    assert "sudo apt-get" not in script
    assert "git clone" not in script
    assert "git -C \"${path}\" fetch" not in script
    assert "KISS_MATCHER_REPO" not in script
    assert "SMALL_GICP_REPO" not in script
    assert "github.com" not in script
    assert "require_existing_checkout" in script
    assert "cmake --install" in script


def test_relocalization_dependency_installer_is_documented():
    readme = read_repo_file("README.md")

    assert "## 可选 3D 重定位依赖" in readme
    assert "./scripts/install_relocalization_deps.sh" in readme
    assert "H1_USE_KISS_MATCHER=ON" in readme
    assert "H1_USE_SMALL_GICP=ON" in readme
    assert "install/kiss_matcher" in readme
    assert "install/small_gicp" in readme
    assert "third_party/" in readme
    assert "先准备源码" in readme
    assert "third_party/ROBIN" in readme
    assert "third_party/pmc" in readme
    assert "third_party/xenium" in readme
    assert "不包含 build" in readme
    assert "脚本不会 clone 或 fetch" in readme
