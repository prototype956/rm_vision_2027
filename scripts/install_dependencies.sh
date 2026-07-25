#!/bin/bash

################################################################################
# EX_MiracleVision 依赖安装脚本
# 
# 用途: 自动安装所有系统依赖并验证环境配置
# 支持: Ubuntu 22.04 LTS
# 作者: BJFU RoboMaster Vision Team
# 日期: 2026-02-14
################################################################################

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 打印标题
print_header() {
    echo ""
    echo -e "${BLUE}================================================${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}================================================${NC}"
    echo ""
}

# 检查是否为 root 用户
check_root() {
    if [ "$EUID" -eq 0 ]; then
        log_error "请不要使用 root 用户运行此脚本"
        log_info "正确用法: ./install_dependencies.sh"
        exit 1
    fi
}

# 检查系统版本
check_system() {
    log_info "检查系统版本..."
    
    if [ ! -f /etc/os-release ]; then
        log_error "无法识别的操作系统"
        exit 1
    fi
    
    . /etc/os-release
    
    if [ "$ID" != "ubuntu" ]; then
        log_warning "此脚本为 Ubuntu 设计，其他发行版可能需要调整"
    fi
    
    if [ "$VERSION_ID" != "22.04" ]; then
        log_warning "推荐使用 Ubuntu 22.04，当前版本: $VERSION_ID"
    fi
    
    log_success "系统: $PRETTY_NAME"
}

# 检查 GCC 版本
check_gcc() {
    log_info "检查 GCC 版本..."
    
    if ! command -v gcc &> /dev/null; then
        log_error "未找到 GCC"
        return 1
    fi
    
    GCC_VERSION=$(gcc -dumpversion)
    GCC_MAJOR=$(echo $GCC_VERSION | cut -d. -f1)
    
    if [ "$GCC_MAJOR" -lt 11 ]; then
        log_error "GCC 版本过低: $GCC_VERSION (需要 11.0+)"
        return 1
    elif [ "$GCC_MAJOR" -ge 12 ]; then
        log_warning "检测到 GCC $GCC_VERSION (推荐使用 GCC 11，GCC 12/13 存在已知 ICE bug)"
    else
        log_success "GCC 版本: $GCC_VERSION"
    fi
    
    return 0
}

# 检查 CMake 版本
check_cmake() {
    log_info "检查 CMake 版本..."
    
    if ! command -v cmake &> /dev/null; then
        log_error "未找到 CMake"
        return 1
    fi
    
    CMAKE_VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
    CMAKE_MAJOR=$(echo $CMAKE_VERSION | cut -d. -f1)
    CMAKE_MINOR=$(echo $CMAKE_VERSION | cut -d. -f2)
    
    if [ "$CMAKE_MAJOR" -lt 3 ] || ([ "$CMAKE_MAJOR" -eq 3 ] && [ "$CMAKE_MINOR" -lt 22 ]); then
        log_error "CMake 版本过低: $CMAKE_VERSION (需要 3.22+)"
        return 1
    fi
    
    log_success "CMake 版本: $CMAKE_VERSION"
    return 0
}

# 检查依赖库
check_library() {
    local lib_name=$1
    local pkg_name=$2
    
    if pkg-config --exists $pkg_name 2>/dev/null; then
        local version=$(pkg-config --modversion $pkg_name 2>/dev/null || echo "unknown")
        log_success "$lib_name: $version"
        return 0
    else
        log_error "$lib_name: 未安装"
        return 1
    fi
}

# 检查所有依赖
check_dependencies() {
    log_info "检查依赖库..."
    local all_ok=true
    
    check_library "OpenCV" "opencv4" || all_ok=false
    check_library "fmt" "fmt" || all_ok=false
    check_library "spdlog" "spdlog" || all_ok=false
    check_library "Eigen3" "eigen3" || all_ok=false
    check_library "yaml-cpp" "yaml-cpp" || all_ok=false
    
    # nlohmann-json 可能没有 pkg-config 文件
    if [ -f /usr/include/nlohmann/json.hpp ]; then
        log_success "nlohmann-json: installed"
    else
        log_error "nlohmann-json: 未安装"
        all_ok=false
    fi
    
    # TBB 检查
    if pkg-config --exists tbb 2>/dev/null || [ -f /usr/include/tbb/tbb.h ]; then
        log_success "TBB: installed"
    else
        log_error "TBB: 未安装"
        all_ok=false
    fi
    
    if $all_ok; then
        return 0
    else
        return 1
    fi
}

# 更新系统包列表
update_system() {
    print_header "更新系统包列表"
    log_info "执行 apt update..."
    
    sudo apt update || {
        log_error "apt update 失败"
        exit 1
    }
    
    log_success "系统包列表已更新"
}

# 安装构建工具
install_build_tools() {
    print_header "安装构建工具"
    
    local packages=(
        "build-essential"
        "cmake"
        "git"
        "pkg-config"
    )
    
    log_info "安装: ${packages[*]}"
    
    sudo apt install -y "${packages[@]}" || {
        log_error "构建工具安装失败"
        exit 1
    }
    
    log_success "构建工具安装完成"
}

# 安装核心依赖
install_dependencies() {
    print_header "安装核心依赖"
    
    local packages=(
        "libopencv-dev"
        "libfmt-dev"
        "libspdlog-dev"
        "libeigen3-dev"
        "libyaml-cpp-dev"
        "nlohmann-json3-dev"
        "libtbb-dev"
    )
    
    log_info "安装: ${packages[*]}"
    
    sudo apt install -y "${packages[@]}" || {
        log_error "依赖安装失败"
        exit 1
    }
    
    log_success "核心依赖安装完成"
}

# 配置串口权限
setup_serial_permissions() {
    print_header "配置串口权限"
    
    log_info "将用户 $USER 添加到 dialout 组..."
    
    if groups $USER | grep -q dialout; then
        log_success "用户已在 dialout 组中"
    else
        sudo usermod -a -G dialout $USER
        log_success "已添加到 dialout 组"
        log_warning "需要重新登录或执行 'newgrp dialout' 以使权限生效"
    fi
}

# 检查第三方 SDK
check_third_party_sdks() {
    print_header "检查第三方 SDK"
    
    # MindVision SDK
    if [ -f "3rdparty/mindvision/linux/lib/x64/libMVSDK.so" ]; then
        log_success "MindVision SDK: 已存在"
    else
        log_warning "MindVision SDK: 未找到"
        log_info "路径: 3rdparty/mindvision/linux/lib/x64/libMVSDK.so"
    fi
    
    # Foxglove SDK
    if [ -f "3rdparty/foxglove/lib/libfoxglove.so" ]; then
        log_success "Foxglove SDK: 已存在"
    else
        log_warning "Foxglove SDK: 未找到"
        log_info "路径: 3rdparty/foxglove/lib/libfoxglove.so"
    fi
    
    # ONNX Runtime
    if [ -f "/usr/local/lib/libonnxruntime.so" ]; then
        log_success "ONNX Runtime: 已安装"
    else
        log_warning "ONNX Runtime: 未安装（可选）"
        log_info "如需深度学习功能，请参考文档安装 ONNX Runtime"
    fi
}

# 验证安装
verify_installation() {
    print_header "验证安装"
    
    local all_ok=true
    
    check_gcc || all_ok=false
    check_cmake || all_ok=false
    check_dependencies || all_ok=false
    
    echo ""
    
    if $all_ok; then
        log_success "所有依赖验证通过！"
        return 0
    else
        log_error "部分依赖验证失败"
        return 1
    fi
}

# 打印环境信息
print_environment_info() {
    print_header "环境信息总结"
    
    echo "系统信息:"
    echo "  OS: $(lsb_release -d | cut -f2)"
    echo "  Kernel: $(uname -r)"
    echo "  Architecture: $(uname -m)"
    echo ""
    
    echo "编译工具:"
    echo "  GCC: $(gcc --version | head -n1)"
    echo "  G++: $(g++ --version | head -n1)"
    echo "  CMake: $(cmake --version | head -n1)"
    echo ""
    
    echo "依赖库版本:"
    pkg-config --modversion opencv4 2>/dev/null && echo "  OpenCV: $(pkg-config --modversion opencv4)" || echo "  OpenCV: N/A"
    pkg-config --modversion fmt 2>/dev/null && echo "  fmt: $(pkg-config --modversion fmt)" || echo "  fmt: N/A"
    pkg-config --modversion spdlog 2>/dev/null && echo "  spdlog: $(pkg-config --modversion spdlog)" || echo "  spdlog: N/A"
    pkg-config --modversion eigen3 2>/dev/null && echo "  Eigen3: $(pkg-config --modversion eigen3)" || echo "  Eigen3: N/A"
    pkg-config --modversion yaml-cpp 2>/dev/null && echo "  yaml-cpp: $(pkg-config --modversion yaml-cpp)" || echo "  yaml-cpp: N/A"
    echo ""
}

# 打印下一步说明
print_next_steps() {
    print_header "下一步"
    
    echo "环境配置完成！现在可以编译项目："
    echo ""
    echo -e "${GREEN}  mkdir -p build && cd build${NC}"
    echo -e "${GREEN}  cmake -DCMAKE_BUILD_TYPE=Release ..${NC}"
    echo -e "${GREEN}  make -j\$(nproc)${NC}"
    echo ""
    echo "编译测试程序（可选）："
    echo ""
    echo -e "${GREEN}  cmake -DBUILD_TESTS=ON ..${NC}"
    echo -e "${GREEN}  make -j\$(nproc)${NC}"
    echo ""
    echo "运行程序："
    echo ""
    echo -e "${GREEN}  ./bin/MiracleVision${NC}"
    echo ""
    echo "更多信息请查看文档:"
    echo "  - docs/BUILD_GUIDE.md"
    echo "  - docs/ENVIRONMENT_SETUP.md"
    echo ""
}

# 主函数
main() {
    print_header "EX_MiracleVision 依赖安装"
    
    # 解析命令行参数
    CHECK_ONLY=false
    if [ "$1" == "--check-only" ] || [ "$1" == "-c" ]; then
        CHECK_ONLY=true
    fi
    
    if [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
        echo "用法: $0 [选项]"
        echo ""
        echo "选项:"
        echo "  -c, --check-only    仅检查环境，不安装"
        echo "  -h, --help          显示此帮助信息"
        echo ""
        exit 0
    fi
    
    # 检查基本环境
    check_root
    check_system
    
    if $CHECK_ONLY; then
        # 仅检查模式
        log_info "运行在检查模式（不会安装任何包）"
        echo ""
        check_gcc
        check_cmake
        check_dependencies
        check_third_party_sdks
        echo ""
        log_info "检查完成"
        exit 0
    fi
    
    # 完整安装流程
    update_system
    install_build_tools
    install_dependencies
    setup_serial_permissions
    check_third_party_sdks
    verify_installation
    
    # 打印总结信息
    echo ""
    print_environment_info
    print_next_steps
    
    log_success "安装完成！"
}

# 运行主函数
main "$@"
