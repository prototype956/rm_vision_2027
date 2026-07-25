#!/bin/bash
# =============================================================================
# 代码规范工具快速设置脚本
# =============================================================================

set -e

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}==============================================================================${NC}"
echo -e "${BLUE}  EX_MiracleVision 代码规范工具设置${NC}"
echo -e "${BLUE}==============================================================================${NC}"
echo ""

# 检查是否在项目根目录
if [ ! -f "CMakeLists.txt" ]; then
    echo -e "${RED}错误: 请在项目根目录运行此脚本${NC}"
    exit 1
fi

PROJECT_ROOT=$(pwd)

# Step 1: 检查并安装工具
echo -e "${YELLOW}[1/5] 检查工具安装状态...${NC}"

TOOLS_MISSING=false

if ! command -v clangd &> /dev/null; then
    echo -e "${RED}  ✗ clangd 未安装${NC}"
    TOOLS_MISSING=true
else
    echo -e "${GREEN}  ✓ clangd 已安装 ($(clangd --version | head -n1))${NC}"
fi

if ! command -v clang-format &> /dev/null; then
    echo -e "${RED}  ✗ clang-format 未安装${NC}"
    TOOLS_MISSING=true
else
    echo -e "${GREEN}  ✓ clang-format 已安装 ($(clang-format --version | head -n1))${NC}"
fi

if ! command -v clang-tidy &> /dev/null; then
    echo -e "${RED}  ✗ clang-tidy 未安装${NC}"
    TOOLS_MISSING=true
else
    echo -e "${GREEN}  ✓ clang-tidy 已安装 ($(clang-tidy --version | head -n1))${NC}"
fi

if [ "$TOOLS_MISSING" = true ]; then
    echo ""
    echo -e "${YELLOW}需要安装缺失的工具。${NC}"
    read -p "是否现在安装? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${BLUE}安装工具...${NC}"
        sudo apt update
        sudo apt install -y clangd clang-format clang-tidy
        echo -e "${GREEN}✓ 工具安装完成${NC}"
    else
        echo -e "${RED}已取消。请手动安装后再运行此脚本。${NC}"
        exit 1
    fi
fi

echo ""

# Step 2: 生成编译数据库
echo -e "${YELLOW}[2/5] 生成编译数据库...${NC}"

if [ ! -d "build" ]; then
    mkdir build
    echo -e "${GREEN}  ✓ 创建 build 目录${NC}"
fi

cd build
cmake .. > /dev/null 2>&1
cd ..

if [ -f "build/compile_commands.json" ]; then
    echo -e "${GREEN}  ✓ compile_commands.json 已生成${NC}"
else
    echo -e "${RED}  ✗ compile_commands.json 生成失败${NC}"
    exit 1
fi

echo ""

# Step 3: 创建符号链接
echo -e "${YELLOW}[3/5] 创建符号链接...${NC}"

if [ -L "compile_commands.json" ] || [ -f "compile_commands.json" ]; then
    rm -f compile_commands.json
fi

ln -s build/compile_commands.json compile_commands.json
echo -e "${GREEN}  ✓ 符号链接已创建${NC}"

echo ""

# Step 4: 验证配置文件
echo -e "${YELLOW}[4/5] 验证配置文件...${NC}"

CONFIG_FILES=(
    ".clangd"
    ".clang-format"
    ".clang-tidy"
    ".vscode/settings.json"
    ".vscode/extensions.json"
)

ALL_EXISTS=true
for config in "${CONFIG_FILES[@]}"; do
    if [ -f "$config" ]; then
        echo -e "${GREEN}  ✓ $config${NC}"
    else
        echo -e "${RED}  ✗ $config 不存在${NC}"
        ALL_EXISTS=false
    fi
done

if [ "$ALL_EXISTS" = false ]; then
    echo -e "${RED}配置文件缺失！请检查。${NC}"
    exit 1
fi

echo ""

# Step 5: 检查 VS Code 扩展
echo -e "${YELLOW}[5/5] 检查 VS Code 扩展...${NC}"

if command -v code &> /dev/null; then
    CLANGD_INSTALLED=$(code --list-extensions | grep -c "llvm-vs-code-extensions.vscode-clangd" || true)

    if [ $CLANGD_INSTALLED -eq 0 ]; then
        echo -e "${YELLOW}  ⚠ clangd 扩展未安装${NC}"
        read -p "  是否现在安装? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            code --install-extension llvm-vs-code-extensions.vscode-clangd
            echo -e "${GREEN}  ✓ clangd 扩展已安装${NC}"
        else
            echo -e "${YELLOW}  → 请手动在 VS Code 中安装 'clangd' 扩展${NC}"
        fi
    else
        echo -e "${GREEN}  ✓ clangd 扩展已安装${NC}"
    fi
else
    echo -e "${YELLOW}  ⚠ 未检测到 VS Code 命令行工具${NC}"
    echo -e "${YELLOW}  → 请在 VS Code 中手动安装推荐扩展${NC}"
fi

echo ""
echo -e "${GREEN}==============================================================================${NC}"
echo -e "${GREEN}  ✓ 设置完成！${NC}"
echo -e "${GREEN}==============================================================================${NC}"
echo ""
echo -e "${BLUE}接下来的步骤:${NC}"
echo ""
echo -e "1. ${YELLOW}重启 VS Code${NC}"
echo "   - 关闭并重新打开 VS Code 以激活配置"
echo ""
echo -e "2. ${YELLOW}安装推荐扩展${NC} (如果未自动安装)"
echo "   - 打开 VS Code 后，右下角会提示安装推荐扩展"
echo "   - 或手动安装: llvm-vs-code-extensions.vscode-clangd"
echo ""
echo -e "3. ${YELLOW}验证功能${NC}"
echo "   - 打开任意 .cpp 文件"
echo "   - 检查底部状态栏是否显示 'clangd' 状态"
echo "   - 测试代码补全 (Ctrl+Space)"
echo "   - 测试跳转定义 (F12)"
echo ""
echo -e "4. ${YELLOW}格式化代码${NC}"
echo "   - 使用快捷键: Shift+Alt+F"
echo "   - 或保存时自动格式化"
echo ""
echo -e "5. ${YELLOW}查看使用文档${NC}"
echo "   - 查看: docs/CODE_STYLE_GUIDE.md"
echo ""
echo -e "${BLUE}实用脚本:${NC}"
echo "  - ${GREEN}./scripts/format_code.sh${NC}      # 格式化代码"
echo "  - ${GREEN}./scripts/check_code.sh${NC}       # 静态分析"
echo ""
echo -e "${YELLOW}如有问题，请查阅文档或联系团队成员！${NC}"
echo ""
