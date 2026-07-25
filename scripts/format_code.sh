#!/bin/bash
# =============================================================================
# 代码格式化脚本
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 检查 clang-format 是否安装
if ! command -v clang-format &> /dev/null; then
    echo -e "${RED}错误: clang-format 未安装${NC}"
    echo "请运行: sudo apt install clang-format"
    exit 1
fi

echo -e "${GREEN}==============================================================================${NC}"
echo -e "${GREEN}  代码格式化工具${NC}"
echo -e "${GREEN}==============================================================================${NC}"
echo ""

# 解析参数
DRY_RUN=false
TARGET_DIR=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --dir)
            TARGET_DIR="$2"
            shift 2
            ;;
        -h|--help)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --dry-run       仅检查，不修改文件"
            echo "  --dir <path>    指定要格式化的目录（默认: 整个项目）"
            echo "  -h, --help      显示此帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                    # 格式化整个项目"
            echo "  $0 --dry-run          # 仅检查格式"
            echo "  $0 --dir module/      # 仅格式化 module/ 目录"
            exit 0
            ;;
        *)
            echo -e "${RED}未知选项: $1${NC}"
            echo "使用 -h 查看帮助"
            exit 1
            ;;
    esac
done

# 设置搜索目录
if [ -z "$TARGET_DIR" ]; then
    SEARCH_DIR="$PROJECT_ROOT"
    echo -e "${YELLOW}格式化范围: 整个项目${NC}"
else
    SEARCH_DIR="$PROJECT_ROOT/$TARGET_DIR"
    if [ ! -d "$SEARCH_DIR" ]; then
        echo -e "${RED}错误: 目录不存在: $SEARCH_DIR${NC}"
        exit 1
    fi
    echo -e "${YELLOW}格式化范围: $TARGET_DIR${NC}"
fi

# 排除目录
EXCLUDE_DIRS=(
    "build"
    "3rdparty"
    ".git"
    "cmake-build-*"
)

# 构建 find 命令的排除参数
EXCLUDE_ARGS=""
for dir in "${EXCLUDE_DIRS[@]}"; do
    EXCLUDE_ARGS="$EXCLUDE_ARGS -path '*/$dir/*' -prune -o"
done

echo ""
if [ "$DRY_RUN" = true ]; then
    echo -e "${YELLOW}模式: 仅检查（不会修改文件）${NC}"
else
    echo -e "${YELLOW}模式: 格式化并修改文件${NC}"
fi
echo ""

# 查找所有 C/C++ 文件
FILES=$(eval "find '$SEARCH_DIR' $EXCLUDE_ARGS -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.c' -o -name '*.h' \) -print")

if [ -z "$FILES" ]; then
    echo -e "${YELLOW}未找到需要格式化的文件${NC}"
    exit 0
fi

FILE_COUNT=$(echo "$FILES" | wc -l)
echo -e "${GREEN}找到 $FILE_COUNT 个文件${NC}"
echo ""

# 处理文件
MODIFIED_COUNT=0
ERROR_COUNT=0

while IFS= read -r file; do
    relative_path="${file#$PROJECT_ROOT/}"

    if [ "$DRY_RUN" = true ]; then
        # 检查格式
        if ! clang-format --dry-run --Werror "$file" 2>/dev/null; then
            echo -e "${YELLOW}[需格式化] $relative_path${NC}"
            ((MODIFIED_COUNT++))
        fi
    else
        # 格式化文件
        if clang-format -i "$file" 2>/dev/null; then
            echo -e "${GREEN}[已格式化] $relative_path${NC}"
            ((MODIFIED_COUNT++))
        else
            echo -e "${RED}[错误] $relative_path${NC}"
            ((ERROR_COUNT++))
        fi
    fi
done <<< "$FILES"

echo ""
echo -e "${GREEN}==============================================================================${NC}"
echo -e "${GREEN}  完成${NC}"
echo -e "${GREEN}==============================================================================${NC}"
echo -e "总文件数: $FILE_COUNT"

if [ "$DRY_RUN" = true ]; then
    echo -e "需要格式化: $MODIFIED_COUNT"
else
    echo -e "已格式化: $MODIFIED_COUNT"
    echo -e "错误: $ERROR_COUNT"
fi

if [ $ERROR_COUNT -gt 0 ]; then
    exit 1
fi

exit 0
