#!/bin/bash
# =============================================================================
# 代码静态分析脚本（clang-tidy）
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 检查 clang-tidy 是否安装
if ! command -v clang-tidy &> /dev/null; then
    echo -e "${RED}错误: clang-tidy 未安装${NC}"
    echo "请运行: sudo apt install clang-tidy"
    exit 1
fi

# 检查编译数据库
COMPILE_DB="$PROJECT_ROOT/build/compile_commands.json"
if [ ! -f "$COMPILE_DB" ]; then
    echo -e "${RED}错误: 编译数据库不存在${NC}"
    echo "请先运行: cd build && cmake .."
    exit 1
fi

echo -e "${BLUE}==============================================================================${NC}"
echo -e "${BLUE}  代码静态分析工具 (clang-tidy)${NC}"
echo -e "${BLUE}==============================================================================${NC}"
echo ""

# 解析参数
AUTO_FIX=false
TARGET_FILE=""
TARGET_DIR=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --fix)
            AUTO_FIX=true
            shift
            ;;
        --file)
            TARGET_FILE="$2"
            shift 2
            ;;
        --dir)
            TARGET_DIR="$2"
            shift 2
            ;;
        -h|--help)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --fix           自动修复问题（谨慎使用）"
            echo "  --file <path>   指定要检查的文件"
            echo "  --dir <path>    指定要检查的目录"
            echo "  -h, --help      显示此帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                              # 检查整个项目"
            echo "  $0 --file module/armor/armor.cpp  # 检查单个文件"
            echo "  $0 --dir module/                  # 检查目录"
            echo "  $0 --fix --file xxx.cpp           # 自动修复"
            exit 0
            ;;
        *)
            echo -e "${RED}未知选项: $1${NC}"
            echo "使用 -h 查看帮助"
            exit 1
            ;;
    esac
done

# 确定要检查的文件
FILES=""

if [ -n "$TARGET_FILE" ]; then
    if [ ! -f "$PROJECT_ROOT/$TARGET_FILE" ]; then
        echo -e "${RED}错误: 文件不存在: $TARGET_FILE${NC}"
        exit 1
    fi
    FILES="$PROJECT_ROOT/$TARGET_FILE"
    echo -e "${YELLOW}检查范围: 单个文件 - $TARGET_FILE${NC}"
elif [ -n "$TARGET_DIR" ]; then
    SEARCH_DIR="$PROJECT_ROOT/$TARGET_DIR"
    if [ ! -d "$SEARCH_DIR" ]; then
        echo -e "${RED}错误: 目录不存在: $TARGET_DIR${NC}"
        exit 1
    fi
    FILES=$(find "$SEARCH_DIR" -type f \( -name "*.cpp" -o -name "*.hpp" \))
    echo -e "${YELLOW}检查范围: 目录 - $TARGET_DIR${NC}"
else
    # 检查整个项目（排除 3rdparty 和 build）
    FILES=$(find "$PROJECT_ROOT" -type f \( -name "*.cpp" -o -name "*.hpp" \) \
        ! -path "*/build/*" \
        ! -path "*/3rdparty/*" \
        ! -path "*/.git/*")
    echo -e "${YELLOW}检查范围: 整个项目${NC}"
fi

if [ -z "$FILES" ]; then
    echo -e "${YELLOW}未找到需要检查的文件${NC}"
    exit 0
fi

FILE_COUNT=$(echo "$FILES" | wc -l)
echo -e "${GREEN}找到 $FILE_COUNT 个文件${NC}"
echo ""

if [ "$AUTO_FIX" = true ]; then
    echo -e "${RED}⚠️  警告: 自动修复模式已启用${NC}"
    echo -e "${RED}⚠️  这会直接修改源代码文件${NC}"
    echo ""
    read -p "确认继续? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "已取消"
        exit 0
    fi
    FIX_ARG="-fix"
else
    FIX_ARG=""
fi

# 运行 clang-tidy
TOTAL_WARNINGS=0
TOTAL_ERRORS=0

while IFS= read -r file; do
    relative_path="${file#$PROJECT_ROOT/}"
    echo -e "${BLUE}检查: $relative_path${NC}"

    OUTPUT=$(clang-tidy $FIX_ARG -p "$PROJECT_ROOT/build" "$file" 2>&1 || true)

    # 统计警告和错误
    WARNINGS=$(echo "$OUTPUT" | grep -c "warning:" || true)
    ERRORS=$(echo "$OUTPUT" | grep -c "error:" || true)

    TOTAL_WARNINGS=$((TOTAL_WARNINGS + WARNINGS))
    TOTAL_ERRORS=$((TOTAL_ERRORS + ERRORS))

    if [ $WARNINGS -gt 0 ] || [ $ERRORS -gt 0 ]; then
        echo "$OUTPUT"
        echo -e "${YELLOW}  → 警告: $WARNINGS, 错误: $ERRORS${NC}"
    else
        echo -e "${GREEN}  → ✓ 无问题${NC}"
    fi
    echo ""
done <<< "$FILES"

echo -e "${BLUE}==============================================================================${NC}"
echo -e "${BLUE}  检查完成${NC}"
echo -e "${BLUE}==============================================================================${NC}"
echo -e "检查文件数: $FILE_COUNT"
echo -e "总警告数: $TOTAL_WARNINGS"
echo -e "总错误数: $TOTAL_ERRORS"
echo ""

if [ $TOTAL_ERRORS -gt 0 ]; then
    echo -e "${RED}发现错误！请修复后再提交代码。${NC}"
    exit 1
elif [ $TOTAL_WARNINGS -gt 0 ]; then
    echo -e "${YELLOW}发现警告，建议修复。${NC}"
    exit 0
else
    echo -e "${GREEN}✓ 代码质量检查通过！${NC}"
    exit 0
fi
