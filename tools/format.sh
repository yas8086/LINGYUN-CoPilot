#!/usr/bin/env bash
# 灵云01号飞艇伴飞电脑 — 代码格式化(以 ament_uncrustify 为权威,ROS2 标准)
#
# 用法:
#   ./tools/format.sh            # 格式化所有 C++ 源文件(原地修改)
#   ./tools/format.sh --check    # 仅检查是否有格式差异,不修改(退出码非0表示有差异)
#
# 提交前请始终运行本脚本;.clang-format 仅供 IDE 实时预览,不作为权威。
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# 先 source ROS2 环境(set -u 与 ROS setup.bash 中的未定义变量不兼容,故仅用 -eo pipefail)
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

CPP_FILES=$(find src -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.cc' -o -name '*.cxx' \))
if [ -z "${CPP_FILES}" ]; then
  echo "未找到 C++ 源文件"
  exit 0
fi

if [ "${1:-}" = "--check" ]; then
  echo "==> ament_uncrustify 检查模式(不修改文件)"
  ament_uncrustify ${CPP_FILES}
  echo "==> 格式检查通过"
else
  echo "==> ament_uncrustify --reformat(原地格式化)"
  ament_uncrustify --reformat ${CPP_FILES}
  echo "==> 完成。请用 'git diff' 复核格式化改动后再提交。"
fi
