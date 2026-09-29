#!/bin/bash
# =============================================================================
# ORB_SLAM3 一键构建脚本（适配 Ubuntu 24.04 + g++13 + OpenCV4.6 + Pangolin0.9）
#
# 用法：
#   ./build.sh          增量编译（默认，只重编改动过的部分，可反复运行）
#   ./build.sh clean    干净全量重编（先删除所有旧构建产物，首次约 10~20 分钟）
#
# 说明：
#   1) 本仓库 CMakeLists.txt 已把 C++ 标准改为 -std=c++17（Pangolin 0.9 的 sigslot 需要）
#   2) Thirdparty/Sophus 是纯头文件库，必须关闭 tests/examples，
#      否则 Eigen 3.4 的告警会被 -Werror 当成错误导致编译失败
#   3) 日常只调试单目+IMU 示例时，不必跑本脚本，直接：
#         cd build && make -j8 mono_inertial_euroc
# =============================================================================

set -e                                   # 任一步失败立即退出
cd "$(dirname "$0")"                     # 切到脚本所在目录（仓库根目录），与调用位置无关

# -----------------------------------------------------------------------------
# 可选：加 clean 参数时，先清理旧构建产物，保证从零全量重编
# -----------------------------------------------------------------------------
if [ "$1" = "clean" ]; then
    echo "==> 清理旧构建产物（clean 全量重编）..."
    rm -rf build \
           Thirdparty/DBoW2/build \
           Thirdparty/g2o/build \
           Thirdparty/Sophus/build
fi

# -----------------------------------------------------------------------------
# 并行度：按**可用内存**算，不按核数算（本机 16 核但只有 14 GB 内存）
#   单个编译单元峰值实测 1.1~2.4 GB：System.cc 1.11 GB，而最重的
#   Optimizer.cc / Tracking.cc / LoopClosing.cc 这类塞满 Eigen/g2o 模板的大文件
#   单进程就要 2.14~2.33 GB（cc1plus anon-rss 实测，见 build_notes.md §11）：
#     · -j16 于 2026-09-21 触发全局 OOM（build_notes.md §6）
#     · -j8  于 2026-09-29 把整机卡死、只能硬重启（build_notes.md §11）
#   宁可慢一点，不要再卡机。覆盖方式：
#     JOBS="-j2" ./build.sh     手动指定并行度
#     FORCE=1    ./build.sh     内存不足时也强行编译（不推荐）
# -----------------------------------------------------------------------------
if [ -z "${JOBS:-}" ]; then
    mem_avail_mb=$(awk '/^MemAvailable/{print int($2/1024)}' /proc/meminfo)
    j=$(( (mem_avail_mb - 3072) / 2500 ))  # 留 3 GB 给桌面，每个编译单元按 2.5 GB 算
    [ "$j" -gt 4 ] && j=4                  # 再宽也不超过 -j4
    [ "$j" -lt 1 ] && j=1
    JOBS="-j$j"
    echo "==> 可用内存 ${mem_avail_mb} MB → 并行度 ${JOBS}（单编译单元按 2.5 GB 计，实测最重 2.33 GB）"
fi

# 内存底线：低于 2.5 GB 直接不编，避免把桌面乃至整机拖死
if [ "${FORCE:-0}" != "1" ]; then
    mem_avail_mb=$(awk '/^MemAvailable/{print int($2/1024)}' /proc/meminfo)
    if [ "$mem_avail_mb" -lt 2500 ]; then
        echo "[x] 可用内存仅 ${mem_avail_mb} MB（< 2500 MB），已停止编译。"
        echo "    先关掉 VS Code / Chrome / Docker，或等大任务结束；真要硬上用 FORCE=1 ./build.sh"
        echo "    背景：build_notes.md §6 / §11 —— 编译爆内存会先杀 VS Code，再拖死整机。"
        exit 1
    fi
fi

# 修掉上一次被中断的编译留下的零字节目标文件（build_notes.md §7 / §11）：
# make 认为它们「比源码新」因而不会重编，直接拿去链接就会生成残缺产物。
zero_objs=$(find build Thirdparty -name "*.o" -size 0 2>/dev/null || true)
if [ -n "$zero_objs" ]; then
    echo "==> 发现上次编译被打断留下的零字节目标文件，删除后重编："
    echo "$zero_objs" | sed 's/^/      /'
    find build Thirdparty -name "*.o" -size 0 -delete 2>/dev/null || true
fi

# 让路给交互进程：编译不抢 CPU/IO，桌面才不会卡成幻灯片
if command -v ionice >/dev/null 2>&1; then
    MAKE=(nice -n 15 ionice -c2 -n7 make)
else
    MAKE=(nice -n 15 make)
fi

# -----------------------------------------------------------------------------
# [1/5] DBoW2：词袋模型库（用于回环检测时生成/检索视觉单词向量）
# -----------------------------------------------------------------------------
echo "==> [1/5] 编译 Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
"${MAKE[@]}" $JOBS

# -----------------------------------------------------------------------------
# [2/5] g2o：图优化库（BA、位姿图优化、IMU 预积分等后端优化）
# -----------------------------------------------------------------------------
echo "==> [2/5] 编译 Thirdparty/g2o ..."
cd ../../g2o
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
"${MAKE[@]}" $JOBS

# -----------------------------------------------------------------------------
# [3/5] Sophus：李群/李代数库（SE3/SO3/Sim3，位姿更新与求导）
#        注意：必须关闭 tests/examples，避免 Eigen 3.4 的 -Werror 编译失败
# -----------------------------------------------------------------------------
echo "==> [3/5] 编译 Thirdparty/Sophus ..."
cd ../../Sophus
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
"${MAKE[@]}" $JOBS

# -----------------------------------------------------------------------------
# [4/5] 解压预训练 ORB 视觉词典（系统启动时必须加载）
# -----------------------------------------------------------------------------
echo "==> [4/5] 解压词袋文件 Vocabulary/ORBvoc.txt ..."
cd ../../../
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

# -----------------------------------------------------------------------------
# [5/5] 编译 ORB_SLAM3 核心库 lib/libORB_SLAM3.so + 全部示例可执行文件
# -----------------------------------------------------------------------------
echo "==> [5/5] 配置并编译 ORB_SLAM3 ..."
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
"${MAKE[@]}" $JOBS

echo ""
echo "==> 全部编译完成 ✅"
echo "    核心库:         lib/libORB_SLAM3.so"
echo "    单目+IMU 示例:  Examples/Monocular-Inertial/mono_inertial_euroc"
echo "    其他示例:       Examples/{Monocular,Stereo,RGB-D,...} 目录下"
