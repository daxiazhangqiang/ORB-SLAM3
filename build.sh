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

# 并行编译使用全部 CPU 核心（本机 16 核）
JOBS="-j$(nproc)"

# -----------------------------------------------------------------------------
# [1/5] DBoW2：词袋模型库（用于回环检测时生成/检索视觉单词向量）
# -----------------------------------------------------------------------------
echo "==> [1/5] 编译 Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $JOBS

# -----------------------------------------------------------------------------
# [2/5] g2o：图优化库（BA、位姿图优化、IMU 预积分等后端优化）
# -----------------------------------------------------------------------------
echo "==> [2/5] 编译 Thirdparty/g2o ..."
cd ../../g2o
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $JOBS

# -----------------------------------------------------------------------------
# [3/5] Sophus：李群/李代数库（SE3/SO3/Sim3，位姿更新与求导）
#        注意：必须关闭 tests/examples，避免 Eigen 3.4 的 -Werror 编译失败
# -----------------------------------------------------------------------------
echo "==> [3/5] 编译 Thirdparty/Sophus ..."
cd ../../Sophus
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make $JOBS

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
make $JOBS

echo ""
echo "==> 全部编译完成 ✅"
echo "    核心库:         lib/libORB_SLAM3.so"
echo "    单目+IMU 示例:  Examples/Monocular-Inertial/mono_inertial_euroc"
echo "    其他示例:       Examples/{Monocular,Stereo,RGB-D,...} 目录下"
