# OpenEB 编译指南

## 系统环境

| 项目       | 版本                    |
|------------|-------------------------|
| 操作系统   | Ubuntu 26.04            |
| 架构       | amd64 (x86_64)          |
| GCC        | 15.x（系统默认）        |
| CMake      | 4.2.3                   |
| Python     | 3.14（系统默认，不兼容 openeb） |
| 编译用 Python | 3.12（via deadsnakes PPA） |

## 注意事项

1. **Python 版本**：OpenEB 官方仅支持 Python 3.9~3.12，系统自带 Python 3.14 不兼容（`numba` 等依赖限制）。需通过 deadsnakes PPA 安装 Python 3.12。
2. **GCC 15 兼容性**：GCC 15 不再隐式包含 `<cstdint>`，导致 `uint8_t`、`uint16_t` 等类型未声明。需在 CMakeLists.txt 中添加全局编译选项修复。
3. **包名变化**：Ubuntu 26 中 `libcanberra-gtk-module` 已替换为 `libcanberra-gtk3-module`。

## 编译步骤

### 1. 安装系统依赖

```bash
sudo apt update
sudo apt -y install apt-utils build-essential software-properties-common wget unzip curl git cmake
sudo apt -y install libopencv-dev libboost-all-dev libusb-1.0-0-dev libprotobuf-dev protobuf-compiler
sudo apt -y install libhdf5-dev hdf5-tools libglew-dev libglfw3-dev libcanberra-gtk3-module ffmpeg
sudo apt -y install libgl-dev libglx-dev libopengl-dev
# 可选（测试用）：
sudo apt -y install libgtest-dev libgmock-dev
```

### 2. 安装 Python 3.12（via deadsnakes PPA）

```bash
sudo apt install software-properties-common
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.12 python3.12-venv python3.12-dev
```

### 3. 安装 pybind11 v2.11.0

```bash
cd /tmp
wget https://github.com/pybind/pybind11/archive/v2.11.0.zip
unzip v2.11.0.zip
cd pybind11-2.11.0/
mkdir build && cd build
cmake .. -DPYBIND11_TEST=OFF -DPython3_EXECUTABLE=/usr/bin/python3.12
cmake --build .
sudo cmake --build . --target install
```

### 4. 创建 Python 虚拟环境并安装依赖

```bash
python3.12 -m venv /tmp/prophesee/py3venv --system-site-packages
/tmp/prophesee/py3venv/bin/python -m pip install pip --upgrade
/tmp/prophesee/py3venv/bin/python -m pip install -r OPENEB_SRC_DIR/utils/python/requirements_openeb.txt
```

> ML 依赖（`requirements_pytorch_cpu.txt`）可选安装，其中 `torch==2.9.1` 需确认是否支持 Python 3.12。

### 5. 修复 GCC 15 兼容性问题

在 `OPENEB_SRC_DIR/CMakeLists.txt` 的 `project()` 行之后添加：

```cmake
# GCC 15+ no longer implicitly includes <cstdint>; add it globally to fix uint8_t/uint16_t etc.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "15")
    add_compile_options("-include;cstdint")
endif()
```

### 6. 编译

```bash
cd OPENEB_SRC_DIR
rm -rf build
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/tmp/prophesee/py3venv/bin/python3.12
cmake --build . --config Release -- -j$(nproc)
```

### 7. 配置环境变量（选择一种方式）

**方式一：从 build 目录直接使用**

```bash
source OPENEB_SRC_DIR/build/utils/scripts/setup_env.sh
# 可添加到 ~/.bashrc 使其永久生效
```

**方式二：部署到系统路径**

```bash
sudo cmake --build . --target install
# 然后设置环境变量（添加到 ~/.bashrc）：
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
export HDF5_PLUGIN_PATH=$HDF5_PLUGIN_PATH:/usr/local/lib/hdf5/plugin
```

## 遇到的问题与解决方案

| 问题                                  | 原因                                        | 解决方案                                           |
|---------------------------------------|---------------------------------------------|----------------------------------------------------|
| `numba` 安装失败（Python 3.14）       | numba 仅支持 Python >=3.9,<3.13            | 使用 deadsnakes PPA 安装 Python 3.12              |
| `uint16_t`/`uint8_t` 未声明           | GCC 15 不再隐式包含 `<cstdint>`            | CMakeLists.txt 添加 `-include cstdint`            |
| `libcanberra-gtk-module` 无候选       | Ubuntu 26 中包名已变更                      | 使用 `libcanberra-gtk3-module`                    |
| `OpenGL` 库找不到                     | 未安装 OpenGL 开发库                        | 安装 `libgl-dev libglx-dev libopengl-dev`         |
| `GLEW` 库找不到                       | 未安装 GLEW 开发库                          | 安装 `libglew-dev`                                |
| `glfw3` 配置文件找不到                | 未安装 GLFW3 开发库                         | 安装 `libglfw3-dev`                               |
