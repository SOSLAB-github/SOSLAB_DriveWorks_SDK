# SOSLAB LiDAR Plugin for NVIDIA DRIVEWORKS

## Table of Contents
1. [Getting Started](#getting-started)
2. [Environment and Dependencies](#environment-and-dependencies)
3. [Build](#build)
4. [Configuration](#configuration)
5. [Run](#run)
6. [Contact](#contact)
7. [License](#license)

---

## Getting Started

This plugin enables **SOSLAB LiDAR** sensor support in **NVIDIA DRIVEWORKS 5.20**.  
Supported LiDAR models:

- ML-X Series
- ML-A Series

The plugin is developed using NVIDIA's Custom Lidar Interface.  
For more information, refer to the official NVIDIA documentation:

🔗 [NVIDIA Custom Lidar Interface Documentation](https://developer.nvidia.com/docs/drive/driveworks/latest/nvsdk_dw_html/sensorplugins_lidarsensor.html)

### Directory Structure

- `src/`, `include/` : Source code
- `prebuild/` : Pre-built plugin libraries
    - Includes builds for x86 and ARM architectures
    - Contains shell scripts for execution

---

## Environment and Dependencies

**System Environment:** Linux + NVIDIA DriveWorks SDK  
**Verified Environment:**
- Ubuntu 20.04 with NVIDIA DriveWorks SDK 5.20

> ⚠️ Make sure **DRIVE OS** is installed on your PC or DRIVE system. Also ensure that the NVIDIA-provided sample programs compile successfully on your system.

For detailed DRIVE OS installation instructions, refer to:  
🔗 [DRIVE OS Installation Guide](https://developer.nvidia.com/docs/drive/drive-os/6.0.10/public/drive-os-linux-installation/index.html)

---

## Build

The SOSLAB plugin requires the **NVIDIA DriveWorks SDK** to be properly installed.

### 1. Verify DriveWorks SDK

Confirm that the DriveWorks SDK is installed or compiled.  
Default installation path:

```
/usr/local/driveworks
```

### 2. Create Symbolic Link to Plugin

Create a symbolic link from your downloaded plugin to the DriveWorks sample folder:

```
sudo ln -s [PATH_TO_YOUR_DOWNLOADED_PLUGIN]/SOSLAB_DriveWorks_SDK [PATH_TO_DRIVEWORKS_SAMPLE_FOLDER]/src/SOSLAB_DriveWorks_SDK
```
The default sample folder is usually located at:

```
/usr/local/driveworks/samples
```

### 3. Update CMakeLists.txt

Append the following line to [PATH_TO_DRIVEWORKS_SAMPLE_FOLDER]/CMakeLists.txt to include the plugin module.

```
add_subdirectory(src/SOSLAB_DriveWorks_SDK)
```

### 4. Build and Install

#### Compile for PC (x86)

```
cd [PATH_TO_DRIVEWORKS_SAMPLE_FOLDER]
mkdir build && cd build
cmake -S ../ -B ./
make -j
make install
```

#### Cross-compile for NVIDIA DRIVE AGX Orin

```
cd [PATH_TO_DRIVEWORKS_SAMPLE_FOLDER]
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/usr/local/driveworks/samples/cmake/Toolchain-V5L.cmake -S ../ -B ./
make -j
make install
```

---

## Run

To run the LiDAR example program, follow the steps below using the default DriveWorks path.

### Using Compiled Plugin

After building DriveWorks with the plugin, execute the following:

```
cd [PATH_TO_DRIVEWORKS_SAMPLE_BUILD_FOLDER]/install/usr/local/driveworks/samples/bin/

./sample_lidar_replay --protocol=lidar.custom --params=ip=192.168.1.10,port=2000,decoder-path=./libSOSLAB_DriveWorks_plugin.so
```

### Using Prebuilt Plugin

If you are using the prebuilt libraries, move the appropriate file to the sample folder and then run:

```
cp [PLUGIN_PATH]/prebuild/[ARCHITECTURE]/libSOSLAB_DriveWorks_plugin.so /usr/local/driveworks/bin/libSOSLAB_DriveWorks_plugin.so

cd /usr/local/driveworks/bin/

./sample_lidar_replay --protocol=lidar.custom --params=ip=192.168.1.10,port=2000,decoder-path=/usr/local/driveworks/bin/libSOSLAB_DriveWorks_plugin.so
```

### ML-A

![Alt text](./Etc/ML-A_Screenshot.png?raw=true "ML-A LiDAR Image")

---

## Contact

- For plugin-specific support, please use the **GitHub Issues** tab in this repository.
- For general SOSLAB product support, please contact **SOSLAB Customer Support**.

---

## License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

> The MIT License is a permissive license that allows for maximum freedom in using, modifying, and distributing the software while providing minimal restrictions.
