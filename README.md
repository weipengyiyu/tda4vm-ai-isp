# TDA4VM RAW AI ISP

从 TDA4VM（J721E）工程提取的 TIOVX Host、Target Kernel 和 Vision Apps 源码。保留原 SDK 相对路径以及源码中的版权和许可条款。

```text
CSI2-RX Capture → C66x_1 RAW 预处理 → C7x/MMA TIDL → C66x_2 RAW 重建 → VPAC VISS
```

## 代码入口

- `vision_apps/apps/dl_demos/app_tidl_cam/` 包含 A72 应用、TIOVX 图调度、RAW 前后处理模块和示例配置。
- `vision_apps/kernels/img_proc/host/` 包含 OCPreProc 和 OCPostProc 注册、参数验证及节点 API。对应 C66x 实现在 `vision_apps/kernels/img_proc/c66/vx_oc_{pre,post}_proc_target.c`。
- `vision_apps/modules/` 包含 Capture、TIDL、VISS、AEWB、LDC、Mosaic 和 Display 模块。Capture 的 Host 和 Target 位于 `video_io/kernels/video_io/`。
- `c7x-mma-tidl/arm-tidl/tiovx_kernels/tidl/` 包含 TIDL 节点接口和 DSP Target。推理实现涉及 `c7x-mma-tidl/ti_dl/algo/src/` 和 SDK 中的其余 TIDL 组件。
- `imaging/` 包含相机驱动、DCC 源配置及供固件编译使用的 DCC 头文件。`vision_apps/kernels/fileio/` 包含 RAW 调试写出相关代码。

## 分支与来源

目标仓库保留六个有相关源码的源分支快照。各分支提取提交中的 `Source-Commit` 记录完整源 SHA。目标 `main` 指向 SC2210 降噪快照。源 `main`（`ac140be5c`）只有说明文件。

- `2210_denoise_ai_isp_develop`（`092b17883`）包含 SC2210 线性与三 VC 采集配置、RAW 1C/4C 转换、前帧状态反馈、离线回放和 ITT 抓数。
- `2210_hdr_3ch_isp_develop`（`135a80901`）增加三 VC RAW 重映射及写出节点。该节点接在 Capture 的调试写出路径上，AI 图中的 VISS 仍接收 OCPostProc 输出。
- `623_feature_isp_develop`（`0b7aef98f`）加入 IMX623 / MAX623、DCC 和 VISS PWL 适配。Capture 的 RAW 对象数按启用相机数配置。
- `x3c_built_in_isp_develop`（`29eda308f`）加入 OX03C10 built-in WDR、AEWB 和 DCC 适配。
- `x3c_denoise_ai_isp_develop`（`805a67d35`）保留 OX03C10 线性 RAW AI 图和 1C/4C 路径。该快照早于前帧状态反馈。
- `tda4vm_base_rtos_j721e_evm_09000101`（`60ca142b8`）为 9.0 SDK 历史参考。其 APP 将 Capture 接到 VISS，再经 LDC 和 Scaler 进入推理。

## 集成与构建

开发分支基于 TI Processor SDK RTOS J721E 09.01.00.06。历史参考分支使用 09.00.01.01。仓库是 SDK 源码覆盖层，依赖完整的对应版本 SDK、TI 编译器和配套 Linux ADAS 文件系统。完整 SDK、固件、模型、测试 RAW 和实验备份由构建环境单独提供。

在 Linux 上准备一份干净的对应版本 SDK。将当前分支已跟踪文件按原路径覆盖到 SDK 根目录，然后用 SDK 自带的环境初始化和构建流程编译。更换分支时使用一份干净的 SDK 副本，避免保留上一个分支独有的文件。

```bash
# 在已检出所需分支的提取仓库执行
export PSDKR_PATH=/absolute/path/to/ti-processor-sdk-rtos-j721e-evm
git archive HEAD c7x-mma-tidl imaging sdk_builder tiovx video_io vision_apps |
  tar -xf - -C "$PSDKR_PATH"

cd "$PSDKR_PATH"
./sdk_builder/scripts/setup_psdk_rtos.sh --skip_pc_emulation
cd sdk_builder
make sdk
```

完成 SDK 的 Linux 文件系统 staging 和固件部署后，在板端按实际相机、VC 和模型配置运行：

```bash
/opt/vision_apps/vx_app_tidl_cam.out --cfg /opt/vision_apps/app_tidl_cam.cfg
```

- 在支持配置开关的开发分支，`c7x_viss_display=1` 选择 RAW AI 图，`0` 选择 Capture RAW 直通 VISS。设置 `tidl_config` 和 `tidl_network` 为同一次模型导入生成的 I/O 描述与网络文件，模型张量的位宽和布局须与前后处理一致。示例配置中的路径需要按板端环境调整。
- `config/app_4c_zhitong.cfg` 演示离线回放，需提供同一 Bayer 布局的 RAW 和匹配的 TIDL 模型文件。
- SC2210 快照的 Capture 通道数按启用相机数乘三，单曝光板端运行前需核对实际 VC 配置。该分支的 AI 预处理从第一个 RAW 缓冲取样，三曝光 RAW10 调试路径需要单独验收。
