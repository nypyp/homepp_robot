# Homepp_robot

源码：[nypyp/homepp_robot: home++ robot ros2 packages (github.com)](https://github.com/nypyp/homepp_robot)

## 功能

* ER-NeRF 数字人接入大语言模型问答网页

  * 数字人大语言模型对话框（包含语音输入及文本框输入）
  * 数字孪生三维模型、生理参数展示网页（渲染视频流中人物动作为三维模型）
  * 表情识别（调用设备摄像头上传视频到服务器进行表情识别渲染后返回到网页）
  * 生理参数采集（通过 mqtt 接收生理参数采集模块数据，展示在网页）
* TTS 实时语音合成、语音克隆
* 边缘端离线语音识别、语音唤醒
* 边缘端摔倒检测并发送 mqtt 消息
* 边缘端目标检测算法
* 边缘端目标跟随算法、跟随 PID 运动规划
* 手机端目标检测算法
* 边缘端 VINS-Fusion 视觉里程计算法
* ROS2 NeRF 实时训练算法 Nerf_bridge

## 平台信息

#### 深度学习服务器

系统：Ubuntu 20.04

GPU：RTX 3090

显卡驱动版本：535.183.01

CUDA：12.2

#### 公网服务器

公网网址：[8.134.150.174](https://ossrs.net/lts/zh-cn/docs/v6/doc/getting-started-build) | [www.jianhuguide.top](http://www.jianhuguide.top)

双核 CPU 服务器

硬盘大小 40G

## ER-NeRF 数字人

<span data-type="text" id="">metahuman笔记</span>

项目地址：[github: metahuman](https://github.com/lipku/LiveTalking)

### 安装项目环境

安装附加指导：<span data-type="text" id="">nerfstream</span>

> info
>
> 现在仓库更新后，基本环境已经改变，以实际为准

### SRS 服务器

SRS 官方手册：[Docker 镜像](https://ossrs.net/lts/zh-cn/docs/v6/doc/getting-started)

可以使用 Docker 方式以及源码编译方式简单部署 SRS 本地服务器，在公网端服务器采用源码编译方式部署，项目中配置了特定的端口启动 SRS 服务器

#### docker 安装

Docker 官方手册：[Docker Engine](https://docs.docker.com/engine/install/ubuntu/)

首先需要安装 Docker Engine，根据官方手册使用添加官方源直接安装 Docker，安装后验证 Docker 安装是否成功

```shell
sudo docker run hello-world
# 列出镜像
docker ps
# 关闭运行中的镜像
docker stop <PID>
```

接下来安装 docker 版本 SRS：[Docker | SRS](https://ossrs.net/lts/zh-cn/docs/v6/doc/getting-started)

#### Build 源码编译

SRS 帮助手册：[源码编译](https://ossrs.net/lts/zh-cn/docs/v6/doc/getting-started-build)

在公网端服务器考虑到空间大小有限，使用源码编译方式直接构建

### 部署 TTS 服务器

> info
>
> 当前更新版本已经全部署为 edge tts

xtts 仓库：[github: xtts-streaming-server](https://github.com/coqui-ai/xtts-streaming-server)

部署 cuda 12.2 版本的 xtts Docker 镜像，安装过程中由于网络问题可能会导致下载模型失败，解决方法见 <span data-type="text" id="">XTTS 声音克隆</span>，通过对 Docker 配置 Proxy 端口解决问题，其中要分辨 Docker Proxy 分为对 Docker 进行代理以及对 Docker 镜像进行代理，镜像代理需要配置相应的代理端口

当在镜像中安装部署成功模型后，每次关闭镜像都会重置镜像内模型缓存，通过对模型进行外部挂载将模型映射到镜像中去：<span data-type="text" id="">XTTS Docker启动</span>

```shell
docker run  --gpus=all -v "/path/to/xtts_v2":"/app/tts_models" -e COQUI_TOS_AGREED=1 --rm -p 9000:80 ghcr.io/coqui-ai/xtts-streaming-server:latest-cuda121
```

‍
