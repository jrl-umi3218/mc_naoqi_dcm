# mc_naoqi_dcm

Fast communication module between NAO/PEPPER robot sensors and actuators and [`mc_rtc`](https://jrl-umi3218.github.io/mc_rtc/index.html) control framework.
This is a local robot module, which needs to be cross-compiled for the desired platform (NAO or Pepper), and uploaded on the robot. This module provides fast access to the low level [Device Communication Manager](https://developer.softbankrobotics.com/pepper-naoqi-25/naoqi-developer-guide/naoqi-apis/dcm) module of [NAOqi OS](https://developer.softbankrobotics.com/pepper-naoqi-25), it allows to set/get actuator values under 12ms.

## Building

This module needs to be cross-compiled and sent to the robot.

### 1. Installing `qiBuild` and creating a worktree

1. Install **qiBuild**: `pip install qibuild --user`
1. Configure **qiBuild**: `qibuild config --wizard`
1. Create a **qiBuild worktree**: `mkdir qiBuild_wt`
1. Enter **qiBuild worktree**: `cd qiBuild_wt`
1. Initialize your worktree: `qibuild init`

### 2. Installing and configuring NAOqi C++ SDK

1. Download [NAOqi C++ SDK](https://developer.softbankrobotics.com/pepper-naoqi-25-downloads-linux): `naoqi-sdk-x.x.x-[OS].tar.gz`
1. Extract this archive into (e.g.) `naoqi-sdk` folder
1. Create toolchain for cross compilation: `qitoolchain create naoqi-cct naoqi-sdk/toolchain.xml`
1. Enter previously created **qiBuild worktree**: `cd qiBuild_wt`
1. Create toolchain build configuration and set it as default in your **qiBuild worktree**: `qibuild add-config naoqi-cct-config -t naoqi-cct --default`

**Note:** For more information or troubleshooting regarding `qiBuild` or `NAOqi C++ SDK` see the official [NAOqi C++ SDK installation guide](https://developer.softbankrobotics.com/pepper-naoqi-25/naoqi-developer-guide/sdks/c-sdk/c-sdk-installation-guide).

### 3. Clone and build `mc_naoqi_dcm`

1. Clone this project into your **qiBuild worktree**: `git clone git@gite.lirmm.fr:softbankrobotics/mc_naoqi_dcm.git`
1. Enter the project folder: `cd mc_naoqi_dcm`
1. Configure the project (to build for either Pepper or NAO): `qibuild configure --release -DROBOT_NAME=<pepper|nao>`
1. Build the local robot module: `qibuild make`

### 4. Upload module to the robot and configure autoload

1. Transfer `libmc_naoqi_dcm.so` file to the robot:
```bash
rsync build-naoqi-cct-config/sdk/lib/naoqi/libmc_naoqi_dcm.so nao@pepper.local:/home/nao/naoqi/
```
1. Login to robot system:
```bash
ssh nao@<robot_ip>
```
1. Configure `mc_naoqi_dcm` module to be auto-loaded when robot starts operating. Modify configuration file:
```bash
nano naoqi/preferences/autoload.ini
```
to contain the following line:
```bash
[user]
/home/nao/naoqi/libmc_naoqi_dcm.so
```
1. Restart robot system:
```bash
nao restart
```

### 5. All done | Next steps
The robot is now running our uploaded local module `mc_naoqi_dcm` and is ready to be controlled via [`mc_rtc`](https://jrl-umi3218.github.io/mc_rtc/index.html) controller using [`mc_naoqi`](https://gite.lirmm.fr/multi-contact/mc_naoqi) interface.

You can refer to the sample Pepper Finite State Machine (FSM) `mc_rtc` controller  project: [`PepperFSMController`](https://gite.lirmm.fr/mc-controllers/pepperfsmcontroller) for an example (or a starting point) for creating your own `mc_rtc` controller for Pepper.

---
The URLs in this file were valid on 20.07.20
