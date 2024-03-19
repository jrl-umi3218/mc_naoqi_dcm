# mc_naoqi_dcm
[![CI of mc_naoqi_dcm with Docker](https://github.com/jrl-umi3218/mc_naoqi_dcm/actions/workflows/build-docker.yml/badge.svg)](https://github.com/jrl-umi3218/mc_naoqi_dcm/actions/workflows/build-docker.yml)[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/jrl-umi3218/mc_naoqi_dcm/master.svg)](https://results.pre-commit.ci/latest/github/jrl-umi3218/mc_naoqi_dcm/master)

Fast communication module between NAO/PEPPER robot sensors and actuators and [`mc_rtc`](https://jrl-umi3218.github.io/mc_rtc/index.html) control framework.
This is a local robot module, which needs to be cross-compiled for the desired platform (NAO or Pepper), and uploaded on the robot. This module provides fast access to the low level [Device Communication Manager](https://developer.softbankrobotics.com/pepper-naoqi-25/naoqi-developer-guide/naoqi-apis/dcm) module of [NAOqi OS](https://developer.softbankrobotics.com/pepper-naoqi-25), it allows to set/get actuator values under 12ms.

# Building

This module needs to be cross-compiled and sent to the robot. For this you have three options (choose which one fits you best):

## Option 1: Build locally using the cross compilation toolchain

### 1. Installing `qiBuild` and creating a worktree

1. Install **qiBuild**: `pip install qibuild --user`
1. Configure **qiBuild**: `qibuild config --wizard`
1. Create a **qiBuild worktree**: `mkdir qibuild_ws`
1. Enter **qiBuild worktree**: `cd qibuild_ws`
1. Initialize your worktree: `qibuild init`

### 2. Downloading and configuring cross-compilation toolchain

1. Download the [Cross Compilation Toolchain](https://developer.softbankrobotics.com/pepper-naoqi-25-downloads-linux). Unfortunately the toolchain required for naoqi 2.5 is no longer officially provided, you may download a copy here:
   ```sh
   wget -c https://seafile.lirmm.fr/f/5389d0a64d79481ab49e/?dl=1 -O ctc-naoqi.tar.xz
   ```
   We advise you to keep a copy of that archive in case the above link becomes unavailable as this archive cannot be found elsewhere online.
1. Extract this archive
   ```sh
   tar -xf ctc-naoqi.tar.xz .
   ```
1. Create toolchain for cross compilation: `qitoolchain create ctc-naoqi ctc-naoqi-2.5.0/toolchain.xml`
1. Enter previously created **qiBuild worktree**: `cd qibuild_ws`
1. Create toolchain build configuration and set it as default in your **qiBuild worktree**: `qibuild add-config ctc-naoqi-config -t ctc-naoqi --default`

### 3. Clone and build `mc_naoqi_dcm`

1. Clone this project into your **qiBuild worktree**: `git clone https://github.com/jrl-umi3218/mc_naoqi_dcm.git`
1. Enter the project folder: `cd mc_naoqi_dcm`
1. Configure the project (to build for either Pepper or NAO): `qibuild configure --release -DROBOT_NAME=<pepper|nao>`
1. Build the local robot module: `qibuild make`
  * **Note**: you may need to run `export LC_ALL=C` if you encounter the following error while building:
  ```bash
  as: loadlocale.c:129: _nl_intern_locale_data: Assertion `cnt < (sizeof (_nl_value_type_LC_TIME) / sizeof (_nl_value_type_LC_TIME[0]))' failed.
  ```

You may now upload `qibuild_ws/mc_naoqi_dcm/build-ctc-naoqi-config/sdk/lib/naoqi/libmc_naoqi_dcm.so` to the robot.

## Option 2: Build using the provided Dockerfile

You may use the provided `Dockerfile` that sets up the cross-compilation environment for you. If you already followed `Option 1` this is not necessary.

```sh
cd <mc_naoqi_dcm>
docker build -t docker-naoqi-dcm -f Dockerfile
id=$(docker create docker-naoqi-dcm)
docker cp $id:/libmc_naoqi_dcm.so /tmp/libmc_naoqi_dcm.so
docker rm -v $id
```
You may now upload the `/tmp/libmc_naoqi_dcm.so` on the robot

## Option 3: Using the pre-built library

You may find a pre-built version of `libmc_naoqi_dcm.so` in the [Github Artefacts](https://github.com/arntanguy/mc_naoqi_dcm/actions/workflows/build-docker.yml): click on `[CI of mc_naoqi_dcm with Docker]` then click on the latest successful action and scroll down to the Artefacts section where you can download the pre-compiled `libmc_naoqi_dcm.so` library.

# Installing on the robot

The installation consists of uploading the module to the robot and making it automatically load on startup

1. Transfer `libmc_naoqi_dcm.so` file to the robot:
```bash
rsync build-ctc-naoqi-config/sdk/lib/naoqi/libmc_naoqi_dcm.so nao@pepper.local:/home/nao/naoqi/ # For Option 1
# Or:
rsync /tmp/libmc_naoqi_dcm.so nao@pepper.local:/home/nao/naoqi/ # For Option 2
```
2. Login to robot system:
```bash
ssh nao@<robot_ip>
```
3. Configure `mc_naoqi_dcm` module to be auto-loaded when robot starts operating. Modify configuration file:
```bash
nano naoqi/preferences/autoload.ini
```
to contain the following line:
```bash
[user]
/home/nao/naoqi/libmc_naoqi_dcm.so
```
4. Restart robot system:
```bash
nao restart
```

# All done | Next steps
The robot is now running our uploaded local module `mc_naoqi_dcm` and is ready to be controlled via [`mc_rtc`](https://jrl-umi3218.github.io/mc_rtc/index.html) controller using [`mc_naoqi`](https://github.com/jrl-umi3218/mc_naoqi) interface.

You can refer to the sample Pepper Finite State Machine (FSM) `mc_rtc` controller  project: [`PepperFSMController`](https://github.com/jrl-umi3218/pepper-fsm-controller) for an example (or a starting point) for creating your own `mc_rtc` controller for Pepper.
