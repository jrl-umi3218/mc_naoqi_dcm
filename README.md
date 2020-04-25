mc_naoqi_dcm
==

Fast communication module with NAO/PEPPER sensors and actuators and `mc_rtc` control framework.
This is a local robot module, that needs to be cross-compiled for the desired platform (NAO or Pepper), and uploaded on the robot.

How to build
==

First, you'll need to install the NAOqi SDK and building tools. To do so, follow the [getting started](http://doc.aldebaran.com/qibuild/beginner/getting_started.html) instructions on how to install qibuild and the NAOqi SDK.

As we are cross-compiling, you need to first create a toolchain for the CTC (cross-compilation toolchain).

```
qitoolchain create ctc-naoqi-toolchain /path/to/ctc/toolchain.xml
```

Create `qibuild` worktree

```
qibuild init
```

Add the toolchain to qibuild worktree

```
qibuild add-config tc-naoqi--config -t ctc-naoqi-toolchain --default
```

Clone this project into `qibuild worktree`

```
git clone git@gite.lirmm.fr:softbankrobotics/mc_naoqi_dcm.git
```

Now, build the project

```
# Choose either "pepper" or "nao"
qibuild configure --release -DROBOT_NAME=<pepper|nao>
qibuild make
```

Once the local module is built, transfer it to the robot

```
scp build-ctc-naoqi-config/sdk/lib/naoqi/libmc_naoqi_dcm.so nao@robot_ip:/home/nao/naoqi/locallib/
```

A final step is required to autoload the module on the robot.
Edit the file ~/naoqi/preferences/autoload.ini on the robot, and add

```
[user]
/home/nao/naoqi/locallib/libmc_naoqi_dcm.so
```

You will also need to disable the ALMotion module to prevent it from conflicting with the controls from this module. On robot, edit the file `/etc/naoqi/autoload.ini`, and comment out the motion module


```
...
# motion
...
```

You can now use it directly by restarting naoqi on the robot.
Make sure the robot stable before doing so, as it will servo off.

```
nao stop
nao start
```

The robot is now ready to be controlled via `mc_rtc` controller.
