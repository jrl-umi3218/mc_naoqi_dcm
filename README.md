fastgetsetdcm
==

Fast communication module with NAO/PEPPER sensors and actuators. 
This is a local robot module, that needs to be cross-compiled for the desired platform (NAO or Pepper), and uploaded on the robot.

How to build
==

As we are cross-compiling, you need to first create a toolchain for the CTC (cross-compilation toolchain).

```
qitoolchain create cross-atom /path/to/ctc/toolchain.xml
```

Add the toolchain to qibuild

```
qibuild add-config cross-atom --toolchain cross-atom
```

Create a worktree, and clone the project
```
mkdir /path/to/worktree
cd /path/to/worktree
git clone <fastgetsetdcm.git>
```

Now, build the project 

```
qibuild init
qisrc add fastgetsetdcm
qibuild configure -c cross-atom fastgetsetdcm
cd build-cross-atom
# Choose either "pepper" or "nao"
cmake .. -DROBOT_NAME=<pepper|nao>
make
```


And finally, let's deploy it on the robot:

```
qibuild deploy -c cross-atom --url nao@192.168.2.3:naoqi fastgetsetdcm
```

A final step is required to autoload the module on the robot.
Edit the file ~/naoqi/preferences/autoload.ini on the robot, and add

```
[user]
/home/nao/naoqi/lib/naoqi/libfastgetsetdcm.so
```

You will also need to diable the ALMotion module to prevent it from conflicting with the controls from this module.

You can now use it directly by restarting naoqi on the robot.
Make sure the robot stable before doing so, as it will servo off.

```
nao stop
nao start
```

More info on http://janebotics.blogspot.fr

