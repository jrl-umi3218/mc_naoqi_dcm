fastgetsetdcm
==

Fast communication module with NAO joints. 
This is a local NAO module, that needs to be cross-compiled
for NAO, and put on the robot.

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
qibuild make -c cross-atom fastgetsetdcm
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

You can use it directly by restarting naoqi on the robot.
Make sure NAO is stable before doing so, as it will servo off.

```
nao stop
nao start
```

More info on http://janebotics.blogspot.fr

