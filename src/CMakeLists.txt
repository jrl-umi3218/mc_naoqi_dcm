set(_srcs
    main.cpp
    fastgetsetdcm.cpp
    RobotModule.cpp
    NAORobotModule.cpp
    PepperRobotModule.cpp
)

qi_create_lib(fastgetsetdcm SHARED ${_srcs} SUBFOLDER naoqi)
qi_use_lib(fastgetsetdcm ALCOMMON ALMEMORYFASTACCESS)
