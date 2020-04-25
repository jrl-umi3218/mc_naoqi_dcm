set(_srcs
    main.cpp
    mc_naoqi_dcm.cpp
    RobotModule.cpp
    NAORobotModule.cpp
    PepperRobotModule.cpp
)

qi_create_lib(mc_naoqi_dcm SHARED ${_srcs} SUBFOLDER naoqi)
qi_use_lib(mc_naoqi_dcm ALCOMMON ALMEMORYFASTACCESS)
