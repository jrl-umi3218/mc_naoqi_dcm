# Deactivate safety reflexes
# First, go to http://pepper.local/advanced/#/settings to enable the deactivation

import qi
import sys

# Connect to Naoqi session
session = qi.Session()
try:
    session.connect("tcp://127.0.0.1:9559")
except RuntimeError:
    print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
           "Please check your script arguments. Run with -h option for help.")
    sys.exit(1)

# Get the service ALMotion.
motion_service  = session.service("ALMotion")

# Deactivate safety reflexes
motion_service.setCollisionProtectionEnabled("RArm", False)
motion_service.setCollisionProtectionEnabled("LArm", False)
motion_service.setDiagnosisEffectEnabled(False)
motion_service.setSmartStiffnessEnabled(False)
motion_service.setExternalCollisionProtectionEnabled("All", False)
motion_service.setFallManagerEnabled(False)
motion_service.setPushRecoveryEnabled(False)
