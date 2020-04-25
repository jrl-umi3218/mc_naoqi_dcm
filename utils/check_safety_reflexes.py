# Check which safety reflexes are enabled

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

# Display which safety reflexes are active
print "Right arm anti selfcollision enabled: " + str(motion_service.getCollisionProtectionEnabled("RArm"))
print "Left arm anti selfcollision enabled: " + str(motion_service.getCollisionProtectionEnabled("LArm"))
print "Diagnosis effect enabled: " + str(motion_service.getDiagnosisEffectEnabled())
print "Smart stiffness enabled: " + str(motion_service.getSmartStiffnessEnabled())
print "Anti external collision enabled: " + str(motion_service.getExternalCollisionProtectionEnabled("All"))
print "Fall manager enabled: " + str(motion_service.getFallManagerEnabled())
print "Push recovery enabled: " + str(motion_service.getPushRecoveryEnabled())

# All true when robot is just turned on
# All true after restart
# All true except diagnosis and smart stiffness after disactivating all and then calling 'Wake up'
