# turns out we don't need this, will keep it just in case

Import("env")
import serial
import time

def before_upload(source, target, env):
    print("Resetting board...")    
    ser = serial.Serial(env.subst("$UPLOAD_PORT"), 1200)
    ser.dtr(False)
    time.sleep(0.5)
    ser.dtr(True)
    ser.close()
    print("Complete?")    

env.AddPreAction("upload", before_upload)