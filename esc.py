#imports
from machine import PWM, Pin, I2C, Timer
import time
import math
from imu import MPU6050
from vl53l5cx import DATA_TARGET_STATUS, DATA_DISTANCE_MM
from vl53l5cx import RESOLUTION_8X8, RANGING_MODE_CONTINUOUS
from vl53l5cx.mp import VL53L5CXMP
from sensor import make_sensor


#receiver duty constants
SAMPLES = 5 #number of samples taken per read
FAST_SAMPLES = 3 #faster samples, used for manual input
TOLERANCE = 20 #used for errant inputs, since the clock is not 100% accurate
LOW = 800 #initial input, will be used as a kill switch (will trigger deep sleep for the pico, halting all PPMs)
MED = 1495 #used for manual control (runs passthrough)
HIGH = 2190 #used for autonomous control (will use the TOF sensor)


#drive motor variables (i don't know why they're reversed)
NEUTRAL = 4915 #1.5us
FORWARD = 5300 #1.413us
BACKWARD = 4530 #1.587us
DRIVE_TOLERANCE = 0.7

WEAPON_NEUTRAL = 3120

#pins, consult picowiring
throttleC1 = Pin(12, Pin.IN)
weaponC3 = Pin(11, Pin.IN)
pivotC4 = Pin(13, Pin.IN)
gearC5 = Pin(10, Pin.IN)

tof_power = Pin(22, mode=Pin.OUT)
gyro_power = Pin(8, mode=Pin.OUT)

throttle = Pin(27, mode=Pin.OUT)
pivot = Pin(26, mode=Pin.OUT)
weapon_pin = Pin(16, mode=Pin.OUT)


#intialize power pins
tof_power.value(1)
gyro_power.value(1)

#initialize PPM
throttle_pwm = PWM(throttle)
pivot_pwm = PWM(pivot)
weapon_pwm = PWM(weapon_pin)
throttle_pwm.freq(50)
pivot_pwm.freq(50)
weapon_pwm.freq(50)



mpu_scl, mpu_sda = (6, 7)

i2c1 = I2C(1, sda=Pin(6), scl=Pin(7), freq=400000)
imu = MPU6050(i2c1)
time.sleep(0.25)
tof = make_sensor()
tof.reset()

if not tof.is_alive():
    raise ValueError("VL53L5CX not detected")

tof.init()

time.sleep(0.25) #quickly wait a bit for everything to initialize

tof.init()
tof.resolution = RESOLUTION_8X8
tof.rangingmode = RANGING_MODE_CONTINUOUS
tof.ranging_freq = 5
tof.start_ranging({DATA_DISTANCE_MM, DATA_TARGET_STATUS})

#initialize PPM to the motors, set them to neutral
weapon_pwm.duty_u16(4100)
throttle_pwm.duty_u16(NEUTRAL)
pivot_pwm.duty_u16(NEUTRAL)     

doubleCheck = 2
init = False
firstCheck = 1

print("First Loop")
##--------------------------------------------------------------------##
#this is the first loop, waits for a low input and then a medium input
while True: #first waits for a low input, then will have a double check to prevent any errors
    dutysum = 0
    for i in range (SAMPLES):
        while gearC5.value() == 0:
          pass
        start = time.ticks_us()
        while gearC5.value() == 1:
            pass
        end = time.ticks_us()
        duty = end - start
        dutysum += duty
    trueduty = dutysum/SAMPLES
    if(trueduty < (LOW + TOLERANCE) and trueduty > (LOW - TOLERANCE)):
        firstCheck = 0
        doubleCheck = 2
    elif(trueduty < (MED + TOLERANCE) and trueduty > (MED - TOLERANCE)):
        doubleCheck = doubleCheck - 1
    if(doubleCheck == 0 and firstCheck == 0):
        break
    
##--------------------------------------------------------------------##
def manualControl():
    reverseDrive = 1  
    ax=round(imu.accel.x,2)*-1
    if(ax < -0.7):
        reverseDrive = -1
    dutysum = 0
    for i in range (FAST_SAMPLES):
        while throttleC1.value() == 0:
            pass
        start = time.ticks_us()
        while throttleC1.value() == 1:
            pass
        end = time.ticks_us()
        duty = end - start
        dutysum += duty
    throttleduty = round(((dutysum*65536*50)/(10**6*FAST_SAMPLES)-NEUTRAL)*DRIVE_TOLERANCE)
    throttle_pwm.duty_u16(throttleduty * reverseDrive + NEUTRAL)
    dutysum = 0
    duty = 0
#     print(throttleduty)
    for i in range (FAST_SAMPLES):
        while pivotC4.value() == 0:
            pass
        start = time.ticks_us()
        while pivotC4.value() == 1:
            pass
        end = time.ticks_us()
        duty = end - start
        dutysum += duty
    pivotduty = round(((dutysum*65536*50)/(10**6*FAST_SAMPLES)-NEUTRAL)*DRIVE_TOLERANCE)
    pivot_pwm.duty_u16(pivotduty * reverseDrive + NEUTRAL)
#     print(dutysum/3)
    duty = 0
    dutysum = 0
    for i in range (FAST_SAMPLES):
        while weaponC3.value() == 0:
            pass
        start = time.ticks_us()
        while weaponC3.value() == 1:
            pass
        end = time.ticks_us()
        duty = end - start
        dutysum += duty
    weapon_duty = round((dutysum*65536*50)/(10**6*3)-WEAPON_NEUTRAL)+WEAPON_NEUTRAL-200
    weapon_pwm.duty_u16(weapon_duty)
#     print(weapon_duty)

##--------------------------------------------------------------------##
def autonomous():
    reverseDrive = 1  
    ax=round(imu.accel.x,2)*-1
    if(ax < -0.7):
        reverseDrive = -1
    cols = [0] * 8
    if tof.check_data_ready():
        results = tof.get_ranging_data()
        distance = results.distance_mm
        status = results.target_status
        for i in range(32):
            temp = distance[i * 2 + 1]
            distance[i * 2 + 1] = distance[i * 2 + 0]
            distance[i * 2 + 0] = temp
            if distance[i] - distance[i+1] > 500:
                distance[i] = distance[i+1]
            elif distance[i+1] - distance[i] > 500:
                distance[i+1] = distance[i]
        distance = distance[::-1]
        for i in range(8):
            cols[i] = sum(distance[i:40+i:8])/8
        if(sum(distance[47:63])/16 < 35): #too close to wall
            throttle_pwm.duty_u16((BACKWARD - NEUTRAL) * reverseDrive + NEUTRAL) #reverse
            time.sleep(0.5)
            throttle_pwm.duty_u16(NEUTRAL) #stop
            pivot_pwm.duty_u16((BACKWARD - NEUTRAL) * reverseDrive + NEUTRAL) #turn right
            time.sleep(0.5)
            pivot_pwm.duty_u16(NEUTRAL) #stop
            print("too close")
        elif(sum(distance[0:15])/16 < 180 and abs(cols[1]-cols[2]) < 10 and abs(cols[3]-cols[4]) < 10 and abs(cols[5]-cols[6]) < 10):
            if((cols[0] + 6) < cols[3] and (cols[4] + 6) < cols[7]): #increasing values
                pivot_pwm.duty_u16((BACKWARD - NEUTRAL) * reverseDrive + NEUTRAL) #turn right
                time.sleep(0.5)
                pivot_pwm.duty_u16(NEUTRAL) #stop
                print("increasing")
            elif(cols[0] > (cols[3] + 6) and cols[4] > (cols[7] + 6)): #decreasing values
                pivot_pwm.duty_u16((FORWARD - NEUTRAL) * reverseDrive + NEUTRAL) #turn left
                time.sleep(0.5)
                pivot_pwm.duty_u16(NEUTRAL) #stop
                print("decreasing")
            else: #back up
                throttle_pwm.duty_u16((BACKWARD - NEUTRAL) * reverseDrive + NEUTRAL) #reverse
                time.sleep(0.5)
                throttle_pwm.duty_u16(NEUTRAL) #stop
                print("back up")
        else:
            if(cols[4]-cols[3] > 30 or cols[3] - cols[2] > 30 or cols[2] - cols[1] > 30 or cols[1] - cols[0] > 30): #close left!
                weapon_pwm.duty_u16(4200)
                pivot_pwm.duty_u16((FORWARD - NEUTRAL) * reverseDrive + NEUTRAL) #turn left
                time.sleep(0.1)
                pivot_pwm.duty_u16(NEUTRAL) #stop
                throttle_pwm.duty_u16((FORWARD - NEUTRAL) * reverseDrive + NEUTRAL) #forward
                time.sleep(0.5)
                throttle_pwm.duty_u16(NEUTRAL) #stop
                weapon_pwm.duty_u16(WEAPON_NEUTRAL)
                print("close left")
            elif(cols[5]-cols[6] > 30 or cols[6] - cols[7] > 30): #close right!
                weapon_pwm.duty_u16(4200)
                pivot_pwm.duty_u16((BACKWARD - NEUTRAL) * reverseDrive + NEUTRAL) #turn right
                time.sleep(0.3)
                pivot_pwm.duty_u16(NEUTRAL) #stop
                throttle_pwm.duty_u16((FORWARD - NEUTRAL) * reverseDrive + NEUTRAL) #forward
                time.sleep(0.5)
                throttle_pwm.duty_u16(NEUTRAL) #stop
                weapon_pwm.duty_u16(WEAPON_NEUTRAL)
                print("close right!")
            elif(abs(cols[5]-cols[6]) > 30 and abs(cols[2]-cols[1]) < 10):
                weapon_pwm.duty_u16(4200)
                throttle_pwm.duty_u16((FORWARD - NEUTRAL) * reverseDrive + NEUTRAL) #forward
                time.sleep(0.6)
                throttle_pwm.duty_u16(NEUTRAL) #stop
                weapon_pwm.duty_u16(WEAPON_NEUTRAL)
                print("drive straight!")
            elif(cols[2]-cols[1] < -30 and abs(cols[6]-cols[5]) < 10):
                weapon_pwm.duty_u16(4200)
                pivot_pwm.duty_u16((BACKWARD - NEUTRAL) * reverseDrive + NEUTRAL) #turn right
                time.sleep(0.1)
                pivot_pwm.duty_u16(NEUTRAL) #stop
                throttle_pwm.duty_u16((FORWARD - NEUTRAL) * reverseDrive + NEUTRAL) #forward
                time.sleep(0.5)
                throttle_pwm.duty_u16(NEUTRAL) #stop
                weapon_pwm.duty_u16(WEAPON_NEUTRAL)
                print("slight right!")
            elif((cols[0] + 20) < cols[3] and (cols[4] + 20) < cols[7]):
                pivot_pwm.duty_u16((BACKWARD - NEUTRAL) * reverseDrive + NEUTRAL) #turn right
                time.sleep(0.5)
                pivot_pwm.duty_u16(NEUTRAL) #stop
                print("increasing")
            elif(cols[0] > (cols[3] + 20) and cols[4] > (cols[7] + 20)):
                pivot_pwm.duty_u16((FORWARD - NEUTRAL) * reverseDrive + NEUTRAL) #turn left
                time.sleep(0.5)
                pivot_pwm.duty_u16(NEUTRAL) #stop
                print("decreasing")
            else:
                pivot_pwm.duty_u16((BACKWARD - NEUTRAL) * reverseDrive + NEUTRAL) #turn right
                time.sleep(0.1)
                pivot_pwm.duty_u16(NEUTRAL) #stop
                print("spin slowly")

##--------------------------------------------------------------------##
print("Main Loop")
while True:
    dutysum = 0
    for i in range (SAMPLES):
        while gearC5.value() == 0:
          pass
        start = time.ticks_us()
        while gearC5.value() == 1:
            pass
        end = time.ticks_us()
        duty = end - start
        dutysum += duty
    trueduty = dutysum/SAMPLES
    if(trueduty < (LOW + TOLERANCE) and trueduty > (LOW - TOLERANCE)):
        machine.deepsleep(2000)
    elif(trueduty < (MED + TOLERANCE) and trueduty > (MED - TOLERANCE)):
        manualControl()
    elif(trueduty < (HIGH + TOLERANCE) and trueduty > (HIGH - TOLERANCE)):
        autonomous()
    time.sleep(0.1) #give it some time
    