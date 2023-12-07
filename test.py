from machine import I2C, Pin
from vl53l5cx import DATA_TARGET_STATUS, DATA_DISTANCE_MM
from vl53l5cx import STATUS_VALID, RESOLUTION_8X8, RANGING_MODE_CONTINUOUS
from vl53l5cx.mp import VL53L5CXMP
from time import sleep
from sensor import make_sensor

tof_power = Pin(22, mode=Pin.OUT)
tof_power.value(1)
# i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=400_000)
tof = make_sensor()
# tof = VL53L5CXMP(i2c, lpn=Pin(13, Pin.OUT, value=1))
if not tof.is_alive():
    raise ValueError("VL53L5CX not detected")

# print(i2c.scan())
tof.init()

tof.resolution = RESOLUTION_8X8

tof.rangingmode = RANGING_MODE_CONTINUOUS

grid = 7

tof.ranging_freq = 8



def main():

    tof.start_ranging({DATA_DISTANCE_MM, DATA_TARGET_STATUS})
    while True:
        cols = [0] * 8
        if tof.check_data_ready():
            results = tof.get_ranging_data()
            distance = results.distance_mm
            status = results.target_status
            for i in range(32):
                temp = distance[i * 2 + 1]
                distance[i * 2 + 1] = distance[i * 2 + 0]
                distance[i * 2 + 0] = temp
                
            distance = distance[::-1]
            for i, d in enumerate(distance):
                print("{:8}".format(d), end=" ")

                if (i & grid) == grid:
                    print("")
            print("")
             
input()
main()