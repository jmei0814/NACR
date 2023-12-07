from machine import PWM, Pin, I2C, Timer
import time

# weapon_pwm.duty_u16(4800)
WEAPON_NEUTRAL = 3110
weaponC3 = Pin(11, Pin.IN)

duty = 0
dutysum = 0
while True:
    dutysum = 0
    for i in range (3):
        while weaponC3.value() == 0:
            pass
        start = time.ticks_us()
        while weaponC3.value() == 1:
            pass
        end = time.ticks_us()
        duty = end - start
        dutysum += duty
    weapon_duty = round((dutysum*65536*50)/(10**6*3)-WEAPON_NEUTRAL)+WEAPON_NEUTRAL-200
    # weapon_pwm.duty_u16(weapon_duty)
    print(weapon_duty)
