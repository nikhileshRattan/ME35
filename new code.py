from machine import Pin, PWM, Timer, I2C
import time
import lis3dh
import servo

# -------------------
# Encoder / Motor
# -------------------
class Count(object):
    def __init__(self,A,B):
        self.A = Pin(A, Pin.IN)
        self.B = Pin(B, Pin.IN)
        self.counter = 0

        self.A.irq(self.cb,self.A.IRQ_FALLING|self.A.IRQ_RISING)
        self.B.irq(self.cb,self.B.IRQ_FALLING|self.B.IRQ_RISING)

    def cb(self,msg):
        other,inc = (self.B,1) if msg == self.A else (self.A,-1)
        self.counter += -inc if msg.value()!=other.value() else  inc 
        
    def value(self):
        return self.counter

class Motor(Count):
    def __init__(self,m1,m2, A, B):
        self.enc = Count(A,B)
        self.M1 = PWM(m1, freq=100, duty_u16=0)
        self.M2 = PWM(m2, freq=100, duty_u16=0)
        self.stop()
                    
    def pos(self):
        return self.enc.value()
            
    def stop(self):
        self.M1.duty_u16(0) 
        self.M2.duty_u16(0) 

    def start(self, direction = 0, speed = 50):
        if direction:
            self.M1.duty_u16(int(speed*65535/100)) 
            self.M2.duty_u16(0)
        else:
            self.M1.duty_u16(0)
            self.M2.duty_u16(int(speed*65535/100)) 

# -------------------
# Servo helper
# -------------------
class Servo:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin), freq=50)
        
    def write_angle(self, angle):
        # convert 0-180 deg to 0.5-2.5 ms pulse width
        # duty_u16 range: 0-65535
        min_us = 500
        max_us = 2500
        us = min_us + (angle / 180) * (max_us - min_us)
        duty = int(us / 20000 * 65535)  # 20ms period for 50Hz
        self.pwm.duty_u16(duty)

# -------------------
# Setup hardware
# -------------------
MotorLower1 = Motor(14,27,32,39)      # DC motor with encoder
ServoLower1 = Servo(15)               # Servo attached to GPIO15 (example)

# H3LIS331DL accelerometer setup
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
accel = lis3dh.H3LIS331DL(sda_pin=21, scl_pin=22)

# Button on D34
button = Pin(34, Pin.IN)  # external pull-down required

# -------------------
# Dataset
# -------------------
dataset = []

# -------------------
# Training loop
# -------------------
print("Press button to record Servo + Accel data...")
while True:
    if button.value() == 1:  # button pressed
        # read motor position
        motor_pos = MotorLower1.pos()
        
        # map motor counts (0-3840) to servo angle (0-180)
        servo_angle = (motor_pos%3840/21.5)
        if servo_angle > 180:  # clamp max
            servo_angle = 180
        elif servo_angle < 0:  # clamp min
            servo_angle = 0
        
        # move servo
        ServoLower1.write_angle(servo_angle)
        
        # read accelerometer X/Z in g
        accl_g = accel.read_accl_g()
        accel_x = accl_g['x']
        accel_z = accl_g['z']
        
        # store as tuple
        data_point = (servo_angle, accel_x, accel_z)
        dataset.append(data_point)
        
        print("Data recorded:", data_point)
        print("Current dataset:", dataset)
        
        # simple debounce: wait until button released
        while button.value() == 1:
            time.sleep(0.05)
    
    time.sleep(0.05)  # loop delay
    
while True:
    servoUpper2.write_angle(0)
    time.sleep(1.5)
    print(MotorLower1.pos())
    print(MotorUpper2.pos())
    servoUpper2.write_angle(round(MotorLower1.pos()%3840/21.5))
    #print(servoUpper2.write_angle(round(MotorLower1.pos()%3840/180))
    time.sleep(1.5)

