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
        # Convert 0-180 deg to 0.5-2.5 ms pulse width
        min_us = 500
        max_us = 2500
        us = min_us + (angle / 180) * (max_us - min_us)
        duty = int(us / 20000 * 65535)  # 20ms period for 50Hz
        self.pwm.duty_u16(duty)

# -------------------
# Setup hardware
# -------------------
MotorLower1 = Motor(14,27,32,39)
MotorUpper2 = Motor(13,12,26,25)

ServoLower1 = servo.Servo(19)
ServoUpper2 = servo.Servo(18)  # example GPIO for second servo

# H3LIS331DL accelerometer
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
    ServoUpper2.write_angle(0)
    ServoLower1.write_angle(0)
    # --- Read motor positions ---
    motor_lower_pos = MotorLower1.pos()
    motor_upper_pos = MotorUpper2.pos()
    
    # --- Map to servo angles (cyclic 3840 counts → 0-180°) ---
    servo_lower_angle = round(round((motor_lower_pos % 3840)) / 21.5//30 * 30)
    servo_upper_angle = round(round((motor_upper_pos % 3840)) /  21.5//30 * 30)
    
    # --- Move servos ---
    ServoLower1.write_angle(servo_lower_angle)
    ServoUpper2.write_angle(servo_upper_angle)
    
    MotorLower1 = Motor(14,27, 32,39)
    MotorUpper2 = Motor(13,12, 26,25)
 
    #Motor1.pos() # to read the encoder value for Motor 1
    
    
    # --- Record dataset if button pressed ---
    if button.value() == 1:
        # Read accelerometer X/Z in g
        accl_g = accel.read_accl_g()
        accel_x = accl_g['x']
        accel_z = accl_g['z']
        
        # Store as tuple: (servo_lower_angle, servo_upper_angle, accel_x, accel_z)
        data_point = (servo_lower_angle, servo_upper_angle, accel_x, accel_z)
        dataset.append(data_point)
        
        print("Data recorded:", data_point)
        print("Current dataset:", dataset)
        
        # Debounce: wait until button released
        while button.value() == 1:
            time.sleep(0.05)
    
    time.sleep(0.05)  # loop delay
    
def nearestNeighbor(data, sensor_value): #data is an array of [(sensor_values, motor_position)]
	if len(data) == 0: #check if data exists
		return 0
	diff = 10000 #start with a big number 
	for i in data: # for every element in data check if the sensors value is closer to the data 
		if abs(i[0] - sensor_value) <= diff: #compare different sensors values in data with current sensor value
			diff = abs(i[0] - sensor_value) # if smaller than the big number set the new difference as 
			motor_position = i[1] # use the motor position as the desired position 
		return (motor_position) #return the motor position
		
nearestNeighbor(data,sensor_value)
