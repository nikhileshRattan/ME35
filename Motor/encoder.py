from machine import Pin



class Count(object):
    def __init__(self,A,B):
        self.A = Pin(A, Pin.IN)
        self.B = Pin(B, Pin.IN)
        self.counter = 0

        self.A.irq(self.cb,self.A.IRQ_FALLING|self.A.IRQ_RISING) #interrupt on line A
        self.B.irq(self.cb,self.B.IRQ_FALLING|self.B.IRQ_RISING) #interrupt on line B
        print("Initialized")

    def cb(self,msg):
        other,inc = (self.B,1) if msg == self.A else (self.A,-1) #define other line and increment
        self.counter += -inc if msg.value()!=other.value() else  inc 
        
    def value(self):
        return self.counter
    
class Motor(Count):
    def __init__(self,m1,m2, A, B):
        enc = Count(A,B)
        self.M1 = Pin(m1, Pin.OUT)
        self.M2 = Pin(m2, Pin.OUT)
        self.stop()
        
    
    def stop(self):
        self.M1.off()
        self.M2.off()

    def start(self, direction = 0):
        if direction:
            self.M1.on()
            self.M2.off()
        else:
            self.M1.off()
            self.M2.on()
            
            
Motor1 = Motor(14,27, 33,25)


