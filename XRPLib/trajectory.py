import math
import time
from .controller import Controller

class TrapezoidPositionGenerator(Controller) :
    def __init__(self, start_pos, end_pos, max_vel, accel):
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.max_vel = max_vel
        self.accel = accel
        self.current_pos = start_pos

        # Calculate key timings. Assume it's a trapezoid trajectory at first
        self.x_total = abs(end_pos - start_pos)
        
        # Acceleration phase time and distance
        self.t_accel = max_vel / accel  # Time to reach max velocity
        #print(f"Time to reach max velocity is {self.t_accel}")
        self.x_accel = 0.5 * self.accel * self.t_accel**2  # Distance covered in acceleration phase
        
        # Will we have a trapezoid or triangle trajectory?
        if (self.x_accel * 2) > self.x_total : # We won't reach maximum velocity in half the distance, make a triangle trajectory
            self.t_accel = math.sqrt((0.5 * self.x_total) / (0.5 * self.accel)) #time to accelerate to 1/2 the total distance
            # print(f"TRIANGLE MODE: Time to reach max velocity is {self.t_accel}")
            self.x_accel = 0.5 * self.accel * self.t_accel**2  # Distance covered in acceleration phase
            self.x_const = 0
            self.t_const = 0
            self.max_vel = self.t_accel * self.accel # Our new maximum velocity since we can't reach the requested max velocity
        else : # Constant phase time and distance when in trapezoid mode
            self.x_const = self.x_total - 2 * self.x_accel  # Distance covered at constant velocity
            self.t_const = self.x_const / self.max_vel  # Time at max velocity

        self.t_total = 2 * self.t_accel + self.t_const  # Total duration
        # print(f"expected time = {self.t_total}")
        
        self.running = False
        
    def get_target(self):
        if self.running == False:
            self.running = True
            self.mode = "Off"
            self.t_start = time.ticks_ms() # Remember the time at the start of the trajectory generator
            
        """Compute the next position in the trajectory."""
        self.dt = time.ticks_diff(time.ticks_ms(), self.t_start)  / 1000 # How long in seconds has it been since the start of the trajectory generator
        
        if self.dt < self.t_accel:
            # Acceleration phase
            if self.mode != "Accelerating":
                self.mode = "Accelerating"
                # print(self.mode)
            pos = self.start_pos + 0.5 * self.accel * self.dt**2
        elif self.dt < self.t_accel + self.t_const:
            # Constant velocity phase
            if self.mode != "Constant velocity":
                self.mode = "Constant velocity"
                # print(self.mode)
            pos = self.start_pos + self.x_accel + self.max_vel * (self.dt - self.t_accel)
        elif self.dt < self.t_total:
            # Deceleration phase
            if self.mode != "Delecerating":
                self.mode = "Delecerating"
                # print(self.mode)

            t_dec = self.dt - self.t_accel - self.t_const
            pos = self.start_pos + self.x_accel + self.x_const + (self.max_vel * t_dec - 0.5 * self.accel * t_dec**2)
        else:
            # End of trajectory
            self.mode = "Off"
            t_dec = self.dt - self.t_accel - self.t_const
            pos = self.start_pos + self.x_accel + self.x_const + (self.max_vel * t_dec - 0.5 * self.accel * t_dec**2)
            # print(f"final position was {pos}")
            
            pos = self.end_pos
            # print(f"End after {self.dt} seconds")
            self.running = False
        
        return self.running, pos
if False:
    # This makes a trapezpod trajectory
    trajectory = TrapezoidPositionGenerator(start_pos=0, end_pos=100, max_vel=100, accel=100)
    while True:
        running, target = trajectory.get_target()
        print(f"target = {target:.0f}")
        if running == False:
            break
        time.sleep_ms(20)
        
    # This makes a triangle trajectory
    trajectory = TrapezoidPositionGenerator(start_pos=0, end_pos=100, max_vel=3000, accel=50)
    while True:
        running, target = trajectory.get_target()
        print(f"target = {target:.0f}")
        if running == False:
            break
        time.sleep_ms(20)
