import math
import time
from .controller import Controller

class TrapezoidPositionGenerator(Controller) :
    def __init__(self, start_pos, end_pos, max_vel, accel, debuglevel):
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.max_vel = max_vel
        self.accel = accel
        self.current_pos = start_pos
        self.debuglevel = debuglevel

        if self.debuglevel > 1:
            print(f"start {start_pos} end {end_pos} vel {max_vel} accel {accel}")
            
        # Calculate key timings. Assume it's a trapezoid trajectory at first
        self.x_total = abs(end_pos - start_pos)
        
        # Acceleration phase time and distance
        self.t_accel = max_vel / accel  # Time to reach max velocity
        if self.debuglevel > 1:
            print(f"Time to reach max velocity is {self.t_accel}")
        self.x_accel = 0.5 * self.accel * self.t_accel**2  # Distance covered in acceleration phase
        
        # Will we have a trapezoid or triangle trajectory?
        if (self.x_accel * 2) > self.x_total : # We won't reach maximum velocity in half the distance, make a triangle trajectory
            self.t_accel = math.sqrt((0.5 * self.x_total) / (0.5 * self.accel)) #time to accelerate to 1/2 the total distance
            if debuglevel > 1:
                print(f"TRIANGLE MODE: Time to reach max velocity is {self.t_accel}")
            self.x_accel = 0.5 * self.accel * self.t_accel**2  # Distance covered in acceleration phase
            self.x_const = 0
            self.t_const = 0
            self.max_vel = self.t_accel * self.accel # Our new maximum velocity since we can't reach the requested max velocity
        else : # Constant phase time and distance when in trapezoid mode
            self.x_const = self.x_total - 2 * self.x_accel  # Distance covered at constant velocity
            self.t_const = self.x_const / self.max_vel  # Time at max velocity

        self.t_total = 2 * self.t_accel + self.t_const  # Total duration
        if debuglevel > 1:
            print(f"expected time = {self.t_total}")
        
        self.running = False
        
    def get_target(self):
        if self.running == False:
            self.running = True
            self.mode = "Off"
            self.t_start = time.ticks_ms() # Remember the time at the start of the trajectory generator
            
        """Compute the next position in the trajectory."""
        self.dt = time.ticks_diff(time.ticks_ms(), self.t_start)  / 1000 # How long in seconds has it been since the start of the trajectory generator
        if self.debuglevel > 1:
            print(f"end_pos {self.end_pos}")
        if self.dt < self.t_accel:
            # Acceleration phase
            if self.mode != "Accelerating":
                self.mode = "Accelerating"
                if self.debuglevel > 1:
                    print(self.mode)
            if self.end_pos > 0:
                pos = self.start_pos + 0.5 * self.accel * self.dt**2
            else:
                pos = self.start_pos - 0.5 * self.accel * self.dt**2
        elif self.dt < self.t_accel + self.t_const:
            # Constant velocity phase
            if self.mode != "Constant velocity":
                self.mode = "Constant velocity"
                if self.debuglevel > 1:
                    print(self.mode)
            if self.end_pos > 0:
                pos = self.start_pos + self.x_accel + self.max_vel * (self.dt - self.t_accel)
            else:
                pos = self.start_pos - self.x_accel - self.max_vel * (self.dt - self.t_accel)
        elif self.dt < self.t_total:
            # Deceleration phase
            if self.mode != "Delecerating":
                self.mode = "Delecerating"
                if self.debuglevel > 1:
                    print(self.mode)

            t_dec = self.dt - self.t_accel - self.t_const
            if self.end_pos > 0:
                pos = self.start_pos + self.x_accel + self.x_const + (self.max_vel * t_dec - 0.5 * self.accel * t_dec**2)
            else:
                pos = self.start_pos - self.x_accel - self.x_const - (self.max_vel * t_dec - 0.5 * self.accel * t_dec**2)
        else:
            # End of trajectory
            self.mode = "Off"
            if self.debuglevel > 1:
                    print(self.mode)
            t_dec = self.dt - self.t_accel - self.t_const
            if self.end_pos > 0:
                pos = self.start_pos + self.x_accel + self.x_const + (self.max_vel * t_dec - 0.5 * self.accel * t_dec**2)
            else:
                pos = self.start_pos - self.x_accel + self.x_const + (self.max_vel * t_dec - 0.5 * self.accel * t_dec**2)
            # print(f"final position was {pos}")
            
            pos = self.end_pos
            # print(f"End after {self.dt} seconds")
            self.running = False
        
        return self.running, pos
