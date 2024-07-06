import math


class InertialMeasurementUnit:
    def __init__(self, start_orientation = 0):
        self.drone_orientation = start_orientation


class OpticalFlowSensor:
    def __init__(self):
        self.acceleration = 1  # Acceleration rate in meters per second squared
        self.max_speed = 3  # Maximum speed in meters per second
        self.current_speed = 0  # Current speed in meters per second

    def update_speed_acceleration(self):
        self.current_speed += self.acceleration if self.current_speed < self.max_speed else 0


    def update_speed_deceleration(self):
        self.current_speed -= self.acceleration if self.current_speed > 0 else 0

class PID:
    def __init__(self, p, i, d, max_i):
        self.P = p
        self.I = i
        self.D = d
        self.max_i = max_i
        self.integral = 0
        self.last_error = 0
        self.first_run = True

    def constrain(self, value, max_value, min_value):
        if value > max_value:
            return max_value
        elif value < min_value:
            return min_value
        else:
            return value

    def update(self, error, dt):
        if self.first_run:
            self.last_error = error
            self.first_run = False

        self.integral += self.I * error * dt
        diff = (error - self.last_error) / dt if dt != 0 else 0
        const_integral = self.constrain(self.integral, self.max_i, -self.max_i)
        control_out = self.P * error + self.D * diff + const_integral
        self.last_error = error
        return control_out
    
    def update_P_value(self,value):
        self.P += value
        
    def update_I_value(self,value):
        self.I += value

    def update_D_value(self,value):
        self.D += value        

class BatterySensor:
    def __init__(self, battery_percentage=100):
        self.battery_percentage = battery_percentage
        # Update the value 10 times per second, and the battery life is 480 seconds.
        self.total_ticks = 4800  # 480 sec * 10
        self.remaining_ticks = self.total_ticks

    '''
    This function is called 10 times per second. Each call decreases the remaining ticks by 1
    and updates the battery percentage accordingly.
    '''
    def update_battery_percentage(self):
        if self.battery_percentage > 0:
            self.remaining_ticks -= 1
            self.battery_percentage = (self.remaining_ticks / self.total_ticks) * 100  # Update the battery percentage

    def reset_battery(self):
        self.battery_percentage = 100
        self.remaining_ticks = self.total_ticks

    def add_percentage_to_battery(self, percentage):
        ticks_to_add = (self.total_ticks / 100) * percentage
        self.remaining_ticks += ticks_to_add
        if self.remaining_ticks > self.total_ticks:
            self.remaining_ticks = self.total_ticks
        self.battery_percentage = (self.remaining_ticks / self.total_ticks) * 100  # Update the battery percentage


class TwoDimensionalDistanceSensor:
    def __init__(self, sensor_direction, distance=0):
        self.distance = distance  # the distance from an obstacle
        self.direction = sensor_direction  # forward, backward, left, right , forward_right_diagonal , forward_left_diagonal
        self.max_range = 120 #120 px * 2.5 = 3 meters

    # update the current distance from an obstacle using the map and drone's position
    def update_values(self, map_matrix, location_on_map, drone_radius, drone_orientation):
        directions = {
            "forward": 0,
            "backward": 180,
            "leftward": 270,
            "rightward": 90,
            "forward_right_diagonal": 40,
            "forward_left_diagonal": 320
        }
        
        angle = drone_orientation + directions[self.direction]
        dx = math.cos(math.radians(angle))  # defind the x direction relatively to the drone's angle 
        dy = math.sin(math.radians(angle))  # defind the y direction relatively to the drone's angle
        drone_x, drone_y = location_on_map
        
        # sensor_x = drone_x + dx * drone_radius
        # sensor_y = drone_y + dy * drone_radius
        sensor_x = drone_x - drone_radius if dx < 0 else drone_x + drone_radius
        sensor_y = drone_y - drone_radius if dy < 0 else drone_y + drone_radius
        
        
        dist = 1
        while dist <= self.max_range+1:
            nx = int(sensor_x + dx * dist)
            ny = int(sensor_y + dy * dist)
            
            # checking if found an obstacle
            if nx < 0 or nx >= len(map_matrix[0]) or ny < 0 or ny >= len(map_matrix):
                self.distance = dist * 2.5  # Distance in cm
                return
            if map_matrix[ny][nx] == 1:
                self.distance = dist * 2.5  # Distance in cm
                return
        
            dist += 1

        self.distance = self.max_range * 2.5  # If no obstacle is found within range
