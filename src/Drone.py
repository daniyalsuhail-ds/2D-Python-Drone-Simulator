from Utils import OpticalFlowSensor, PID, InertialMeasurementUnit, BatterySensor, TwoDimensionalDistanceSensor
import math, time


class Drone:
    def __init__(self):
        self.battery_sensor = BatterySensor() #battery is initialing with 100%
        self.optical_flow_sensor = OpticalFlowSensor()
        self.forward_distance_sensor = TwoDimensionalDistanceSensor("forward")
        self.forward_right_diagonal_distance_sensor = TwoDimensionalDistanceSensor("forward_right_diagonal")
        self.forward_left_diagonal_distance_sensor = TwoDimensionalDistanceSensor("forward_left_diagonal")
        self.backward_distance_sensor = TwoDimensionalDistanceSensor("backward")
        self.leftward_distance_sensor = TwoDimensionalDistanceSensor("leftward")
        self.rightward_distance_sensor = TwoDimensionalDistanceSensor("rightward")

        self.desired_floor_distance = 100 # 100 px * 2.5 = 2.5 meters

        self.orientation_sensor = InertialMeasurementUnit()

        self.pid_controller = PID(0.07, 0, 0.04, 4.5)
        self.forward_pid_controller = PID(0.9,0, 0.015, 4.5)
        self.narrow_pid_controller = PID(0.03,0, 0.03, 4.5)

        self.initial_desired_wall_distance = 25 # Desired distance from the wall in cm
        self.desired_wall_distance = self.initial_desired_wall_distance # Desired distance from the wall in cm
        self.desired_distance_switching_wall_delta = 3 # an eplsion to diff bettween turnning on the PID to finding a wall in wall switching mode
        # Variables for wall following
        self.is_hugging_right = True  # Start by hugging the right wall
        self.starting_position = None # starting postion of the drone
        self.trail = []
        self.to_start = False
        self.use_pid = True
        self.cooldown = False
        self.cooldown_start_time_wall_switching = 0
        self.drone_idle = False # flag to check if the drone stop because it was about to it a wall

        # returning home parameters:
        self.home_path = []  # empty list to store all the calculated path to home
        self.exploring_path = []  # empty list to store all the calculated path back to exploring
        self.to_explore = False
        self.charging_drone = False # for charging the battery after getting back home

    def update_sensors(self, map_matrix, position, drone_radius, orientation,floor_level, ceiling_level): # added some parameters for height sensors
        self.forward_distance_sensor.update_values(map_matrix, position, drone_radius, orientation)
        self.backward_distance_sensor.update_values(map_matrix, position, drone_radius,orientation)
        self.leftward_distance_sensor.update_values(map_matrix, position, drone_radius,orientation)
        self.rightward_distance_sensor.update_values(map_matrix, position, drone_radius,orientation)
        self.forward_right_diagonal_distance_sensor.update_values(map_matrix, position, drone_radius,orientation)
        self.forward_left_diagonal_distance_sensor.update_values(map_matrix, position, drone_radius,orientation)


        # when the drone is in charging mode, don't use the battery:
        if not self.charging_drone:
            self.battery_sensor.update_battery_percentage()

    # the wall distance should be adjusted relative to the drone's height:
    def update_desired_wall_distance(self,deviation):
        self.desired_wall_distance = int(self.initial_desired_wall_distance +
                                         deviation * self.initial_desired_wall_distance
                                         )


    def move_drone(self,drone_pos , direction):
        directions = {
            "forward": 0,
            "backward": 180,
            "leftward": 270,
            "rightward": 90
        }

        dx = math  .cos(math.radians(self.orientation_sensor.drone_orientation + directions[direction])) * self.optical_flow_sensor.current_speed  # moves in the x axis in a speed relatively to the drone's angle
        dy = math.sin(math.radians(self.orientation_sensor.drone_orientation + directions[direction])) * self.optical_flow_sensor.current_speed  # moves in the y axis in a speed relatively to the drone's angle
        new_pos = [drone_pos[0] + dx, drone_pos[1] + dy]

        return new_pos

    def update_drone_angle(self,angle_delta):
        self.orientation_sensor.drone_orientation = ((self.orientation_sensor.drone_orientation + angle_delta) % 360)


    def switch_wall(self ):
        self.use_pid = False
        # Adjust drone to look 10 degrees away from the current wall
        angle_delta = -10 if self.is_hugging_right else 10
        self.update_drone_angle(angle_delta)

    def wall_following(self, drone_pos, dt):
        delta = self.desired_wall_distance - self.desired_distance_switching_wall_delta
        #if we dont use PID
        if not self.use_pid:
            if self.is_hugging_right: # drone is hugging the right side and is looking for the left wall
                #checking if the drone should move without the PID
                if  self.forward_distance_sensor.distance > delta and \
                        self.leftward_distance_sensor.distance > delta and \
                        self.rightward_distance_sensor.distance > delta and \
                        self.forward_left_diagonal_distance_sensor.distance > delta and \
                        self.forward_right_diagonal_distance_sensor.distance > delta:
                    # Move forward without PID control
                    new_pos = self.move_drone(drone_pos, "forward")
                    return new_pos

                #if found the left wall , hug it
                elif self.forward_distance_sensor.distance <= delta \
                        or self.leftward_distance_sensor.distance <= delta \
                        or self.forward_left_diagonal_distance_sensor.distance <= delta:
                    self.is_hugging_right = not self.is_hugging_right

                # Re-enable PID because we are close to a wall
                self.use_pid = True
            else: # drone is hugging the left side and is looking for the right wall
                if  self.forward_distance_sensor.distance > delta and \
                        self.leftward_distance_sensor.distance > delta and \
                        self.rightward_distance_sensor.distance > delta and \
                        self.forward_left_diagonal_distance_sensor.distance > delta and \
                        self.forward_right_diagonal_distance_sensor.distance > delta:
                    # Move forward without PID control
                    new_pos = self.move_drone(drone_pos, "forward")
                    return new_pos
                #if found the right wall , hug it
                elif self.forward_distance_sensor.distance <= delta \
                        or self.rightward_distance_sensor.distance <= delta \
                        or self.forward_right_diagonal_distance_sensor.distance <= delta:
                    self.is_hugging_right = not self.is_hugging_right

                # Re-enable PID because we are close to a wall
                self.use_pid = True


        # Calculate the error from the desired wall distance
        if not self.is_hugging_right: #self.leftward_distance_sensor.distance < 35:  # Detect the wall on the left side
            error = -0.62 *(self.leftward_distance_sensor.distance - self.desired_wall_distance)
        else:
            error = 0.62* (self.rightward_distance_sensor.distance - self.desired_wall_distance)

        # Calculate the left / right wall hugging correction using the PID controller
        overall_correction = self.pid_controller.update(error, dt)

        # Calculate the correction for case the drone's front is getting too close to a wall
        front_danger_distance = 50 * self.optical_flow_sensor.current_speed
        if(self.forward_distance_sensor.distance >= front_danger_distance):
            forward_distance_error = 0
        else:
            forward_distance_error = front_danger_distance - self.forward_distance_sensor.distance

        forward_correction = self.forward_pid_controller.update(forward_distance_error, dt)

        # calculate the correction for the case where the drone is in a narrow path and needs
        # to adjust to both walls:
        narrow_path_error = 0

        if self.is_hugging_right and self.leftward_distance_sensor.distance < self.rightward_distance_sensor.distance :
            narrow_path_error = self.rightward_distance_sensor.distance - self.leftward_distance_sensor.distance

        elif not self.is_hugging_right and self.leftward_distance_sensor.distance > self.rightward_distance_sensor.distance :
            narrow_path_error = self.rightward_distance_sensor.distance - self.leftward_distance_sensor.distance

        narrow_correction = self.narrow_pid_controller.update(narrow_path_error , dt)

        turnning_direction = -2 if self.is_hugging_right else 2 
        #Sum up the corrections for the wall hugging and the drone's front error correction
        overall_correction +=  (forward_correction * turnning_direction) + narrow_correction

        # Limit the correction to prevent aggressive maneuvers
        max_correction = 10  # a maximum correction angle
        overall_correction = max(-max_correction, min(overall_correction, max_correction))

        # Adjust the drone's angle based on the correction
        self.update_drone_angle(overall_correction)

        # Move the drone forward
        new_pos = self.move_drone(drone_pos, "forward")

        return new_pos


    def reload_battery(self):
        self.battery_sensor.add_percentage_to_battery(0.2)
        if self.battery_sensor.battery_percentage == 100:
            self.charging_drone = False
            self.to_explore = True


    def update_position_by_algorithm(self, drone_pos, dt,map_matrix, drone_radius):
        new_pos = drone_pos
        # Check battery level and initiate return home mode if necessary:
        if self.battery_sensor.battery_percentage <= 50 and not self.charging_drone:
            self.to_start = True

        #checking if in returning home mode is activated:
        if self.to_start:
            # Get the next position on the return trail
            new_pos = self.get_next_position_from_returning_home_algo(map_matrix,drone_pos, drone_radius)

        #checking if in charging mode is activated:
        elif self.charging_drone:
            new_pos = drone_pos
            self.reload_battery()

        #checking if in returing to explore mode is activated:
        elif self.to_explore:
            new_pos = self.get_back_to_exploring_from_last_point(drone_pos)

        else:
            # checking if the drone can fly
            if not self.drone_idle and not self.drone_about_to_touch_wall():
                if self.optical_flow_sensor.current_speed == 0:
                    self.optical_flow_sensor.update_speed_acceleration()
                # Then perform wall-following
                new_pos = self.wall_following(drone_pos,dt)

                #update cooldown mode  - False = there is no cooldown , cooldown is over
                if time.time() - self.cooldown_start_time_wall_switching >= 2:
                    self.cooldown = False

                #if cooldown is over you can switch wall    
                if not self.cooldown:
                    if self.is_in_trail_environment(new_pos):
                        self.switch_wall()
                        self.cooldown = True
                        self.cooldown_start_time_wall_switching = time.time()

            else:
                # checking if the drone is idle and if so it need to correct it's angle to avoid touching the wall
                if self.drone_idle:
                    new_pos = self.wall_following(drone_pos,dt) # in this state the drone is not moving , only adjusting is angle
                    #if the drone is no longer about to touch the wall, switch the flag to not idle
                    if not self.drone_about_to_touch_wall():
                        self.drone_idle = False
                # if the drone is not idle - flying , check if its about to touch the wall , if so the drone stops and will adjust
                elif self.drone_about_to_touch_wall():
                    # Stop moving forward
                    new_pos = drone_pos
                    self.drone_idle = True
                    self.optical_flow_sensor.update_speed_deceleration()

        return new_pos

    # returning home imporvements:

    # to make the drone be able to find all points of trail which are at line of sight:
    def is_line_of_sight_clear(self, start, end, map_matrix, drone_radius):

        drone_radius *= 2 # it makes sure it doesnt get points that will make the drone clash on a wall
        def bresenham_line(x0, y0, x1, y1):
            """Bresenham's Line Algorithm"""
            points = []
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            err = dx - dy

            while True:
                points.append((x0, y0))
                if (x0, y0) == (x1, y1):
                    break
                e2 = err * 2
                if e2 > -dy:
                    err -= dy
                    x0 += sx
                if e2 < dx:
                    err += dx
                    y0 += sy
            return points

        def is_clear_path(path):
            for (x, y) in path:
                if x < 0 or x >= len(map_matrix[0]) or y < 0 or y >= len(map_matrix):
                    return False
                if map_matrix[y][x] == 1:  # There's a wall in the way
                    return False
            return True

        # Main line - make a path from curr position to desired point and check if the path is clear from walls:
        main_path = bresenham_line(int(start[0]), int(start[1]), int(end[0]), int(end[1]))
        if not is_clear_path(main_path):
            return False

        # Calculate the offsets perpendicular to the line, in order to check if the drone with it's thick
        # radius could safely go through the path:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.sqrt(dx ** 2 + dy ** 2)
        dx /= length +1
        dy /= length +1

        # Perpendicular offsets
        offset_x = -dy * drone_radius
        offset_y = dx * drone_radius

        # Right offset
        right_offset_start = (start[0] + offset_x, start[1] + offset_y)
        right_offset_end = (end[0] + offset_x, end[1] + offset_y)
        right_path = bresenham_line(int(right_offset_start[0]), int(right_offset_start[1]), int(right_offset_end[0]),
                                    int(right_offset_end[1]))
        if not is_clear_path(right_path):
            return False

        # Left offset
        left_offset_start = (start[0] - offset_x, start[1] - offset_y)
        left_offset_end = (end[0] - offset_x, end[1] - offset_y)
        left_path = bresenham_line(int(left_offset_start[0]), int(left_offset_start[1]), int(left_offset_end[0]),
                                   int(left_offset_end[1]))
        if not is_clear_path(left_path):
            return False

        return True


    # this is the main function to return the drone to starting point in an improved way:
    def get_next_position_from_returning_home_algo(self, map_matrix, drone_pos, drone_radius):

        # if gotten close to the starting point then the returning home process is done:
        allowed_field_error = 10
        if (abs(drone_pos[0] - self.trail[0][0]) <= allowed_field_error
                and abs(drone_pos[1] - self.trail[0][1]) <= allowed_field_error):
            self.to_start = False
            self.charging_drone = True
            self.home_path.clear()
            self.exploring_path.insert(0,drone_pos) # stack behavior - LIFO insert, to record the trail back
            return drone_pos

        if self.home_path:
            next_position = self.home_path.pop(0)
            self.exploring_path.insert(0,next_position) # record the path back from home
            return next_position

        current_position = drone_pos
        trail_points_within_radius = []

        # finding all tail points in range and with clear line of sight to current position:
        for idx, point in enumerate(self.trail):
            distance = math.sqrt((current_position[0] - point[0]) ** 2 + (current_position[1] - point[1]) ** 2)
            if distance <= self.leftward_distance_sensor.max_range * 2:
                if self.is_line_of_sight_clear(current_position, point, map_matrix, drone_radius):
                    trail_points_within_radius.append((idx, point))

        if not trail_points_within_radius: # this shouldn't happen because the drone always has a trail, this is for extreme buggy cases
            print("no points on trail have been found")
            self.to_start = False
            return current_position


        # Find the point with the closest index to the starting point (which is trail[0])
        closest_point = min(trail_points_within_radius, key=lambda p: abs(p[0]))

        # make a path from current pos to the closest point to start:
        path = self.get_path_from_current_to_desired(drone_pos, closest_point[1])

        self.home_path.extend(path)  # Append all points from the path to home_path

        next_position = self.home_path.pop(0)
        self.update_angle_by_position(drone_pos,path[-1]) # each path is a straight line, make the drone look towards the end of that line
        return next_position

    def get_path_from_current_to_desired(self, current_pos, desired_pos):
        """
        Generates a list of points forming a path from current_pos to desired_pos using Bresenham's line algorithm.

        Args:
            current_pos (tuple): The starting position of the drone (x, y).
            desired_pos (tuple): The desired position to reach (x, y).

        Returns:
            List[tuple]: List of points from current_pos to desired_pos.
        """
        path = []

        x1, y1 = current_pos
        x1 = int(x1)
        y1 = int(y1)
        x2, y2 = desired_pos
        x2 = int(x2)
        y2 = int(y2)

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while (x1, y1) != (x2, y2):

            path.append((x1, y1))

            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy

        path.append((x2, y2))  # Include the final position
        return path

    def drone_about_to_touch_wall(self):
        return ((self.forward_distance_sensor.distance < 22.5)
                or (self.rightward_distance_sensor.distance < 7.5)
                or (self.leftward_distance_sensor.distance < 7.5)
                or (self.forward_right_diagonal_distance_sensor.distance < 7.5)
                or (self.forward_left_diagonal_distance_sensor.distance < 7.5))

    def set_position(self, position):
        self.starting_position = position
        self.trail = [position]  # Initialize the trail with the starting position

    def update_position(self, position):
        if not self.to_start:
            self.trail.append(position)


    # getting the drone back to the place it returned home from:
    def get_back_to_exploring_from_last_point(self,drone_pos):

        new_pos = self.exploring_path.pop(0) # get the next point for the drone

        # for making the drone look at a good distance:
        pos_to_aim_the_drone_at = None
        if len(self.exploring_path) >= 3:
            pos_to_aim_the_drone_at = self.exploring_path[2]


        # if we reached the end of the path then we flag it:
        if not self.exploring_path:
            self.exploring_path.clear()
            self.to_explore = False


        if pos_to_aim_the_drone_at:
            self.update_angle_by_position(drone_pos, pos_to_aim_the_drone_at)
        return new_pos

    def update_angle_by_position(self, current_position, next_position):
        # Calculate the angle needed to face the next position
        dx = next_position[0] - current_position[0]
        dy = next_position[1] - current_position[1]
        angle_to_next_position = math.degrees(math.atan2(dy, dx))

        # Update the drone's angle to face the next position
        self.orientation_sensor.drone_orientation = angle_to_next_position

    # in case the drone is touching the trail of itself or is very close to it, for wall switching:
    def is_in_trail_environment(self, point, radius = 0.5):
        for trail_point in self.trail:
            distance = math.sqrt((point[0] - trail_point[0]) ** 2 + (point[1] - trail_point[1]) ** 2)
            if distance <= radius:
                return True
        return False
