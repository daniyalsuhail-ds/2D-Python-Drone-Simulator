import pygame
from PIL import Image
import random
import math
import os
from Drone import Drone
import time


# DroneSimulation class
class DroneSimulation:
    def __init__(self):
        pygame.init()
        self.map_width = 1366 #1000
        self.map_height = 768 #600
        self.screen_width = self.map_width + 290  # Increase width by some pixels
        self.screen_height = self.map_height  # Increase height by some pixels
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        self.last_key_state = pygame.key.get_pressed() #for button pressing
        pygame.display.set_caption("Drone Simulation")
        self.show_legend = True  # Flag to control legend visibility
        
        # 3d expantion:
        self.ceiling_level = int(500 / 2.5) # 500 in cm, so dividing bt 2.5 for pixels
        self.floor_level = 0

        # Load map
        script_dir = os.path.dirname(os.path.abspath(__file__))
        parent_directory = os.path.dirname(script_dir)
        input_filepath = os.path.join(parent_directory, 'maps')
        self.load_map_paths(input_filepath)  # Update with the correct path to your maps folder
        self.load_map(self.map_paths[self.current_map_index])
        self.sensor_texts = {
            "Autonomous_Mode": "Autonomous Mode: True",
            "Drone's battery": "0 %",
            "Drone's speed": "0",
            "Covered Area": "Green Percentage: 0.00%",
        }
        # Define the legend text
        self.legend_texts = [
            "Controls Legend:",
            "Arrow Keys: Move Drone",
            "A/D: Rotate Drone",
            "E: Toggle Autonomous Mode",
            "1/2: Increase/Decrease Height",
            "INFO: "
        ]
        
        # Initialize drone
        self.drone_radius = int(10 / 2.5)  # Convert cm to pixels
        self.initial_drone_radius = self.drone_radius  # for saving the initial radius
        self.drone = Drone()
        self.drone_pos = None

        self.respawn_drone()

        self.clock = pygame.time.Clock()
        self.game_over = False

        # List to store drone positions for leaving a trail
        self.drone_positions = []

        # Array to remember painted pixels
        self.detected_pixels = set()
        self.detected_green_pixels = set()

        # Count initial white pixels
        self.total_white_pixels = self.count_white_pixels()

        self.drone.set_position(self.drone_pos)
        self.drone.set_position(self.drone_pos)
        
        
        self.button_color = (80, 80, 80)
        self.hover_color = (130, 130, 130)
        self.buttons = {
            "Return Home": pygame.Rect(1380, 250, 260, 35),
            "Increase Speed": pygame.Rect(1380, 295, 260, 35),
            "Dercease Speed": pygame.Rect(1380, 340, 260, 35),
            "Reset Simulation": pygame.Rect(1380, 385, 260, 35),
            "Change Map": pygame.Rect(1380, 430, 260, 35),
            "Autonomous Mode": pygame.Rect(1380, 475, 260, 35)
            }
        self.mouse_pos = pygame.mouse.get_pos()
        
    def draw_button(self, rect, color, text):
        text_color = (255, 255, 255)  # White
        pygame.draw.rect(self.screen, color, rect)
        font = pygame.font.SysFont(None, 24, bold=True)
        text_surf = font.render(text, True, text_color)
        text_rect = text_surf.get_rect(center=rect.center)
        self.screen.blit(text_surf, text_rect)

    def load_map(self, filename):
        map_img = Image.open(filename)
        map_img = map_img.resize((self.map_width, self.map_height))
        # Convert black pixels to gray
        gray_img = map_img.convert('RGBA')
        pixels = gray_img.load()
        
        for y in range(self.map_height):
            for x in range(self.map_width):
                if pixels[x, y][:3] <= (50, 50, 50):  # If the pixel is black
                    pixels[x, y] = (128, 128, 128, pixels[x, y][3])  # Change to gray

        new_img = Image.new('RGB', (self.screen_width, self.screen_height), (255, 255, 255))
        new_img.paste(gray_img, (0, 0))
        
        self.map_img = pygame.image.fromstring(new_img.tobytes(), new_img.size, new_img.mode)
        self.map_matrix = self.convert_to_matrix(map_img)

    def convert_to_matrix(self, img):
        bw_img = img.convert("L")  # Convert to grayscale
        threshold = 128  # Threshold value for black/white
        bw_matrix = []
        for y in range(self.map_height):
            row = []
            for x in range(self.map_width):
                pixel = bw_img.getpixel((x, y))
                if pixel < threshold:
                    row.append(1)  # Black pixel
                else:
                    row.append(0)  # White pixel
            bw_matrix.append(row)
        return bw_matrix
    
    def count_white_pixels(self):
        count = 0
        for row in self.map_matrix:
            count += row.count(0)
        return count

    def respawn_drone(self):
        
        while True:
            x = random.randint(self.drone_radius, self.map_width - self.drone_radius - 1)
            y = random.randint(self.drone_radius, self.map_height - self.drone_radius - 1)
            if not self.check_collision(x, y):
                self.drone_pos = [x, y]
                self.helipad_pos = [x, y]
                break

    # checking collision with floor/ceiling/walls/obstacles:
    def check_collision(self, x, y, radius=None):

        if radius is None:
            radius = self.drone_radius

        # check walls collision:
        for i in range(int(x - radius), int(x + radius)):
            for j in range(int(y - radius), int(y + radius)):
                if 0 <= i < self.map_width and 0 <= j < self.map_height:
                    if self.map_matrix[j][i] == 1:
                        return True
        return False

    # checks if the drone is inside the map and also is not collided, if it did reset the game
    def check_move_legality(self , new_pos):
        if (self.drone_radius <= new_pos[0] < self.map_width - self.drone_radius and
                self.drone_radius <= new_pos[1] < self.map_height - self.drone_radius):
            if not self.check_collision(new_pos[0], new_pos[1]):
                self.drone_pos = new_pos
                self.drone.update_position(new_pos)  # Track the trail
                self.drone_positions.append(self.drone_pos[:])  # Add position to the trail
            else:
                self.reset_simulation()
        else:
            self.reset_simulation()


    # move with user input keys
    def move_drone_by_direction(self, direction = "forward"):  
        new_pos = self.drone.move_drone(self.drone_pos , direction)
        self.check_move_legality(new_pos)

    # for controling the drone manually:
    def update_drone_angle(self, angle_delta):
        self.drone.update_drone_angle(angle_delta)

    def update_sensors(self):
        # updating drone's sensors:
        self.drone.update_sensors(self.map_matrix, self.drone_pos, self.drone_radius,
                                  self.drone.orientation_sensor.drone_orientation,
                                  self.floor_level,self.ceiling_level) # for updating up/down distance sensors
       
    def paint_detected_points(self):
        def detected_points(sensor_distance, angle_offset):
            angle_rad = math.radians((self.drone.orientation_sensor.drone_orientation + angle_offset) % 360)
            points = []
            for dist in range(1, int(min(sensor_distance, 300) / 2.5) + 1):
                x = self.drone_pos[0] + dist * math.cos(angle_rad)
                y = self.drone_pos[1] + dist * math.sin(angle_rad)
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    if self.map_matrix[int(y)][int(x)] == 0:  # Check if the point is in the white area
                        points.append((int(x), int(y)))
            return points

        # Get detected points for left and right sensors
        left_points = detected_points(self.drone.leftward_distance_sensor.distance, -90)
        right_points = detected_points(self.drone.rightward_distance_sensor.distance, 90)

        # Calculate the new points to be added
        new_detected_points = set(left_points + right_points)
        points_to_paint = new_detected_points - self.detected_pixels

        # Function to get all points within a radius of 2 around a point
        def get_points_in_radius(x, y, radius=2):
            points = set()
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if dx * dx + dy * dy <= radius * radius:
                        new_x, new_y = x + dx, y + dy
                        if 0 <= new_x < self.map_width and 0 <= new_y < self.map_height:
                            if self.map_matrix[new_y][new_x] == 0:  # Check if the point is in the white area
                                points.add((new_x, new_y))
            return points

        # Update the main set of detected pixels with the new points and their surrounding points
        expanded_points_to_paint = set()
        for x, y in points_to_paint:
            expanded_points_to_paint.update(get_points_in_radius(x, y))

        self.detected_pixels.update(expanded_points_to_paint)

        # Create a surface for detected points if not exists
        if not hasattr(self, 'detected_surface'):
            self.detected_surface = pygame.Surface((self.map_width, self.map_height))
            self.detected_surface.set_colorkey((0, 0, 0))  # Set transparent color

        # Paint only the new points on the detected surface
        for i, (x, y) in enumerate(expanded_points_to_paint):
            if i%2 == 0:
                 self.detected_surface.set_at((x, y), (170, 250, 187))

        # Store detected points for green collection
        self.detected_green_pixels.update(expanded_points_to_paint)

        # Blit the detected surface onto the main screen
        self.screen.blit(self.detected_surface, (0, 0))

    def reset_simulation(self):
        #reseting flags for the autonomous flight algorithm:
        self.drone.to_start = False
        self.drone.to_explore = False
        self.drone.charging_drone = False

        self.drone.exploring_path.clear() # clearing previously saved return to explore paths
        self.drone.home_path.clear() # clearing previously saved return home paths

        self.detected_pixels.clear()  # Clear detected points
        self.respawn_drone()  # Respawn the drone
        self.drone_positions.clear()  # Clear trail
        # empty the surface, ensuring that no previously detected points are displayed on the screen.
        if hasattr(self, 'detected_surface'):
            self.detected_surface.fill((0, 0, 0))  # Fill the detected surface with black color
        self.drone.optical_flow_sensor.current_speed = 0
        self.drone.battery_sensor.reset_battery()
        self.detected_green_pixels.clear()  # Clear green detected points
        self.sensor_texts["Covered Area"] = "Green Percentage: 0.00%"
        #making the drone start flying
        self.drone.optical_flow_sensor.update_speed_acceleration()
        #clearing the drone trail array for wall switching
        self.drone.trail.clear()

    def calculate_green_percentage(self):
        green_pixels_count = len(self.detected_green_pixels)
        percentage = (green_pixels_count / self.total_white_pixels) * 100
        return 100.0 if percentage > 100 else percentage

    def draw_legend_menu(self):
        legend_surface = pygame.Surface((260, 200))
        font = pygame.font.SysFont(None, 24, bold=True)
        for i, text in enumerate(self.legend_texts):
            text_surface = font.render(text, True, (255, 255, 255))
            legend_surface.blit(text_surface, (10, 10 + i * 30))
        self.screen.blit(legend_surface, (1380, 10))

    def load_map_paths(self, folder_path):
        self.map_paths = [os.path.join(folder_path, f) for f in os.listdir(folder_path) if f.endswith(('png', 'jpg', 'jpeg'))]
        self.current_map_index = 0

    def load_next_map(self):
        self.current_map_index = (self.current_map_index + 1) % len(self.map_paths)
        self.load_map(self.map_paths[self.current_map_index])
        self.reset_simulation()

    def background_task(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.game_over = True
                
            if event.type == pygame.MOUSEBUTTONDOWN:
                for k, v in self.buttons.items():
                    if self.buttons[k].collidepoint(self.mouse_pos) and k=="Return Home":
                        self.legend_texts[-1] = "INFO: Returning Home"
                        self.drone.to_start = True
                    if self.buttons[k].collidepoint(self.mouse_pos) and k=="Change Map":
                        self.load_next_map()
                        self.legend_texts[-1] = "INFO: Map Changed"
                    if self.buttons[k].collidepoint(self.mouse_pos) and k=="Reset Simulation":
                        self.legend_texts[-1] = "INFO: Reset Simulation"
                        self.reset_simulation()
                    if self.buttons[k].collidepoint(self.mouse_pos) and k=="Increase Speed":
                        self.legend_texts[-1] = "INFO: Speed Increased"
                        self.drone.optical_flow_sensor.update_speed_acceleration()
                    if self.buttons[k].collidepoint(self.mouse_pos) and k=="Dercease Speed":
                        self.legend_texts[-1] = "INFO: Speed Decreased"
                        self.drone.optical_flow_sensor.update_speed_deceleration()
                    if self.buttons[k].collidepoint(self.mouse_pos) and k=="Autonomous Mode":
                        self.is_autonomous = not self.is_autonomous
                        self.legend_texts[-1] = "INFO: Controll Setting changed"
                        
                self.draw_legend_menu()
                
            
    def run_simulation(self):
        frequency = 10  # Hz
        interval = 1000 // frequency  # Convert frequency to milliseconds
        sensors_update_timer = pygame.time.get_ticks()  # Initialize timer for sensors update
        last_time = time.time()
        self.is_autonomous = True # a flag for enabling/disabling autonomous flight mode
        
        #making the drone start flying
        self.drone.optical_flow_sensor.update_speed_acceleration()
        
        while not self.game_over:
            current_time_test = time.time()
            dt = current_time_test - last_time
            last_time = current_time_test
            self.mouse_pos = pygame.mouse.get_pos()
            self.background_task()
            keys = pygame.key.get_pressed()

            if keys[pygame.K_LEFT]:
                self.move_drone_by_direction("backward")
            if keys[pygame.K_RIGHT]:
                self.move_drone_by_direction("forward")
            if keys[pygame.K_UP]:
                self.move_drone_by_direction("leftward")
            if keys[pygame.K_DOWN]:
                self.move_drone_by_direction("rightward")
            if keys[pygame.K_a]:
                self.update_drone_angle(-10)
            if keys[pygame.K_d]:
                self.update_drone_angle(10)
            # Update the last key state
            self.last_key_state = keys

            # Check if it's time to update the sensors , we want to update 10 times per second
            current_time = pygame.time.get_ticks()
            if current_time - sensors_update_timer >= interval:
                # Update sensors
                self.update_sensors()
                sensors_update_timer = current_time  # Reset the timer


            if self.is_autonomous:

                # Update drone position by algorithm
                self.drone_pos = self.drone.update_position_by_algorithm(
                    self.drone_pos,
                    dt,
                    self.map_matrix,
                    self.drone_radius)
            else:
                self.move_drone_by_direction()
            #checking if the drone crashing into the wall or not  
            self.check_move_legality(self.drone_pos)

            # Blit the map image onto the screen
            self.screen.blit(self.map_img, (0, 0))

            # Paint detected points (green markers)
            self.paint_detected_points()

            green_percentage=self.calculate_green_percentage()

            # this calculation get the factor of change the circle itself had, and we will use it to change the drone's arrow as well:
            precentage_of_drone_height_deviation = (self.drone_radius / self.initial_drone_radius)

            # Blit the drone trail onto the screen
            for pos in self.drone_positions:
                pygame.draw.circle(self.screen, (0, 0, 255), pos, 2)

            # Draw arrow on the drone indicating its direction
            angle_rad = math.radians(self.drone.orientation_sensor.drone_orientation)
            end_x = self.drone_pos[0] + 15 * precentage_of_drone_height_deviation * math.cos(angle_rad)
            end_y = self.drone_pos[1] + 15 * precentage_of_drone_height_deviation * math.sin(angle_rad)
            pygame.draw.line(self.screen, (0, 0, 0), self.drone_pos, (end_x, end_y), int(2* precentage_of_drone_height_deviation) )

            
            # Blit the helipad image onto the screen at the drone's respawn location
            drone_img = pygame.image.load('drone.png')  # replace with the actual file name and path
            drone_img = pygame.transform.scale(drone_img, (20,20))  # adjust the size of the helipad image as needed
            drone_x, drone_y = self.drone_pos
            self.screen.blit(drone_img, (drone_x - drone_img.get_width() // 2, drone_y - drone_img.get_height() // 2))
            
            # Blit the drone onto the screen
            # pygame.draw.circle(self.screen, (255, 0, 0), self.drone_pos, self.drone_radius)

            # Blit the helipad image onto the screen at the drone's respawn location
            helipad_img = pygame.image.load('helipad.png')  # replace with the actual file name and path
            helipad_img = pygame.transform.scale(helipad_img, (30, 39))  # adjust the size of the helipad image as needed
            helipad_x, helipad_y = self.helipad_pos
            self.screen.blit(helipad_img, (helipad_x - helipad_img.get_width() // 2, helipad_y - helipad_img.get_height() // 2))
            
            # Update sensor texts
            self.sensor_texts["Drone's battery"] = f"Drone's battery: {self.drone.battery_sensor.battery_percentage:.1f} %"
            self.sensor_texts["Drone's speed"] = f"Drone's speed: {self.drone.optical_flow_sensor.current_speed:.1f}"
            self.sensor_texts["Covered Area"] = f"Green_Percentage: {green_percentage:.2f} %"
            self.sensor_texts["Autonomous_Mode"] = f"Autonomous_Mode: {self.is_autonomous}"
            
            # Display sensor texts
            font_size = 24
            font = pygame.font.SysFont(None, font_size)
            for i, (key, text) in enumerate(self.sensor_texts.items()):
                font.set_underline(False)
                font.set_bold(False)
                text_surface = font.render(text, True, (0, 0, 0))
                self.screen.blit(text_surface, (1380, self.map_height - 50 - (i * 30)))

            # Draw legend menu if the flag is set
            if self.show_legend:
                self.draw_legend_menu()
            
            for k, v in self.buttons.items():
                if v.collidepoint(self.mouse_pos):
                    self.draw_button(self.buttons[k], self.hover_color, k)
                else:
                    self.draw_button(self.buttons[k], self.button_color, k)
            pygame.display.update()
            self.clock.tick(60)

        pygame.quit()

def main():
    simulation = DroneSimulation()
    simulation.run_simulation()

if __name__ == "__main__":
    main()
