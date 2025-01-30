from carla_include import *
import argparse
import pygame
from pygame.locals import K_ESCAPE, K_q

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

vehicle_list = []
latitudes, longitudes = [], []
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()
original_settings = world.get_settings()

############################################################################################################ Everyone's code
############################################################################################################
############################################### Client Setup ###############################################
############################################################################################################

# Ensure the CARLA server is running and fully loaded
try:
    world = client.get_world()
except RuntimeError as e:
    print(f"Error connecting to CARLA server: {e}")
    sys.exit(1)

# client.load_world('Town05')

############################################################################################################ Roberto's code
############################################################################################################
################################## Ego vehicle and traffic vehicles setup ##################################
############################################################################################################

# Spawn the ego vehicle
ego_vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
ego_vehicle_bp.set_attribute('color', '255,0,0')  # Set color to red
ego_vehicle = world.spawn_actor(ego_vehicle_bp, random.choice(world.get_map().get_spawn_points()))

# Customize the ego vehicle's attributes
        
        
vehicle_list.append(ego_vehicle)
ego_vehicle.set_autopilot(True)  # Enable autopilot for the ego vehicle

############################################################################################################ chris's code
############################################################################################################
############################################### Logger setup ###############################################
############################################################################################################

# Set up logging for each sensor
logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
gnss_logger = logging.getLogger('GNSS')
imu_logger = logging.getLogger('IMU')
collision_logger = logging.getLogger('Collision')
lane_invasion_logger = logging.getLogger('LaneInvasion')

# Remove the default StreamHandler to prevent logging to the console
gnss_logger.propagate = False
imu_logger.propagate = False
collision_logger.propagate = False
lane_invasion_logger.propagate = False

gnss_handler = logging.FileHandler('gnss.log', mode='w')  # Overwrite the file
imu_handler = logging.FileHandler('imu.log', mode='w')
collision_handler = logging.FileHandler('collision.log', mode='w')
lane_invasion_handler = logging.FileHandler('lane_invasion.log', mode='w')

gnss_logger.addHandler(gnss_handler)
imu_logger.addHandler(imu_handler)
collision_logger.addHandler(collision_handler)
lane_invasion_logger.addHandler(lane_invasion_handler)

############################################################################################################
############################################### Camera setup ###############################################
############################################################################################################

# # Transform to place the camera on top of the vehicle
# camera_init_trans = carla.Transform(carla.Location(z=1.5))

# # Create the camera through a blueprint that defines its properties
# camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')

# # Spawn the camera and attach it to the ego vehicle
# camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)
# vehicle_list.append(camera)

# # Set up the Camera listener to save images
# camera.listen(lambda image: image.save_to_disk('out/camera/%06d.png' % image.frame))


# ############################################################################################################
# ############################################ Depth camera setup ############################################
# ############################################################################################################

# # Transform to place the depth camera on top of the vehicle
# depth_cam = None
# depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
# depth_transform = carla.Transform(carla.Location(x=1.0, y=0.0, z=2.5), carla.Rotation(pitch=-15))

# # Spawn the depth camera and attach it to the ego vehicle
# depth_cam = world.spawn_actor(depth_bp, depth_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
# vehicle_list.append(depth_cam)

# # Set up the depth camera listener to save images
# depth_cam.listen(lambda image: image.save_to_disk('out/depth_camera/%06d.png' % image.frame, carla.ColorConverter.LogarithmicDepth))


# ############################################################################################################
# ############################################ LiDAR Sensor setup ############################################
# ############################################################################################################

# # Transform to place the LiDAR on top of the vehicle
lidar_init_trans = carla.Transform(carla.Location(x=1.0, y=0.0, z=2.5))

# Create the LiDAR sensor through a blueprint
lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('range', '100')

# Set LiDAR attributes (optional)
lidar_bp.set_attribute('rotation_frequency', '20')
lidar_bp.set_attribute('channels', '64')
lidar_bp.set_attribute('points_per_second', '250000')

# Spawn the LiDAR and attach it to the ego vehicle
lidar = world.spawn_actor(lidar_bp, lidar_init_trans, attach_to=ego_vehicle)
vehicle_list.append(lidar)

# Start LiDAR with a callback and save the point cloud data to disk
lidar.listen(lambda point_cloud: point_cloud.save_to_disk('out/lidar/lidar_%06d.ply' % point_cloud.frame))


# ############################################################################################################
# ############################################ GNSS Sensor setup #############################################
# ############################################################################################################

# # Transform to place the GNSS sensor on top of the vehicle
gnss_init_trans = carla.Transform(carla.Location(x=1.0, y=0.0, z=2.8))

# Create the GNSS sensor through a blueprint
gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')

# Spawn the GNSS sensor and attach it to the ego vehicle
gnss = world.spawn_actor(gnss_bp, gnss_init_trans, attach_to=ego_vehicle)
vehicle_list.append(gnss)

# Start GNSS with a callback
def gnss_callback(data):
    global longitudes, latitudes
    latitudes.append(data.latitude)
    longitudes.append(data.longitude)
    gnss_logger.info(f"GNSS: {data.latitude}, {data.longitude}, {data.altitude}")

gnss.listen(gnss_callback)


# ############################################################################################################
# ############################################# IMU Sensor setup #############################################
# ############################################################################################################

# # Transform to place the IMU sensor on top of the vehicle
imu_init_trans = carla.Transform(carla.Location(x=1.0, y=0.0, z=2.8))

# Create the IMU sensor through a blueprint
imu_bp = world.get_blueprint_library().find('sensor.other.imu')

# Spawn the IMU sensor and attach it to the ego vehicle
imu = world.spawn_actor(imu_bp, imu_init_trans, attach_to=ego_vehicle)
vehicle_list.append(imu)

# Start IMU with a callback
imu.listen(lambda data: imu_logger.info(f"IMU: {data.accelerometer}, {data.gyroscope}, {data.compass}"))


# ############################################################################################################
# ######################################## Collision Detector setup ##########################################
# ############################################################################################################

# # Create the collision sensor through a blueprint
# collision_bp = world.get_blueprint_library().find('sensor.other.collision')

# # Spawn the collision sensor and attach it to the ego vehicle
# collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=ego_vehicle)
# vehicle_list.append(collision_sensor)

# # Start collision sensor with a callback
# collision_sensor.listen(lambda event: collision_logger.info(f"Collision detected: {event}"))


# ############################################################################################################
# ###################################### Lane Invasion Detector setup ########################################
# ############################################################################################################

# # Create the lane invasion sensor through a blueprint
lane_invasion_bp = world.get_blueprint_library().find('sensor.other.lane_invasion')

# Spawn the lane invasion sensor and attach it to the ego vehicle
lane_invasion_sensor = world.spawn_actor(lane_invasion_bp, carla.Transform(), attach_to=ego_vehicle)
vehicle_list.append(lane_invasion_sensor)

# Start lane invasion sensor with a callback
lane_invasion_sensor.listen(lambda event: lane_invasion_logger.info(f"Lane invasion detected: {event}"))

############################################################################################################ Kyler's code
############################################################################################################
####################################### Live-time Camera Display Setup #####################################
############################################################################################################

class DisplayManager:
    def __init__(self, grid_size, window_size):
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)

        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []

    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self):
        return [int(self.window_size[0]/self.grid_size[1]), int(self.window_size[1]/self.grid_size[0])]

    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def get_sensor_list(self):
        return self.sensor_list

    def render(self):
        for s in self.sensor_list:
            s.render()

        pygame.display.flip()
    
    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    def render_enabled(self):
        return self.display != None

class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()


############################################################################################################
########################################### Sensor Manager Class ###########################################
############################################################################################################

class SensorManager:
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()

        self.time_processing = 0.0
        self.tics_processing = 0

        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera
        
        # elif sensor_type == 'DepthCamera':
        #     depth_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
        #     disp_size = self.display_man.get_display_size()
        #     depth_bp.set_attribute('image_size_x', str(disp_size[0]))
        #     depth_bp.set_attribute('image_size_y', str(disp_size[1]))

        #     for key in sensor_options:
        #         depth_bp.set_attribute(key, sensor_options[key])
            
        #     depth_camera = self.world.spawn_actor(depth_bp, transform, attach_to=attached)

        #     depth_camera.listen(self.save_depth_image)
            

        #     return depth_camera

        elif sensor_type == 'LiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('range', '100')
            lidar_bp.set_attribute('dropoff_general_rate', lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
            lidar_bp.set_attribute('dropoff_intensity_limit', lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
            lidar_bp.set_attribute('dropoff_zero_intensity', lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_lidar_image)

            return lidar
        
        # elif sensor_type == 'SemanticLiDAR':
        #     lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
        #     lidar_bp.set_attribute('range', '100')

        #     for key in sensor_options:
        #         lidar_bp.set_attribute(key, sensor_options[key])

        #     lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

        #     lidar.listen(self.save_semanticlidar_image)

        #     return lidar
        
        # elif sensor_type == "Radar":
        #     radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')

        #     for key in sensor_options:
        #         radar_bp.set_attribute(key, sensor_options[key])

        #     radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
        #     radar.listen(self.save_radar_image)

        #     return radar
    
        
        # elif sensor_type == "SemanticSegmentationCamera":
        #     segmentation_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
        #     disp_size = self.display_man.get_display_size()
        #     segmentation_bp.set_attribute('image_size_x', str(disp_size[0]))
        #     segmentation_bp.set_attribute('image_size_y', str(disp_size[1]))

        #     for key in sensor_options:
        #         segmentation_bp.set_attribute(key, sensor_options[key])

        #     segmentation = self.world.spawn_actor(segmentation_bp, transform, attach_to=attached)
        #     segmentation.listen(self.save_semanticsegmenation_image)

        #     return segmentation
    
        # elif sensor_type == 'InstanceSegmentationCamera':
        #     instance_bp = self.world.get_blueprint_library().find('sensor.camera.instance_segmentation')
        #     disp_size = self.display_man.get_display_size()
        #     instance_bp.set_attribute('image_size_x', str(disp_size[0]))
        #     instance_bp.set_attribute('image_size_y', str(disp_size[1]))

        #     for key in sensor_options:
        #         instance_bp.set_attribute(key, sensor_options[key])
            
        #     instance = self.world.spawn_actor(instance_bp, transform, attach_to=attached)
        #     instance.listen(self.save_instance_segmentation_image)

        #     return instance
        
        else:
            return None

    def get_sensor(self):
        return self.sensor

    def save_rgb_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_lidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    # def save_semanticlidar_image(self, image):
    #     t_start = self.timer.time()

    #     disp_size = self.display_man.get_display_size()
    #     lidar_range = 2.0*float(self.sensor_options['range'])

    #     points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
    #     points = np.reshape(points, (int(points.shape[0] / 6), 6))
    #     lidar_data = np.array(points[:, :2])
    #     lidar_data *= min(disp_size) / lidar_range
    #     lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
    #     lidar_data = np.fabs(lidar_data)
    #     lidar_data = lidar_data.astype(np.int32)
    #     lidar_data = np.reshape(lidar_data, (-1, 2))
    #     lidar_img_size = (disp_size[0], disp_size[1], 3)
    #     lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

    #     lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

    #     if self.display_man.render_enabled():
    #         self.surface = pygame.surfarray.make_surface(lidar_img)

    #     t_end = self.timer.time()
    #     self.time_processing += (t_end-t_start)
    #     self.tics_processing += 1
    
    # def save_depth_image(self, image):
    #     t_start = self.timer.time()

    #     image.convert(carla.ColorConverter.LogarithmicDepth)
    #     array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    #     array = np.reshape(array, (image.height, image.width, 4))
    #     array = array[:, :, :3]
    #     array = array[:, :, ::-1]

    #     if self.display_man.render_enabled():
    #         self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        
    #     t_end = self.timer.time()
    #     self.time_processing += (t_end-t_start)
    #     self.tics_processing += 1


    # def save_radar_image(self, radar_data):
    #     t_start = self.timer.time()
    #     if self.display_man.render_enabled():
    #         # Create a blank surface
    #         disp_size = self.display_man.get_display_size()
    #         self.surface = pygame.Surface(disp_size)

    #         # Process radar data and draw points
    #         for detection in radar_data:
    #             # Convert detection to screen coordinates
    #             x = int(detection.depth * disp_size[0] / 100.0)
    #             y = int(detection.altitude * disp_size[1] / 100.0)
    #             velocity = detection.velocity

    #             # Draw the point on the surface
    #             color = (255, 0, 0) if velocity > 0 else (0, 255, 0)
    #             pygame.draw.circle(self.surface, color, (x, y), 2)

    #         # Blit the surface to the display
    #         offset = self.display_man.get_display_offset(self.display_pos)
    #         self.display_man.display.blit(self.surface, offset)

    #     self.time_processing += self.timer.time() - t_start
    #     self.tics_processing += 1

    # def save_semanticsegmenation_image(self, image):
    #     t_start = self.timer.time()

    #     image.convert(carla.ColorConverter.CityScapesPalette)
    #     array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    #     array = np.reshape(array, (image.height, image.width, 4))
    #     array = array[:, :, :3]
    #     array = array[:, :, ::-1]

    #     if self.display_man.render_enabled():
    #         self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        
    #     t_end = self.timer.time()
    #     self.time_processing += (t_end-t_start)
    #     self.tics_processing += 1
    
    # def save_instance_segmentation_image(self, image):
    #     t_start = self.timer.time()

    #     image.convert(carla.ColorConverter.CityScapesPalette)
    #     array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    #     array = np.reshape(array, (image.height, image.width, 4))
    #     array = array[:, :, :3]
    #     array = array[:, :, ::-1]

    #     if self.display_man.render_enabled():
    #         self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        
    #     t_end = self.timer.time()
    #     self.time_processing += (t_end-t_start)
    #     self.tics_processing += 1

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()


############################################################################################################ Matthew's code
############################################################################################################
################################# Function to run the CARLA simulation #####################################
############################################################################################################

def run_simulation(args, client):

    display_manager = None
    timer = CustomTimer()
    try:

        # Getting the world and
        world = client.get_world()
        original_settings = world.get_settings()

        if args.sync:
            traffic_manager = client.get_trafficmanager(8000)
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)

        # Connect to the client and retrieve the world object
        

        
        
        # Set up the Display Manager
        display_manager = DisplayManager(grid_size=[2, 3], window_size=[1280, 720])

        ############################################################################################################
        ########################################### Display Sensor Data ############################################ 
        ############################################################################################################

        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=-90)), 
                      ego_vehicle, {}, display_pos=[0, 0])
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)), 
                      ego_vehicle, {}, display_pos=[0, 1])
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+90)), 
                      ego_vehicle, {}, display_pos=[0, 2])
        SensorManager(world, display_manager, 'LiDAR', carla.Transform(carla.Location(x=0, z=2.4)), 
                      ego_vehicle, {'channels' : '64', 'range' : '100',  'points_per_second': '250000', 'rotation_frequency': '20'}, display_pos=[1, 0])
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=180)), 
                      ego_vehicle, {}, display_pos=[1, 1])
        # SensorManager(world, display_manager, 'SemanticSegmentationCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)),
        #                 ego_vehicle, {}, display_pos=[1, 2])
        
        # SensorManager(world, display_manager, 'DepthCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)),
        #                 ego_vehicle, {}, display_pos=[2, 0])
        # SensorManager(world, display_manager, 'Radar', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)),
        #                 ego_vehicle, {}, display_pos=[2, 1])
        # SensorManager(world, display_manager, 'SemanticLiDAR', carla.Transform(carla.Location(x=0, z=2.4)), 
        #               ego_vehicle, {'channels' : '64', 'range' : '100', 'points_per_second': '100000', 'rotation_frequency': '20'}, display_pos=[1, 2])
        # SensorManager(world, display_manager, 'InstanceSegmentationCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)),
        #                 ego_vehicle, {}, display_pos=[2, 2])
        
        #Simulation loop
        call_exit = False
        while True:

            # if args.sync:
            #     traffic_manager = client.get_trafficmanager(8000)
            #     settings = world.get_settings()
            #     traffic_manager.set_synchronous_mode(True)
            #     settings.synchronous_mode = True
            #     settings.fixed_delta_seconds = 0.05
            #     world.apply_settings(settings)

            # # Carla Tick
            # if args.sync:
            #     world.tick()
            # else:
            #     world.wait_for_tick()

            # Render received data
            display_manager.render()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE or event.key == K_q:
                        call_exit = True
                        break

            if call_exit:
                break

    finally:
        if display_manager:
            display_manager.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])

        world.apply_settings(original_settings)


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Sensor tutorial')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--async',
        dest='sync',
        action='store_false',
        help='Asynchronous mode execution')
    argparser.set_defaults(sync=True)
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        run_simulation(args, client)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()


############################################################################################################ Alex's code
############################################################################################################
######################## Function to visualize and save LiDAR data using matplotlib ########################
############################################################################################################

def visualize_lidar_data():
    # Get the list of saved LiDAR point cloud files
    lidar_files = glob.glob('out/lidar/*.ply')
    print(f"Found {len(lidar_files)} LiDAR files.")

    # Ensure the output directory exists
    output_dir = 'out/lidar_plots'
    os.makedirs(output_dir, exist_ok=True)

    for i, file in enumerate(lidar_files):
        print(f"Processing file: {file}")
        
        # Read the point cloud file
        plydata = PlyData.read(file)
        
        # Extract the point cloud data
        points = np.vstack([plydata['vertex'].data['x'], 
                            plydata['vertex'].data['y'], 
                            plydata['vertex'].data['z']]).T
        
        # Create a figure with subplots for different views
        fig, axs = plt.subplots(1, 3, figsize=(15, 5))
        
        # Top-down view (XY plane)
        axs[0].scatter(points[:, 0], points[:, 1], s=1)
        axs[0].set_title('Top-down view (XY plane)')
        axs[0].set_xlabel('X')
        axs[0].set_ylabel('Y')
        
        # Side view (XZ plane)
        axs[1].scatter(points[:, 0], points[:, 2], s=1)
        axs[1].set_title('Side view (XZ plane)')
        axs[1].set_xlabel('X')
        axs[1].set_ylabel('Z')
        
        # Front view (YZ plane)
        axs[2].scatter(points[:, 1], points[:, 2], s=1)
        axs[2].set_title('Front view (YZ plane)')
        axs[2].set_xlabel('Y')
        axs[2].set_ylabel('Z')
        
        # Save the plots to a file
        plot_filename = os.path.join(output_dir, f'lidar_plot_{i:06d}.png')
        plt.savefig(plot_filename)
        print(f"Saved plot to {plot_filename}")
        
        # Show the plots
        # plt.show()

        # Close the figure to free up memory
        plt.close(fig)


############################################################################################################
########################## Run the CARLA simulation for 30 seconds or when stopped #########################
############################################################################################################

############################################################################################################
################################## Set up real-time plotting for GNSS data #################################
############################################################################################################

fig, ax = plt.subplots()

def update(frame):
    ax.clear()
    ax.plot(longitudes, latitudes, 'bo-')
    ax.set_title('GNSS Data')
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    plt.savefig(f'out/gnss/gnss_plot_{frame:06d}.png')  # Save each frame as an image

ani = FuncAnimation(fig, update, interval=1000)
plt.show()

visualize_lidar_data()