from carla_include import *
import argparse
import pygame
import math
from pygame.locals import K_ESCAPE, K_q
import cv2
import numpy as np
import queue
latitudes, longitudes = [], []

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# setting up the display manager to organize the sensors in a grid
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

# Manage the sensors and their data
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

    # Initialize the sensor and attach it to the vehicle
    def init_sensor(self, sensor_type, transform, attached, sensor_options):

        # Initializing the rgb camera
        if sensor_type == 'RGBCamera':
            
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera
        
        # Initializing the LiDAR sensor
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
        
        # Initializing the Semantic LiDAR sensor
        elif sensor_type == 'SemanticLiDAR':

            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            lidar_bp.set_attribute('range', '100')

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_semanticlidar_image)

            return lidar
        
        # Initializing the Radar sensor
        elif sensor_type == "Radar":

            rad_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            rad_bp.set_attribute('horizontal_fov', str(35))
            rad_bp.set_attribute('vertical_fov', str(20))
            rad_bp.set_attribute('range', str(20))

            rad_ego = self.world.spawn_actor(rad_bp, transform, attach_to=attached)

            def rad_callback(radar_data):
                velocity_range = 7.5  # m/s
                current_rot = radar_data.transform.rotation

                for detect in radar_data:
                    azi = math.degrees(detect.azimuth)
                    alt = math.degrees(detect.altitude)
                    fw_vec = carla.Vector3D(x=detect.depth - 0.25)
                    carla.Transform(
                        carla.Location(),
                        carla.Rotation(
                            pitch=current_rot.pitch + alt,
                            yaw=current_rot.yaw + azi,
                            roll=current_rot.roll)).transform(fw_vec)

                    def clamp(min_v, max_v, value):
                        return max(min_v, min(value, max_v))

                    norm_velocity = detect.velocity / velocity_range
                    r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
                    g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
                    b = int(abs(clamp(-1.0, 0.0, -1.0 - norm_velocity)) * 255.0)

                    self.world.debug.draw_point(
                        radar_data.transform.location + fw_vec,
                        size=0.075,
                        life_time=0.06,
                        persistent_lines=False,
                        color=carla.Color(r, g, b))

            rad_ego.listen(lambda radar_data: rad_callback(radar_data))

            return rad_ego
        
        # Initializing the Depth Camera sensor
        elif sensor_type == 'DepthCamera':

            depth_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
            disp_size = self.display_man.get_display_size()
            depth_bp.set_attribute('image_size_x', str(disp_size[0]))
            depth_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                depth_bp.set_attribute(key, sensor_options[key])
            
            depth_camera = self.world.spawn_actor(depth_bp, transform, attach_to=attached)

            depth_camera.listen(self.save_depth_image)

            return depth_camera
        
        # Initializing the Semantic Segmentation Camera sensor
        elif sensor_type == "SemanticSegmentationCamera":

            segmentation_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
            disp_size = self.display_man.get_display_size()
            segmentation_bp.set_attribute('image_size_x', str(disp_size[0]))
            segmentation_bp.set_attribute('image_size_y', str(disp_size[1]))


            for key in sensor_options:
                segmentation_bp.set_attribute(key, sensor_options[key])

            segmentation = self.world.spawn_actor(segmentation_bp, transform, attach_to=attached)
            segmentation.listen(self.save_semanticsegmenation_image)

            return segmentation
        
        # Initializing the IMU sensor
        elif sensor_type == "IMU":

            imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
            imu = self.world.spawn_actor(imu_bp, transform, attach_to=attached)
            imu.listen(self.save_imu_data)

            return imu
        
        # Initializing the GNSS sensor
        elif sensor_type == "GNSS":

            gnss_bp = self.world.get_blueprint_library().find('sensor.other.gnss')
            gnss = self.world.spawn_actor(gnss_bp, transform, attach_to=attached)
            gnss.listen(self.save_gnss_data)

            return gnss
        # Initializing the Collision sensor
        elif sensor_type == "Collision":

            collision_bp = self.world.get_blueprint_library().find('sensor.other.collision')
            collision = self.world.spawn_actor(collision_bp, transform, attach_to=attached)
            collision.listen(self.save_collision_data)

            return collision
        
        # Initializing the Lane Invasion sensor
        elif sensor_type == "LaneInvasion":

            lane_bp = self.world.get_blueprint_library().find('sensor.other.lane_invasion')
            lane = self.world.spawn_actor(lane_bp, transform, attach_to=attached)
            lane.listen(self.save_lane_invasion_data)

            return lane
        
        else:
            return None

    def get_sensor(self):
        return self.sensor

    
    def save_rgb_image(self, image):

        t_start = self.timer.time()

        #image.save_to_disk('out/rgb_camera/%06d.png' % image.frame)

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

        # image.save_to_disk('out/lidar/lidar_%06d.ply' % image.frame)

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
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

    def save_semanticsegmenation_image(self, image):

        t_start = self.timer.time()

        # image.save_to_disk('out/semantic_segmentation/%06d.png' % image.frame, carla.ColorConverter.CityScapesPalette)

        image.convert(carla.ColorConverter.CityScapesPalette)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    def save_radar_image(self, radar_data):

        t_start = self.timer.time()
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_depth_image(self, image):

        t_start = self.timer.time()

        #image.save_to_disk('out/depth_camera/%06d.png' % image.frame, carla.ColorConverter.LogarithmicDepth)

        image.convert(carla.ColorConverter.LogarithmicDepth)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        
        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()

    
# Project 3D points to 2D
def get_image_point(loc, K, w2c):

    point = np.array([loc.x, loc.y, loc.z, 1])
    point_camera = np.dot(w2c, point)

    # Convert from UE4's coordinate system to a standard one
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

    # Project 3D -> 2D using the camera matrix
    point_img = np.dot(K, point_camera)
    point_img[0] /= point_img[2]
    point_img[1] /= point_img[2]

    return point_img[0:2]

# Check if a point is within the image canvas
def point_in_canvas(pos, img_h, img_w):
    """Return true if point is in canvas."""
    return 0 <= pos[0] < img_w and 0 <= pos[1] < img_h

# Camera projection matrix
def build_projection_matrix(w, h, fov, is_behind_camera=False):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)

    if is_behind_camera:
        K[0, 0] = K[1, 1] = -focal
    else:
        K[0, 0] = K[1, 1] = focal

    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K

# Main function to run the simulation
def run_simulation(args, client):
    
    # Function to draw bounding boxes for pedestrians, vehicles, lights, and signs
    def draw_bounding_boxes(world, ego_vehicle):

            actors = world.get_actors()

            # Filter actors by type
            vehicles = list(actors.filter('vehicle.*'))
            pedestrians = list(actors.filter('walker.pedestrian.*'))
            traffic_signs = list(actors.filter('static.prop.traffic_sign'))

            # Draw bounding boxes
            for actor in vehicles + pedestrians + traffic_signs:
                if actor.id == ego_vehicle.id:  # Skip the ego vehicle
                    continue

                bounding_box = actor.bounding_box
                bounding_box.location = actor.get_transform().transform(bounding_box.location)
                world.debug.draw_box(
                    bounding_box,
                    actor.get_transform().rotation,
                    thickness=0.01,
                    color=carla.Color(5, 0, 0),  # Dark Red color for bounding boxes
                    life_time=0.1
                )

    display_manager = None
    vehicle = None
    vehicle_list = []
    all_id = []

    try:

        # Getting the world and original settings
        world = client.get_world()
        original_settings = world.get_settings()

        # set weather
        # weather = carla.WeatherParameters.ClearNoon
        # weather = carla.WeatherParameters.CloudyNoon
        # weather = carla.WeatherParameters.WetNoon
        # weather = carla.WeatherParameters.WetCloudyNoon
        # weather = carla.WeatherParameters.MidRainyNoon
        # weather = carla.WeatherParameters.HardRainNoon
        # weather = carla.WeatherParameters.SoftRainNoon
        # weather = carla.WeatherParameters.ClearSunset
        # weather = carla.WeatherParameters.CloudySunset
        # weather = carla.WeatherParameters.WetSunset
        # weather = carla.WeatherParameters.WetCloudySunset
        # weather = carla.WeatherParameters.MidRainSunset
        # weather = carla.WeatherParameters.HardRainSunset
        # weather = carla.WeatherParameters.SoftRainSunset

        weather = carla.WeatherParameters(
            cloudiness=0.0,
            precipitation=0.0,
            wind_intensity=0.0,
        )
        # cloudiness
        # precipitation
        # precipitation_deposits
        # wind_intensity
        # sun_azimuth_angle
        # sun_altitude_angle
        # fog_density
        # fog_distance
        # fog_falloff
        # wetness
        # scattering_intensity
        # mie_scattering_scale
        # rayleigh_scattering_scale

        world.set_weather(weather)
        

        # Concatenate file location and filename
        # recorder_path = os.path.join(args.file_location, args.recorder_filename)
        # print("Recording on file: %s" % recorder_path)
        # client.start_recorder(recorder_path)

        if args.sync:
            traffic_manager = client.get_trafficmanager(8000)
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)


        # spawning the ego_vehicle
        bp = world.get_blueprint_library().filter('charger_2020')[0]
        vehicle = world.spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))
        vehicle_list.append(vehicle)
        vehicle.set_autopilot(True, traffic_manager.get_port())

        # Set up the rgb camera for bounding box rendering
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(args.width))
        camera_bp.set_attribute('image_size_y', str(args.height))
        camera_bp.set_attribute('fov', '90')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

        # Create a queue to store and retrieve the sensor data
        image_queue = queue.Queue()
        camera.listen(image_queue.put)

        # Retrieve bounding boxes for traffic lights and traffic signs
        bounding_box_set = world.get_level_bbs(carla.CityObjectLabel.TrafficLight)
        bounding_box_set.extend(world.get_level_bbs(carla.CityObjectLabel.TrafficSigns))

        # Define edges for bounding box rendering
        edges = [[0, 1], [1, 3], [3, 2], [2, 0], [0, 4], [4, 5], [5, 1], [5, 7], [7, 6], [6, 4], [6, 2], [7, 3]]

        # Enable the TrafficManager for the ego_vehicle
        traffic_manager.ignore_lights_percentage(vehicle, 0)
        traffic_manager.ignore_walkers_percentage(vehicle, 0)  # Ensure the vehicle respects pedestrians

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

        # Display Manager organize all the sensors an its display in a window
        display_manager = DisplayManager(grid_size=[3, 3], window_size=[args.width, args.height])
        
        # Spawn sensors and attach them to the vehicle
        SensorManager(world, display_manager, 'LiDAR', carla.Transform(carla.Location(x=0, z=2.4)), 
                      vehicle, {'channels' : '64', 'range' : '100',  'points_per_second': '250000', 'rotation_frequency': '20'}, display_pos=[0, 0])
        SensorManager(world, display_manager, 'DepthCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=00)), 
                      vehicle, {}, display_pos=[0, 1])
        SensorManager(world, display_manager, 'SemanticSegmentationCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=00)), 
                      vehicle, {}, display_pos=[0, 2])
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=-90)), 
                      vehicle, {}, display_pos=[1, 0])
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)), 
                      vehicle, {}, display_pos=[1, 1])
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+90)), 
                      vehicle, {}, display_pos=[1, 2])
        SensorManager(world, display_manager, 'Radar', carla.Transform(carla.Location(x=0, z=2.4) , carla.Rotation(yaw=180)), 
                      vehicle, {}, display_pos=[])
        SensorManager(world, display_manager, 'Radar', carla.Transform(carla.Location(x=0, z=2.4) , carla.Rotation(yaw=+00)), 
                      vehicle, {}, display_pos=[])
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=180)), 
                      vehicle, {}, display_pos=[2, 1])
        # SensorManager(world, display_manager, 'SemanticLiDAR', carla.Transform(carla.Location(x=0, z=2.4)), 
        #               vehicle, {'channels' : '64', 'range' : '100', 'points_per_second': '100000', 'rotation_frequency': '20'}, display_pos=[1, 2])
        

        # Spawn NPC pedestrians and vehicles
        blueprintsWalkers = world.get_blueprint_library().filter("walker.pedestrian.*")
        blueprintsWalkersController = world.get_blueprint_library().find('controller.ai.walker')
        bp_lib = world.get_blueprint_library()
        spawn_points = world.get_map().get_spawn_points()
        
        for i in range(50): # Number of vehicles
            vehicle_bp = random.choice(bp_lib.filter('vehicle'))
            npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
            if npc:
                npc.set_autopilot(True)
                vehicle_list.append(npc)



        spawn_points = []
        for i in range(50):  # Number of pedestrians
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if loc:
                spawn_point.location = loc
                spawn_points.append(spawn_point)

        batch = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

        results = client.apply_batch_sync(batch, True)
        walker_ids = [result.actor_id for result in results if not result.error]

        batch = []
        for walker_id in walker_ids:
            batch.append(carla.command.SpawnActor(blueprintsWalkersController, carla.Transform(), walker_id))

        results = client.apply_batch_sync(batch, True)
        controller_ids = [result.actor_id for result in results if not result.error]

        all_id = walker_ids + controller_ids

        
        def point_in_canvas(pos, img_h, img_w):
            """Return true if point is in canvas"""
            if (pos[0] >= 0) and (pos[0] < img_w) and (pos[1] >= 0) and (pos[1] < img_h):
                return True
            return False
        
        call_exit = False
        
        while True:
            # Carla Tick
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

            # Render received data
            display_manager.render()
            
            # Retrieve and process the image
            # image = image_queue.get()
            # img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

            ego_vehicle = vehicle

            draw_bounding_boxes(world, ego_vehicle)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE or event.key == K_q:
                        call_exit = True
                        break

            if call_exit:
                break

        #     # Display on an OpenCV display window
        #     cv2.namedWindow('BoundingBoxWindow', cv2.WINDOW_AUTOSIZE)
        #     cv2.imshow('BoundingBoxWindow',img)
        #     cv2.waitKey(1)

        #     world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

        #     image_w = camera_bp.get_attribute("image_size_x").as_int()
        #     image_h = camera_bp.get_attribute("image_size_y").as_int()
        #     fov = camera_bp.get_attribute("fov").as_float()

        #     # Calculate the camera projection matrix to project from 3D -> 2D
        #     K = build_projection_matrix(image_w, image_h, fov)
        #     K_b = build_projection_matrix(image_w, image_h, fov, is_behind_camera=True)

        #     # Render bounding boxes
        #     for bb in bounding_box_set:

        #         # Filter for distance from ego vehicle
        #         if bb.location.distance(vehicle.get_transform().location) < 50:
        #             forward_vec = vehicle.get_transform().get_forward_vector()
        #             ray = bb.location - vehicle.get_transform().location

        #             # Only render bounding boxes in front of the camera
        #             if forward_vec.dot(ray) > 0:
        #                 verts = [v for v in bb.get_world_vertices(carla.Transform())]
        #                 for edge in edges:
        #                     p1 = get_image_point(verts[edge[0]], K, world_2_camera)
        #                     p2 = get_image_point(verts[edge[1]], K, world_2_camera)

        #                     if point_in_canvas(p1, image_h, image_w) and point_in_canvas(p2, image_h, image_w):
        #                         cv2.line(img, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (0, 0, 255, 255), 1)

        #     for npc in world.get_actors().filter('*vehicle*'):

        #         # Filter out the ego vehicle
        #         if npc.id != vehicle.id:

        #             bb = npc.bounding_box
        #             dist = npc.get_transform().location.distance(vehicle.get_transform().location)

        #             # Filter for the vehicles within 50m
        #             if dist < 50:

        #             # Calculate the dot product between the forward vector
        #             # of the vehicle and the vector between the vehicle
        #             # and the other vehicle. We threshold this dot product
        #             # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
        #                 forward_vec = vehicle.get_transform().get_forward_vector()
        #                 ray = npc.get_transform().location - vehicle.get_transform().location

        #                 if forward_vec.dot(ray) > 0:
        #                     verts = [v for v in bb.get_world_vertices(npc.get_transform())]
        #                     for edge in edges:
        #                         p1 = get_image_point(verts[edge[0]], K, world_2_camera)
        #                         p2 = get_image_point(verts[edge[1]],  K, world_2_camera)

        #                         p1_in_canvas = point_in_canvas(p1, image_h, image_w)
        #                         p2_in_canvas = point_in_canvas(p2, image_h, image_w)

        #                         if not p1_in_canvas and not p2_in_canvas:
        #                             continue

        #                         ray0 = verts[edge[0]] - camera.get_transform().location
        #                         ray1 = verts[edge[1]] - camera.get_transform().location
        #                         cam_forward_vec = camera.get_transform().get_forward_vector()

        #                         # One of the vertex is behind the camera
        #                         if not (cam_forward_vec.dot(ray0) > 0):
        #                             p1 = get_image_point(verts[edge[0]], K_b, world_2_camera)
        #                         if not (cam_forward_vec.dot(ray1) > 0):
        #                             p2 = get_image_point(verts[edge[1]], K_b, world_2_camera)

        #                         cv2.line(img, (int(p1[0]),int(p1[1])), (int(p2[0]),int(p2[1])), (255,0,0, 255), 1)        

        #     cv2.imshow('BoundingBoxWindow',img)
        #     if cv2.waitKey(1) == ord('q'):
        #         break
        # cv2.destroyAllWindows()

    finally:
        if display_manager:
            display_manager.destroy()

        if vehicle:
            vehicle.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        world.apply_settings(original_settings)
        cv2.destroyAllWindows()
        print("Simulation resources have been cleaned up.")

        # print("Stop recording")
        # client.stop_recorder()
        # # set the time factor for the replayer
        # client.set_replayer_time_factor(args.time_factor)

        # # set to ignore the hero vehicles or not
        # client.set_replayer_ignore_hero(args.ignore_hero)

        # # set to ignore the spectator camera or not
        # client.set_replayer_ignore_spectator(not args.move_spectator)
        # print(client.replay_file(recorder_path, args.start, args.duration, args.camera, args.spawn_sensors))
        # print("Replaying on file: %s" % recorder_path)


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
    argparser.add_argument(
        '-f', '--recorder_filename',
        metavar='F',
        default="test1.log",
        help='recorder filename (test1.log)')
    argparser.add_argument(
        '-t', '--recorder_time',
        metavar='T',
        default=0,
        type=int,
        help='recorder duration (auto-stop)')
    argparser.add_argument(
        '--file_location',
        metavar='FL',
        default=r"C:\Users\Ky\Desktop\CARLA testing\WindowsNoEditor\PythonAPI\carla_testing\out", # r"Path\to\file"
        help='recorder file location (carla_testing\out)')
    argparser.add_argument(
        '--spawn-sensors',
        action='store_true',
        help='spawn sensors in the replayed world')
    argparser.add_argument(
        '-c', '--camera',
        metavar='C',
        default=0,
        type=int,
        help='camera follows an actor (ex: 82)')
    argparser.add_argument(
        '-x', '--time-factor',
        metavar='X',
        default=1.0,
        type=float,
        help='time factor (default 1.0)')
    argparser.add_argument(
        '-s', '--start',
        metavar='S',
        default=0.0,
        type=float,
        help='starting time (default: 0.0)')
    argparser.add_argument(
        '-d', '--duration',
        metavar='D',
        default=0.0,
        type=float,
        help='duration (default: 0.0)')
    argparser.add_argument(
        '-i', '--ignore-hero',
        action='store_true',
        help='ignore hero vehicles')
    argparser.add_argument(
        '--move-spectator',
        action='store_true',
        help='move spectator camera')
    
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

# # Function to visualize and save LiDAR data using matplotlib
# def visualize_lidar_data():
#     # Get the list of saved LiDAR point cloud files
#     lidar_files = glob.glob('out/lidar/*.ply')
#     print(f"Found {len(lidar_files)} LiDAR files.")

#     # Ensure the output directory exists
#     output_dir = 'out/lidar_plots'
#     os.makedirs(output_dir, exist_ok=True)

#     for i, file in enumerate(lidar_files):
#         print(f"Processing file: {file}")
        
#         # Read the point cloud file
#         plydata = PlyData.read(file)
        
#         # Extract the point cloud data
#         points = np.vstack([plydata['vertex'].data['x'], 
#                             plydata['vertex'].data['y'], 
#                             plydata['vertex'].data['z']]).T
        
#         # Create a figure with subplots for different views
#         fig, axs = plt.subplots(1, 3, figsize=(15, 5))
        
#         # Top-down view (XY plane)
#         axs[0].scatter(points[:, 0], points[:, 1], s=1)
#         axs[0].set_title('Top-down view (XY plane)')
#         axs[0].set_xlabel('X')
#         axs[0].set_ylabel('Y')
        
#         # Side view (XZ plane)
#         axs[1].scatter(points[:, 0], points[:, 2], s=1)
#         axs[1].set_title('Side view (XZ plane)')
#         axs[1].set_xlabel('X')
#         axs[1].set_ylabel('Z')
        
#         # Front view (YZ plane)
#         axs[2].scatter(points[:, 1], points[:, 2], s=1)
#         axs[2].set_title('Front view (YZ plane)')
#         axs[2].set_xlabel('Y')
#         axs[2].set_ylabel('Z')
        
#         # Save the plots to a file
#         plot_filename = os.path.join(output_dir, f'lidar_plot_{i:06d}.png')
#         plt.savefig(plot_filename)
#         print(f"Saved plot to {plot_filename}")

#         # Close the figure to free up memory
#         plt.close(fig)

# Call the function to visualize and save LiDAR data
# simulation_thread.join()  # Wait for the simulation to finish
# visualize_lidar_data()