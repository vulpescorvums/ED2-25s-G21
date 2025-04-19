from carla_include import *
import argparse
import pygame
import math
from pygame.locals import K_ESCAPE, K_q
from rl_agent import CarlaPerceptionEnv, RLAgent

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


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
        
        elif sensor_type == 'SemanticLiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            lidar_bp.set_attribute('range', '100')

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_semanticlidar_image)

            return lidar
        
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

                    norm_velocity = detect.velocity / velocity_range  # range [-1, 1]
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
        
        ###################################################################################################################
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
        
        ###################################################################################################################
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
        
        elif sensor_type == "IMU":
            imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
            imu = self.world.spawn_actor(imu_bp, transform, attach_to=attached)
            imu.listen(self.save_imu_data)

            return imu
        
        elif sensor_type == "GNSS":
            gnss_bp = self.world.get_blueprint_library().find('sensor.other.gnss')
            gnss = self.world.spawn_actor(gnss_bp, transform, attach_to=attached)
            gnss.listen(self.save_gnss_data)

            return gnss
        
        elif sensor_type == "Collision":
            collision_bp = self.world.get_blueprint_library().find('sensor.other.collision')
            collision = self.world.spawn_actor(collision_bp, transform, attach_to=attached)
            collision.listen(self.save_collision_data)

            return collision
        
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

    def save_semanticlidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 6), 6))
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

    def save_radar_image(self, radar_data):
        t_start = self.timer.time()
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    ###################################################################################################################
    def save_depth_image(self, image):
        t_start = self.timer.time()

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

    ###################################################################################################################
    def save_semanticsegmenation_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.CityScapesPalette)
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

def run_simulation(args, client):
    """This function performed one test run using the args parameters
    and connecting to the carla client passed.
    """

    display_manager = None
    vehicle = None
    vehicle_list = []
    timer = CustomTimer()

    try:

        # Getting the world and
        world = client.get_world()
        original_settings = world.get_settings()

        # # Concatenate file location and filename
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


        # Instanciating the vehicle to which we attached the sensors
        bp = world.get_blueprint_library().filter('charger_2020')[0]
        vehicle = world.spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))
        vehicle_list.append(vehicle)
        # vehicle.set_autopilot(True)
        # === PPO RL Agent Setup - commenting out CARLA autopilotting (above) to have the PPO drive, essentially (below) ===
        env = CarlaPerceptionEnv(vehicle, world)
        agent = RLAgent()  # Can use "ppo_carla_model" string to load a pretrained model
        agent.attach_env(env)
        obs = env.reset()

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
        # If can easily configure the grid and the total window size
        display_manager = DisplayManager(grid_size=[3, 3], window_size=[args.width, args.height])

        # Then, SensorManager can be used to spawn RGBCamera, LiDARs and SemanticLiDARs as needed
        # and assign each of them to a grid position,
        
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
        

        #Simulation loop
        call_exit = False
        time_init_sim = timer.time()
        while True:
            # # Carla Tick
            # if args.sync:
            #     world.tick()
            # else:
            #     world.wait_for_tick()
            
            world.tick()

            # RL control logic
            action = agent.get_action(obs)
            obs, reward, done, _ = env.step(action)

            if done:
                obs = env.reset()

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

        # print("Stop recording")
        # client.stop_recorder()
        # # set the time factor for the replayer
        # client.set_replayer_time_factor(args.time_factor)

        # # Set to ignore the hero vehicles or not
        # client.set_replayer_ignore_hero(args.ignore_hero)

        # # Set to ignore the spectator camera or not
        # client.set_replayer_ignore_spectator(not args.move_spectator)
        # print(client.replay_file(recorder_path, args.start, args.duration, args.camera, args.spawn_sensors))
        # print("Replaying on file: %s" % recorder_path)

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
