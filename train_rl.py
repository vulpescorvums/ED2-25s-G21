# train_rl.py
import carla
import random
from rl_agent import CarlaPerceptionEnv, RLAgent

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Enable sync mode for training
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # Spawn vehicle
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('charger_2020')[0]
    spawn_point = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    vehicle.set_autopilot(False)

    try:
        # Set up env and train agent
        env = CarlaPerceptionEnv(vehicle, world)
        agent = RLAgent()
        agent.attach_env(env)
        agent.train(total_timesteps=100000)

    finally:
        print("Cleaning up...")
        vehicle.destroy()
        world.apply_settings(settings)

if __name__ == "__main__":
    main()
