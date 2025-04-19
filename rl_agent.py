# ---------- How it's being integrated into demo.py ----------
# In demo.py, import the following:
# `from rl_agent import CarlaPerceptionEnv, RLAgent`
#
# Then, create the environment and agent (instead of `vehicle.set_autopilot(True)`):
# `env = CarlaPerceptionEnv(vehicle, world)
#  agent = RLAgent("ppo_carla_model")  # or None if training
#  agent.attach_env(env)`
#
#  obs = env.reset()`
#
# Finally, inside the simulation loop:
# `action = agent.get_action(obs)
#  obs, reward, done, _ = env.step(action)`
#
# if done:
#     `obs = env.reset()`
# ----------------------------------------------------

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from gym import spaces
import numpy as np
import carla
import random
import torch

class CarlaPerceptionEnv:
    """
    Simplified CARLA Gym-style environment for PPO.
    This version expects to be plugged into demo.py's simulation loop.
    """
    def __init__(self, ego_vehicle, world):
        self.ego_vehicle = ego_vehicle
        self.world = world
        self.action_space = spaces.Box(low=np.array([-1.0, 0.0, 0.0]),  # steering, throttling, braking
                                       high=np.array([1.0, 1.0, 1.0]),
                                       dtype=np.float32)

        # Placeholder observation space: [speed, distance_front, obstacle_left, obstacle_right]
        self.observation_space = spaces.Box(low=0, high=100, shape=(4,), dtype=np.float32)

        self.prev_distance = None
        self.current_step = 0

    def reset(self):
        self.current_step = 0
        self.ego_vehicle.set_transform(random.choice(self.world.get_map().get_spawn_points()))
        self.ego_vehicle.set_autopilot(False)
        return self._get_obs()

    def step(self, action):
        steer, throttle, brake = action
        control = carla.VehicleControl()
        control.steer = float(steer)
        control.throttle = float(throttle)
        control.brake = float(brake)
        self.ego_vehicle.apply_control(control)

        self.world.tick()  # must be in sync mode

        obs = self._get_obs()
        reward = self._calculate_reward(obs)
        done = self._is_done()

        self.current_step += 1

        return obs, reward, done, {}

    def _get_obs(self):
        velocity = self.ego_vehicle.get_velocity()
        speed = 3.6 * np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # km/h

        # Placeholder: randomized dummy sensor values
        distance_front = random.uniform(0, 50)
        obstacle_left = random.uniform(0, 50)
        obstacle_right = random.uniform(0, 50)

        return np.array([speed, distance_front, obstacle_left, obstacle_right], dtype=np.float32)

    def _calculate_reward(self, obs):
        speed, distance_front, _, _ = obs
        reward = 0.1 * speed
        if distance_front < 5:
            reward -= 10  # collision penalty
        return reward

    def _is_done(self):
        return self.current_step > 300


class RLAgent:
    def __init__(self, model_path=None):
        self.env = None
        self.model = None

        if model_path:
            self.model = PPO.load(model_path)

    def attach_env(self, env):
        self.env = env

    def get_action(self, obs):
        action, _states = self.model.predict(obs, deterministic=True)
        return action

    def train(self, total_timesteps=10000):
        self.model = PPO("MlpPolicy", self.env, verbose=1)
        self.model.learn(total_timesteps=total_timesteps)
        self.model.save("ppo_carla_model")