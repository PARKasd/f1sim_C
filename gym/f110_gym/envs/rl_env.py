import gym
from gym import spaces
import numpy as np

from f110_gym.envs.f110_env import F110Env


class F110RLEnv(gym.Env):
    """
    Single-agent RL wrapper around F110Env.

    Action:
        np.ndarray shape (2,): [desired steering angle, desired speed]

    Observation:
        np.ndarray shape (num_beams + 4,):
        normalized scan, normalized speed, normalized yaw rate,
        sin(theta), cos(theta)
    """

    metadata = {'render.modes': ['human', 'human_fast']}

    def __init__(self, **kwargs):
        self.max_episode_steps = kwargs.pop('max_episode_steps', 1000)
        self.start_pose = np.asarray(
            kwargs.pop('start_pose', [0.0, 0.0, 0.0]),
            dtype=np.float64)
        self.max_scan_range = float(kwargs.pop('max_scan_range', 30.0))
        kwargs['num_agents'] = 1

        self.env = F110Env(**kwargs)
        self.num_beams = self.env.num_beams
        self.params = self.env.params
        self.steps = 0
        self.prev_x = None

        self.action_space = spaces.Box(
            low=np.array([self.params['s_min'], self.params['v_min']], dtype=np.float32),
            high=np.array([self.params['s_max'], self.params['v_max']], dtype=np.float32),
            dtype=np.float32)
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.num_beams + 4,),
            dtype=np.float32)

    def _flatten_obs(self, obs):
        scan = np.asarray(obs['scans'][0], dtype=np.float32)
        scan = np.nan_to_num(scan, nan=self.max_scan_range, posinf=self.max_scan_range, neginf=0.0)
        scan = np.clip(scan, 0.0, self.max_scan_range) / self.max_scan_range

        speed = float(obs['linear_vels_x'][0]) / max(abs(self.params['v_max']), 1e-6)
        yaw_rate = float(obs['ang_vels_z'][0]) / 10.0
        theta = float(obs['poses_theta'][0])
        state = np.array([speed, yaw_rate, np.sin(theta), np.cos(theta)], dtype=np.float32)
        return np.concatenate((scan.astype(np.float32), state), axis=0)

    def reset(self):
        self.steps = 0
        poses = self.start_pose.reshape(1, 3)
        obs, _, _, _ = self.env.reset(poses)
        self.prev_x = float(obs['poses_x'][0])
        return self._flatten_obs(obs)

    def step(self, action):
        action = np.asarray(action, dtype=np.float64)
        action = np.clip(action, self.action_space.low, self.action_space.high)
        obs, _, done, info = self.env.step(action.reshape(1, 2))

        self.steps += 1
        x = float(obs['poses_x'][0])
        progress = 0.0 if self.prev_x is None else x - self.prev_x
        self.prev_x = x

        collision = bool(obs['collisions'][0])
        reward = progress + 0.01
        if collision:
            reward -= 1.0

        if self.steps >= self.max_episode_steps:
            done = True

        info = dict(info)
        info['collision'] = collision
        return self._flatten_obs(obs), float(reward), bool(done), info

    def render(self, mode='human'):
        return self.env.render(mode)

    def close(self):
        pass
