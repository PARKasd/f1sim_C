import gymnasium as gym
import numpy as np

import f110_gym  # noqa: F401


def main():
    env = gym.make(
        'f110_gym:f110-rl-v0',
        map='levine',
        map_ext='.pgm',
        max_episode_length=200,
        start_pose=np.array([0.0, 0.0, 0.0]),
    )

    obs = env.reset()
    total_reward = 0.0

    for step in range(200):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        total_reward += reward
        if done:
            print(f'episode ended at step={step}, reward={total_reward:.3f}, info={info}')
            break
    else:
        print(f'episode finished without done, reward={total_reward:.3f}')

    print(f'observation shape: {obs.shape}')
    print(f'action space: {env.action_space}')


if __name__ == '__main__':
    main()
