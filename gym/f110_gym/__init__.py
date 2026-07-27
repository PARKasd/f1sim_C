from gymnasium.envs.registration import register

# order_enforce/disable_env_checker are off so gym.make() returns the bare env:
# these envs keep the original f1tenth_gym API (reset(poses) -> 4-tuple), which
# gymnasium's wrappers would otherwise reject.
register(
	id='f110-v0',
	entry_point='f110_gym.envs:F110Env',
	order_enforce=False,
	disable_env_checker=True,
	)
register(
	id='f110-rl-v0',
	entry_point='f110_gym.envs:F110RLEnv',
	order_enforce=False,
	disable_env_checker=True,
	)
