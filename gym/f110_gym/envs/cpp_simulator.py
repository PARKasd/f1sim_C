import os

import numpy as np
from PIL import Image
import yaml

try:
    from f110_gym import _cpp_backend
    CPP_AVAILABLE = True
except ImportError:
    _cpp_backend = None
    CPP_AVAILABLE = False


class CppSimulator:
    """
    Python adapter for the C++ simulator core.

    Map image and metadata loading stay in Python; physics, scan simulation,
    and collision checking run in the C++ extension module.
    """

    def __init__(
            self,
            params,
            num_agents,
            seed,
            time_step=0.01,
            ego_idx=0,
            integrator=None,
            lidar_dist=0.0,
            num_beams=1080,
            fov=4.7):
        if not CPP_AVAILABLE:
            raise ImportError('f110_gym._cpp_backend is not built')

        integrator_value = getattr(integrator, 'value', integrator)
        if integrator_value is None:
            integrator_value = 1

        self.num_agents = num_agents
        self.seed = seed
        self.time_step = time_step
        self.ego_idx = ego_idx
        self.params = params
        self.num_beams = num_beams
        self.fov = fov
        self._core = _cpp_backend.create(
            params,
            int(num_agents),
            int(seed),
            float(time_step),
            int(ego_idx),
            int(integrator_value),
            float(lidar_dist),
            int(num_beams),
            float(fov),
        )

    def set_map(self, map_path, map_ext):
        map_img_path = os.path.splitext(map_path)[0] + map_ext
        map_img = np.array(
            Image.open(map_img_path).convert('L').transpose(Image.FLIP_TOP_BOTTOM),
            dtype=np.uint8)

        with open(map_path, 'r') as yaml_stream:
            map_metadata = yaml.safe_load(yaml_stream)

        resolution = float(map_metadata['resolution'])
        origin = map_metadata['origin']
        map_img = np.ascontiguousarray(map_img)
        return _cpp_backend.set_map_image(self._core, map_img, resolution, origin)

    def update_params(self, params, agent_idx=-1):
        self.params = params
        _cpp_backend.update_params(self._core, params, int(agent_idx))

    def reset(self, poses):
        poses = np.ascontiguousarray(poses, dtype=np.float64)
        _cpp_backend.reset(self._core, poses)

    def step(self, control_inputs):
        control_inputs = np.ascontiguousarray(control_inputs, dtype=np.float64)
        return _cpp_backend.step(self._core, control_inputs)
