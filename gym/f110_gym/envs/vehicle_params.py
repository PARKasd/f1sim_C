import yaml


VEHICLE_PARAM_KEYS = (
    'mu',
    'C_Sf',
    'C_Sr',
    'lf',
    'lr',
    'h',
    'm',
    'I',
    's_min',
    's_max',
    'sv_min',
    'sv_max',
    'v_switch',
    'a_max',
    'v_min',
    'v_max',
    'width',
    'length',
)

DEFAULT_VEHICLE_PARAMS = {
    'mu': 1.0489,
    'C_Sf': 4.718,
    'C_Sr': 5.4562,
    'lf': 0.15875,
    'lr': 0.17145,
    'h': 0.074,
    'm': 3.74,
    'I': 0.04712,
    's_min': -0.4189,
    's_max': 0.4189,
    'sv_min': -3.2,
    'sv_max': 3.2,
    'v_switch': 7.319,
    'a_max': 9.51,
    'v_min': -5.0,
    'v_max': 20.0,
    'width': 0.31,
    'length': 0.58,
}


def normalize_vehicle_params(params=None):
    normalized = dict(DEFAULT_VEHICLE_PARAMS)
    if params:
        normalized.update(params)

    missing = [key for key in VEHICLE_PARAM_KEYS if key not in normalized]
    if missing:
        raise ValueError(f'Missing vehicle parameters: {missing}')

    return {key: float(normalized[key]) for key in VEHICLE_PARAM_KEYS}


def load_vehicle_params(path):
    with open(path, 'r') as stream:
        data = yaml.safe_load(stream) or {}

    if 'vehicle' in data:
        data = data['vehicle'] or {}

    return normalize_vehicle_params(data)
