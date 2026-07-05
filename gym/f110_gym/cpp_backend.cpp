#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <numpy/arrayobject.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr const char * kCapsuleName = "f110_gym.SimulatorCore";
constexpr double kPi = 3.141592653589793238462643383279502884;

struct Params {
  double mu;
  double c_sf;
  double c_sr;
  double lf;
  double lr;
  double h;
  double m;
  double inertia;
  double s_min;
  double s_max;
  double sv_min;
  double sv_max;
  double v_switch;
  double a_max;
  double v_min;
  double v_max;
  double width;
  double length;
};

double dict_double(PyObject * dict, const char * key)
{
  PyObject * value = PyDict_GetItemString(dict, key);
  if (!value) {
    throw std::runtime_error(std::string("missing vehicle parameter: ") + key);
  }
  const double out = PyFloat_AsDouble(value);
  if (PyErr_Occurred()) {
    throw std::runtime_error(std::string("vehicle parameter is not numeric: ") + key);
  }
  return out;
}

Params parse_params(PyObject * dict)
{
  if (!PyDict_Check(dict)) {
    throw std::runtime_error("vehicle params must be a dict");
  }
  return Params{
    dict_double(dict, "mu"),
    dict_double(dict, "C_Sf"),
    dict_double(dict, "C_Sr"),
    dict_double(dict, "lf"),
    dict_double(dict, "lr"),
    dict_double(dict, "h"),
    dict_double(dict, "m"),
    dict_double(dict, "I"),
    dict_double(dict, "s_min"),
    dict_double(dict, "s_max"),
    dict_double(dict, "sv_min"),
    dict_double(dict, "sv_max"),
    dict_double(dict, "v_switch"),
    dict_double(dict, "a_max"),
    dict_double(dict, "v_min"),
    dict_double(dict, "v_max"),
    dict_double(dict, "width"),
    dict_double(dict, "length"),
  };
}

double accl_constraints(
  double vel, double accl, double v_switch, double a_max, double v_min, double v_max)
{
  const double pos_limit = vel > v_switch ? a_max * v_switch / vel : a_max;

  if ((vel <= v_min && accl <= 0.0) || (vel >= v_max && accl >= 0.0)) {
    accl = 0.0;
  } else if (accl <= -a_max) {
    accl = -a_max;
  } else if (accl >= pos_limit) {
    accl = pos_limit;
  }
  return accl;
}

double steering_constraint(
  double steering_angle, double steering_velocity, double s_min, double s_max,
  double sv_min, double sv_max)
{
  if ((steering_angle <= s_min && steering_velocity <= 0.0) ||
    (steering_angle >= s_max && steering_velocity >= 0.0))
  {
    steering_velocity = 0.0;
  } else if (steering_velocity <= sv_min) {
    steering_velocity = sv_min;
  } else if (steering_velocity >= sv_max) {
    steering_velocity = sv_max;
  }
  return steering_velocity;
}

std::array<double, 2> pid(
  double speed, double steer, double current_speed, double current_steer,
  double max_sv, double max_a, double max_v, double min_v)
{
  const double steer_diff = steer - current_steer;
  double sv = 0.0;
  if (std::fabs(steer_diff) > 1e-4) {
    sv = (steer_diff / std::fabs(steer_diff)) * max_sv;
  }

  const double vel_diff = speed - current_speed;
  double accl = 0.0;
  if (current_speed > 0.0) {
    if (vel_diff > 0.0) {
      accl = 10.0 * max_a / max_v * vel_diff;
    } else {
      accl = 10.0 * max_a / (-min_v) * vel_diff;
    }
  } else {
    if (vel_diff > 0.0) {
      accl = 2.0 * max_a / max_v * vel_diff;
    } else {
      accl = 2.0 * max_a / (-min_v) * vel_diff;
    }
  }
  return {accl, sv};
}

std::array<double, 5> vehicle_dynamics_ks(
  const std::array<double, 5> & x, double sv, double accl, const Params & p)
{
  const double lwb = p.lf + p.lr;
  const double constrained_sv =
    steering_constraint(x[2], sv, p.s_min, p.s_max, p.sv_min, p.sv_max);
  const double constrained_accl =
    accl_constraints(x[3], accl, p.v_switch, p.a_max, p.v_min, p.v_max);
  return {
    x[3] * std::cos(x[4]),
    x[3] * std::sin(x[4]),
    constrained_sv,
    constrained_accl,
    x[3] / lwb * std::tan(x[2]),
  };
}

std::array<double, 7> vehicle_dynamics_st(
  const std::array<double, 7> & x, double sv, double accl, const Params & p)
{
  constexpr double g = 9.81;
  const double constrained_sv =
    steering_constraint(x[2], sv, p.s_min, p.s_max, p.sv_min, p.sv_max);
  const double constrained_accl =
    accl_constraints(x[3], accl, p.v_switch, p.a_max, p.v_min, p.v_max);

  if (std::fabs(x[3]) < 0.5) {
    const std::array<double, 5> x_ks{x[0], x[1], x[2], x[3], x[4]};
    const auto f_ks = vehicle_dynamics_ks(x_ks, constrained_sv, constrained_accl, p);
    const double lwb = p.lf + p.lr;
    return {
      f_ks[0],
      f_ks[1],
      f_ks[2],
      f_ks[3],
      f_ks[4],
      constrained_accl / lwb * std::tan(x[2]) +
        x[3] / (lwb * std::pow(std::cos(x[2]), 2.0)) * constrained_sv,
      0.0,
    };
  }

  const double lr_lf = p.lr + p.lf;
  return {
    x[3] * std::cos(x[6] + x[4]),
    x[3] * std::sin(x[6] + x[4]),
    constrained_sv,
    constrained_accl,
    x[5],
    -p.mu * p.m / (x[3] * p.inertia * lr_lf) *
      (p.lf * p.lf * p.c_sf * (g * p.lr - constrained_accl * p.h) +
      p.lr * p.lr * p.c_sr * (g * p.lf + constrained_accl * p.h)) * x[5] +
      p.mu * p.m / (p.inertia * lr_lf) *
      (p.lr * p.c_sr * (g * p.lf + constrained_accl * p.h) -
      p.lf * p.c_sf * (g * p.lr - constrained_accl * p.h)) * x[6] +
      p.mu * p.m / (p.inertia * lr_lf) *
      p.lf * p.c_sf * (g * p.lr - constrained_accl * p.h) * x[2],
    (p.mu / (x[3] * x[3] * lr_lf) *
      (p.c_sr * (g * p.lf + constrained_accl * p.h) * p.lr -
      p.c_sf * (g * p.lr - constrained_accl * p.h) * p.lf) - 1.0) * x[5] -
      p.mu / (x[3] * lr_lf) *
      (p.c_sr * (g * p.lf + constrained_accl * p.h) +
      p.c_sf * (g * p.lr - constrained_accl * p.h)) * x[6] +
      p.mu / (x[3] * lr_lf) *
      (p.c_sf * (g * p.lr - constrained_accl * p.h)) * x[2],
  };
}

double cross2(const std::array<double, 2> & a, const std::array<double, 2> & b)
{
  return a[0] * b[1] - a[1] * b[0];
}

double dot2(const std::array<double, 2> & a, const std::array<double, 2> & b)
{
  return a[0] * b[0] + a[1] * b[1];
}

std::array<double, 2> sub2(const std::array<double, 2> & a, const std::array<double, 2> & b)
{
  return {a[0] - b[0], a[1] - b[1]};
}

std::array<double, 2> neg2(const std::array<double, 2> & a)
{
  return {-a[0], -a[1]};
}

std::array<double, 2> perpendicular(std::array<double, 2> pt)
{
  const double temp = pt[0];
  pt[0] = pt[1];
  pt[1] = -temp;
  return pt;
}

std::array<double, 2> triple_product(
  const std::array<double, 2> & a, const std::array<double, 2> & b,
  const std::array<double, 2> & c)
{
  const double ac = dot2(a, c);
  const double bc = dot2(b, c);
  return {b[0] * ac - a[0] * bc, b[1] * ac - a[1] * bc};
}

std::array<double, 2> avg_point(const std::array<std::array<double, 2>, 4> & vertices)
{
  std::array<double, 2> out{0.0, 0.0};
  for (const auto & v : vertices) {
    out[0] += v[0];
    out[1] += v[1];
  }
  out[0] /= 4.0;
  out[1] /= 4.0;
  return out;
}

int index_of_furthest_point(
  const std::array<std::array<double, 2>, 4> & vertices,
  const std::array<double, 2> & d)
{
  int best = 0;
  double best_dot = dot2(vertices[0], d);
  for (int i = 1; i < 4; ++i) {
    const double candidate = dot2(vertices[i], d);
    if (candidate > best_dot) {
      best = i;
      best_dot = candidate;
    }
  }
  return best;
}

std::array<double, 2> support(
  const std::array<std::array<double, 2>, 4> & vertices1,
  const std::array<std::array<double, 2>, 4> & vertices2,
  const std::array<double, 2> & d)
{
  const int i = index_of_furthest_point(vertices1, d);
  const int j = index_of_furthest_point(vertices2, neg2(d));
  return sub2(vertices1[i], vertices2[j]);
}

bool collision(
  const std::array<std::array<double, 2>, 4> & vertices1,
  const std::array<std::array<double, 2>, 4> & vertices2)
{
  int index = 0;
  std::array<std::array<double, 2>, 3> simplex{};
  std::array<double, 2> d = sub2(avg_point(vertices1), avg_point(vertices2));
  if (d[0] == 0.0 && d[1] == 0.0) {
    d[0] = 1.0;
  }

  auto a = support(vertices1, vertices2, d);
  simplex[index] = a;
  if (dot2(d, a) <= 0.0) {
    return false;
  }

  d = neg2(a);
  int iter_count = 0;
  while (iter_count < 1000) {
    a = support(vertices1, vertices2, d);
    index += 1;
    simplex[index] = a;
    if (dot2(d, a) <= 0.0) {
      return false;
    }

    const auto ao = neg2(a);
    if (index < 2) {
      const auto b = simplex[0];
      const auto ab = sub2(b, a);
      d = triple_product(ab, ao, ab);
      if (std::hypot(d[0], d[1]) < 1e-10) {
        d = perpendicular(ab);
      }
      continue;
    }

    const auto b = simplex[1];
    const auto c = simplex[0];
    const auto ab = sub2(b, a);
    const auto ac = sub2(c, a);
    const auto acperp = triple_product(ab, ac, ac);

    if (dot2(acperp, ao) >= 0.0) {
      d = acperp;
    } else {
      const auto abperp = triple_product(ac, ab, ab);
      if (dot2(abperp, ao) < 0.0) {
        return true;
      }
      simplex[0] = simplex[1];
      d = abperp;
    }
    simplex[1] = simplex[2];
    index -= 1;
    iter_count += 1;
  }
  return false;
}

std::array<std::array<double, 2>, 4> get_vertices(
  const std::array<double, 3> & pose, double length, double width)
{
  const double x = pose[0];
  const double y = pose[1];
  const double th = pose[2];
  const double c = std::cos(th);
  const double s = std::sin(th);
  const std::array<std::array<double, 2>, 4> local{{
    {{-length / 2.0, width / 2.0}},
    {{-length / 2.0, -width / 2.0}},
    {{length / 2.0, -width / 2.0}},
    {{length / 2.0, width / 2.0}},
  }};
  std::array<std::array<double, 2>, 4> out{};
  for (std::size_t i = 0; i < 4; ++i) {
    out[i] = {c * local[i][0] - s * local[i][1] + x, s * local[i][0] + c * local[i][1] + y};
  }
  return out;
}

bool are_collinear(
  const std::array<double, 2> & a, const std::array<double, 2> & b,
  const std::array<double, 2> & c)
{
  const auto ba = sub2(b, a);
  const auto ca = sub2(a, c);
  return std::fabs(cross2(ba, ca)) < 1e-8;
}

double get_range(
  const std::array<double, 3> & pose, double beam_theta,
  const std::array<double, 2> & va, const std::array<double, 2> & vb)
{
  const std::array<double, 2> o{pose[0], pose[1]};
  const auto v1 = sub2(o, va);
  const auto v2 = sub2(vb, va);
  const std::array<double, 2> v3{std::cos(beam_theta + kPi / 2.0), std::sin(beam_theta + kPi / 2.0)};
  const double denom = dot2(v2, v3);
  double distance = std::numeric_limits<double>::infinity();
  if (std::fabs(denom) > 0.0) {
    const double d1 = cross2(v2, v1) / denom;
    const double d2 = dot2(v1, v3) / denom;
    if (d1 >= 0.0 && d2 >= 0.0 && d2 <= 1.0) {
      distance = d1;
    }
  } else if (are_collinear(o, va, vb)) {
    distance = std::min(
      std::hypot(va[0] - o[0], va[1] - o[1]),
      std::hypot(vb[0] - o[0], vb[1] - o[1]));
  }
  return distance;
}

struct MapData {
  int height = 0;
  int width = 0;
  double resolution = 0.0;
  double orig_x = 0.0;
  double orig_y = 0.0;
  double orig_c = 1.0;
  double orig_s = 0.0;
  std::vector<double> dt;
};

std::vector<double> edt_1d(const std::vector<double> & f, int n)
{
  constexpr double inf = 1e20;
  std::vector<double> d(static_cast<std::size_t>(n), 0.0);
  std::vector<int> v(static_cast<std::size_t>(n), 0);
  std::vector<double> z(static_cast<std::size_t>(n + 1), 0.0);

  int k = 0;
  v[0] = 0;
  z[0] = -inf;
  z[1] = inf;

  for (int q = 1; q < n; ++q) {
    double s = 0.0;
    while (true) {
      const int vk = v[static_cast<std::size_t>(k)];
      s = ((f[static_cast<std::size_t>(q)] + q * q) -
        (f[static_cast<std::size_t>(vk)] + vk * vk)) /
        (2.0 * q - 2.0 * vk);
      if (s <= z[static_cast<std::size_t>(k)]) {
        --k;
      } else {
        break;
      }
    }
    ++k;
    v[static_cast<std::size_t>(k)] = q;
    z[static_cast<std::size_t>(k)] = s;
    z[static_cast<std::size_t>(k + 1)] = inf;
  }

  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[static_cast<std::size_t>(k + 1)] < q) {
      ++k;
    }
    const double diff = q - v[static_cast<std::size_t>(k)];
    d[static_cast<std::size_t>(q)] =
      diff * diff + f[static_cast<std::size_t>(v[static_cast<std::size_t>(k)])];
  }
  return d;
}

std::vector<double> distance_transform_2d(
  const unsigned char * image, int height, int width, double resolution)
{
  constexpr double inf = 1e20;
  std::vector<double> temp(static_cast<std::size_t>(height * width), 0.0);
  std::vector<double> f(static_cast<std::size_t>(std::max(height, width)), 0.0);

  for (int r = 0; r < height; ++r) {
    for (int c = 0; c < width; ++c) {
      const unsigned char value = image[r * width + c];
      f[static_cast<std::size_t>(c)] = value <= 128 ? 0.0 : inf;
    }
    const auto row = edt_1d(f, width);
    for (int c = 0; c < width; ++c) {
      temp[static_cast<std::size_t>(r * width + c)] = row[static_cast<std::size_t>(c)];
    }
  }

  std::vector<double> dt(static_cast<std::size_t>(height * width), 0.0);
  for (int c = 0; c < width; ++c) {
    for (int r = 0; r < height; ++r) {
      f[static_cast<std::size_t>(r)] = temp[static_cast<std::size_t>(r * width + c)];
    }
    const auto col = edt_1d(f, height);
    for (int r = 0; r < height; ++r) {
      dt[static_cast<std::size_t>(r * width + c)] =
        std::sqrt(col[static_cast<std::size_t>(r)]) * resolution;
    }
  }

  return dt;
}

class SimulatorCore {
public:
  SimulatorCore(
    const Params & params, int num_agents, unsigned int seed, double time_step,
    int ego_idx, int integrator, double lidar_dist, int num_beams, double fov)
  : params_(params),
    num_agents_(num_agents),
    seed_(seed),
    time_step_(time_step),
    ego_idx_(ego_idx),
    integrator_(integrator),
    lidar_dist_(lidar_dist),
    num_beams_(num_beams),
    fov_(fov),
    angle_increment_(fov / static_cast<double>(num_beams - 1)),
    theta_index_increment_(theta_dis_ * angle_increment_ / (2.0 * kPi)),
    agent_poses_(static_cast<std::size_t>(num_agents)),
    collisions_(static_cast<std::size_t>(num_agents), 0.0),
    collision_idx_(static_cast<std::size_t>(num_agents), -1.0)
  {
    if (num_agents < 1) {
      throw std::runtime_error("num_agents must be positive");
    }
    if (ego_idx < 0 || ego_idx >= num_agents) {
      throw std::runtime_error("ego_idx is out of bounds");
    }
    agents_.reserve(static_cast<std::size_t>(num_agents));
    for (int i = 0; i < num_agents; ++i) {
      agents_.push_back(Agent{});
      agents_.back().rng.seed(seed_);
    }
    precompute_scan_tables();
  }

  void set_map_data(const std::vector<double> & dt, int height, int width, double resolution,
    double origin_x, double origin_y, double origin_theta)
  {
    map_.dt = dt;
    map_.height = height;
    map_.width = width;
    map_.resolution = resolution;
    map_.orig_x = origin_x;
    map_.orig_y = origin_y;
    map_.orig_s = std::sin(origin_theta);
    map_.orig_c = std::cos(origin_theta);
  }

  void set_map_image(const unsigned char * image, int height, int width, double resolution,
    double origin_x, double origin_y, double origin_theta)
  {
    map_.dt = distance_transform_2d(image, height, width, resolution);
    map_.height = height;
    map_.width = width;
    map_.resolution = resolution;
    map_.orig_x = origin_x;
    map_.orig_y = origin_y;
    map_.orig_s = std::sin(origin_theta);
    map_.orig_c = std::cos(origin_theta);
  }

  void update_params(const Params & params, int agent_idx)
  {
    (void)agent_idx;
    params_ = params;
    precompute_vehicle_scan_tables();
  }

  void reset(const double * poses, npy_intp rows, npy_intp cols)
  {
    if (rows != num_agents_ || cols != 3) {
      throw std::runtime_error("poses must have shape (num_agents, 3)");
    }
    std::fill(collisions_.begin(), collisions_.end(), 0.0);
    std::fill(collision_idx_.begin(), collision_idx_.end(), -1.0);
    for (int i = 0; i < num_agents_; ++i) {
      auto & agent = agents_[static_cast<std::size_t>(i)];
      agent.state.fill(0.0);
      agent.state[0] = poses[i * 3 + 0];
      agent.state[1] = poses[i * 3 + 1];
      agent.state[4] = poses[i * 3 + 2];
      agent.accel = 0.0;
      agent.steer_angle_vel = 0.0;
      agent.in_collision = false;
      agent.steer_buffer.clear();
      agent.rng.seed(seed_);
      agent_poses_[static_cast<std::size_t>(i)] = {agent.state[0], agent.state[1], agent.state[4]};
    }
  }

  PyObject * step(const double * controls, npy_intp rows, npy_intp cols)
  {
    if (rows != num_agents_ || cols != 2) {
      throw std::runtime_error("control inputs must have shape (num_agents, 2)");
    }
    std::vector<std::vector<double>> agent_scans;
    agent_scans.reserve(static_cast<std::size_t>(num_agents_));

    for (int i = 0; i < num_agents_; ++i) {
      const double raw_steer = controls[i * 2 + 0];
      const double vel = controls[i * 2 + 1];
      agent_scans.push_back(update_pose(agents_[static_cast<std::size_t>(i)], raw_steer, vel));
      const auto & state = agents_[static_cast<std::size_t>(i)].state;
      agent_poses_[static_cast<std::size_t>(i)] = {state[0], state[1], state[4]};
    }

    check_collision();

    for (int i = 0; i < num_agents_; ++i) {
      auto & agent = agents_[static_cast<std::size_t>(i)];
      check_ttc(agent, agent_scans[static_cast<std::size_t>(i)]);
      ray_cast_agents(i, agent_scans[static_cast<std::size_t>(i)]);
      if (agent.in_collision) {
        collisions_[static_cast<std::size_t>(i)] = 1.0;
      }
    }

    return build_observations(agent_scans);
  }

private:
  struct Agent {
    std::array<double, 7> state{};
    double accel = 0.0;
    double steer_angle_vel = 0.0;
    bool in_collision = false;
    std::deque<double> steer_buffer;
    std::mt19937 rng;
  };

  void precompute_scan_tables()
  {
    sines_.resize(static_cast<std::size_t>(theta_dis_));
    theta_cosines_.resize(static_cast<std::size_t>(theta_dis_));
    for (int i = 0; i < theta_dis_; ++i) {
      const double theta = static_cast<double>(i) * 2.0 * kPi / static_cast<double>(theta_dis_ - 1);
      sines_[static_cast<std::size_t>(i)] = std::sin(theta);
      theta_cosines_[static_cast<std::size_t>(i)] = std::cos(theta);
    }
    precompute_vehicle_scan_tables();
  }

  void precompute_vehicle_scan_tables()
  {
    scan_angles_.assign(static_cast<std::size_t>(num_beams_), 0.0);
    beam_cosines_.assign(static_cast<std::size_t>(num_beams_), 0.0);
    side_distances_.assign(static_cast<std::size_t>(num_beams_), 0.0);

    const double dist_sides = params_.width / 2.0;
    const double dist_fr = (params_.lf + params_.lr) / 2.0;
    for (int i = 0; i < num_beams_; ++i) {
      const double angle = -fov_ / 2.0 + i * angle_increment_;
      scan_angles_[static_cast<std::size_t>(i)] = angle;
      beam_cosines_[static_cast<std::size_t>(i)] = std::cos(angle);

      if (angle > 0.0) {
        if (angle < kPi / 2.0) {
          side_distances_[static_cast<std::size_t>(i)] =
            std::min(dist_sides / std::sin(angle), dist_fr / std::cos(angle));
        } else {
          side_distances_[static_cast<std::size_t>(i)] =
            std::min(dist_sides / std::cos(angle - kPi / 2.0), dist_fr / std::sin(angle - kPi / 2.0));
        }
      } else {
        if (angle > -kPi / 2.0) {
          side_distances_[static_cast<std::size_t>(i)] =
            std::min(dist_sides / std::sin(-angle), dist_fr / std::cos(-angle));
        } else {
          side_distances_[static_cast<std::size_t>(i)] =
            std::min(dist_sides / std::cos(-angle - kPi / 2.0), dist_fr / std::sin(-angle - kPi / 2.0));
        }
      }
    }
  }

  double distance_transform(double x, double y) const
  {
    const double x_trans = x - map_.orig_x;
    const double y_trans = y - map_.orig_y;
    const double x_rot = x_trans * map_.orig_c + y_trans * map_.orig_s;
    const double y_rot = -x_trans * map_.orig_s + y_trans * map_.orig_c;

    int r = -1;
    int c = -1;
    if (x_rot >= 0.0 && x_rot < map_.width * map_.resolution &&
      y_rot >= 0.0 && y_rot < map_.height * map_.resolution)
    {
      c = static_cast<int>(x_rot / map_.resolution);
      r = static_cast<int>(y_rot / map_.resolution);
    }

    if (r < 0 || c < 0 || r >= map_.height || c >= map_.width) {
      r = std::max(0, map_.height - 1);
      c = std::max(0, map_.width - 1);
    }
    return map_.dt[static_cast<std::size_t>(r * map_.width + c)];
  }

  double trace_ray(double x, double y, double theta_index) const
  {
    int theta_index_i = static_cast<int>(theta_index);
    if (theta_index_i < 0) {
      theta_index_i = 0;
    } else if (theta_index_i >= theta_dis_) {
      theta_index_i = theta_dis_ - 1;
    }
    const double s = sines_[static_cast<std::size_t>(theta_index_i)];
    const double c = theta_cosines_[static_cast<std::size_t>(theta_index_i)];
    double dist_to_nearest = distance_transform(x, y);
    double total_dist = dist_to_nearest;

    while (dist_to_nearest > eps_ && total_dist <= max_range_) {
      x += dist_to_nearest * c;
      y += dist_to_nearest * s;
      dist_to_nearest = distance_transform(x, y);
      total_dist += dist_to_nearest;
    }

    return total_dist > max_range_ ? max_range_ : total_dist;
  }

  std::vector<double> get_scan(const std::array<double, 3> & pose, Agent & agent)
  {
    if (map_.height <= 0 || map_.width <= 0 || map_.dt.empty()) {
      throw std::runtime_error("map is not set for C++ simulator");
    }

    std::vector<double> scan(static_cast<std::size_t>(num_beams_));
    double theta_index = theta_dis_ * (pose[2] - fov_ / 2.0) / (2.0 * kPi);
    theta_index = std::fmod(theta_index, theta_dis_);
    while (theta_index < 0.0) {
      theta_index += theta_dis_;
    }

    for (int i = 0; i < num_beams_; ++i) {
      scan[static_cast<std::size_t>(i)] = trace_ray(pose[0], pose[1], theta_index);
      theta_index += theta_index_increment_;
      while (theta_index >= theta_dis_) {
        theta_index -= theta_dis_;
      }
    }

    std::normal_distribution<double> noise(0.0, scan_noise_std_);
    for (auto & value : scan) {
      value += noise(agent.rng);
    }
    return scan;
  }

  std::vector<double> update_pose(Agent & agent, double raw_steer, double vel)
  {
    double steer = 0.0;
    if (agent.steer_buffer.size() < steer_buffer_size_) {
      agent.steer_buffer.push_front(raw_steer);
    } else {
      steer = agent.steer_buffer.back();
      agent.steer_buffer.pop_back();
      agent.steer_buffer.push_front(raw_steer);
    }

    const auto control = pid(
      vel, steer, agent.state[3], agent.state[2],
      params_.sv_max, params_.a_max, params_.v_max, params_.v_min);
    const double accl = control[0];
    const double sv = control[1];

    if (integrator_ == 1) {
      const auto k1 = vehicle_dynamics_st(agent.state, sv, accl, params_);
      auto k2_state = agent.state;
      for (int i = 0; i < 7; ++i) {
        k2_state[static_cast<std::size_t>(i)] += time_step_ * (k1[static_cast<std::size_t>(i)] / 2.0);
      }
      const auto k2 = vehicle_dynamics_st(k2_state, sv, accl, params_);
      auto k3_state = agent.state;
      for (int i = 0; i < 7; ++i) {
        k3_state[static_cast<std::size_t>(i)] += time_step_ * (k2[static_cast<std::size_t>(i)] / 2.0);
      }
      const auto k3 = vehicle_dynamics_st(k3_state, sv, accl, params_);
      auto k4_state = agent.state;
      for (int i = 0; i < 7; ++i) {
        k4_state[static_cast<std::size_t>(i)] += time_step_ * k3[static_cast<std::size_t>(i)];
      }
      const auto k4 = vehicle_dynamics_st(k4_state, sv, accl, params_);
      for (int i = 0; i < 7; ++i) {
        agent.state[static_cast<std::size_t>(i)] +=
          time_step_ * (1.0 / 6.0) *
          (k1[static_cast<std::size_t>(i)] + 2.0 * k2[static_cast<std::size_t>(i)] +
          2.0 * k3[static_cast<std::size_t>(i)] + k4[static_cast<std::size_t>(i)]);
      }
    } else if (integrator_ == 2) {
      const auto f = vehicle_dynamics_st(agent.state, sv, accl, params_);
      for (int i = 0; i < 7; ++i) {
        agent.state[static_cast<std::size_t>(i)] += time_step_ * f[static_cast<std::size_t>(i)];
      }
    } else {
      throw std::runtime_error("invalid integrator, expected 1 for RK4 or 2 for Euler");
    }

    if (agent.state[4] > 2.0 * kPi) {
      agent.state[4] -= 2.0 * kPi;
    } else if (agent.state[4] < 0.0) {
      agent.state[4] += 2.0 * kPi;
    }

    const double scan_x = agent.state[0] + lidar_dist_ * std::cos(agent.state[4]);
    const double scan_y = agent.state[1] + lidar_dist_ * std::sin(agent.state[4]);
    return get_scan({scan_x, scan_y, agent.state[4]}, agent);
  }

  void check_collision()
  {
    std::fill(collisions_.begin(), collisions_.end(), 0.0);
    std::fill(collision_idx_.begin(), collision_idx_.end(), -1.0);

    std::vector<std::array<std::array<double, 2>, 4>> all_vertices;
    all_vertices.reserve(static_cast<std::size_t>(num_agents_));
    for (const auto & agent : agents_) {
      all_vertices.push_back(get_vertices({agent.state[0], agent.state[1], agent.state[4]}, params_.length, params_.width));
    }

    for (int i = 0; i < num_agents_ - 1; ++i) {
      for (int j = i + 1; j < num_agents_; ++j) {
        if (collision(all_vertices[static_cast<std::size_t>(i)], all_vertices[static_cast<std::size_t>(j)])) {
          collisions_[static_cast<std::size_t>(i)] = 1.0;
          collisions_[static_cast<std::size_t>(j)] = 1.0;
          collision_idx_[static_cast<std::size_t>(i)] = static_cast<double>(j);
          collision_idx_[static_cast<std::size_t>(j)] = static_cast<double>(i);
        }
      }
    }
  }

  void check_ttc(Agent & agent, const std::vector<double> & scan)
  {
    bool in_collision = false;
    if (agent.state[3] != 0.0) {
      for (int i = 0; i < num_beams_; ++i) {
        const double proj_vel = agent.state[3] * beam_cosines_[static_cast<std::size_t>(i)];
        if (proj_vel == 0.0) {
          continue;
        }
        const double ttc =
          (scan[static_cast<std::size_t>(i)] - side_distances_[static_cast<std::size_t>(i)]) / proj_vel;
        if (ttc < ttc_thresh_ && ttc >= 0.0) {
          in_collision = true;
          break;
        }
      }
    }

    if (in_collision) {
      for (int i = 3; i < 7; ++i) {
        agent.state[static_cast<std::size_t>(i)] = 0.0;
      }
      agent.accel = 0.0;
      agent.steer_angle_vel = 0.0;
    }
    agent.in_collision = in_collision;
  }

  std::pair<int, int> get_blocked_view_indices(
    const std::array<double, 3> & pose,
    const std::array<std::array<double, 2>, 4> & vertices) const
  {
    int min_ind = num_beams_;
    int max_ind = 0;
    const double ego_angle = std::atan2(std::sin(pose[2]), std::cos(pose[2]));

    for (int i = 0; i < 4; ++i) {
      const double dx = vertices[static_cast<std::size_t>(i)][0] - pose[0];
      const double dy = vertices[static_cast<std::size_t>(i)][1] - pose[1];
      const double norm = std::hypot(dx, dy);
      double angle = ego_angle - std::atan2(dy / norm, dx / norm);
      if (angle > kPi) {
        angle -= 2.0 * kPi;
      } else if (angle < -kPi) {
        angle += 2.0 * kPi;
      }
      const double angle_with_x = -angle;

      int best = 0;
      double best_abs = std::fabs(scan_angles_[0] - angle_with_x);
      for (int j = 1; j < num_beams_; ++j) {
        const double candidate = std::fabs(scan_angles_[static_cast<std::size_t>(j)] - angle_with_x);
        if (candidate < best_abs) {
          best_abs = candidate;
          best = j;
        }
      }
      min_ind = std::min(min_ind, best);
      max_ind = std::max(max_ind, best);
    }
    return {min_ind, max_ind};
  }

  void ray_cast_agents(int agent_index, std::vector<double> & scan)
  {
    const auto & agent = agents_[static_cast<std::size_t>(agent_index)];
    const std::array<double, 3> pose{agent.state[0], agent.state[1], agent.state[4]};
    for (int i = 0; i < num_agents_; ++i) {
      if (i == agent_index) {
        continue;
      }
      const auto vertices = get_vertices(agent_poses_[static_cast<std::size_t>(i)], params_.length, params_.width);
      const auto blocked = get_blocked_view_indices(pose, vertices);
      std::array<std::array<double, 2>, 5> looped{};
      for (int j = 0; j < 4; ++j) {
        looped[static_cast<std::size_t>(j)] = vertices[static_cast<std::size_t>(j)];
      }
      looped[4] = vertices[0];

      for (int beam = blocked.first; beam <= blocked.second; ++beam) {
        for (int edge = 0; edge < 4; ++edge) {
          const double scan_range = get_range(
            pose, pose[2] + scan_angles_[static_cast<std::size_t>(beam)],
            looped[static_cast<std::size_t>(edge)], looped[static_cast<std::size_t>(edge + 1)]);
          if (scan_range < scan[static_cast<std::size_t>(beam)]) {
            scan[static_cast<std::size_t>(beam)] = scan_range;
          }
        }
      }
    }
  }

  PyObject * make_float_list(const std::vector<double> & values) const
  {
    PyObject * list = PyList_New(static_cast<Py_ssize_t>(values.size()));
    if (!list) {
      return nullptr;
    }
    for (std::size_t i = 0; i < values.size(); ++i) {
      PyList_SET_ITEM(list, static_cast<Py_ssize_t>(i), PyFloat_FromDouble(values[i]));
    }
    return list;
  }

  PyObject * build_observations(const std::vector<std::vector<double>> & agent_scans) const
  {
    PyObject * dict = PyDict_New();
    if (!dict) {
      return nullptr;
    }
    PyObject * ego_idx = PyLong_FromLong(ego_idx_);
    if (!ego_idx) {
      Py_DECREF(dict);
      return nullptr;
    }
    PyDict_SetItemString(dict, "ego_idx", ego_idx);
    Py_DECREF(ego_idx);

    PyObject * scans = PyList_New(num_agents_);
    PyObject * poses_x = PyList_New(num_agents_);
    PyObject * poses_y = PyList_New(num_agents_);
    PyObject * poses_theta = PyList_New(num_agents_);
    PyObject * linear_vels_x = PyList_New(num_agents_);
    PyObject * linear_vels_y = PyList_New(num_agents_);
    PyObject * ang_vels_z = PyList_New(num_agents_);
    if (!scans || !poses_x || !poses_y || !poses_theta || !linear_vels_x || !linear_vels_y || !ang_vels_z) {
      Py_XDECREF(scans);
      Py_XDECREF(poses_x);
      Py_XDECREF(poses_y);
      Py_XDECREF(poses_theta);
      Py_XDECREF(linear_vels_x);
      Py_XDECREF(linear_vels_y);
      Py_XDECREF(ang_vels_z);
      Py_DECREF(dict);
      return nullptr;
    }

    for (int i = 0; i < num_agents_; ++i) {
      npy_intp dims[1] = {num_beams_};
      PyObject * scan_array = PyArray_SimpleNew(1, dims, NPY_DOUBLE);
      if (!scan_array) {
        Py_DECREF(dict);
        return nullptr;
      }
      auto * data = static_cast<double *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(scan_array)));
      const auto & scan = agent_scans[static_cast<std::size_t>(i)];
      std::copy(scan.begin(), scan.end(), data);
      PyList_SET_ITEM(scans, i, scan_array);

      const auto & state = agents_[static_cast<std::size_t>(i)].state;
      PyList_SET_ITEM(poses_x, i, PyFloat_FromDouble(state[0]));
      PyList_SET_ITEM(poses_y, i, PyFloat_FromDouble(state[1]));
      PyList_SET_ITEM(poses_theta, i, PyFloat_FromDouble(state[4]));
      PyList_SET_ITEM(linear_vels_x, i, PyFloat_FromDouble(state[3]));
      PyList_SET_ITEM(linear_vels_y, i, PyFloat_FromDouble(0.0));
      PyList_SET_ITEM(ang_vels_z, i, PyFloat_FromDouble(state[5]));
    }

    npy_intp collision_dims[1] = {num_agents_};
    PyObject * collisions = PyArray_SimpleNew(1, collision_dims, NPY_DOUBLE);
    if (!collisions) {
      Py_DECREF(dict);
      return nullptr;
    }
    auto * collision_data = static_cast<double *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(collisions)));
    std::copy(collisions_.begin(), collisions_.end(), collision_data);

    PyDict_SetItemString(dict, "scans", scans);
    PyDict_SetItemString(dict, "poses_x", poses_x);
    PyDict_SetItemString(dict, "poses_y", poses_y);
    PyDict_SetItemString(dict, "poses_theta", poses_theta);
    PyDict_SetItemString(dict, "linear_vels_x", linear_vels_x);
    PyDict_SetItemString(dict, "linear_vels_y", linear_vels_y);
    PyDict_SetItemString(dict, "ang_vels_z", ang_vels_z);
    PyDict_SetItemString(dict, "collisions", collisions);

    Py_DECREF(scans);
    Py_DECREF(poses_x);
    Py_DECREF(poses_y);
    Py_DECREF(poses_theta);
    Py_DECREF(linear_vels_x);
    Py_DECREF(linear_vels_y);
    Py_DECREF(ang_vels_z);
    Py_DECREF(collisions);
    return dict;
  }

  Params params_;
  int num_agents_;
  unsigned int seed_;
  double time_step_;
  int ego_idx_;
  int integrator_;
  double lidar_dist_;
  int num_beams_;
  double fov_;
  int theta_dis_ = 2000;
  double eps_ = 0.0001;
  double max_range_ = 30.0;
  double angle_increment_;
  double theta_index_increment_;
  double scan_noise_std_ = 0.01;
  double ttc_thresh_ = 0.005;
  std::size_t steer_buffer_size_ = 2;

  MapData map_;
  std::vector<double> sines_;
  std::vector<double> theta_cosines_;
  std::vector<double> scan_angles_;
  std::vector<double> beam_cosines_;
  std::vector<double> side_distances_;
  std::vector<Agent> agents_;
  std::vector<std::array<double, 3>> agent_poses_;
  std::vector<double> collisions_;
  std::vector<double> collision_idx_;
};

SimulatorCore * core_from_capsule(PyObject * capsule)
{
  auto * core = static_cast<SimulatorCore *>(PyCapsule_GetPointer(capsule, kCapsuleName));
  if (!core) {
    throw std::runtime_error("invalid C++ simulator capsule");
  }
  return core;
}

void capsule_destructor(PyObject * capsule)
{
  auto * core = static_cast<SimulatorCore *>(PyCapsule_GetPointer(capsule, kCapsuleName));
  delete core;
}

PyObject * py_create(PyObject *, PyObject * args)
{
  try {
    if (PyTuple_GET_SIZE(args) != 9) {
      PyErr_SetString(PyExc_TypeError, "create expects 9 arguments");
      return nullptr;
    }
    PyObject * params_obj = PyTuple_GET_ITEM(args, 0);
    const int num_agents = static_cast<int>(PyLong_AsLong(PyTuple_GET_ITEM(args, 1)));
    const unsigned int seed = static_cast<unsigned int>(PyLong_AsUnsignedLong(PyTuple_GET_ITEM(args, 2)));
    const double time_step = PyFloat_AsDouble(PyTuple_GET_ITEM(args, 3));
    const int ego_idx = static_cast<int>(PyLong_AsLong(PyTuple_GET_ITEM(args, 4)));
    const int integrator = static_cast<int>(PyLong_AsLong(PyTuple_GET_ITEM(args, 5)));
    const double lidar_dist = PyFloat_AsDouble(PyTuple_GET_ITEM(args, 6));
    const int num_beams = static_cast<int>(PyLong_AsLong(PyTuple_GET_ITEM(args, 7)));
    const double fov = PyFloat_AsDouble(PyTuple_GET_ITEM(args, 8));
    if (PyErr_Occurred()) {
      return nullptr;
    }

    auto core = std::make_unique<SimulatorCore>(
      parse_params(params_obj), num_agents, seed, time_step, ego_idx, integrator, lidar_dist, num_beams, fov);
    return PyCapsule_New(core.release(), kCapsuleName, capsule_destructor);
  } catch (const std::exception & ex) {
    PyErr_SetString(PyExc_RuntimeError, ex.what());
    return nullptr;
  }
}

PyObject * py_set_map_data(PyObject *, PyObject * args)
{
  PyObject * capsule = nullptr;
  PyObject * dt_obj = nullptr;
  double resolution = 0.0;
  PyObject * origin_obj = nullptr;
  if (!PyArg_ParseTuple(args, "OOdO", &capsule, &dt_obj, &resolution, &origin_obj)) {
    return nullptr;
  }

  PyObject * dt_array_obj = PyArray_FROM_OTF(dt_obj, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY);
  if (!dt_array_obj) {
    return nullptr;
  }

  try {
    auto * dt_array = reinterpret_cast<PyArrayObject *>(dt_array_obj);
    if (PyArray_NDIM(dt_array) != 2) {
      throw std::runtime_error("distance transform must be a 2D float64 array");
    }
    const int height = static_cast<int>(PyArray_DIM(dt_array, 0));
    const int width = static_cast<int>(PyArray_DIM(dt_array, 1));
    const auto * data = static_cast<const double *>(PyArray_DATA(dt_array));
    std::vector<double> dt(data, data + static_cast<std::size_t>(height * width));

    if (!PySequence_Check(origin_obj) || PySequence_Size(origin_obj) < 3) {
      throw std::runtime_error("map origin must be a sequence with at least 3 values");
    }
    PyObject * ox = PySequence_GetItem(origin_obj, 0);
    PyObject * oy = PySequence_GetItem(origin_obj, 1);
    PyObject * ot = PySequence_GetItem(origin_obj, 2);
    const double origin_x = PyFloat_AsDouble(ox);
    const double origin_y = PyFloat_AsDouble(oy);
    const double origin_theta = PyFloat_AsDouble(ot);
    Py_XDECREF(ox);
    Py_XDECREF(oy);
    Py_XDECREF(ot);
    if (PyErr_Occurred()) {
      Py_DECREF(dt_array_obj);
      return nullptr;
    }

    core_from_capsule(capsule)->set_map_data(dt, height, width, resolution, origin_x, origin_y, origin_theta);
    Py_DECREF(dt_array_obj);
    Py_RETURN_TRUE;
  } catch (const std::exception & ex) {
    Py_DECREF(dt_array_obj);
    PyErr_SetString(PyExc_RuntimeError, ex.what());
    return nullptr;
  }
}

PyObject * py_set_map_image(PyObject *, PyObject * args)
{
  PyObject * capsule = nullptr;
  PyObject * image_obj = nullptr;
  double resolution = 0.0;
  PyObject * origin_obj = nullptr;
  if (!PyArg_ParseTuple(args, "OOdO", &capsule, &image_obj, &resolution, &origin_obj)) {
    return nullptr;
  }

  PyObject * image_array_obj = PyArray_FROM_OTF(image_obj, NPY_UINT8, NPY_ARRAY_IN_ARRAY);
  if (!image_array_obj) {
    return nullptr;
  }

  try {
    auto * image_array = reinterpret_cast<PyArrayObject *>(image_array_obj);
    if (PyArray_NDIM(image_array) != 2) {
      throw std::runtime_error("map image must be a 2D uint8 array");
    }
    const int height = static_cast<int>(PyArray_DIM(image_array, 0));
    const int width = static_cast<int>(PyArray_DIM(image_array, 1));

    if (!PySequence_Check(origin_obj) || PySequence_Size(origin_obj) < 3) {
      throw std::runtime_error("map origin must be a sequence with at least 3 values");
    }
    PyObject * ox = PySequence_GetItem(origin_obj, 0);
    PyObject * oy = PySequence_GetItem(origin_obj, 1);
    PyObject * ot = PySequence_GetItem(origin_obj, 2);
    const double origin_x = PyFloat_AsDouble(ox);
    const double origin_y = PyFloat_AsDouble(oy);
    const double origin_theta = PyFloat_AsDouble(ot);
    Py_XDECREF(ox);
    Py_XDECREF(oy);
    Py_XDECREF(ot);
    if (PyErr_Occurred()) {
      Py_DECREF(image_array_obj);
      return nullptr;
    }

    core_from_capsule(capsule)->set_map_image(
      static_cast<const unsigned char *>(PyArray_DATA(image_array)),
      height,
      width,
      resolution,
      origin_x,
      origin_y,
      origin_theta);
    Py_DECREF(image_array_obj);
    Py_RETURN_TRUE;
  } catch (const std::exception & ex) {
    Py_DECREF(image_array_obj);
    PyErr_SetString(PyExc_RuntimeError, ex.what());
    return nullptr;
  }
}

PyObject * py_update_params(PyObject *, PyObject * args)
{
  PyObject * capsule = nullptr;
  PyObject * params_obj = nullptr;
  int agent_idx = -1;
  if (!PyArg_ParseTuple(args, "OOi", &capsule, &params_obj, &agent_idx)) {
    return nullptr;
  }
  try {
    core_from_capsule(capsule)->update_params(parse_params(params_obj), agent_idx);
    Py_RETURN_NONE;
  } catch (const std::exception & ex) {
    PyErr_SetString(PyExc_RuntimeError, ex.what());
    return nullptr;
  }
}

PyObject * py_reset(PyObject *, PyObject * args)
{
  PyObject * capsule = nullptr;
  PyObject * poses_obj = nullptr;
  if (!PyArg_ParseTuple(args, "OO", &capsule, &poses_obj)) {
    return nullptr;
  }
  PyObject * poses_array_obj = PyArray_FROM_OTF(poses_obj, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY);
  if (!poses_array_obj) {
    return nullptr;
  }
  try {
    auto * poses_array = reinterpret_cast<PyArrayObject *>(poses_array_obj);
    if (PyArray_NDIM(poses_array) != 2) {
      throw std::runtime_error("poses must be a 2D float64 array");
    }
    core_from_capsule(capsule)->reset(
      static_cast<const double *>(PyArray_DATA(poses_array)),
      PyArray_DIM(poses_array, 0),
      PyArray_DIM(poses_array, 1));
    Py_DECREF(poses_array_obj);
    Py_RETURN_NONE;
  } catch (const std::exception & ex) {
    Py_DECREF(poses_array_obj);
    PyErr_SetString(PyExc_RuntimeError, ex.what());
    return nullptr;
  }
}

PyObject * py_step(PyObject *, PyObject * args)
{
  PyObject * capsule = nullptr;
  PyObject * controls_obj = nullptr;
  if (!PyArg_ParseTuple(args, "OO", &capsule, &controls_obj)) {
    return nullptr;
  }
  PyObject * controls_array_obj = PyArray_FROM_OTF(controls_obj, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY);
  if (!controls_array_obj) {
    return nullptr;
  }
  try {
    auto * controls_array = reinterpret_cast<PyArrayObject *>(controls_array_obj);
    if (PyArray_NDIM(controls_array) != 2) {
      throw std::runtime_error("control inputs must be a 2D float64 array");
    }
    PyObject * result = core_from_capsule(capsule)->step(
      static_cast<const double *>(PyArray_DATA(controls_array)),
      PyArray_DIM(controls_array, 0),
      PyArray_DIM(controls_array, 1));
    Py_DECREF(controls_array_obj);
    return result;
  } catch (const std::exception & ex) {
    Py_DECREF(controls_array_obj);
    PyErr_SetString(PyExc_RuntimeError, ex.what());
    return nullptr;
  }
}

PyMethodDef methods[] = {
  {"create", py_create, METH_VARARGS, "Create a C++ F110 simulator core."},
  {"set_map_data", py_set_map_data, METH_VARARGS, "Set map distance transform data."},
  {"set_map_image", py_set_map_image, METH_VARARGS, "Set map image data and compute distance transform in C++."},
  {"update_params", py_update_params, METH_VARARGS, "Update vehicle parameters."},
  {"reset", py_reset, METH_VARARGS, "Reset simulator poses."},
  {"step", py_step, METH_VARARGS, "Step simulator controls."},
  {nullptr, nullptr, 0, nullptr}
};

PyModuleDef module = {
  PyModuleDef_HEAD_INIT,
  "_cpp_backend",
  "C++ backend for f110_gym.",
  -1,
  methods,
};

}  // namespace

PyMODINIT_FUNC PyInit__cpp_backend()
{
  import_array();
  return PyModule_Create(&module);
}
