#include "target.hpp"

#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, int armor_num,
  std::unique_ptr<FilterStrategy> filter)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  is_switch_(false),
  is_converged_(false),
  switch_count_(0),
  priority(armor.priority),
  armor_num_(armor_num),
  filter_(std::move(filter)),
  isinit(true)
{}

EKFStrategy::EKFStrategy(
    std::chrono::steady_clock::time_point t,
    const Eigen::VectorXd & x0, 
    const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
    : ekf_(x0,P0, x_add),
      t_(t)
{}

EKFStrategy::EKFStrategy(
    const Eigen::VectorXd & x0, 
    const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
    : ekf_(x0,P0, x_add)
{}

Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[state::a] = tools::limit_rad(c[state::a]);
    return c;
  };

  filter_ = std::make_unique<EKFStrategy>(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

Target::Target(double x,double direction, double v, double vyaw, double radius, double h) : armor_num_(4)
{
  Eigen::VectorXd x0{{x, v*std::cos(direction), 0, v*std::sin(direction), 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  filter_ = std::make_unique<EKFStrategy>(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void EKFStrategy::predict(std::chrono::steady_clock::time_point t, Target& target)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt,target);
  t_ = t;
}

void EKFStrategy::predict(double dt, Target& target)
{
  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;
  if (target.name == ArmorName::outpost) {
    v1 = 10;   // 前哨站加速度方差
    v2 = 0.1;  // 前哨站角加速度方差
  } else {
    v1 = 100;  // 加速度方差
    v2 = 400;  // 角加速度方差
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (target.convergened() && target.name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  ekf_.predict(F, Q, f);
  // ekf_.predict(F, Eigen::MatrixXd::Zero(11,11), f);
}

void Target::update(const Armor & armor)
{
  // 装甲板匹配
  int id;
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });

  // 取前3个distance最小的装甲板
  for (int i = 0; i < 3; i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  if (id != 0) jumped = true;

  if (id != last_id) {
    is_switch_ = true;
  } else {
    is_switch_ = false;
  }

  if (is_switch_) switch_count_++;

  last_id = id;
  update_count_++;

  filter_->update_ypda(armor, id, *this);
}

void EKFStrategy::update_ypda(const Armor & armor, int id, Target& target)
{
  //观测jacobi
  Eigen::MatrixXd H = h_jacobian(target, id);
  // Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  // Eigen::VectorXd R_dig{
  //   {4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
  //    log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};
    
  Eigen::VectorXd R_dig_fin{
    {target.R_dig[0], target.R_dig[1], 
      target.R_dig[2]*std::pow(armor.ypd_in_world[2],4),//* delta_angle是装甲板相对自身中心的转角
      target.R_dig[3]/(armor.ratio+ 0.1)
      //* ratio看作灯条长度不变，width大致为cos(delta_angle)这样lenth/width
      //* 1/cos正好是在(0,pie/2)上升非常陡的
     }};
      //TODO 这里距离用装甲板和中心的夹角作为不确定性
      //TODO 装甲板姿态用距离衡量不确定性，一个四次方，一个长度比例，不然他可能是开环的
      //TODO 只能说有点道理，但准备先用上交的不确定性，之后测试

  //测量过程噪声偏差的方差
  Eigen::MatrixXd R = R_dig_fin.asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = target.h_armor_xyz(id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / target.armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };  

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  //获得观测量
  //* ypd[0]是在odom下的位置方向信息，ypr[0]是装甲板的朝向

  ekf_.update(z, H, R, h, z_subtract);
}

const Eigen::VectorXd& Target::get_state() const { return filter_->get_state(); }
const Eigen::VectorXd& EKFStrategy::get_state() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & EKFStrategy::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(get_state()[state::a] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  const auto& filter_state = get_state();
  auto r_ok = filter_state[state::r] > 0.05 && filter_state[state::r] < 0.5;
  auto l_ok = filter_state[state::r] + filter_state[state::l] > 0.05 && filter_state[state::r] + filter_state[state::l] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", filter_state[state::r], filter_state[state::l]);
  return true;
}

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz( int id) const
{
  const auto& filter_state = get_state();
  auto angle = tools::limit_rad(filter_state[state::a] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? filter_state[state::r] + filter_state[state::l] : filter_state[state::r];
  auto armor_x = filter_state[state::x] - r * std::cos(angle);
  auto armor_y = filter_state[state::y] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? filter_state[state::z] + filter_state[state::h] : filter_state[state::z];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd EKFStrategy::h_jacobian(Target& target, int id) const
{
  const auto& state = ekf_.x;
  auto angle = tools::limit_rad(state[Target::state::a] + id * 2 * CV_PI / target.armor_num_);
  auto use_l_h = (target.armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? state[Target::state::r] + state[Target::state::l] : state[Target::state::r];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = target.h_armor_xyz(id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

bool EKFStrategy::has_bad_convergence() const
{
  const auto& ekf_instance = this->ekf();
  
  // 计算近期 NIS 失败的总数
  int recent_failures = std::accumulate(
    ekf_instance.recent_nis_failures.begin(), ekf_instance.recent_nis_failures.end(), 0);

  // 判断失败率是否超过阈值
  return recent_failures >= (0.4 * ekf_instance.window_size);
}

bool Target:: has_bad_convergence() const
{
  if (filter_) {
    return filter_->has_bad_convergence();
  }
  return false;
}

void Target::predict(const double dt)
{
  filter_->predict(dt, *this);
}
void Target::predict(std::chrono::steady_clock::time_point t)
{
  filter_->predict(t, *this);
}

std::unique_ptr<FilterStrategy> EKFStrategy::clone() const
{
 return std::make_unique<EKFStrategy>(*this);
}

Target::Target(const Target& other)
  : name(other.name),
    armor_type(other.armor_type),
    priority(other.priority),
    jumped(other.jumped),
    last_id(other.last_id),
    isinit(other.isinit),
    armor_num_(other.armor_num_),
    switch_count_(other.switch_count_),
    update_count_(other.update_count_),
    is_switch_(other.is_switch_),
    is_converged_(other.is_converged_)
{
  if (other.filter_) {
    filter_ = other.filter_->clone();
  }
}

Target& Target::operator=(const Target& other) 
{
  // 1. 防止自我赋值 (a = a;)
  if (this == &other) {
    return *this;
  }

  // 2. 复制所有普通成员
  name = other.name;
  armor_type = other.armor_type;
  priority = other.priority;
  jumped = other.jumped;
  last_id = other.last_id;
  isinit = other.isinit;
  armor_num_ = other.armor_num_;
  switch_count_ = other.switch_count_;
  update_count_ = other.update_count_;
  is_switch_ = other.is_switch_;
  is_converged_ = other.is_converged_;
  // ... 其他所有成员 ...

  // 3. 处理 unique_ptr (最危险的部分)
  if (other.filter_) {
    // 克隆对方的 filter，这可能会抛出异常 (比如内存不足)
    filter_ = other.filter_->clone(); 
  } else {
    filter_.reset(); // 如果对方没有，我也要释放掉我自己的
  }

  return *this;
}
}  // namespace auto_aim
