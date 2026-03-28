#include "extended_kalman_filter.hpp"

#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  Eigen::VectorXd x_prior = x;
  Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

  x = x_add(x, K * z_subtract(z, h(x)));

  /// 卡方检验
  Eigen::VectorXd residual = z_subtract(z, h(x));
  // 新增检验
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  double nis = residual.transpose() * S.inverse() * residual;
  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

  // 卡方检验阈值（自由度=4，取置信水平95%）
  constexpr double nis_threshold = 0.711;
  constexpr double nees_threshold = 0.711;

  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  data["residual_yaw"] = residual[0];
  data["residual_pitch"] = residual[1];
  data["residual_distance"] = residual[2];
  data["residual_angle"] = residual[3];
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

/************************************************************************************************************************
 * @brief   EKF 类构造函数
 ***********************************************************************************************************************/
 RWExtendedKalmanFilter::RWExtendedKalmanFilter(
  const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
  const VoidMatFunc & u_q, const VoidMatFunc & u_r, const Eigen::MatrixXd & P0)
  : f(f), h(h), jacobian_f(j_f), jacobian_h(j_h), update_Q(u_q), update_R(u_r),
    n(P0.rows()), I(Eigen::MatrixXd::Identity(n, n)), P_post(P0), x_post(n)
  {}
  
  /************************************************************************************************************************
   * @brief   EKF 预测
   * @return  VectorXd	预测后状态向量
   ***********************************************************************************************************************/
  Eigen::VectorXd RWExtendedKalmanFilter::predict()
  {
    F = jacobian_f(x_post), Q = update_Q();
  
    x_pri = f(x_post);
    P_pri = F * P_post * F.transpose() + Q;
  
    // 处理只有预测没有更新情况
    x_post = x_pri;
    P_post = P_pri;
  
    return x_pri;
  }

  [[deprecated("use predict instead")]]Eigen::VectorXd RWExtendedKalmanFilter::fake_predict()
  {
    auto temp_F = jacobian_f(x_post), temp_Q = update_Q();
  
    auto temp_x_pri = f(x_post);
  
    return temp_x_pri;
  }
  
  /************************************************************************************************************************
   * @brief   EKF 更新
   * @param   z			测量向量
   * @return  VectorXd	更新后状态向量
   ***********************************************************************************************************************/
  Eigen::VectorXd RWExtendedKalmanFilter::update(const Eigen::VectorXd & z)
  {
    H = jacobian_h(x_pri), R = update_R();
  
    K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
    x_post = x_pri + K * (z - h(x_pri));
  
    P_post = (I - K * H) * P_pri;
    return x_post;
  }
  
  /************************************************************************************************************************
   * @brief   EKF 设置当前状态量
   * @param   x0	设置的状态向量
   ***********************************************************************************************************************/
  void RWExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }
  
  /************************************************************************************************************************
   * @brief   EKF 重新初始化P矩阵
   ***********************************************************************************************************************/
  void RWExtendedKalmanFilter::reinitP() { P_post = I; }
  
  /************************************************************************************************************************
   * @brief   EKF 获取当前状态量
   * @return  VectorXd	当前状态向量
   ***********************************************************************************************************************/
  Eigen::VectorXd RWExtendedKalmanFilter::getState() const { return x_post; }

KalmanFilter::KalmanFilter(
    const Eigen::VectorXd& x0,
    const Eigen::MatrixXd& P0,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& Q
) : x(x0), P(P0), F(F), Q(Q), R(R), K(Eigen::MatrixXd::Zero(x0.rows(), H.rows())) {}

KalmanFilter::KalmanFilter(
  const double& x0,
  const double& P0,
  const double& H_val,
  const double& R_val,
  const double& F_val, 
  const double& Q_val
) {
  // 初始化向量（1维）
  x = Eigen::VectorXd::Constant(1, x0);
  
  // 初始化矩阵（1x1）
  P = Eigen::MatrixXd::Constant(1, 1, P0);
  F = Eigen::MatrixXd::Constant(1, 1, F_val);
  Q = Eigen::MatrixXd::Constant(1, 1, Q_val);
  R = Eigen::MatrixXd::Constant(1, 1, R_val);
  H = Eigen::MatrixXd::Constant(1, 1, H_val);
  K = Eigen::MatrixXd::Zero(1, 1);
  I = Eigen::MatrixXd::Identity(1, 1);
}

Eigen::VectorXd KalmanFilter::predict() {
    x = F * x;
    P = F * P * F.transpose() + Q;
    return x;
}

Eigen::VectorXd KalmanFilter::update(const Eigen::VectorXd& z) {
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x = x + K * (z - H * x);
    P = (I - K * H) * P;
    return x;
}
}  // namespace tools