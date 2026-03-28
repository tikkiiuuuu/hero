#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>

namespace tools
{
class ExtendedKalmanFilter
{
public:
  Eigen::VectorXd x;
  Eigen::MatrixXd P;

  ExtendedKalmanFilter() = default;

  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; });

  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q);

  Eigen::VectorXd predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f);

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, 
    const Eigen::MatrixXd & H, 
    const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function
      <Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> 
        z_subtract =
          [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
            return a - b; });

  std::map<std::string, double> data;  //卡方检验数据
  std::deque<int> recent_nis_failures{0};
  size_t window_size = 100;
  double last_nis;

private:
  Eigen::MatrixXd I;
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add;

  int nees_count_ = 0;
  int nis_count_ = 0;
  int total_count_ = 0;
};

class RWExtendedKalmanFilter {
public:
    using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
    using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;
    using VoidMatFunc = std::function<Eigen::MatrixXd()>;

    /* ---------------------------------------- 函数 --------------------------------------------- */

    // 构造函数
    RWExtendedKalmanFilter() = default;
    explicit RWExtendedKalmanFilter(
        const VecVecFunc& f,
        const VecVecFunc& h,
        const VecMatFunc& j_f,
        const VecMatFunc& j_h,
        const VoidMatFunc& u_q,
        const VoidMatFunc& u_r,
        const Eigen::MatrixXd& P0
    );

    // 核心功能函数
    Eigen::VectorXd predict();
    Eigen::VectorXd update(const Eigen::VectorXd& z);

    // 接口函数
    [[deprecated("use predict instead")]]Eigen::VectorXd fake_predict();
    void setState(const Eigen::VectorXd& x0);
    void reinitP();
    Eigen::VectorXd getState() const;

private:
    /* ---------------------------------------- 常量 --------------------------------------------- */

    // EKF 变量生成函数
    VecVecFunc f; // Process nonlinear vector function
    VecVecFunc h; // Observation nonlinear vector function
    VecMatFunc jacobian_f; // Jacobian of f()
    VecMatFunc jacobian_h; // Jacobian of h()
    VoidMatFunc update_Q; // Process noise covariance matrix
    VoidMatFunc update_R; // Measurement noise covariance matrix

    // 初始化常量
    int n; // System dimensions
    Eigen::MatrixXd I; // N-size identity

    /* ---------------------------------------- 变量 --------------------------------------------- */

    // EKF 核心变量
    Eigen::MatrixXd F; // Jacobian of f()
    Eigen::MatrixXd H; // Jacobian of h()
    Eigen::MatrixXd Q; // Process noise covariance matrix
    Eigen::MatrixXd R; // Measurement noise covariance matrix
    Eigen::MatrixXd K; // Kalman gain

    Eigen::MatrixXd P_pri; // Priori error estimate covariance matrix
    Eigen::MatrixXd P_post; // Posteriori error estimate covariance matrix

    Eigen::VectorXd x_pri; // Priori state
    Eigen::VectorXd x_post; // Posteriori state
};

class KalmanFilter {
public:
    KalmanFilter() = default;
    KalmanFilter(
        const Eigen::VectorXd& x0,
        const Eigen::MatrixXd& P0,
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& F, 
        const Eigen::MatrixXd& Q
    );
    KalmanFilter(
        const double& x0,
        const double& P0,
        const double& H,
        const double& R,
        const double& F, 
        const double& Q
    );
    Eigen::VectorXd predict();
    Eigen::VectorXd update(const Eigen::VectorXd& z);
    double getx() const{
      return x[0];
    }
private:
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    Eigen::MatrixXd F;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd K;
    Eigen::MatrixXd I;
    Eigen::MatrixXd H;
};

} // namespace tools

#endif  // TOOLS__EXTENDED_KALMAN_FILTER_HPP