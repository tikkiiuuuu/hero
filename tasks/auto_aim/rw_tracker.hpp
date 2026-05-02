#ifndef AUTO_AIM__RW_TRACKER_HPP
#define AUTO_AIM__RW_TRACKER_HPP

#include "armor.hpp"
#include "solver.hpp"
#include "tools/extended_kalman_filter.hpp"
#include <Eigen/Dense>
#include <list>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <optional>
namespace auto_aim {
/************************************************************************************************************************
 * @brief RW跟踪器
 * 和sp的tracker同级，sp的tracker和target虽然实现了目标队列，但没什么必要，太屎了
 * 本来功能就是耦合的，还放在同一级有相互调用，之前重构sp的也有些屎，详情见那里的注释
 * 这里接受识别到的所有装甲板和装甲板以外的灯条，进行跟踪
 * 另外由于不希望将target和tracker分开，这里实现了ShowTargetInterface接口，方便planner使用
***********************************************************************************************************************/
class RWTracker {
    //! 为了保持兼容，先用别名暂时适配
public:
    using Light = auto_aim::Lightbar;
    /************************************************************************************************************************
    * @brief 面向planner的接口，因为planner需要部分target的功能
    * 我们暂时只对他要求的接口进行实现
    ***********************************************************************************************************************/
    class ShowTargetInterface {
    public:
        explicit ShowTargetInterface(RWTracker& tracker):tracker_(tracker) {};
        ~ShowTargetInterface() = default;
        std::vector<Eigen::Vector4d> armor_xyza_list() const;
        // void predict(double dt);
        // void predict(std::chrono::steady_clock::time_point t) const;
        //* 使用绝对时间的方法是考虑与系统时间的对齐
            //*还有防止多次预测时延时的累计误差
        const Eigen::VectorXd& get_state() const;
        void init_ekf(){
            state_ = tracker_.target_state;
            ekf_ = tracker_.ekf;
        };
    // private:
        RWTracker& tracker_;
        Eigen::VectorXd state_;
        tools::RWExtendedKalmanFilter ekf_;
        // mutable Eigen::VectorXd temp_state_;
        // mutable bool has_temp_state_ = false;

        // mutable std::chrono::steady_clock::time_point last_predict_time_;
    };
    // unique 版本的接口，返回更完整的 state
    class ShowTargetInterfaceUnique {
    public:
        ShowTargetInterfaceUnique():tracker_(nullptr) {};
        explicit ShowTargetInterfaceUnique(RWTracker& tracker):tracker_(&tracker) {};
        ~ShowTargetInterfaceUnique() = default;
        const Eigen::VectorXd& get_state() const;
        RWTracker* tracker_;
        Eigen::VectorXd state_;
        tools::RWExtendedKalmanFilter ekf_;
    };
    enum MatchType {
        NONE,
        LIGHT_LEFT,
        LIGHT_RIGHT,
        ARMOR,
    };

    explicit RWTracker(const std::string& config_path, Solver & solver);
    void init(const std::list<Armor>& armors);
    void initEKF(const Armor& armor);
    void tracker_loop();
    //!因为要获取灯条目前先用传统方法
    void update(const std::list<Armor>& armors, std::list<Light> light_array);
    void update_ceres(const std::list<Armor>& armors, std::list<Light> light_array);
    void updateTrackedArmorsNum(const Armor& armor);
    double getError_Distance(const Eigen::VectorXd & predict_points, const Eigen::VectorXd & measure_points);
    double getError_Angle(const Eigen::VectorXd & predict_points, const Eigen::VectorXd & measure_points);
    double getError_Area(const Eigen::VectorXd & predict_points, const Eigen::VectorXd & measure_points);
    Eigen::VectorXd Singal_Armor_h(const Eigen::VectorXd & x, int armor_id, MatchType match_type);
    Eigen::MatrixXd Singal_Armor_jh(const Eigen::VectorXd & x, int armor_id, MatchType match_type);
    double normalize_angle(double angle);
    void drawResults(cv::Mat & img);
    void set_enemy_color(uint8_t enemycolor);
    ShowTargetInterface& get_interface();
    ShowTargetInterfaceUnique& get_interface_unique();
    int getCircleIndex() const { return circle_index; }  // 获取前哨站当前装甲板索引

    double dt_;//! 在图像回调中计算

    enum TrackState {
        LOST,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    };
    std::optional<Armor> tracked_armor;
    TrackState tracker_state;

    // class ShowTargetInterface;
    // friend class ShowTargetInterface;//* 自动管理生命周期，同时有所有权限

// private:
    std::chrono::steady_clock::time_point last_update_time_; //* 记录上一次更新时间
    ShowTargetInterface show_target_interface_;
    Eigen::Matrix3d cam_info_;
    
    
    struct EkfSigam2Params {
        double q_xyz;
        double q_xy;
        double q_z;
        double q_yaw;
        double q_r;
        double r_u, r_v;
    };
    struct MatchArmor {
        bool need_match;//处于fov门控的考虑，比如他的预测位置在视野外、背面等看不到
        double error_distance;
        double error_angle;
        MatchType match_type;
        Eigen::VectorXd predict_points;
        Eigen::VectorXd measure_points;
    };
    enum class State { XC, VX, YC, VY, ZA1, VZA1, YAW, VYAW, R1, ZA2, R2 };
    // enum class OutpostState { XC, VX, YC, VY, ZA1, VZA1, YAW, VYAW, R1, DZA2, DZA3 };
    MatchArmor MATCH_ARMOR_INIT =
    {false, 0.0f, 0.0f, MatchType::NONE, Eigen::VectorXd::Zero(0), Eigen::VectorXd::Zero(0)};

    
    Eigen::Vector3d odom2camera_tvec;
    Eigen::Matrix4d odom_camera_matrix;
    // EKF 相关
    EkfSigam2Params s2_normal;                      /*!< EKF 方差参数（敌方车辆）*/
    EkfSigam2Params s2_outpost;                     /*!< EKF 方差参数（前哨站）*/
    std::vector<MatchArmor> match_armors;
    int measure_dimension;
    int tracked_armors_num;

    ArmorName tracked_id;
    Eigen::VectorXd target_state;
    Eigen::VectorXd ekf_prediction;
    tools::RWExtendedKalmanFilter ekf;
    Solver & solver;
    Color enemy_color_;

    // 新增：用于一致性遮挡调试的数据结构
    struct ConsistentOcclusionDebugInfo {
        bool valid = false;
        double cos_side_left = 0.0, cos_side_right = 0.0;
        double cos_extend_left = 0.0, cos_extend_right = 0.0;
        double alpha_occ_left = 0.0, alpha_occ_right = 0.0;
        double alpha_ext_left = 0.0, alpha_ext_right = 0.0;
        double side_offset_left = 0.0, side_offset_right = 0.0;
        double up_offset_left = 0.0, up_offset_right = 0.0;
        double forward_offset_left = 0.0, forward_offset_right = 0.0;
        int armor_id = -1;
        int match_type = 0;
        double theta = 0.0;
        double r = 0.0;
        double z = 0.0;
        Eigen::MatrixXd jh;
        Eigen::MatrixXd djh;
        double jh_norm = 0.0;
        double jh_max_abs = 0.0;
        double djh_norm = 0.0;
        double djh_max_abs = 0.0;
    };

    // 切换使用一致性遮挡模型
    bool use_consistent_occlusion_model_ = false;
    // 填充的一致性遮挡调试信息
    ConsistentOcclusionDebugInfo consistent_occlusion_debug_;
    const ConsistentOcclusionDebugInfo& consistent_occlusion_debug() const;

    // 额外的 state / 接口（完整16维）
    Eigen::VectorXd target_state_unique;
    ShowTargetInterfaceUnique show_target_interface_unique_;

    // 最近一次真实预测时间
    std::chrono::steady_clock::time_point last_true_predict_time_;

    // YAML 参数
    double r_distance = 1.0;

    // Ceres 版本的观测函数（声明）
    Eigen::VectorXd Singal_Armor_h_ceres(const Eigen::VectorXd& x, int armor_id, MatchType match_type);
    Eigen::MatrixXd Singal_Armor_jh_ceres(const Eigen::VectorXd& x, int armor_id, MatchType match_type);
    Eigen::VectorXd Singal_Armor_h_ceres_consis(const Eigen::VectorXd& x, int armor_id, MatchType match_type);
    Eigen::MatrixXd Singal_Armor_jh_ceres_consis(const Eigen::VectorXd& x, int armor_id, MatchType match_type);
    Eigen::MatrixXd last_consistent_occlusion_jh_;

    /* ---------------------------------------- 变量 --------------------------------------------- */
    // 阈值
    double track_thres;                             // 跟踪器跟踪阈值（s）
    double lost_thres;                              // 跟踪器器丢失阈值（s）
    double max_armor_distance_;
    double max_match_error;
    
    // 装甲板尺寸
    static constexpr double SMALL_ARMOR_LENGTH = 0.13;
    static constexpr double SMALL_ARMOR_WIDTH = 0.05;
    static constexpr double LARGE_ARMOR_LENGTH = 0.22;
    static constexpr double LARGE_ARMOR_WIDTH = 0.05;
    static constexpr double ARMOR_PITCH = M_PI/12;

    float self_height;
    float outpost_height[3];
    int circle_type[3] = {1,0,-1};
    int circle_index = 0;
    int outpost_errcount = 0;

    bool matched = false;                           // 当前帧是否有匹配的装甲板、灯条
    double track_time_count = 0;
    double lost_time_count = 0;
    double last_yaw_;

    //? 待调整参数
    float match_thre_distance = 200.0f; // 装甲板匹配误差阈值（距离）都是经验参数
    float match_thre_angle = 2.0f; // 装甲板匹配误差阈值（角度）
    float match_thre_distance_light = 60.0f; // 灯条匹配误差阈值（距离）
    float match_thre_angle_light = 0.1f; // 灯条匹配误差阈值（角度）
    float match_thre_length_light = 0.1f; // 灯条匹配误差阈值（百分比长度）
};

} // namespace auto_aim

#endif // AUTO_AIM__RW_TRACKER_HPP