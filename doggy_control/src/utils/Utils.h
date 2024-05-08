

#ifndef A1_CPP_UTILS_H
#define A1_CPP_UTILS_H

#define COUT_RESET   "\033[0m"
#define COUT_BLACK   "\033[30m"      /* Black */
#define COUT_RED     "\033[31m"      /* Red */
#define COUT_GREEN   "\033[32m"      /* Green */
#define COUT_YELLOW  "\033[33m"      /* Yellow */
#define COUT_BLUE    "\033[34m"      /* Blue */

#define PI 3.141592653589793115997963468544185161590576171875
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "dog_params.h"

#define SIGN(x) (((x) > 0) - ((x) < 0))

class Utils {
public:
    // compare to Eigen's default eulerAngles
    
    static Eigen::Quaterniond euler_to_quat(Eigen::Vector3d rpy);
    static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);// this function returns yaw angle within -pi to pi
    static Eigen::Matrix3d euler_to_angular_velocity_mapping(Eigen::Vector3d rpy);
    static Eigen::Matrix3d skew(Eigen::Vector3d vec);
    static Eigen::Matrix3d pseudo_inverse(const Eigen::Matrix3d &mat);
    static double cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2);
    static double periodical_clamp(double value, double lower_bound, double upper_bound, double T);
    static double clamp(double value, double lower_bound, double upper_bound);
    static Eigen::VectorXd clamp(Eigen::VectorXd value, Eigen::VectorXd lower_bound, Eigen::VectorXd upper_bound);
    static Eigen::Quaterniond xyz_pos_to_quaternion(Eigen::Vector3d pos);
    static Eigen::Vector3d frame_transformation(Eigen::Vector3d src_vector, Eigen::Matrix3d rotation, Eigen::Vector3d transition);
    // template <typename T> 
    static std::vector<Eigen::Triplet<double>> matrix_to_none_zero_tripletList(Eigen::MatrixXd matrix, int start_i=0, int start_j=0, double eps=1e-7);
    static Eigen::Vector3d raibert_heuristic_delta_pos(Eigen::Vector3d hip_pos_abs, Eigen::Vector3d com_lin_vel_r, Eigen::Vector3d com_lin_vel_r_set, Eigen::Vector3d com_ang_vel_r, Eigen::Vector3d com_ang_vel_r_set, double swing_time, double foot_delta_x_limit=0.0, double foot_delta_y_limit=0.0);
    static Eigen::Vector2d calculate_displacement(Eigen::Vector2d lin_vel, double yaw_rate, double dt, bool use_approximated=false);
    static Eigen::Vector3d calculate_surface_normal(Eigen::Matrix<double, 3, NUM_LEG> points);
    static Eigen::Vector3d calculate_surface_euler(Eigen::Matrix<double, 3, NUM_LEG> points, double yaw);
    static std::string getCurrentTimeAsString(std::string format = "%Y%m%d-%H%M%S");
    static std::string getStringFromEigen(Eigen::MatrixXd mat, int *numOfVal=nullptr);
    static double calculate_included_angle(Eigen::Vector3d vec1, Eigen::Vector3d vec2);
};
class BezierUtils {
    // TODO: allow degree change? may not be necessary, we can stick to degree 4
public:
    BezierUtils () {
        curve_constructed = false;
        bezier_degree = 4;
    }
    // set of functions create bezier curves, get points, reset
    
    std::pair<Eigen::Vector3d, Eigen::Vector3d> get_foot_pos_curve(double t,
                                   Eigen::Vector3d foot_pos_start,
                                   Eigen::Vector3d foot_pos_final, 
                                   double sub_foot_height, double foot_height, double touchdown_clearence);

    void reset_foot_pos_curve() {curve_constructed = false;}
private:
    double bezier_curve(double t, const std::vector<double> &P);

    bool curve_constructed;
    float bezier_degree;
};

#endif 
