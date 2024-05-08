

#include "Utils.h"
#include <ctime>
#include <iomanip>
#include <algorithm>

// Triplet<double> 

Eigen::Quaterniond Utils::euler_to_quat(Eigen::Vector3d rpy)
{
    Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy(2), Eigen::Vector3d::UnitZ());

    //Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
    Eigen::Quaterniond q =  yawAngle * pitchAngle * rollAngle;
    return q;
}


Eigen::Matrix3d Utils::euler_to_angular_velocity_mapping(Eigen::Vector3d rpy)
{
    Eigen::Matrix3d mapping;
    mapping << cos(rpy(1)) * cos(rpy(2)), -sin(rpy(2)), 0,
               cos(rpy(1)) * sin(rpy(2)), cos(rpy(2)), 0,
               -sin(rpy(1)), 0, 1;
    return mapping;
}


std::vector<Eigen::Triplet<double>> Utils::matrix_to_none_zero_tripletList(Eigen::MatrixXd matrix, int start_i, int start_j, double eps)
{
    std::vector<Eigen::Triplet<double>> triplet_lists;
    for(int i = 0; i < matrix.rows(); i++)
    {
        for(int j = 0; j < matrix.cols(); j++)
        {
            if(abs(matrix(i, j)) > eps)
            {
                triplet_lists.emplace_back(Eigen::Triplet<double>(i + start_i, j + start_j, matrix(i, j)));
            }
        }
    }
    return triplet_lists;
}

Eigen::Vector3d Utils::quat_to_euler(Eigen::Quaterniond quat) {
    Eigen::Vector3d rst;

    // order https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
    Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();
    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double y_sqr = y*y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y_sqr);

    rst[0] = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    rst[1] = asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y_sqr + z * z);
    rst[2] = atan2(t3, t4);
    return rst;
}
//seems to be the cross product of matrix
Eigen::Matrix3d Utils::skew(Eigen::Vector3d vec) {
    Eigen::Matrix3d rst; rst.setZero();
    rst <<            0, -vec(2),  vec(1),
            vec(2),             0, -vec(0),
            -vec(1),  vec(0),             0;
    return rst;
}

// https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f
Eigen::Matrix3d Utils::pseudo_inverse(const Eigen::Matrix3d &mat) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double epsilon = std::numeric_limits<double>::epsilon();
    // For a non-square matrix
    // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(mat.cols(), mat.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
           svd.matrixU().adjoint();
}

double Utils::cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2) {
    // surface 1: a1 * x + b1 * y + c1 * z + d1 = 0, coef: [a1, b1, c1]
    // surface 2: a2 * x + b2 * y + c2 * z + d2 = 0, coef: [a1, b2, c2]
    double angle_cos =
            abs(surf_coef_1[0] * surf_coef_2[0] + surf_coef_1[1] * surf_coef_2[1] + surf_coef_1[2] * surf_coef_2[2])
            / (sqrt(surf_coef_1[0] * surf_coef_1[0] + surf_coef_1[1] * surf_coef_1[1] + surf_coef_1[2] * surf_coef_1[2]) *
               sqrt(surf_coef_2[0] * surf_coef_2[0] + surf_coef_2[1] * surf_coef_2[1] + surf_coef_2[2] * surf_coef_2[2]));
    return acos(angle_cos);
}

double Utils::periodical_clamp(double value, double lower_bound, double upper_bound, double T)
{
    assert(("T must be positive!", T > 0.0));
    assert(("upper_bound - lower_bound must >= T!", upper_bound - lower_bound >= T - 1e-4));
    while(value > upper_bound)
    {
        value -= T;
    }
    while(value < lower_bound)
    {
        value += T;
    }
    return value;
}

double Utils::clamp(double value, double lower_bound, double upper_bound)
{
    assert(("the up bound must be larger than or equal to the lower bound!", upper_bound >= lower_bound));
    return std::max(lower_bound, std::min(upper_bound, value));
}

Eigen::VectorXd Utils::clamp(Eigen::VectorXd value, Eigen::VectorXd lower_bound, Eigen::VectorXd upper_bound)
{
    Eigen::VectorXd result(value.rows());
    for(int i = 0; i < value.rows(); ++i)
    {
        result(i) = std::max(lower_bound(i), std::min(upper_bound(i), value(i)));
    }
    return result;
}
Eigen::Vector3d Utils::frame_transformation(Eigen::Vector3d src_vector, Eigen::Matrix3d rotation, Eigen::Vector3d transition)
{
    return rotation * src_vector + transition;
}


Eigen::Quaterniond Utils::xyz_pos_to_quaternion(Eigen::Vector3d pos)
{
    Eigen::Matrix<double, 3, 3> rotation_matrix;
    if(pos.isZero())
    {
        rotation_matrix = Eigen::Matrix<double, 3, 3>::Identity();
    }
    else
    {
        //std::cout<<"pos:"<<std::endl;
        //std::cout<<pos<<std::endl;
        Eigen::Vector3d pos_normalized = pos.normalized();
        //std::cout<<"pos_normalized:"<<std::endl;
        //std::cout<<pos_normalized<<std::endl;
        Eigen::Vector3d pos_normalized_per;
        if(abs(pos_normalized(0)) + abs(pos_normalized(1)) > 0)
        {
            pos_normalized_per << -pos_normalized(1), pos_normalized(0), 0;
        }
        else
        {
            pos_normalized_per << 0, -pos_normalized(2), pos_normalized(1);
        }
        pos_normalized_per = pos_normalized_per.normalized(); 
        //std::cout<<"pos_normalized_per:"<<std::endl;
        //std::cout<<pos_normalized_per<<std::endl;
        Eigen::Vector3d pos_normalized_per2 = pos_normalized.cross(pos_normalized_per);
        //std::cout<<"pos_normalized_per2:"<<std::endl;
        //std::cout<<pos_normalized_per2<<std::endl;
        rotation_matrix << pos_normalized, pos_normalized_per, pos_normalized_per2;
        //std::cout<<"rotation_matrix:"<<std::endl;
        //std::cout<<rotation_matrix<<std::endl;
        //rotation_matrix.transposeInPlace();
    }
    //std::cout<<"rotation_matrix:"<<std::endl;
    //std::cout<<rotation_matrix<<std::endl;
    Eigen::Quaterniond quaternion(rotation_matrix);
    return quaternion;
}

Eigen::Vector3d Utils::raibert_heuristic_delta_pos(Eigen::Vector3d hip_pos_abs, Eigen::Vector3d com_lin_vel_r, Eigen::Vector3d com_lin_vel_r_set, Eigen::Vector3d com_ang_vel_r, Eigen::Vector3d com_ang_vel_r_set, double contact_time, double foot_delta_x_limit, double foot_delta_y_limit)
{
    Eigen::Vector3d delta_pose;
    delta_pose.setZero();
    double vel_x_now = com_lin_vel_r(0) + (Utils::skew(com_ang_vel_r) * hip_pos_abs)(0);
    double vel_x_set = com_lin_vel_r_set(0) + (Utils::skew(com_ang_vel_r_set) * hip_pos_abs)(0);
    double delta_x = std::sqrt(std::abs(hip_pos_abs(2)) / 9.8) * (vel_x_now - vel_x_set) + contact_time / 2.0 * vel_x_set;
    double vel_y_now = com_lin_vel_r(1) + (Utils::skew(com_ang_vel_r) * hip_pos_abs)(1);
    double vel_y_set = com_lin_vel_r_set(1) + (Utils::skew(com_ang_vel_r_set) * hip_pos_abs)(1);
    double delta_y = std::sqrt(std::abs(hip_pos_abs(2)) / 9.8) * (vel_y_now - vel_y_set) + contact_time / 2.0 * vel_y_set;
    if(foot_delta_x_limit > 0)
    {
        delta_x = Utils::clamp(delta_x, -foot_delta_x_limit, foot_delta_x_limit);
    }
    if(foot_delta_y_limit > 0)
    {
        delta_y = Utils::clamp(delta_y, -foot_delta_y_limit, foot_delta_y_limit);
    }
    delta_pose(0) = delta_x;
    delta_pose(1) = delta_y;
    return delta_pose;

}

Eigen::Vector2d Utils::calculate_displacement(Eigen::Vector2d lin_vel, double yaw_rate, double dt, bool use_approximated)
{
    Eigen::Vector2d displacement;
    if(yaw_rate != 0)
    {
        displacement(0) = 1.0 / yaw_rate * (lin_vel(0) * sin(yaw_rate * dt) - lin_vel(1) * (1 - cos(yaw_rate * dt)));
        displacement(1) = 1.0 / yaw_rate * (lin_vel(0) * (1 - cos(yaw_rate * dt)) + lin_vel(1) * sin(yaw_rate * dt));
    }
    else
    {
        displacement = lin_vel * dt;
    }
    return displacement;
}

Eigen::Vector3d Utils::calculate_surface_normal(Eigen::Matrix<double, 3, NUM_LEG> points)
{
    Eigen::Matrix<double, NUM_LEG, 3> W;
    Eigen::VectorXd foot_pos_z;
    Eigen::Vector3d a;
    Eigen::Vector3d surf_coef;

    W.block<NUM_LEG, 1>(0, 0).setOnes();
    W.block<NUM_LEG, 2>(0, 1) = points.block<2, NUM_LEG>(0, 0).transpose();

    foot_pos_z.resize(NUM_LEG);
    foot_pos_z = points.block<1, NUM_LEG>(2, 0).transpose();

    a = Utils::pseudo_inverse(W.transpose() * W) * W.transpose() * foot_pos_z;
    // surface: a1 * x + a2 * y - z + a0 = 0, coefficient vector: [a1, a2, -1]
    surf_coef << -a[1], -a[2], 1;
    return surf_coef.normalized();
}

Eigen::Vector3d Utils::calculate_surface_euler(Eigen::Matrix<double, 3, NUM_LEG> points, double yaw)
{
    Eigen::Vector3d normal = calculate_surface_normal(points);
    normal = normal * SIGN(normal(2));
    Eigen::Vector3d dir_vec = Eigen::Vector3d(cos(yaw), sin(yaw), 0);
    Eigen::Vector3d dir_projection = Eigen::Vector3d(dir_vec(0), dir_vec(1), -(normal(0) * dir_vec(0) + normal(1) * dir_vec(1)) / normal(2));
    double pitch = - calculate_included_angle(dir_vec, dir_projection) * SIGN(dir_projection(2));
    if(std::isnan(pitch))
    {
        pitch = 0;
    }
    Eigen::Vector3d dir_left_projection = normal.cross(dir_projection);
    // std::cout << "dir_left_projection" << std::endl;
    // std::cout << dir_left_projection << std::endl;
    Eigen::Vector3d dir_left = Eigen::Vector3d(dir_left_projection(0), dir_left_projection(1), 0);
    // std::cout << "dir_left" << std::endl;
    // std::cout << dir_left << std::endl;
    double roll = calculate_included_angle(dir_left_projection, dir_left) * SIGN(dir_left_projection(2));
    // std::cout << "roll" << std::endl;
    // std::cout << roll << std::endl;
    if(std::isnan(roll))
    {
        roll = 0;
    }
    Eigen::Vector3d euler = Eigen::Vector3d(roll, pitch, 0);

    return euler;
}

double Utils::calculate_included_angle(Eigen::Vector3d vec1, Eigen::Vector3d vec2)
{
    if((vec1.transpose() * vec2)(0) == 0)
    {
        return 0.5 * PI;
    }
    else
    {
        return acos((vec1.transpose() * vec2)(0) / (vec1.norm() * vec2.norm()));
    }
}

std::string Utils::getCurrentTimeAsString(std::string format) {

    std::time_t currentTime = std::time(nullptr); // 获取当前时间的时间戳
    std::tm* localTime = std::localtime(&currentTime); // 转换为本地时间

    std::stringstream ss;
    ss << std::put_time(localTime, format.c_str()); // 格式化时间字符串
    return ss.str();
}

std::string Utils::getStringFromEigen(Eigen::MatrixXd mat, int *numOfVal)
{
    std::string str;
    if (numOfVal)
    {
        *numOfVal = mat.cols() * mat.rows();
    }
    std::ostringstream buffer;
    Eigen::VectorXd flatten_mat(Eigen::Map<Eigen::VectorXd>(mat.data(), mat.cols() * mat.rows()));
    buffer << flatten_mat.transpose();
    str = buffer.str();
    str.erase(std::unique(str.begin(), str.end(), [](char a, char b)
                          { return std::isspace(a) && std::isspace(b); }),
              str.end());
    std::replace_if(
        str.begin(), str.end(), [](char c)
        { return std::isspace(c); },
        ',');
    if (!str.empty() && str[0] == ',')
    {
        str.erase(0, 1); // 删除第一个字符
    }
    

    return str;
}

// t: 0.0 - 1.0
std::pair<Eigen::Vector3d, Eigen::Vector3d> BezierUtils::get_foot_pos_curve(double t,
                                   Eigen::Vector3d foot_pos_start,
                                   Eigen::Vector3d foot_pos_final, 
                                    double sub_foot_height = 0.0, double foot_height = 0.4, double touchdown_clearence = 0.0)
{
    assert(("t must between 0 and 1", t >= 0.0 && t <= 1.0));
    Eigen::Vector3d foot_pos_target;
    Eigen::Vector3d foot_vel_target;
    double vel_dt = t < 0.5 ? 0.02 : -0.02;
    // X-axis
    std::vector<double> bezierX{foot_pos_start(0),
                               foot_pos_start(0),
                                0.5 * (foot_pos_final(0) + foot_pos_start(0)),
                                foot_pos_final(0),
                                foot_pos_final(0)};
    foot_pos_target(0) = bezier_curve(t, bezierX);
    //foot_vel_target(0)

    // Y-axis
    std::vector<double> bezierY{foot_pos_start(1),
                                foot_pos_start(1),
                                0.5 * (foot_pos_final(1) + foot_pos_start(1)),
                                foot_pos_final(1),
                                foot_pos_final(1)};
    foot_pos_target(1) = bezier_curve(t, bezierY);

    // Z-axis
    std::vector<double> bezierZ{foot_pos_start(2),
                                foot_pos_start(2) + sub_foot_height,
                                foot_pos_final(2) + foot_height,
                                foot_pos_final(2) + sub_foot_height,
                                foot_pos_final(2) + touchdown_clearence};
    foot_pos_target(2) = bezier_curve(t, bezierZ);

    foot_vel_target(0) = (bezier_curve(t + vel_dt, bezierX) - foot_pos_target(0)) / vel_dt;
    foot_vel_target(1) = (bezier_curve(t + vel_dt, bezierY) - foot_pos_target(1)) / vel_dt;
    foot_vel_target(2) = (bezier_curve(t + vel_dt, bezierZ) - foot_pos_target(2)) / vel_dt;

    return std::pair<Eigen::Vector3d, Eigen::Vector3d>(foot_pos_target, foot_vel_target);
}

double BezierUtils::bezier_curve(double t, const std::vector<double> &P) {
    std::vector<double> coefficients{1, 4, 6, 4, 1};
    double y = 0;
    for (int i = 0; i < P.size(); i++) {
        y += coefficients[i] * std::pow(t, i) * std::pow(1 - t, P.size() - 1 - i) * P[i];
    }
    return y;
}

