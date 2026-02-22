#include "DynamicsUtil.h"

Eigen::VectorXd DynamicsUtil::SetQState(const Eigen::VectorXd& q_vec)
{
    Eigen::VectorXd q_state(z6.size() + q_vec.size());
    q_state << z6, q_vec;   // 앞 6개는 0, 뒤는 q_vec
    return q_state;
}

Eigen::VectorXd DynamicsUtil::SetDQState(const Eigen::VectorXd& dq_vec)
{
    Eigen::VectorXd dq_state(z6.size() + dq_vec.size());
    dq_state << z6, dq_vec;   // 앞 6개는 0, 뒤는 q_vec
    return dq_state;
}


Eigen::Matrix3d DynamicsUtil::Quat2RotMatrix(std::vector<double>& quat)
{
    Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
    double yaw = std::atan2(
        2.0 * (q.w()*q.z() + q.x()*q.y()),
        1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z())
    );
    Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_rp = q_yaw.inverse() * q;
    return q_rp.toRotationMatrix();
}

Eigen::Vector3d DynamicsUtil::Quat2EulerClamped(const std::vector<double>& quat) {
    // Eigen quaternion 은 w,x,y,z 순
    Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);

    // ── 1) Roll (x-axis rotation) ─────────────────────────────────────
    double sinr_cosp = 2.0 * (q.w()*q.x() + q.y()*q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x()*q.x() + q.y()*q.y());
    double roll = std::atan2(sinr_cosp, cosr_cosp);  // (–π, +π]

    // ── 2) Pitch (y-axis rotation) ────────────────────────────────────
    double sinp = 2.0 * (q.w()*q.y() - q.z()*q.x());
    // asin 입력값이 [-1,1] 넘어가면 NaN 되기 때문에 clamp 처리
    sinp = std::clamp(sinp, -1.0, 1.0);
    double pitch = std::asin(sinp);                    // [–π/2, +π/2]

    // ── 3) Yaw (z-axis rotation) ──────────────────────────────────────
    double siny_cosp = 2.0 * (q.w()*q.z() + q.x()*q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z());
    double yaw = std::atan2(siny_cosp, cosy_cosp);    // (–π, +π]

    return { roll, pitch, yaw };
}


Eigen::VectorXd DynamicsUtil::ReMapLegOrder(const std::vector<double>& q_in) 
{
    assert(q_in.size() == 12 && "ReMapLegOrder: input must have length 12");
    // static constexpr int idx_map[12] = {
    //     9,10,11,
    //     6, 7, 8,
    //     3, 4, 5,
    //     0, 1, 2
    // };
    static constexpr int idx_map[12] = {
        3, 4, 5,
        0, 1, 2, 
        9, 10, 11,
        6, 7, 8
    };
    Eigen::VectorXd q_out(12);
    for(int i = 0; i < 12; ++i) {
        q_out[i] = q_in[idx_map[i]];
    }
    return q_out;
}