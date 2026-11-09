#include "diag.hpp"

#include <algorithm>
#include <cmath>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{

// 逐帧一行 33 列；外部脚本按列序解析，列定义以本处为准（角度均为 deg）：
//   1 frame  2 t[s]  3 量测 pitch  4 预测 pitch  5 z  6 vz  7 h  8 云台 pitch
//   9 量测 yaw  10 预测 yaw  11 云台 yaw  12~14 瞄点仰角/方位/有效
//   15~16 指令 pitch/yaw  17 击发  18~25 EKF 状态 x vx y vy a w r l
//   26 瞄点板 id  27 本帧同 name+type 的实测板数  28 瞄点板相位角
//   29 管道延迟  30 飞行时间  31 量测板角  32 板角新息  33 板角新息 sd
void Diag::log(const DiagFrame & f)
{
  if (!on_ || f.targets.empty() || f.armors.empty()) return;

  const auto & tg = f.targets.front();
  const auto ex = tg.ekf_x();
  const auto plates = tg.armor_xyza_list();
  Eigen::Vector3d pred = Eigen::Vector3d::Zero();
  if (!plates.empty()) pred = plates.front().head(3);
  const double pred_el = std::atan2(pred[2], std::hypot(pred[0], pred[1])) * 57.29578;
  const double pred_az = std::atan2(pred[1], pred[0]) * 57.29578;

  // 瞄点已按飞行时间前推，与同帧量测角之差即视觉提前量
  double aim_el = 0, aim_az = 0;
  const int aim_id = f.aim_point.id;
  double aim_ang = 0;
  if (f.aim_point.valid) {
    const Eigen::Vector3d aim = f.aim_point.xyza.head(3);
    aim_el = std::atan2(aim[2], std::hypot(aim[0], aim[1])) * 57.29578;
    aim_az = std::atan2(aim[1], aim[0]) * 57.29578;
    aim_ang = f.aim_point.xyza[3] * 57.29578;
  }

  tools::logger()->info(
    "[diag] {} {:.6f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} "
    "{:.4f} {:.4f} {} {:.4f} {:.4f} {} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} "
    "{:.5f} {:.5f} {} {} {:.4f} {:.5f} {:.5f} {:.4f} {:.4f} {:.4f}",
    f.frame, f.t, f.armors.front().ypd_in_world[1] * 57.29578, pred_el, ex[4], ex[5], ex[10],
    f.ypr[1] * 57.29578, f.armors.front().ypd_in_world[0] * 57.29578, pred_az, f.ypr[0] * 57.29578,
    aim_el, aim_az, f.aim_point.valid ? 1 : 0,
    f.command.pitch * 57.29578, f.command.yaw * 57.29578,
    // 17 为本帧是否允许击发，不等于实际发弹数（仿真下位机限制射速）
    f.command.shoot ? 1 : 0,
    ex[0], ex[1], ex[2], ex[3], ex[6], ex[7], ex[8], ex[9], aim_id,
    // 27 按本帧实测计数，plates.size() 恒为模型板数
    static_cast<int>(std::count_if(
      f.armors.begin(), f.armors.end(), [&](const auto & a) {
        return a.name == f.armors.front().name && a.type == f.armors.front().type;
      })),
    aim_ang, f.aim_dt, f.fly_time,
    f.armors.front().ypr_in_world[0] * 57.29578,
    tg.res4() * 57.29578, tg.sd4() * 57.29578);
}

}  // namespace auto_aim
