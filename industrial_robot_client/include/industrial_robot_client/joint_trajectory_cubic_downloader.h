#ifndef JOINT_TRAJECTORY_FULL_DOWNLOADER_H
#define JOINT_TRAJECTORY_FULL_DOWNLOADER_H

#include "industrial_robot_client/joint_trajectory_full_interface.h"
#define INTER_RATE 100

namespace industrial_robot_client{
namespace joint_trajectory_downloader{

using industrial_robot_client::joint_trajectory_interface::JointTrajectoryFullInterface;
using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;

class JointTrajectoryCubicDownloader : public JointTrajectoryFullInterface{

public:

  bool send_to_robot(const std::vector<JointTrajPtFullMessage>& messages);

	/** 
	* @brief		粗插补
	* @details		粗插补
	* @param[in]	points:需要插补的轨迹  time:时间 
	* @param[out]	null
	* @exception	null
	* @return		目标点位
	* @autor		xukunlin
	* @date			2018/10/31
	*/
  JointTrajPtFullMessage sample_traj(std::vector<JointTrajPtFullMessage>& points, int time);

private:
	/** 
	* @brief		三次多项式插补
	* @details		三次多项式插补
	* @param[in]	star:起始点  end：目标点 time:时间 
	* @param[out]	null
	* @exception	null
	* @return		目标点位
	* @autor		xukunlin
	* @date			2018/10/31
	*/
  JointTrajPtFullMessage cubic(JointTrajPtFullMessage start, JointTrajPtFullMessage end, int time);

};

}	//namespace joint_trajectory_downloader
}	//namespace industrial_robot_client

#endif
