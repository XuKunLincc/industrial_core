#include "industrial_robot_client/joint_trajectory_cubic_downloader.h"

namespace industrial_robot_client
{
namespace joint_trajectory_downloader
{
using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;
using industrial::simple_message::SimpleMessage;
namespace SpecialSeqValues = industrial::joint_traj_pt::SpecialSeqValues;

bool JointTrajectoryCubicDownloader::send_to_robot(const std::vector<JointTrajPtFullMessage>& messages){

    bool rslt=true;
    std::vector<JointTrajPtFullMessage> points(messages);
    SimpleMessage msg;
    JointTrajPtFullMessage targetPoint;

    ros::Duration startDur(0.05);
    ros::Rate cubicRate(10);
    int pointSize = points.size();

    if (pointSize < 2)
        points.push_back(JointTrajPtFullMessage(points[0]));

    // The first and last points are assigned special sequence values
    points.begin()->setSequence(SpecialSeqValues::START_TRAJECTORY_DOWNLOAD);
    points.back().setSequence(SpecialSeqValues::END_TRAJECTORY);

    if (!this->connection_->isConnected())
    {
        ROS_WARN("Attempting robot reconnection");
        this->connection_->makeConnect();
    }

    ROS_INFO("Sending trajectory points, size: %d", (int)points.size());

    int t = 0;
    int seq = 1;
    // send the first point and after 50ms will send second point
    targetPoint = sample_traj(points, t);
    targetPoint.toTopic(msg);
    this->connection_->sendMsg(msg);

    // 休眠50ms马上发送第二点，为控制器预留50MS的窗口
    startDur.sleep();

    // 当seq为END_TRAJECTORY，表明最后一个点已经发送
    while(targetPoint.point_.getSequence() != SpecialSeqValues::END_TRAJECTORY){

        t += INTER_RATE;
        targetPoint = sample_traj(points, t);

        // 当处于最后一个点时，不修改其seq
        if(targetPoint.point_.getSequence() != SpecialSeqValues::END_TRAJECTORY)
            targetPoint.setSequence(seq);
        seq++;

        // 发送点位
        targetPoint.toTopic(msg);
        bool ptRslt = this->connection_->sendMsg(msg);
        if (ptRslt)
            ROS_DEBUG("Point[%d] sent to controller",seq);
        else
            ROS_WARN("Failed sent joint point, skipping point");
        rslt &= ptRslt;

        // 休眠100ms，此处会导致轨迹运行比预先的慢100ms
        cubicRate.sleep();
    }

    return rslt;
}

JointTrajPtFullMessage JointTrajectoryCubicDownloader::sample_traj(std::vector<JointTrajPtFullMessage>& points, int time){
    int pointSize = points.size();
    industrial::shared_types::shared_real endTime, pointTime;

    // 获取结束点位的时间
    points[pointSize -1].point_.getTime(endTime);

    // 第一个点位
    if(time == 0)
        return points[0];
    if(time >= endTime)
        return points[pointSize-1];

    // 寻找可以满足当前时间的点位
    int i = 1;
    points[i].point_.getTime(pointTime);
    while(time > pointTime){
        i++;
        points[i].point_.getTime(pointTime);
    }

    return cubic(points[i-1], points[i], time);
}

JointTrajPtFullMessage JointTrajectoryCubicDownloader::cubic(JointTrajPtFullMessage start, JointTrajPtFullMessage end, int time){

    return start;
}

}
}
