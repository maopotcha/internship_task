#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <sstream>

const static int NUM_AXIS = 4;//ショベルの動く軸(swing, boom, arm, bucket)の数
const static int SWING = 0, BOOM = 1, ARM = 2, BUCKET = 3; //各軸のIDの定義
const static int MODE0 = 0, MODE1 = 1, MODE2 = 2, MODE3 = 3; //各モードIDの定義
const static double SRES_ERROR = 0.1; //角度偏差の許容値

std::vector<double> joint_pos(NUM_AXIS);

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  //ここにはjoint_stateメッセージを受信した時に行う処理を記述する
  for(int i=0; i<msg->name.size(); i++){
      if(msg->name[i]=="swing_joint"){joint_pos[SWING] = msg->position[i];}
      else if(msg->name[i]=="boom_joint"){joint_pos[BOOM] = msg->position[i];}
      else if(msg->name[i]=="arm_joint"){joint_pos[ARM] = msg->position[i];}
      else if(msg->name[i]=="bucket_joint"){joint_pos[BUCKET] = msg->position[i];}
  }
}

int main(int argc, char **argv)
{
  std::vector<std_msgs::Float64> joint_cmd(NUM_AXIS);
  std::vector<double> joint_error(NUM_AXIS);
  double error_norm = 0.0;

  ros::init(argc, argv, "sample_node");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Publisher swing_cmd_pub = n.advertise<std_msgs::Float64>("zx120/swing/cmd", 100);
  ros::Publisher boom_cmd_pub = n.advertise<std_msgs::Float64>("zx120/boom/cmd", 100);
  ros::Publisher arm_cmd_pub = n.advertise<std_msgs::Float64>("zx120/arm/cmd", 100);
  ros::Publisher bucket_cmd_pub = n.advertise<std_msgs::Float64>("zx120/bucket/cmd", 100);

  ros::Subscriber joint_state_sub = n.subscribe("zx120/joint_states", 100, jointStateCallback);

  ros::Rate loop_rate(20);

  int count = 0;
  int mode = MODE0;

  joint_cmd[SWING].data = 0.0;
  joint_cmd[BOOM].data = -1.0;
  joint_cmd[ARM].data = 1.0;
  joint_cmd[BUCKET].data = 0.0;

  while (ros::ok())
  {
    //ROS_INFO("msg=%f", joint_pos_cmd[SWING].data);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    for(int i=0;i<NUM_AXIS;i++){
        joint_error[i] = joint_cmd[i].data - joint_pos[i];
        error_norm += pow(joint_error[i],2.0);
    }
    error_norm = sqrt(error_norm);

    switch(mode){
        case MODE0:
            joint_cmd[SWING].data = 0.0;
            joint_cmd[BOOM].data = -1.0;
            joint_cmd[ARM].data = 1.0;
            joint_cmd[BUCKET].data = 0.0;

            if(error_norm < SRES_ERROR){
                mode = MODE1;
            }
        break;

        case MODE1:
            joint_cmd[SWING].data = -1.0;
            joint_cmd[BOOM].data = -0.7;
            joint_cmd[ARM].data = 1.5;
            joint_cmd[BUCKET].data = 1.5;

            if(error_norm < SRES_ERROR){
                mode = MODE2;
            }
        break;

        case MODE2:
            ROS_INFO("Sample motion completes!");
        break;

        case MODE3:
        break;

        default:
            ROS_WARN("mode has undefined value!");
        break;
    }

    swing_cmd_pub.publish(joint_cmd[SWING]);
    boom_cmd_pub.publish(joint_cmd[BOOM]);
    arm_cmd_pub.publish(joint_cmd[ARM]);
    bucket_cmd_pub.publish(joint_cmd[BUCKET]);

    ROS_INFO("error_norm=%f, mode=%d", error_norm, mode);

    ros::spinOnce();
    loop_rate.sleep();
    error_norm = 0.0;
    ++count;
  }

  return 0;//プログラム終了
}