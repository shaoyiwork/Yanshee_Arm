#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "control.h"


Robot::Robot(ros::NodeHandle& n)
{
    joint_pub = n.advertise<ubt_msgs::angles_set>("hal_angles_set", 1);
    
    joint_angle_.angles.resize(17);
    
    min_rad[head] = -0.78;  max_rad[head] = 0.78;
    min_rad[l_arm1] = -1.58;max_rad[l_arm1] = 1.58;
    min_rad[l_arm2] = 0;max_rad[l_arm2] = 3.15;
    min_rad[l_arm3] = -1.58;max_rad[l_arm3] = 1.58;
    min_rad[r_arm1] = -1.58;max_rad[r_arm1] = 1.58;
    min_rad[r_arm2] = 0;max_rad[r_arm2] = 3.15;
    min_rad[r_arm3] = -1.58;max_rad[r_arm3] = 1.58;

    target_rad[head] = 0;
    target_rad[l_arm1] = 0;
    target_rad[l_arm2] = 1.5708;
    target_rad[l_arm3] = 0;
    target_rad[r_arm1] = 0;
    target_rad[r_arm2] = 1.5708;
    target_rad[r_arm3] = 0;
    
    current_rad[head] = 0;
    current_rad[l_arm1] = 0;
    current_rad[l_arm2] = 1.5708;
    current_rad[l_arm3] = 0;
    current_rad[r_arm1] = 0;
    current_rad[r_arm2] = 1.5708;
    current_rad[r_arm3] = 0;

    left_current_position.x = 0;
    left_current_position.y = 0;
    left_current_position.z = a2+a3;
    right_current_position.x = 0;
    right_current_position.y = 0;
    right_current_position.z = a2+a3;

    joint_angle_.angles[0] = int((90 + 0)*2048/180);
    joint_angle_.angles[1] = int((90 + 0)*2048/180);
    joint_angle_.angles[2] = int((90 + 0)*2048/180);
    joint_angle_.angles[3] = int((90 + 0)*2048/180);
    joint_angle_.angles[4] = int((40 + 0)*2048/180);
    joint_angle_.angles[5] = int((15 + 0)*2048/180);
    joint_angle_.angles[6] = int((90 + 0)*2048/180);
    joint_angle_.angles[7] = int((60 + 0)*2048/180);
    joint_angle_.angles[8] = int((76 - 0)*2048/180);
    joint_angle_.angles[9] = int((110 - 0)*2048/180);
    joint_angle_.angles[10] = int((90 + 0)*2048/180);
    joint_angle_.angles[11] = int((90 + 0)*2048/180);
    joint_angle_.angles[12] = int((120 - 0)*2048/180);
    joint_angle_.angles[13] = int((104 + 0)*2048/180);
    joint_angle_.angles[14] = int((70 + 0)*2048/180);
    joint_angle_.angles[15] = int((90 + 0)*2048/180);
    joint_angle_.angles[16] = int((90 + 0)*2048/180);
    joint_angle_.time = 25;
    
    stop_flag = false;
}

/************************************************************
Function:transform()
Description:将每个关节的弧度转化为舵机的输出
*************************************************************/
void Robot::transform()
{  
    joint_angle_.angles[16] = int((pi/2-current_rad[head])*2048/pi);
    joint_angle_.angles[0] = int((pi/2-current_rad[r_arm1])*2048/pi);
    joint_angle_.angles[1] = int((pi-current_rad[r_arm2])*2048/pi);
    joint_angle_.angles[2] = int((pi/2-current_rad[r_arm3])*2048/pi);
    joint_angle_.angles[3] = int((pi/2-current_rad[l_arm1])*2048/pi);
    joint_angle_.angles[4] = int(current_rad[l_arm2]*2048/pi);
    joint_angle_.angles[5] = int((pi/2+current_rad[l_arm3])*2048/pi);
    joint_angle_.time = 20;
    
}

/************************************************************
Function:robot_action()
Description:将target_rad[]限制在合适范围内，同步到current_rad[]中，经过
transform()转换给joint_msg[i].data并发布给gazebo中每个关节的话题
*************************************************************/
void Robot::robot_action()
{
    int wait_time = 0,temp = 0;

    if(stop_flag)
    {
        return;//如果无解时不再运行action
    }

    for(int i = 0;i<link_num;i++)
    {
        if(target_rad[i] < min_rad[i])
            current_rad[i] = min_rad[i];
        else if(target_rad[i] > max_rad[i])
            current_rad[i] = max_rad[i];
        else
        {
            current_rad[i] = target_rad[i];
        }
        
    }
    transform();
    joint_pub.publish(joint_angle_);

}

/************************************************************
Function:get_position(ARM_ID arm_id,Position& position)
Description:得到手臂末端的坐标
Input:
    arm_id:left_arm/right_arm  选择左臂或者右臂
    position:Position引用类型，通过此引用将坐标传出
Output:通过引用position将坐标传出
*************************************************************/
void Robot::get_position(ARM_ID arm_id,Position& position)
{
    float theta_1,theta_2,theta_3;
    if(arm_id == left_arm)
    {
        theta_1 = current_rad[l_arm1];
        theta_2 = current_rad[l_arm2];
        theta_3 = current_rad[l_arm3];
    }
    else if(arm_id == right_arm)
    {
        theta_1 = current_rad[r_arm1];
        theta_2 = current_rad[r_arm2];
        theta_3 = current_rad[r_arm3];
    }
    else
    {
        ROS_INFO("ARM_ID：%d error!",arm_id);
        return;
    }
    position.x = cos(theta_1)*((a2*cos(theta_2))+(a3*(cos(theta_2+theta_3))));
    position.y = sin(theta_1)*((a2*cos(theta_2))+(a3*(cos(theta_2+theta_3))));
    position.z = a2*sin(theta_2)+(a3*(sin(theta_2+theta_3)));
}


/************************************************************
Function:set_position(ARM_ID arm_id,Position& position)
Description:设定手臂末端的坐标
Input:
    arm_id:left_arm/right_arm  选择左臂或者右臂
    position:Position引用类型，通过此引用得到位置输入
*************************************************************/
void Robot::set_position(ARM_ID arm_id,Position& position)
{
    float th1,  //第一关节角
          th2a, //第二关节角a解
          th3a, //第三关节角a解
          th2b, //第二关节角b解
          th3b, //第三关节角b解
          i_2,  //中间变量，坐标原点到末端的距离的平方
          i,    //中间变量，坐标原点到末端的距离
          th21, //求第二关节角的中间变量
          th22, //求第二关节角的中间变量
          offset_a,//a解距离当前关节位置的偏差
          offset_b;//b解距离当前关节位置的偏差

    stop_flag = false;

    i_2 = position.x*position.x+position.y*position.y+position.z*position.z;
    i = sqrt(i_2);

//正常位置

        th1 = atan(position.y/position.x);
        th21 = acos(((a2*a2)-(a3*a3)+i_2)/(2*a2*i));  //没有多解的情况，但是会出现无解的情况
        if(th21 < 0 || th21 > pi)//无解
        {
            ROS_INFO("position error!");
            stop_flag = true;//无解则停止action
            return;
        }

        if(position.x >=0)  //th22的多解问题，通过判断坐标区域选择解，实际最终只有一个解
            th22 = asin(position.z/i);
        else
            th22 = pi-(asin(position.z/i));
//a，b两组解
        th2a = th21+th22;
        th3a = acos(((a2*a2)+(a3*a3)-i_2)/(2*a2*a3));
        th3a = th3a - pi;

        th2b = th22 - th21;
        th3b = -th3a;

//验证两组解的可行性

        if((th2a >= min_rad[arm_id+2])&&(th2a <= max_rad[arm_id+2])&&(th3a >= min_rad[arm_id+3])&&(th3a <= max_rad[arm_id+3]))
            offset_a = fabs(current_rad[arm_id+2]-th2a)+fabs(current_rad[arm_id+3]-th3a);
        else
            offset_a = 100;//置一个大的偏差使不在限制范围内的解不会被选到
        if((th2b >= min_rad[arm_id+2])&&(th2b <= max_rad[arm_id+2])&&(th3b >= min_rad[arm_id+3])&&(th3b <= max_rad[arm_id+3]))
            offset_b = fabs(current_rad[arm_id+2]-th2b)+fabs(current_rad[arm_id+3]-th3b);
        else
            offset_b = 100;//置一个大的偏差使不在限制范围内的解不会被选到
        if(offset_a == 100 && offset_b == 100)
        {
            ROS_INFO("position limit error!");
            stop_flag = true;//所有的解都不再限制范围内也停止action
            return;
        }
//选择可行且总旋转弧度小的那个解
        if(offset_a <= offset_b)
        {
            target_rad[arm_id+1] = th1;
            target_rad[arm_id+2] = th2a;
            target_rad[arm_id+3] = th3a;
        }else
        {
            target_rad[arm_id+1] = th1;
            target_rad[arm_id+2] = th2b;
            target_rad[arm_id+3] = th3b;
        }

        switch (arm_id)
        {
        case left_arm:
            left_current_position.x = position.x;
            left_current_position.y = position.y;
            left_current_position.z = position.z;
            break;
        case right_arm:
            right_current_position.x = position.x;
            right_current_position.y = position.y;
            right_current_position.z = position.z;
            break;
        default:
            break;
        }

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"control");
    ros::NodeHandle yanshee_node;
    ros::NodeHandle& n = yanshee_node;
    Robot yanshee(n);
    Position position = {100.0,-100,10};//手臂末端坐标值初始化
    Position current_position;
    float time = 0;
    
    ros::Rate loop_rate(50);//设置ros消息发布频率
    while(ros::ok())
    {
        yanshee.robot_action();
        yanshee.set_position(left_arm,position);//通过逆运动学求得输出角度值
        yanshee.set_position(right_arm,position);//通过逆运动学求得输出角度值
        yanshee.get_position(left_arm,current_position);//通过正运动学求得当前位置坐标
        ROS_INFO("position(%f,%f,%f)",current_position.x,current_position.y,current_position.z);//打印正运动学当前位置
        time+=0.04;
        position.y = 90*(sin(time));//通过周期函数改变水平运动末端坐标值
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

