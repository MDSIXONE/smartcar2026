/**************************************************************************
作者：caidx1
功能：命令控制器，命令词识别结果转化为对应的执行动作
**************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <iostream>
#include <stdio.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

using namespace std;
ros::Publisher vel_pub;    //创建底盘运动话题发布者
ros::Publisher cmd_vel_pub;
ros::Publisher follow_flag_pub;    //创建寻找声源标志位话题发布者
ros::Publisher cmd_vel_flag_pub;    //创建底盘运动控制器标志位话题发布者
ros::Publisher awake_flag_pub;    //创建唤醒标志位话题发布者
ros::Publisher navigation_auto_pub;    //创建自主导航目标点话题发布者
ros::Publisher tts_pub;		//创建语音合成词话题发布者
ros::Publisher arrive_A1_pub;
ros::Publisher arrive_A2_pub;
ros::Publisher arrive_C_pub;
ros::Publisher arrive_D_pub;
ros::Publisher turnover_pub;
ros::Publisher stopover1_pub;
ros::Publisher stopover2_pub;
ros::Publisher stopover3_pub;

ros::Publisher task_start_flag_pub;

geometry_msgs::Twist cmd_msg;    //底盘运动话题消息数据
geometry_msgs::PoseStamped target;    //导航目标点消息数据

int turn_angle;
float choice_;
float identify_;

float position_x ;
float position_y ;
float orientation_z ;
float orientation_w ;

float A2_position_x ;
float A2_position_y ;
float A2_orientation_z ;
float A2_orientation_w ;

float B_position_x ;
float B_position_y ;
float B_orientation_z ;
float B_orientation_w ;

float C_position_x ;
float C_position_y ;
float C_orientation_z ;
float C_orientation_w ;

float D_position_x ;
float D_position_y ;
float D_orientation_z ;
float D_orientation_w ;

float line_vel_x ;
float ang_vel_z ;
float turn_line_vel_x ;

int s_count = 0;
extern int detover_flag;
int detover_flag = 0;
int flag1 = 1;
int flag2 = 1;
int flag3 = 1;
int flag4 = 1;
int flag5 = 1;
string result = " ";

//D1,D2,D3为三个车库
double pos_D1[4] = {1.020, -1.000, -0.705, 0.710};
//double pos_D2[4] = {0.500, -1.000, -0.705, 0.705};
double pos_D2[4] = {0.500, -1.000, -0.705, -0.705};
double pos_D3[4] = {0.055, -1.000, -0.710, 0.705};
 
/**************************************************************************
函数功能：离线命令词识别结果sub回调函数
入口参数：命令词字符串  voice_control.cpp等
返回  值：无
**************************************************************************/
void voice_words_callback(const std_msgs::String& msg)
{
	/***指令***/
	string str1 = msg.data.c_str();    //取传入数据
	string str2 = "小车前进";
	string str3 = "小车后退"; 
	string str4 = "小车左转";
	string str5 = "小车右转";
	string str6 = "小车停";
	string str7 = "小车休眠";
	string str8 = "小车过来";
	string str9 = "任务开始";
	string str10 = "小车去起点";
	string str11 = "小车去终点";
	string str12 = "失败5次";
	string str13 = "失败10次";
	string str14 = "遇到障碍物";
	string str15 = "小车唤醒";
	string str16 = "小车雷达跟随";
	string str17 = "小车色块跟随";
	string str18 = "关闭雷达跟随";
	string str19 = "关闭色块跟随";


/***********************************
指令：小车前进
动作：底盘运动控制器使能，发布速度指令
***********************************/
	if(str1 == str2)
	{
		cmd_msg.linear.x = line_vel_x;
		cmd_msg.angular.z = 0;
		vel_pub.publish(cmd_msg);

		std_msgs::Int8 cmd_vel_flag_msg;
    	cmd_vel_flag_msg.data = 1;
    	cmd_vel_flag_pub.publish(cmd_vel_flag_msg);
		cout<<"好的：小车前进"<<endl;
	}
/***********************************
指令：小车后退
动作：底盘运动控制器使能，发布速度指令
***********************************/
	else if(str1 == str3)
	{
		cmd_msg.linear.x = -line_vel_x;
		cmd_msg.angular.z = 0;
		vel_pub.publish(cmd_msg);

		std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 1;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);
		cout<<"好的：小车后退"<<endl;
	}
/***********************************
指令：小车左转
动作：底盘运动控制器使能，发布速度指令
***********************************/
	else if(str1 == str4)
	{
		cmd_msg.linear.x = turn_line_vel_x;
		cmd_msg.angular.z = ang_vel_z;
		vel_pub.publish(cmd_msg);

		std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 1;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);
		cout<<"好的：小车左转"<<endl;
	}
/***********************************
指令：小车右转
动作：底盘运动控制器使能，发布速度指令
***********************************/
	else if(str1 == str5)
	{
		cmd_msg.linear.x = turn_line_vel_x;
		cmd_msg.angular.z = -ang_vel_z;
		vel_pub.publish(cmd_msg);

		std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 1;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);
		cout<<"好的：小车右转"<<endl;
	}
/***********************************
指令：小车停
动作：底盘运动控制器失能，发布速度空指令
***********************************/
	else if(str1 == str6)
	{
		cmd_msg.linear.x = 0;
		cmd_msg.angular.z = 0;
		vel_pub.publish(cmd_msg);


		std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 1;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);
		cout<<"好的：小车停"<<endl;
	}
/***********************************
指令：小车休眠
动作：底盘运动控制器失能，发布速度空指令，唤醒标志位置零
***********************************/
	else if(str1 == str7)
	{
		cmd_msg.linear.x = 0;
		cmd_msg.angular.z = 0;
		vel_pub.publish(cmd_msg);

		std_msgs::Int8 awake_flag_msg;
        awake_flag_msg.data = 0;
        awake_flag_pub.publish(awake_flag_msg);

        std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 1;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);
		cout<<"小车休眠，等待下一次唤醒"<<endl;

		// std_msgs::String tts_words;
		// tts_words.data = "已休眠";
		// tts_pub.publish(tts_words);
	}
/***********************************
指令：小车过来
动作：寻找声源标志位置位
***********************************/
	else if(str1 == str8)
	{
		std_msgs::Int8 follow_flag_msg;
		follow_flag_msg.data = 1;
		follow_flag_pub.publish(follow_flag_msg);
		cout<<"好的：小车寻找声源"<<endl;

		std_msgs::String tts_words;
		tts_words.data = "好的,小车寻找声源";
		tts_pub.publish(tts_words);
	}
/***********************************
指令：小车去第一个坐标点
动作：底盘运动控制器失能(导航控制)，发布目标点
***********************************/
	else if(str1 == str9)
	{
		// target.pose.position.x = position_x;
		// target.pose.position.y = position_y;
		// target.pose.orientation.z = orientation_z;
		// target.pose.orientation.w = orientation_w;
		// navigation_auto_pub.publish(target);

		// std_msgs::Int8 cmd_vel_flag_msg;
        // cmd_vel_flag_msg.data = 0;
        // cmd_vel_flag_pub.publish(cmd_vel_flag_msg);

		// std_msgs::Int8 task_start_flag_msg;
		// task_start_flag_msg.data = 1;
		// task_start_flag_pub.publish(task_start_flag_msg);

		// std_msgs::String tts_words;
		// tts_words.data = "好的,主人";
		// tts_pub.publish(tts_words);
	}
/***********************************
指令：小车去C点
动作：底盘运动控制器失能(导航控制)，发布目标点
***********************************/
	else if(str1 == str10)
	{
		target.pose.position.x = C_position_x;
		target.pose.position.y = C_position_y;
		target.pose.orientation.z = C_orientation_z;
		target.pose.orientation.w = C_orientation_w;
		//navigation_auto_pub.publish(target);

		std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 0;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);
		cout<<"好的：小车自主导航至C点"<<endl;
	}
/***********************************
指令：小车去D点
动作：底盘运动控制器失能(导航控制)，发布目标点
***********************************/
	else if(str1 == str11)
	{
		target.pose.position.x = D_position_x;
		target.pose.position.y = D_position_y;
		target.pose.orientation.z = D_orientation_z;
		target.pose.orientation.w = D_orientation_w;
		//navigation_auto_pub.publish(target);

		std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 0;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);
		cout<<"好的：小车自主导航至D点"<<endl;
	}
/***********************************
辅助指令：失败5次
动作：用户界面打印提醒
***********************************/
	else if(str1 == str12)
	{
		cout<<"您已经连续【输入空指令or识别失败】5次，累计达15次自动进入休眠，输入有效指令后计数清零"<<endl;
	}
/***********************************
辅助指令：失败10次
动作：用户界面打印提醒
***********************************/
	else if(str1 == str13)
	{
		cout<<"您已经连续【输入空指令or识别失败】10次，累计达15次自动进入休眠，输入有效指令后计数清零"<<endl;
	}
/***********************************
辅助指令：遇到障碍物
动作：用户界面打印提醒
***********************************/
	else if(str1 == str14)
	{
		cout<<"小车遇到障碍物，已停止运动"<<endl;
	}
/***********************************
辅助指令：小车唤醒
动作：用户界面打印提醒
***********************************/
	else if(str1 == str15)
	{
		cout<<"小车已被唤醒，请说语音指令"<<endl;
		std_msgs::String tts_words;
		tts_words.data = "已唤醒";
		tts_pub.publish(tts_words);
		
	}
	
	
	
	else if(str1 == str16)
	{
		system("dbus-launch gnome-terminal -- roslaunch simple_follower laserfollow.launch");
		cout<<"好的：小车雷达跟随"<<endl;
	}
	else if(str1 == str17)
	{
		system("dbus-launch gnome-terminal -- rosnode kill /laser_tracker");
		system("dbus-launch gnome-terminal -- roslaunch simple_follower voi_visual_follower.launch");
		cout<<"好的：小车色块跟随"<<endl;
	}
	else if(str1 == str18)
	{
		system("dbus-launch gnome-terminal -- rosnode kill /follower");
		cout<<"好的：关闭雷达跟随"<<endl;
	}
	else if(str1 == str19)
	{
		system("dbus-launch gnome-terminal -- rosrun xf_mic_asr_offline kill");
		system("dbus-launch gnome-terminal -- roslaunch simple_follower laserTracker.launch");
		cout<<"好的：关闭色块跟随"<<endl;
	}
	
}

/*
void kill_pro(char pro_name[])
{
	char get_pid[30] = "pidof ";
	strcat(get_pid,pro_name);
	FILE *fp = popen(get_pid,"r");
	
	char pid[10] = {0};
	fgets(pid,10,fp);
	pclose(fp);
	
	char cmd[20] = "kill -9 ";
	strcat(cmd,pid);
	system(cmd);
	
}
*/

void turn_clockwise(int angle)
{
	int ticks;    //计数变量
	float angular_turn_msg = ang_vel_z;  //转向速度弧度制
	float angular_duration;    //转向时间
	float rate1 = 50;    //控制频率
	geometry_msgs::Twist cmd_vel_msg;    //速度控制信息数据
	ros::Rate loopRate1(rate1);

	//printf("angle =%d\n",angle);
	cmd_vel_msg.linear.x = 0;    //保持X轴方向速度为0
	cmd_vel_msg.angular.z = -angular_turn_msg;		//给Z轴速度

	
	angular_duration = angle / angular_turn_msg /180 * 3.14159;    //计算转弯时间
	//printf("time =%f\n",angular_duration);
	ticks = int(angular_duration * rate1);    //计算控制次数
	//printf("ticks =%d\n",ticks);

	/***控制转向***/
	for(int i = 0; i < ticks; i++)
	{	
		cmd_vel_pub.publish(cmd_vel_msg); // 将速度指令发送给机器人
		loopRate1.sleep();    //控制频率50Hz
		// printf("i =%d\n",i);
	}
	cmd_vel_msg.angular.z = 0;    //速度置零
	cmd_vel_pub.publish(geometry_msgs::Twist());
}


void stop_to_detect(int ticks=10)
{
	// int ticks = 5;    //计数变量
	ros::Rate looprate(10);

	//停顿ticks乘以0.2s去识别
	for(int i=0; i<ticks; i++)
	{
		looprate.sleep();
	}	
}


void det_callback(const std_msgs::String::ConstPtr& msg)
{
	result = msg->data.c_str();
}

void status_callback(const move_base_msgs:: MoveBaseActionResult & msg)
{
	extern int s_count;
	if (msg.status.status == 3)
	{
		s_count++;

		//到达第一个人物识别点
		if (s_count == 1 && flag1 == 1)		
		{
			flag1 = 0;

			//到达第一个人物识别点
			stop_to_detect(5);
			std_msgs::Int8 arrive_A1_flag_msg;
			arrive_A1_flag_msg.data = 1;
			arrive_A1_pub.publish(arrive_A1_flag_msg);
			stop_to_detect();

			//第一个人物识别点完成
			std_msgs::Int8 stopover_msg1;
			stopover_msg1.data = 1;
			stopover1_pub.publish(stopover_msg1);

			//去第二个人物识别点
			target.pose.position.x = A2_position_x;
			target.pose.position.y = A2_position_y;
			target.pose.orientation.z = A2_orientation_z;
			target.pose.orientation.w = A2_orientation_w;
			//navigation_auto_pub.publish(target);

			std_msgs::Int8 cmd_vel_flag_msg;
			cmd_vel_flag_msg.data = 0;
			cmd_vel_flag_pub.publish(cmd_vel_flag_msg);
		}

		//到达第二个人物识别点
		if (s_count == 2 && flag2 == 1)		
		{
			flag2 = 0;

			//到达第二个人物识别点
			stop_to_detect(8);
			std_msgs::Int8 arrive_A2_flag_msg;
			arrive_A2_flag_msg.data = 1;
			arrive_A2_pub.publish(arrive_A2_flag_msg);
			stop_to_detect();
			
			std_msgs::Int8 stopover_msg2;
			stopover_msg2.data = 1;
			stopover2_pub.publish(stopover_msg2);

			//去B点
			target.pose.position.x = B_position_x;
			target.pose.position.y = B_position_y;
			target.pose.orientation.z = B_orientation_z;
			target.pose.orientation.w = B_orientation_w;
			//navigation_auto_pub.publish(target);

			std_msgs::Int8 cmd_vel_flag_msg;
			cmd_vel_flag_msg.data = 0;
			cmd_vel_flag_pub.publish(cmd_vel_flag_msg);
		}

		//到达B区
		if (s_count == 3 && flag3 == 1)
		{
			flag3 = 0;
			// std_msgs::Int8 ttsover_flag_msg;
			// ttsover_flag_msg.data = 1;
			// arrive_C_pub.publish(ttsover_flag_msg);
			// system("dbus-launch gnome-terminal -- roslaunch ucar_cam lax_yolo.launch");
		}

		//到达C区
		if (s_count == 4 && flag4 == 1)
		{
			flag4 = 0;
			std_msgs::Int8 arrive_C_flag_msg;
			arrive_C_flag_msg.data = 1;
			arrive_C_pub.publish(arrive_C_flag_msg);

			//判断是否识别
			if(identify_ == 1)
			{
				// 向右旋转90度找人物
				turn_clockwise(turn_angle);

				stop_to_detect(8);
				std_msgs::Int8 turnover_msg;
				turnover_msg.data = 1;
				turnover_pub.publish(turnover_msg);
			}

			//去D区
			switch(int(choice_))
			{
				case 1:
					target.pose.position.x = pos_D1[0];
					target.pose.position.y = pos_D1[1];
					target.pose.orientation.z = pos_D1[2];
					target.pose.orientation.w = pos_D1[3];
					//navigation_auto_pub.publish(target);
					break;

				case 2:
					target.pose.position.x = pos_D2[0];
					target.pose.position.y = pos_D2[1];
					target.pose.orientation.z = pos_D2[2];
					target.pose.orientation.w = pos_D2[3];
					//navigation_auto_pub.publish(target);
					break;

				case 3:
					target.pose.position.x = pos_D3[0];
					target.pose.position.y = pos_D3[1];
					target.pose.orientation.z = pos_D3[2];
					target.pose.orientation.w = pos_D3[3];
					//navigation_auto_pub.publish(target);
					break;

				default:
					break;
			}
		}		
		if (s_count == 5 && flag5 == 1)
		{
			flag5 = 0;
			//到达D点
			stop_to_detect(5);
			std_msgs::Int8 arrive_D_msg;
			arrive_D_msg.data = 1;
			arrive_D_pub.publish(arrive_D_msg);
			//停下来识别1s
			stop_to_detect();
			std_msgs::Int8 stopover_msg3;
			stopover_msg3.data = 1;
			stopover3_pub.publish(stopover_msg3);
		}
	}
}


void callback(std_msgs::Int8 msg)
{
	extern int detover_flag;
	detover_flag = msg.data;
	//等待1s,等待det_callback接收result
	stop_to_detect();
	if (detover_flag == 1)
	{
		std_msgs::String tts_words;
		tts_words.data = result;
		tts_pub.publish(tts_words);			
		tts_words.data = "您的菜品已送达，请您取餐";
		tts_pub.publish(tts_words);
	}
}


/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(int argc, char** argv)
{

	ros::init(argc, argv, "cmd_rec");     //初始化ROS节点

	ros::NodeHandle n;    //创建句柄
	
	string if_akm;
	
	/***创建寻找声源标志位话题发布者***/
	follow_flag_pub = n.advertise<std_msgs::Int8>("follow_flag",1);

	/***创建底盘运动控制器标志位话题发布者***/
	cmd_vel_flag_pub = n.advertise<std_msgs::Int8>("cmd_vel_flag",1);

	/***创建底盘运动话题发布者***/
	vel_pub = n.advertise<geometry_msgs::Twist>("ori_vel",1);

	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	/***创建唤醒标志位话题发布者***/
	awake_flag_pub = n.advertise<std_msgs::Int8>("awake_flag", 1);

  	/***创建自主导航目标点话题发布者***/
	navigation_auto_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);

	/***创建语音合成词话题发布者***/
	tts_pub = n.advertise<std_msgs::String>("/voice/xf_tts_topic", 1000);

	task_start_flag_pub = n.advertise<std_msgs::Int8>("task_start_flag",1);

	/***创建离线命令词识别结果话题订阅者***/
	ros::Subscriber voice_words_sub = n.subscribe("voice_words",1,voice_words_callback);


	ros::Subscriber detover_sub = n.subscribe("detover_flag", 1, callback);

	ros::Subscriber detect_sub = n.subscribe("detect", 1, det_callback);

	ros::Subscriber goal_sub = n.subscribe("move_base/result", 10, status_callback);


	arrive_A1_pub = n.advertise<std_msgs::Int8>("/arrive_A1_flag",1);
	arrive_A2_pub = n.advertise<std_msgs::Int8>("/arrive_A2_flag",1);
	arrive_C_pub = n.advertise<std_msgs::Int8>("/arrive_C_flag",1);
	arrive_D_pub = n.advertise<std_msgs::Int8>("/arrive_D_flag",1);
	turnover_pub = n.advertise<std_msgs::Int8>("/turnover_flag",1);
	stopover1_pub = n.advertise<std_msgs::Int8>("/stopover1_flag",1);
	stopover2_pub = n.advertise<std_msgs::Int8>("/stopover2_flag",1);
	stopover3_pub = n.advertise<std_msgs::Int8>("/stopover3_flag",1);

	n.param<int>("turn_angle", turn_angle, 60);
	n.param<float>("choice", choice_, 1);
	n.param<float>("identify", identify_, 1);
	n.param<float>("/position_x", position_x, 1);
	n.param<float>("/position_y", position_y, 0);
	n.param<float>("/orientation_z", orientation_z, 0);
	n.param<float>("/orientation_w", orientation_w, 1);
	n.param<float>("/A2_position_x", position_x, 1);
	n.param<float>("/A2_position_y", position_y, 0);
	n.param<float>("/A2_orientation_z", A2_orientation_z, 0);
	n.param<float>("/A2_orientation_w", A2_orientation_w, 1);
	n.param<float>("/B_position_x", B_position_x, 1);
	n.param<float>("/B_position_y", B_position_y, 0);
	n.param<float>("/B_orientation_z", B_orientation_z, 0);
	n.param<float>("/B_orientation_w", B_orientation_w, 1);
	n.param<float>("/C_position_x", C_position_x, 2);
	n.param<float>("/C_position_y", C_position_y, 0);
	n.param<float>("/C_orientation_z", C_orientation_z, 0);
	n.param<float>("/C_orientation_w", C_orientation_w, 1);
	n.param<float>("/D_position_x", D_position_x, 3);
	n.param<float>("/D_position_y", D_position_y, 0);
	n.param<float>("/D_orientation_z", D_orientation_z, 0);
	n.param<float>("/D_orientation_w", D_orientation_w, 1);
	n.param<float>("/line_vel_x", line_vel_x, 0.2);
	n.param<float>("/ang_vel_z", ang_vel_z, 0.3);
	n.param("/if_akm_yes_or_no", if_akm, std::string("no"));

	if(if_akm == "yes")
		turn_line_vel_x = 0.2;
	else 
		turn_line_vel_x = 0;

	

	/***自主导航目标点数据初始化***/
	target.header.seq = 0;
	//target.header.stamp;
	target.header.frame_id = "map";
	target.pose.position.x = 0;
	target.pose.position.y = 0;
	target.pose.position.z = 0;
	target.pose.orientation.x = 0;
	target.pose.orientation.y = 0;
	target.pose.orientation.z = 0;
	target.pose.orientation.w = 1;

	ros::spin();
}
