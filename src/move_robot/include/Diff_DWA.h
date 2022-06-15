#include "DWA.h"
#include "DWA_test.h"
#include <sensor_msgs/point_cloud_conversion.h>
class diff_dwa: public Move_Robot
{
	public:
		diff_dwa(char *dev_name, int Baudrate);
		~diff_dwa();

		ros::Subscriber Clear_ObsModeSubscriber_;
		ros::Subscriber map_sub_;

		void RevProcess(double receive_period);
		//Callback
		void timerCallback(const ros::TimerEvent& event);
		void ClearCallback(const std_msgs::Int8& msg);
		//void laserCallback(const sensor_msgs::LaserScan& scan);
		void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
		void joystickCallback(const move_robot::joystick& joystick);
		void TriggerCallback_(const std_msgs::Int8& msg);
    void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);//new add
		void change_map_callback(const move_robot::change_map& msg);//new add
		void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);//new add
		void resultCallback(const std_msgs::String& msg);//new add

		//state
		void State_Machine(int& state);
		void Idle();
		void Stop();

		//tracking
		bool Navigation_move(std::vector<SUB_MISSONPATH_SUBPOINT> & myAllSubPath, bool isReSet);
		bool Tracking_Angle_Init(int &subpath_index, bool isReSet);
		bool Tracking_Trajectory(int &subpath_index, bool isReSet);
		bool Trajectory_Tracking(int &subpath_index, bool isReSet);

		//OBS
		void ObsDetect(std::vector<Eigen::Vector3f> &i_obs_container,
				std::vector<Eigen::Vector3f> &i_obs_region_0,
				std::vector<Eigen::Vector3f> &i_obs_region_1,
				std::vector<Eigen::Vector3f> &i_obs_region_2,
				std::vector<Eigen::Vector3f> &i_obs_region_3,
				std::vector<Eigen::Vector3f> &i_obs_region_4,
				std::vector<Eigen::Vector3f> &i_obs_gap_0,
				std::vector<Eigen::Vector3f> &i_obs_gap_1,
				std::vector<Eigen::Vector3f> &i_obs_gap_2,
				std::vector<Eigen::Vector3f> &i_obs_gap_3,
				std::vector<Eigen::Vector3f> &i_obs_gap_4,
				std::vector<Eigen::Vector3f> &all_point);

		void DWA_ObsDetect(std::vector<Eigen::Vector3f> &i_obs_container,
				std::vector<Eigen::Vector3f> &dwa_obs_region, std::vector<Eigen::Vector3f> &dwa_obs_robot_region);

		void ObsStateMachine(std::vector<Eigen::Vector3f> &i_obs_region_0,
				std::vector<Eigen::Vector3f> &i_obs_region_1,
				std::vector<Eigen::Vector3f> &i_obs_region_2,
				std::vector<Eigen::Vector3f> &i_obs_region_3,
				std::vector<Eigen::Vector3f> &i_obs_region_4,
				std::vector<Eigen::Vector3f> &i_obs_gap_0,
				std::vector<Eigen::Vector3f> &i_obs_gap_1,
				std::vector<Eigen::Vector3f> &i_obs_gap_2,
				std::vector<Eigen::Vector3f> &i_obs_gap_3,
				std::vector<Eigen::Vector3f> &i_obs_gap_4,
				std::vector<Eigen::Vector3f> &all_point);


		void RePlanning(int &subpath_index,
				std::vector<Eigen::Vector3f> &i_obs_region_0,
				std::vector<Eigen::Vector3f> &i_obs_region_1,
				std::vector<Eigen::Vector3f> &i_obs_region_2,
				std::vector<Eigen::Vector3f> &i_obs_region_3,
				std::vector<Eigen::Vector3f> &i_obs_region_4,
				std::vector<Eigen::Vector3f> &all_point);

		bool FindKeyPoint(Eigen::Vector3f& target, Eigen::Vector3f& key_p,
				std::vector<Eigen::Vector3f> &i_obs_region_0,
				std::vector<Eigen::Vector3f> &i_obs_region_2,
				std::vector<Eigen::Vector3f> &i_obs_region_4,
				std::vector<Eigen::Vector3f> &all_point);

		void CreateNewPath(Eigen::Vector3f& target, Eigen::Vector3f& key_point, int subpath_index, std::vector<Eigen::Vector3f> &new_path);
		bool WaitObsLeave();
		bool WaitDeadlock();
		bool AvoidObs();

		//joystick
		void joystick_move();

		void Calculate_odom();

		void RevPacketDecorder(std::vector<unsigned char> &buff, unsigned int bufferSize, std::vector<float> &vPacket);




	private:

	float vl, vr ;
	float global_v ;


	float stop_w_max;
	float stop_w_min;
	float move_w_max;
	float predict_turn_dis;
	float predict_turn_R;

  //////////////////////////////////////////////dwa use
	DWA use_dwa;
	std::vector<Eigen::Vector3f> dwa_obs_region;
	std::vector<Eigen::Vector3f> dwa_obs_robot_region;
	std::vector<Eigen::Vector3f> dwa_predict_pose;
	std::vector<Eigen::Vector3f> dwa_choose_line;
	int** oscillation_cost;
	int map_width, map_height;
	float origin_x, origin_y, m_resolution;
	void tra_history(int pixel_x, int pixel_y, float vt, float max_vt, int range_R, bool use_full);
	float CalcgoalHeading(Eigen::Vector3f robot_pose, Eigen::Vector3f goal);
	float CalcObsHeading_nearest(Eigen::Vector3f robot_pose, std::vector<Eigen::Vector3f> ob);
	float CalcObsHeading_average(Eigen::Vector3f robot_pose, std::vector<Eigen::Vector3f> ob);
	int tranfer_state(int state_1, int state_2, int state_3, int state_4);
	std::ofstream oFile;
	//////////////////////////////////////////////dwa use
	/////////////////////////////判斷是長廊用
	int hallway_count, non_hallway_count, hallway_odom_count;
	bool determine_hallway, determine_nonhallway;
	bool use_hallway_odom, use_nonhallway_relocalization;
	/////////////////////////////判斷是長廊用
	// DWA_test joystick_dwa;
	// std::vector<Eigen::Vector3f> predict_pose;
	// std::vector<Eigen::Vector3f> choose_pose;
	// std::vector<Eigen::Vector3f> obs_pose;
};

diff_dwa::diff_dwa(char* dev_name, int Baudrate):Move_Robot(dev_name, Baudrate)
{
	vl = 0.0;
  vr = 0.0;
	global_v = 0.0;


	stop_w_max = 0.65;
	stop_w_min = 0.05;
	move_w_max = 0.5;
	predict_turn_dis = 1.5;
	predict_turn_R = 1.0;
	dwa_obs_region.clear();
	dwa_obs_robot_region.clear();
	dwa_predict_pose.clear();
	dwa_choose_line.clear();
	// predict_pose.clear();
	// choose_pose.clear();
	// obs_pose.clear();
	map_width = 0;
	map_height = 0;
	origin_x = 0.0;
	origin_y = 0.0;
	m_resolution = 0.0;
	oFile.open("/home/user/medical_dwa_ws/src/move_robot/pose_data/pose.csv", std::ios::out | std::ios::trunc);    // 这样就很容易的输出一个需要的excel 文件
	/////////////////////////////判斷是長廊用
	hallway_count = 0;
	non_hallway_count = 0;
	hallway_odom_count = 0;
	determine_hallway = true;
	determine_nonhallway = false;
	use_hallway_odom = false;
	use_nonhallway_relocalization = false;
	/////////////////////////////判斷是長廊用

	std::cout<<"diff_dwa"<<std::endl;
	Clear_ObsModeSubscriber_=node_.subscribe("Clear_ObsMode", 10, &diff_dwa::ClearCallback,this);
	laserSubscriber_ = node_.subscribe("scan", 10, &diff_dwa::laserCallback, this);
	JoysickSubscriber_ = node_.subscribe("joystick", 5, &diff_dwa::joystickCallback, this);
  _pose_init_sub = node_.subscribe("initialpose", 1000, &diff_dwa::init_pose_callback, this);//new add
	change_map_sub = node_.subscribe("change_map", 1000, &diff_dwa::change_map_callback, this);//new add
	map_sub_ = node_.subscribe("map", 1, &diff_dwa::mapCallback, this);//new add
	result_classify_Sub = node_.subscribe("result", 1000, &diff_dwa::resultCallback, this);//new add

	receive_thread_ = new boost::thread(boost::bind(&diff_dwa::RevProcess, this, 0.01));
	control_timer = node_.createTimer(ros::Duration(time_sample), &diff_dwa::timerCallback, this);
	TriggerSubscriber_=node_.subscribe("Trigger", 5, &diff_dwa::TriggerCallback_, this);

}
diff_dwa::~diff_dwa()
{
		for(int j=0;j<map_width;++j) delete [] oscillation_cost[j];         //释放每一维矩阵空间
		delete [] oscillation_cost;                                    //释放存放rows维指针的空间
}

void diff_dwa::mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
		map_width = map->info.width;
		map_height = map->info.height;
		origin_x = map->info.origin.position.x;
		origin_y = map->info.origin.position.y;
		m_resolution = map->info.resolution;
		use_dwa.get_mapinfo(map_width, map_height, origin_x, origin_y, m_resolution);
		//std::cout<<"map_width = "<<map_width<<" map_height = "<<map_height<<" origin_x = "<<origin_x<<" origin_y = "<<origin_y<<std::endl;
		oscillation_cost = new int*[map_width];
		for(int i=0;i<map_width;++i)                     //遍历每行矩阵，确定每行一维矩阵的维度
		{
		    oscillation_cost[i] = new int[map_height];   //确定列数空间内存
		}
		for(int i=0;i<map_width;++i)
		{
				for(int j=0;j<map_height;++j)
				{
						oscillation_cost[i][j] = 0;
				}
		}
}

float diff_dwa::CalcObsHeading_nearest(Eigen::Vector3f robot_pose, std::vector<Eigen::Vector3f> ob)
{
		float dist = 10000.0;
		int idx = -1;
		float obsTheta = 0.0;

		for(int i=0; i<ob.size(); ++i)
		{
				float disttmp = sqrt((ob[i].x() - robot_pose.x()) * (ob[i].x() - robot_pose.x()) + (ob[i].y() - robot_pose.y()) * (ob[i].y() - robot_pose.y()));
				if(dist > disttmp)
				{
						dist = disttmp;
						idx = i;
				}
		}
		obsTheta = atan2((ob[idx].y() - robot_pose.y()), (ob[idx].x() - robot_pose.x()));
		return obsTheta;
}

int diff_dwa::tranfer_state(int state_1, int state_2, int state_3, int state_4)
{
		int current_transfer_state = (state_1 - 1) * 24 + (state_2 - 1) * 8 + (state_3 - 1) * 4 + (state_4 - 1) * 1;
}

float diff_dwa::CalcObsHeading_average(Eigen::Vector3f robot_pose, std::vector<Eigen::Vector3f> ob)
{
		float total_heading = 0.0;
		float average_obs_heading = 0.0;
		for(int i=0;i<ob.size();++i)
		{
				float obsTheta = atan2((ob[i].y() - robot_pose.y()), (ob[i].x() - robot_pose.x()));
				total_heading = total_heading + obsTheta;
		}
		average_obs_heading = (float)total_heading / (float)ob.size();
		return average_obs_heading;
}

float diff_dwa::CalcgoalHeading(Eigen::Vector3f robot_pose, Eigen::Vector3f goal)
{
		float theta =	use_dwa.RadianToDegree(robot_pose.z());
		float goalTheta = use_dwa.RadianToDegree(atan2((goal.y() - robot_pose.y()) , (goal.x() - robot_pose.x())));
		float	targetTheta = theta - goalTheta;

		return targetTheta;
}

void diff_dwa::tra_history(int pixel_x, int pixel_y, float vt, float max_vt, int range_R, bool use_full)
{
		for(int i=-range_R;i<range_R;++i)
		{
				for(int j=-range_R;j<range_R;++j)
				{
						if(pixel_x + i >=0 && pixel_x + i <= map_width && pixel_y + j >=0 && pixel_y + j <= map_height)
						{
								float distance = sqrt(i*i + j*j);
								if(distance < range_R)
								{
										if(!use_full) oscillation_cost[pixel_x + i][pixel_y + j] = ((range_R - distance) * vt) / (range_R * max_vt);
										else oscillation_cost[pixel_x + i][pixel_y + j] = 1.0;
								}
						}
				}
		}
}

void diff_dwa::TriggerCallback_(const std_msgs::Int8& msg)
{
	  std::cout<<"diff_dwa delete"<<std::endl;
		int Trigger = msg.data;
		switch(Trigger)
		{
				case ReloadCarParameter:

							delete this;
							break;
				default:
							break;
		}
}
void diff_dwa::init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)//new add
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    std::cout<<"x = "<<x<<" y = "<<y<<std::endl;
		carto_location(msg->pose.pose);
}

void diff_dwa::resultCallback(const std_msgs::String& msg)
{
		static int local_hallway_count = 0;
		static int local_non_hallway_count = 0;
    int count=0;
    float recognition_rate;
    std::string classify;
    std::string cut_par;
    std::string buf = msg.data;
    std::stringstream cut(buf);
    while(getline(cut,cut_par,','))
    {
        if(count == 0)classify = cut_par;
        else if(count == 1)recognition_rate = std::atof(cut_par.c_str());
        count++;
    }
    //std::cout<<"classify = "<<classify<<" recognitiopn_rate = "<<recognitiopn_rate<<std::endl;
		if(determine_hallway)
		{
				if((classify == "hallway" && recognition_rate > 0.9) || (classify == "non hallway" && recognition_rate < 0.2)) hallway_count++;
				else hallway_count = 0;
		}

		if(determine_nonhallway)
		{
				if((classify == "non hallway" && recognition_rate > 0.9) || (classify == "hallway" && recognition_rate < 0.2)) non_hallway_count++;
				else non_hallway_count = 0;
		}

		if(hallway_count > 20)
		{
				if(local_hallway_count < 2)
				{
					  local_hallway_count++;
						determine_hallway = false;
						determine_nonhallway = true;
						use_hallway_odom = true;
						hallway_count = 0;
				}
		}

		if(non_hallway_count > 20)
		{
			  if(local_non_hallway_count < 1)
				{
						local_non_hallway_count++;
						determine_nonhallway = false;
						determine_hallway = true;
						non_hallway_count = 0;
						transfer_odom = false;
						use_nonhallway_relocalization = true;
				}
		}


}


void diff_dwa::change_map_callback(const move_robot::change_map& msg)//new add
{
    std::cout<<"=========data=========="<<msg.type<<std::endl;
    traj_id = 1;
    /////////////////////////////////////////////////////////////////////////////////////////////
    if(msg.type == "finish change")
    {
			  changemap_finish = true;
        // geometry_msgs::Pose initial_pose;
        // float initial_pose_z = 0.062;
        // initial_pose.position.x = -0.216;
        // initial_pose.position.y = -0.093;
        // initial_pose.position.z = 0.0;
        // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(initial_pose_z);
        // initial_pose.orientation = odom_quat;
				// carto_location(initial_pose);
    }
		else if(msg.type == "reboot map")
		{
				changemap_finish = false;
		}
    ///////////////////////////////////////////////////////////////////////
}


void diff_dwa::timerCallback(const ros::TimerEvent& event)
{
	Calculate_odom();
	std_msgs::Float32 msg;
	std_msgs::Int8 navi_msg;//new add
	msg.data = v_buf;
	v_Publisher_.publish(msg);
	if(isReveice_joystick){
		navi_msg.data = 0;
		navigation_Publisher_.publish(navi_msg);
		v_buf = 0.04;

		switch(btn_id){
			case PUSE_BUTTON_A:
				joystick_move();
				break;

			case PUSE_BUTTON_X:
				joystick_move();
				break;

			case PUSE_BUTTON_Y:
				joystick_move();
				break;

			default:
				break;


		}

		isReveice_joystick = false;
		// if(A_misson.size() > 0)
		// {
		// 		std::cout<<"Joystick dwa======="<<std::endl;
		// 		predict_pose.clear();
		// 		choose_pose.clear();
		// 		obs_pose.clear();
		// 		int pixel_x = fabs(slam_pose_.x() - origin_x) / m_resolution;
		// 		int pixel_y = map_height - fabs(slam_pose_.y() - origin_y) / m_resolution;
		// 		//tra_history(pixel_x, pixel_y, tracking_v, Kinematic[0], 20, false);
		// 		float evalParam[7] = { 1, 3, 1, 0, 0, 0, 2.0 };
		// 		state statex = { slam_pose_.x(),slam_pose_.y(),slam_pose_.z(), 0.0, 0.0 };//机器人初始状态
		// 		int Path_size = A_misson[0].sub_missonPath[0].sub_missonPath_subPoint.size();
		// 		Eigen::Vector3f end_pose = A_misson[0].sub_missonPath[0].sub_missonPath_subPoint[Path_size - 1];
		// 		float obstacleR = 0.4;
		// 		float tdist = sqrt((statex.x - end_pose.x())*(statex.x - end_pose.x()) + (statex.y - end_pose.y())*(statex.y - end_pose.y()));
		// 		joystick_dwa.DynamicWindowApproach(statex, end_pose, evalParam, dwa_obs_region, dwa_obs_robot_region, obstacleR, tdist, oscillation_cost, predict_pose, choose_pose, obs_pose);
		// 		draw(6 , 1.0, 0.1, 0.1, predict_pose);
		// 		draw(7 , 0.1, 0.1, 1.1, choose_pose);
		// 		draw(8 , 0.0, 0.0, 1.1, obs_pose);
		// }
	}
	else{
		//State_Machine(p_state_);
		navi_msg.data = 1;
		navigation_Publisher_.publish(navi_msg);
		LaserMiss += 1;

		if(LaserMiss > 5)
		{
			Stop();
			std_msgs::Int8 msg;
			msg.data = 6;
			Error_Publisher_.publish(msg);

		}
		// else if(v_stop)
		// {
		// 		Stop();
		// }
		else
		{
			State_Machine(p_state_);
		}
	}
}
void diff_dwa::ClearCallback(const std_msgs::Int8& msg)
{
	int obs_clear_buf = msg.data;
	std::vector<SUB_MISSONPATH_SUBPOINT> zzz;

	switch (obs_clear_buf) {
		case Clear_Mode:

			int xxx;
			p_state_ = P_STATE_IDLE;
			if(A_misson.size() > 0)
				A_misson[ready_path_index].ALL_pathnode.clear();

			A_misson.clear();


			memset(&trafficGO_recv,0,sizeof(trafficGO_recv));

			now_A_misson=0;
			now_path_index=0;
			ready_path_index=0;
			Tracking_Angle_Init(xxx, true);
			Tracking_Trajectory(xxx, true);
			Trajectory_Tracking(xxx, true);
			Navigation_move(zzz,true);
			//Misson_state(true);

			AvoidObs_state = P_OBS_NORMAL;
			obs_way_theta = 0.0;
			isAvoidObs = false;
			OBS_limilMode = false;
			all_obs_dis = 0.0;
			decay_obs_v = 0.0;
			isFindObs = false;
			last_subpath_index = 0;
			isblind = false;
			avoid_way = 0;
			obs_avoid_flag =false;
			isCloseNow = true;
			command_OBSMode = 0;
			obs_return =false;          //閉障回來
			avoid_path.clear();

			protect_erase = true;
			traffic_send = false;


			break;
		case Close_AllObs:
			command_OBSMode = 1;

			break;
		case limitObs_Mode:
			command_OBSMode = 2;

			break;
		case Normal_Mode:
			command_OBSMode = 0;

			break;
		default:
			break;
	}


	std::cout<<"A_misson.size  clear"<<A_misson.size() <<std::endl;
}

void diff_dwa::State_Machine(int& state)
{
int type,id;
	switch (state) {

		case P_STATE_IDLE:
			Command_STATE = Command_STATE_IDLE;

			Idle();

			break;
		case P_STATE_MISSON:
			Misson_state();


			break;

		case P_STATE_MOVE:
		   ElevatorGO = false;
			//當不是交管處理中
			//找末點模式與哪個id
			 type = A_misson[now_A_misson].sub_missonPath[now_path_index].end_type;
			 id = A_misson[now_A_misson].sub_missonPath[now_path_index].end;


			if(!traffic_send)
			{
				int id = A_misson[now_A_misson].sub_missonPath[now_path_index].end;
				Command_STATE = Command_STATE_TRACKING;
				Command_id = id;
			}

			if (trafficGO_recv.GO ==2) {
				if(type == Command_STATE_Traffic)
					Stop();
			}
			else
			{
				//導航
				Navigation_move(A_misson[now_A_misson].sub_missonPath, false);


				//std::cout<<"type   "<<  type  <<std::endl;

				if(type == Command_STATE_Traffic)
				{
					if(traffic_send)
					{
						Command_STATE = Command_STATE_Traffic;
						//把陣列合併
						if(trafficGO_recv.id==id)
							if(trafficGO_recv.GO == 1)
							{

								A_misson[now_A_misson].sub_missonPath[now_path_index].start = A_misson[now_A_misson].sub_missonPath[now_path_index+1].start;
								A_misson[now_A_misson].sub_missonPath[now_path_index].end_type = A_misson[now_A_misson].sub_missonPath[now_path_index+1].end_type;
								A_misson[now_A_misson].sub_missonPath[now_path_index].end = A_misson[now_A_misson].sub_missonPath[now_path_index+1].end;
								A_misson[now_A_misson].sub_missonPath[now_path_index].sub_missonPath_subPoint.insert(A_misson[now_A_misson].sub_missonPath[now_path_index].sub_missonPath_subPoint.end(),A_misson[now_A_misson].sub_missonPath[now_path_index+1].sub_missonPath_subPoint.begin(),A_misson[now_A_misson].sub_missonPath[now_path_index+1].sub_missonPath_subPoint.end());
								A_misson[now_A_misson].sub_missonPath.erase(A_misson[now_A_misson].sub_missonPath.begin() + now_path_index + 1);
								trafficGO_recv.GO ==0;

								traffic_send = false;

							}
					}
				}
			}


			break;

	case P_STATE_STOP:

			Stop();

			break;


	case P_STATE_TIME_DELAY:  //LOADING TIME

			if(time_delay_counter >= time_delay_limit){
				///////////////////////////////////
				// ros::Rate delay(10);
				// for(int i=0;i<5;i++)
				// {
				// 		delay.sleep();
				// }
				// move_robot::change_map msg;
				// msg.type = "Clear Map";
				// Command_Publisher_.publish(msg);
				// for(int i=0;i<5;i++)
				// {
				// 		delay.sleep();
				// }
				// msg.type = "Load Map";
				// msg.Name = "mymap1";
				// Command_Publisher_.publish(msg);
				///////////////////////////////////////////////
				time_delay_counter = 0;
				ROS_INFO(" Success!!!\n");
				p_state_ = P_STATE_MOVE;

				if(ready_path_index >= A_misson.size())
				{
					ready_path_index =0;
					A_misson[ready_path_index].ALL_pathnode.clear();
					A_misson.clear();
					p_state_=P_STATE_IDLE;
					isALLfinish =false;
					ROS_INFO("All Success!!!\n");
					now_A_misson = 0;

				}

			}
			else{
				time_delay_counter += 1;
				std::cout<<"time_delay_counter   "<<time_delay_counter<<std::endl;
			}

			break;

	case P_STATE_AVOID_WAIT:

			WaitObsLeave();

			break;

	case P_STATE_AVOID_REPLANNING:

			Stop();

			break;


	case P_STATE_AVOID_OBS:

			AvoidObs();

			break;


	case P_STATE_AVOID_DEADLOCK:

			WaitDeadlock();

			break;

	case P_STATE_ODOM:
			ROS_INFO(" Success!!!\n");
			p_state_ = P_STATE_MOVE;

			if(ready_path_index >= A_misson.size())
			{
				ready_path_index =0;
				A_misson[ready_path_index].ALL_pathnode.clear();
				A_misson.clear();
				p_state_=P_STATE_IDLE;
				isALLfinish =false;
				ROS_INFO("All Success!!!\n");
				now_A_misson = 0;

			}
			break;

	case P_STATE_SET_POSE:
			ROS_INFO(" Success!!!\n");
			p_state_ = P_STATE_MOVE;

			if(ready_path_index >= A_misson.size())
			{
				ready_path_index =0;
				A_misson[ready_path_index].ALL_pathnode.clear();
				A_misson.clear();
				p_state_=P_STATE_IDLE;
				isALLfinish =false;
				ROS_INFO("All Success!!!\n");
				now_A_misson = 0;

			}
			break;

	default:
			break;
}


}

bool diff_dwa::Navigation_move(std::vector<SUB_MISSONPATH_SUBPOINT> & myAllSubPath, bool isReSet)
{
	static int path_index = 0;

	static int cnt = 0;

	float delay_time = 0.1;
	int cnt_limit = int(delay_time/time_sample);
	if(!isReSet){
		if(myAllSubPath.size() > 0){

			if(AvoidObs_state == P_OBS_NORMAL && isFindObs == true && cnt >= cnt_limit && !isCloseNow ){
				//進避障

				cnt = 0;
				Trajectory_Tracking(path_index, true);
				AvoidObs_state = P_OBS_WAIT;
				p_state_ = P_STATE_AVOID_WAIT;

			}
			else{

				if(isFindObs){
					cnt += 1;
				}
				else{
					cnt = 0;
				}

				bool isFInish = false;
				now_path_index=path_index;
				isFInish = Trajectory_Tracking(path_index, false);


				if(isFInish){

					path_index += 1;
					p_state_ = P_STATE_MISSON;

					if(path_index >= myAllSubPath.size()){
						cnt = 0;
						path_index = 0;

						global_v = 0;
						std::vector<unsigned char> command;
						//sendreceive.Package_Diff_encoder(0,0,command);
						sendreceive.Package_publicWheel_encoder(0, 0, 0, 0, command);
						SendPackage(command);



						ready_path_index += 1;
						now_A_misson=ready_path_index -1;

						return true;

					}

				}
			}


		}
		else{
			ready_path_index=0;
			path_index = 0;
			p_state_ = P_STATE_IDLE;
			cnt = 0;
		}
	}
	else{
		ready_path_index=0;
		path_index = 0;
		Trajectory_Tracking(path_index, true);
		isALLfinish =false;
		cnt = 0;
		now_A_misson = 0;

	}
	last_subpath_index = path_index;
	return false;

}

void diff_dwa::Idle()
{

	Navigation_move(A_misson[ready_path_index].sub_missonPath, true);
	global_v = 0;
	std::vector<unsigned char> command;

	// sendreceive.Package_Diff_encoder(0,0,
	// 		command);
	sendreceive.Package_publicWheel_encoder(0, 0, 0, 0, command);

	SendPackage(command);
}

bool diff_dwa::Trajectory_Tracking(int &subpath_index, bool isReSet)
{



	static bool isInitial = true;

	bool isFInish = false;

	bool isreset = isReSet;

	if(!isReSet){

		if(isInitial){

			isInitial = !(Tracking_Angle_Init(subpath_index, isreset));

		}
		else{

			isFInish = Tracking_Trajectory(subpath_index, isreset);

		}

		if(isFInish){
			Tracking_Angle_Init(subpath_index, true);
			Tracking_Trajectory(subpath_index, true);
			isInitial = true;
			return true;
		}


	}
	else{
		Tracking_Angle_Init(subpath_index, true);
		Tracking_Trajectory(subpath_index, true);
		isInitial = true;
	}

	return false;
}
bool diff_dwa::Tracking_Angle_Init(int &subpath_index, bool isReSet)
{
	return true;
	static bool isFInish = false;
	static bool isInitial = true;
	static int target_ind = 0;
	static float pre_angular_error = 0;


	float angular_kp = 0.5;
	float angular_kd = 0.1;
	float angular_ki = 0;

	float angular_error = 0;

	float Vx = 0;
	float Vy = 0;
	float W_rw = 0;

	static int cmd_angular_cnt = 0;
	int cmd_angular_cnt_limit = 10;


	std::string kinematics_model;

	Eigen::Vector3f target_pos;
	Eigen::Vector3f robot_pos;

	if(transfer_odom)
	{
		robot_pos = transfer_odom_pose;
		std::cout<<"==================hallway odom====================="<<std::endl;
	}
	else
	{
			robot_pos = slam_pose_;
	}

	if(!isReSet){

		if(isInitial){

			Command_id=A_misson[ready_path_index].sub_missonPath[subpath_index].end;

				isInitial = false;
				isCloseNow = true;


				int now_index = 0;
				float odom_v = 0.5;

				target_ind = calc_target_index(robot_pos, odom_v, A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint, now_index);

		}
		else{

			float cmd_angular_velocity = 0.0;
			float cmd_angular_velocity_buf = 0.0;


			target_pos = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[target_ind];

			float goal_angle = atan2((target_pos.y() - robot_pos.y()) , (target_pos.x() - robot_pos.x()));

			angular_error = goal_angle - robot_pos.z();


			if(angular_error>M_PI)
				angular_error=angular_error-2*M_PI;
			else if(angular_error<-M_PI)
				angular_error=angular_error+2*M_PI;

			float angular_p_error = angular_error;
			float angular_d_error = angular_error - pre_angular_error;
			pre_angular_error = angular_error;

			cmd_angular_velocity = angular_kp*angular_p_error + angular_kd*angular_d_error;



			if(fabs(cmd_angular_velocity) > stop_w_max){
				cmd_angular_velocity_buf = (cmd_angular_velocity/fabs(cmd_angular_velocity))*stop_w_max;
				cmd_angular_cnt += 1;
				if(cmd_angular_cnt > cmd_angular_cnt_limit)
				{
					cmd_angular_cnt = cmd_angular_cnt_limit;
					cmd_angular_velocity = cmd_angular_velocity_buf;
				}
				else
				{
					cmd_angular_velocity = cmd_angular_velocity_buf * cmd_angular_cnt /cmd_angular_cnt_limit;
				}

			}

			if(fabs(cmd_angular_velocity) < stop_w_min){
				cmd_angular_velocity = (cmd_angular_velocity/fabs(cmd_angular_velocity))*stop_w_min;
			}
			std::cout<<"angular_error = "<<angular_error<<std::endl;
			if(fabs(angular_error) < 0.04){
				isFInish = true;
				cmd_angular_velocity = 0;
			}

			Vx = 0;
			Vy = 0;

			W_rw = cmd_angular_velocity;

			Car.two_wheel_Kinematics(0, W_rw, vl,  vr );


			if(isFInish){
				vl = 0;
				vr = 0;
			}
		}

		global_v = 0;
		std::vector<unsigned char> command;
    if(!back_trajectory)
		{
				// sendreceive.Package_Diff_encoder(0,W_rw,
				// 		command
				// 		);
				sendreceive.Package_publicWheel_encoder(0, 0, W_rw, 0, command);
		}
		else
		{
				// sendreceive.Package_Diff_encoder(0,W_rw,
				// 		command
				// 		);
				sendreceive.Package_publicWheel_encoder(0, 0, W_rw, 0, command);
		}


		SendPackage(command);


		//        ROS_INFO("=====================================");
		ROS_INFO("Tracking_Angle");
		//        ROS_INFO("---------------");
		//        ROS_INFO("now: %f, %f", robot_pos.x(), robot_pos.y());
		//        ROS_INFO("target: %f, %f", target_pos.x(), robot_pos.y());
		//        ROS_INFO("angle_error: %f", angular_error);

		if(isFInish){
			isFInish = false;
			isInitial = true;
			target_ind = 0;
			pre_angular_error = 0;
			cmd_angular_cnt = 0;
			return true;
		}

	}
	else{
		isFInish = false;
		isInitial = true;
		target_ind = 0;
		pre_angular_error = 0;
		cmd_angular_cnt = 0;
	}

	return false;


}


bool diff_dwa::Tracking_Trajectory(int &subpath_index, bool isReSet)
{
	  static bool Endangle = false;
		static bool isFInish = false;
		static float tracking_v = 0.0;
		static float tracking_w = 0.0;
		static float pre_angular_error = 0.0;
		//static float pre_dis_error = 0.0;
		static float pre_angular_velocity_error = 0.0;
		//float dis_error = 0.0;
		float angular_kp = tracking_kp;
		float angular_kd = tracking_kd;
	  Eigen::Vector3f robot_pos = slam_pose_;
		if(transfer_odom)
		{
				robot_pos = transfer_odom_pose;
				std::cout<<"========================use odom================="<<std::endl;
		}
		//std::cout<<"robot_pos.x() = "<<robot_pos.x()<<" robot_pos.y() = "<<robot_pos.y()<<" robot_pos.z() = "<<robot_pos.z()<<std::endl;
		//评价函数参数 [heading,dist,velocity,predictDT],航向得分的比重、距离得分的比重、速度得分的比重、向前模拟轨迹的时间
    float evalParam[7] = { 1, 1, 1, 0, 0, 0, 2.0 };
		//机器人运动学模型参数, 最高速度m / s], 最高旋转速度[rad / s], 加速度[m / ss], 旋转加速度[rad / ss],速度分辨率[m / s], 转速分辨率[rad / s]]
		float Kinematic[6] = { 0.6, 0.5, 0.1, 0.1, 0.01, 0.1 };
		///////////////////////////////障碍物位置列表[x(m) y(m)]
    // std::vector <Eigen::Vector3f> obstacle;
		// obstacle.clear();//先不用
		float obstacleR = 0.4; //冲突判定用的障碍物半径
		int pixel_x = fabs(robot_pos.x() - origin_x) / m_resolution;
		int pixel_y = map_height - fabs(robot_pos.y() - origin_y) / m_resolution;
		tra_history(pixel_x, pixel_y, tracking_v, Kinematic[0], 20, false);
		use_dwa.arrival_goal = false;
		dwa_predict_pose.clear();
		dwa_choose_line.clear();

		/////////////////////////////////////////////////////
		if(!isReSet)
		{
				////////csv
				oFile << robot_pos.x() << "," << robot_pos.y() << "," << pixel_x << "," << pixel_y << std::endl;
				////////csv
				//查找目前所應追尋的點
				int now_index = 0;
				int target_ind = calc_target_index(robot_pos, tracking_v, A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint, now_index);
				//目標點
				Eigen::Vector3f target_pos = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[target_ind];
				int Path_size = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint.size();
				Eigen::Vector3f end_pose = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[Path_size - 1];
				// float x_error = end_pose.x() - robot_pos.x();
				// float y_error = end_pose.y() - robot_pos.y();
				// dis_error = sqrt(x_error*x_error + y_error*y_error);
				state statex = { robot_pos.x(),robot_pos.y(),robot_pos.z(), tracking_v, tracking_w };//机器人初始状态
				std::vector<state> cartraj,realtraj; //cartraj累积存储走过的轨迹点的状态值
				int First_tracking = Path_size - Path_size * 0.1;//60cm以前

				float tdist = sqrt((statex.x - end_pose.x())*(statex.x - end_pose.x()) + (statex.y - end_pose.y())*(statex.y - end_pose.y()));
				std::cout<<"tdist = "<<tdist<<std::endl;
				/////////////////////////////////////////////////////state
				int now_state_1 = 0;
				int now_state_2 = 0;
				int now_state_3 = 0;
				int now_state_4 = 0;
				if(tdist < 3 * obstacleR) now_state_1 = 1;
				else now_state_1 = 2;

				float now_goal_heading = CalcgoalHeading(robot_pos, end_pose);
				if(now_goal_heading >= 0 && now_goal_heading < 60) now_state_2 = 1;
				else if(now_goal_heading < 0 && now_goal_heading >= -60) now_state_2 = 2;
				else now_state_2 = 3;

				int pre_dis_timer = 0;
				state pre_pose = {robot_pos.x(), robot_pos.y(), robot_pos.z(), tracking_v, tracking_w};
				controlU now_vw = {tracking_v, tracking_w};
				while(pre_dis_timer < 10)
				{
						pre_dis_timer = pre_dis_timer + 1;
						pre_pose = use_dwa.CarState(pre_pose, now_vw);
				}
				float one_sec_distance = sqrt((pre_pose.x - robot_pos.x())*(pre_pose.x - robot_pos.x()) + (pre_pose.y - robot_pos.y())*(pre_pose.y - robot_pos.y()));
				if(one_sec_distance <= 0.5) now_state_3 = 1;
				else now_state_3 = 2;

				if(dwa_obs_region.size() == 0) now_state_4 = 4;
				else
				{
						float nearest_obs_heading = CalcObsHeading_nearest(robot_pos, dwa_obs_region);
						float average_obs_heading = CalcObsHeading_average(robot_pos, dwa_obs_region);
						float nearest_average_error = fabs(average_obs_heading - nearest_obs_heading);
						float OBS_heading = 0.0;
						float OBS_heading_buf = 0.0;
						if(nearest_average_error < M_PI / 4.0) OBS_heading_buf = average_obs_heading;
						else OBS_heading_buf = nearest_obs_heading;

						OBS_heading = robot_pos.z() - OBS_heading_buf;

						if(OBS_heading >= 0 && OBS_heading < M_PI / 3.0) now_state_4 = 1;
						else if(OBS_heading < 0 && OBS_heading >= -1.0 * M_PI / 3.0) now_state_4 = 2;
						else now_state_4 = 3;
				}
				int current_transfer_state = tranfer_state(now_state_1, now_state_2, now_state_3, now_state_4);
				std::cout<<"state_1 = "<<now_state_1<<" state_2 = "<<now_state_2<<" state_3 = "<<now_state_3<<" state_4 = "<<now_state_4<<" transfer_state = "<<current_transfer_state<<std::endl;
				if(current_transfer_state == 42)
				{
						evalParam[0] = 1;evalParam[1] = 1;evalParam[2] = 1;evalParam[3] = 1;evalParam[4] = 1;evalParam[5] = 1;evalParam[6] = 1.5;
				}
				else if(current_transfer_state == 46)
				{
						evalParam[0] = 1;evalParam[1] = 1;evalParam[2] = 1;evalParam[3] = 1;evalParam[4] = 1;evalParam[5] = 1;evalParam[6] = 1.5;
				}
				else if(current_transfer_state == 45)
				{
						evalParam[0] = 1;evalParam[1] = 2;evalParam[2] = 2;evalParam[3] = 2;evalParam[4] = 1;evalParam[5] =21;evalParam[6] = 2.0;
				}
				else if(current_transfer_state == 37)
				{
						evalParam[0] = 1;evalParam[1] = 1;evalParam[2] = 2;evalParam[3] = 1;evalParam[4] = 1;evalParam[5] = 1;evalParam[6] = 1.0;
				}
				else if(current_transfer_state == 38)
				{
						evalParam[0] = 1;evalParam[1] = 2;evalParam[2] = 2;evalParam[3] = 2;evalParam[4] = 1;evalParam[5] = 2;evalParam[6] = 1.5;
				}
				else
				{
						evalParam[0] = 1;evalParam[1] = 1;evalParam[2] = 1;evalParam[3] = 1;evalParam[4] = 1;evalParam[5] = 1;evalParam[6] = 2.0;
				}
				/////////////////////////////////////////////////////state
				if (tdist < 0.05)//是否到达目标点
        {
        		std::cout << "Arrive Goal!" << std::endl;
						Endangle = true;
        }

				if(!Endangle)
				{
						if(now_index > First_tracking)
						{
								Kinematic[0] = 0.5;
								use_dwa.arrival_goal = true;
						}
						else Kinematic[0] = 0.6;
						// if(dwa_obs_region.size() != 0)Kinematic[0] = 0.4;
						// else Kinematic[0] = 0.5;
						//std::cout<<"dwa_obs_region = "<<dwa_obs_region.size()<<std::endl;
						//////////////////////////////////////////////////////////dwa
						std::cout<<"first tracking dwa!!"<<std::endl;
						Eigen::Vector3f dwa_choose_pose(0.0, 0.0, 0.0);
						//DWA参数输入 返回控制量 u = [v(m/s),w(rad/s)] 和 轨迹
						controlU cu = use_dwa.DynamicWindowApproach(statex, &cartraj, Kinematic, end_pose, evalParam, dwa_obs_region, dwa_obs_robot_region,obstacleR, tdist, oscillation_cost, dwa_predict_pose, dwa_choose_pose);
						tracking_v = cu.vt;
						tracking_w = cu.wt;
						draw(6 , 1.0, 0.1, 0.1, dwa_predict_pose);
						dwa_choose_line.push_back(robot_pos);
						dwa_choose_line.push_back(dwa_choose_pose);
						drawLine(2, 0.0, 0.0, 1.0, dwa_choose_line);
						//////////////////////////////////////////////////////////dwa
				}
				else
        {
						tracking_v = 0.0;
						std::cout<<"=========Endangle============="<<std::endl;
						float angular_error = end_pose.z() - robot_pos.z();
						if(fabs(angular_error) > M_PI)
						{
								if(angular_error > 0) angular_error = angular_error - 2*M_PI ;
								else                  angular_error = angular_error + 2*M_PI;
						}
						float angular_p_error = angular_error;
						float angular_d_error = angular_error - pre_angular_error;
						pre_angular_error = angular_error;
						tracking_w = angular_kp*angular_p_error + angular_kd*angular_d_error;
						if(fabs(tracking_w)>stop_w_max)
						{
								if(tracking_w > 0) tracking_w =stop_w_max;
								else tracking_w = -1.0*stop_w_max;
						}
						std::cout<<"angular_error  "<< angular_error <<std::endl;
            if(fabs(angular_error) <= 0.05)
						{
								isFInish = true;
						    tracking_v = 0;
						    tracking_w = 0;
						}
        }
				//std::cout<<"tracking_v = "<<tracking_v<<" tracking_v = "<<tracking_w<<std::endl;
				std::vector<unsigned char> command;
				//sendreceive.Package_Diff_encoder(tracking_v,tracking_w,command);
				sendreceive.Package_publicWheel_encoder(tracking_v, 0, tracking_w, 0, command);
				SendPackage(command);
				std::cout<<"=================================="<<std::endl;
				if(isFInish)
				{
						Endangle = false;
						isFInish = false;
						tracking_v = 0.0;
						tracking_w = 0.0;
						pre_angular_error = 0.0;
						//pre_dis_error = 0.0;
						pre_angular_velocity_error = 0.0;
						transfer_odom = false;//new add odom
						oFile.close();
						return true;
				}
		}
		else
		{
				Endangle = false;
				isFInish = false;
				tracking_v = 0.0;
				tracking_w = 0.0;
				pre_angular_error = 0.0;
				//pre_dis_error = 0.0;
				pre_angular_velocity_error = 0.0;
		}
		return false;
}


void diff_dwa::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
		ros::Duration dur (0.5);
		LaserMiss = 0;
		sensor_msgs::PointCloud2 cloud;
		try
		{
		    projector_1_.transformLaserScanToPointCloud("base_link", *scan, cloud, tf_buffer_);
		    sensor_msgs::convertPointCloud2ToPointCloud(cloud, laser_point_cloud_1_);
		}
		catch(tf::TransformException &ex)
		{

		}
		dwa_obs_region.clear();
		dwa_obs_robot_region.clear();
		for(int i=0;i<laser_point_cloud_1_.points.size();++i)
		{
		    const geometry_msgs::Point32& currPoint(laser_point_cloud_1_.points[i]);
		    Eigen::Vector3f robot_laser_pose(currPoint.x, currPoint.y, 0.0);
		    float real_x, real_y;
		    tra.robot2world(robot_laser_pose, slam_pose_, real_x, real_y);
				Eigen::Vector3f laser_pose(real_x, real_y, 0.0);
				if(robot_laser_pose.x() > 0.4 && robot_laser_pose.x() < 2.0 && fabs(robot_laser_pose.y()) < 2.0)
				{
						dwa_obs_region.push_back(laser_pose);
						dwa_obs_robot_region.push_back(robot_laser_pose);
				}
		}
		//std::cout<<"dwa_obs_region.size() = "<<dwa_obs_region.size()<<std::endl;
}

//==============OBS=================
// void diff_dwa::laserCallback(const sensor_msgs::LaserScan& scan)
// {
//
// 	ros::Duration dur (0.5);
//
// 	LaserMiss = 0;
// 	//std::cout<<"ONE LASER"<<std::endl;
// 	//Receive LaserData
// 	int device_num = checkDeviceNum(scan);
//
// 	sensor_msgs::LaserScan device_scan_1;
// 	sensor_msgs::LaserScan device_scan_2;
//
// 	if(device_num == 1){
//
// 		device_scan_1.header          = scan.header;
// 		device_scan_1.angle_min       = scan.angle_min;
// 		device_scan_1.angle_max       = scan.angle_max;
// 		device_scan_1.angle_increment = scan.angle_increment;
// 		device_scan_1.time_increment  = scan.time_increment;
// 		device_scan_1.scan_time       = scan.scan_time;
// 		device_scan_1.range_min       = scan.range_min;
// 		device_scan_1.range_max       = scan.range_max;
// 		device_scan_1.ranges.resize(scan.ranges.size());
// 		device_scan_1.intensities.resize(scan.ranges.size());
//
// 		for(int i=0; i<scan.ranges.size(); i++)
// 			device_scan_1.ranges[i] = scan.ranges[i];
//
// 	}
// 	else{
//
// 		device_scan_1.header          = scan.header;
// 		device_scan_1.angle_min       = scan.angle_min;
// 		device_scan_1.angle_max       = scan.angle_max;
// 		device_scan_1.angle_increment = scan.angle_increment;
// 		device_scan_1.time_increment  = scan.time_increment;
// 		device_scan_1.scan_time       = scan.scan_time;
// 		device_scan_1.range_min       = scan.range_min;
// 		device_scan_1.range_max       = scan.range_max;
// 		device_scan_1.ranges.resize(scan.ranges.size()/2);
// 		device_scan_1.intensities.resize(scan.ranges.size()/2);
//
// 		device_scan_2.header          = scan.header;
// 		std::string frame_id_2_       = std::string("base_laser_link_2");
// 		device_scan_2.header.frame_id = frame_id_2_;
// 		device_scan_2.angle_min       = scan.angle_min;
// 		device_scan_2.angle_max       = scan.angle_max;
// 		device_scan_2.angle_increment = scan.angle_increment;
// 		device_scan_2.time_increment  = scan.time_increment;
// 		device_scan_2.scan_time       = scan.scan_time;
// 		device_scan_2.range_min       = scan.range_min;
// 		device_scan_2.range_max       = scan.range_max;
// 		device_scan_2.ranges.resize(scan.ranges.size()/2);
// 		device_scan_2.intensities.resize(scan.ranges.size()/2);
//
//
// 		for(int i=0; i<(scan.ranges.size()/2); i++){
// 			device_scan_1.ranges[i] = scan.ranges[i];
// 			device_scan_2.ranges[i] = scan.ranges[i+(scan.ranges.size()/2)];
// 		}
// 	}
//
//
// 	if (tf_.waitForTransform(p_base_frame_,device_scan_1.header.frame_id, device_scan_1.header.stamp,dur))
// 	{
//
// 		tf::StampedTransform laserTransform_1;
//
// 		tf_.lookupTransform(p_base_frame_,device_scan_1.header.frame_id, device_scan_1.header.stamp, laserTransform_1);
//
// 		if(device_num == 2)
// 		{
// 			if (tf_.waitForTransform(p_base_frame_,device_scan_2.header.frame_id, device_scan_2.header.stamp,dur))
// 			{
// 				tf::StampedTransform laserTransform_2;
//
// 				tf_.lookupTransform(p_base_frame_,device_scan_2.header.frame_id, device_scan_2.header.stamp, laserTransform_2);
//
//
// 				projector_1_.projectLaser(device_scan_1, laser_point_cloud_1_,20.0);
//
// 				projector_2_.projectLaser(device_scan_2, laser_point_cloud_2_,20.0);
//
//
// 				size_t laser_size_1 = laser_point_cloud_1_.points.size();
//
// 				size_t laser_size_2 = laser_point_cloud_2_.points.size();
//
// 				tf::Vector3 laserPos_1 (laserTransform_1.getOrigin());
//
// 				tf::Vector3 laserPos_2 (laserTransform_2.getOrigin());
//
// 				std::vector<Eigen::Vector3f> obs_container;
//
//
// 				std::vector<Eigen::Vector3f> obs_region_0; //-90 ~ -54
// 				std::vector<Eigen::Vector3f> obs_region_1; //-54 ~ -18
// 				std::vector<Eigen::Vector3f> obs_region_2; //-18 ~ 18
// 				std::vector<Eigen::Vector3f> obs_region_3; // 18 ~ 54
// 				std::vector<Eigen::Vector3f> obs_region_4; // 54 ~ 90
//
//
// 				for (size_t i = 0; i < laser_size_1; ++i){
//
// 					const geometry_msgs::Point32& currPoint(laser_point_cloud_1_.points[i]);
//
// 					float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;
//
// 					if ( (dist_sqr > 0.05) && (dist_sqr < 20) ){
//
// 						if ( (currPoint.x < 0.0f) && (dist_sqr < 0.2f)){
// 							continue;
// 						}
//
// 						tf::Vector3 pointPosBaseFrame(laserTransform_1 * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));
//
// 						double center_dis = sqrt(pointPosBaseFrame.x()*pointPosBaseFrame.x()+ pointPosBaseFrame.y()*pointPosBaseFrame.y());
//
// 						float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos_1.z();
//
// 						if (pointPosLaserFrameZ > -1 && pointPosLaserFrameZ < 1)
// 						{
// 							Eigen::Vector3f p(pointPosBaseFrame.x(), pointPosBaseFrame.y(), pointPosBaseFrame.z());
// 							obs_container.push_back(p);
//
// 						}
// 					}
// 				}
//
//
// 				for (size_t i = 0; i < laser_size_2; ++i)
// 				{
//
// 					const geometry_msgs::Point32& currPoint(laser_point_cloud_2_.points[i]);
//
// 					float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;
//
// 					if ( (dist_sqr > 0.05) && (dist_sqr < 20) ){
//
// 						if ( (currPoint.x < 0.0f) && (dist_sqr < 0.2f)){
// 							continue;
// 						}
//
// 						tf::Vector3 pointPosBaseFrame(laserTransform_2 * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));
//
// 						double center_dis = sqrt(pointPosBaseFrame.x()*pointPosBaseFrame.x()+ pointPosBaseFrame.y()*pointPosBaseFrame.y());
//
// 						float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos_2.z();
//
// 						if (pointPosLaserFrameZ > -1 && pointPosLaserFrameZ < 1)
// 						{
//
// 							Eigen::Vector3f p(pointPosBaseFrame.x(), pointPosBaseFrame.y(), pointPosBaseFrame.z());
// 							obs_container.push_back(p);
//
// 						}
// 					}
//
// 				}
// 				std::vector<Eigen::Vector3f> obs_gap_0;
// 				std::vector<Eigen::Vector3f> obs_gap_1;
// 				std::vector<Eigen::Vector3f> obs_gap_2;
// 				std::vector<Eigen::Vector3f> obs_gap_3;
// 				std::vector<Eigen::Vector3f> obs_gap_4;
// 				std::vector<Eigen::Vector3f> all_point;
//
// 				//--------------------------------------------//
// 				if(command_OBSMode != 1){
// 					obs_region_0.clear();
// 					obs_region_1.clear();
// 					obs_region_2.clear();
// 					obs_region_3.clear();
// 					obs_region_4.clear();
//
// 					//ObsDetect(obs_container, obs_region_0, obs_region_1, obs_region_2, obs_region_3, obs_region_4,obs_gap_0,obs_gap_1,obs_gap_2,obs_gap_3,obs_gap_4,all_point);
//
// 					//ObsStateMachine(obs_region_0, obs_region_1, obs_region_2, obs_region_3, obs_region_4,obs_gap_0,obs_gap_1,obs_gap_2,obs_gap_3,obs_gap_4,all_point);
// 					obs_container.clear();
//
// 				}
//
// 			}
// 		}
// 		else
// 		{//std::cout<<"ONE LASER"<<std::endl;
// 			projector_1_.projectLaser(device_scan_1, laser_point_cloud_1_,20.0);
//
// 			size_t laser_size_1 = laser_point_cloud_1_.points.size();
//
// 			tf::Vector3 laserPos_1 (laserTransform_1.getOrigin());
//
//
// 			std::vector<Eigen::Vector3f> obs_container;
//
//
// 			std::vector<Eigen::Vector3f> obs_region_0; //-90 ~ -54
// 			std::vector<Eigen::Vector3f> obs_region_1; //-54 ~ -18
// 			std::vector<Eigen::Vector3f> obs_region_2; //-18 ~ 18
// 			std::vector<Eigen::Vector3f> obs_region_3; // 18 ~ 54
// 			std::vector<Eigen::Vector3f> obs_region_4; // 54 ~ 90
//
// 			Eigen::Vector3f robot_pos;
// 			robot_pos = slam_pose_;
// 			std::fstream fout;
// 			fout.open("robot_pos.xlsx",std::fstream::out);
// 			fout<<"===================" <<std::endl;
// 			fout<<robot_pos.x() <<"\t" <<robot_pos.y() <<"\t" <<robot_pos.z() <<std::endl;
// 			fout<<"===================" <<std::endl;
//
//
//
// 			for (size_t i = 0; i < laser_size_1; ++i){
//
// 				const geometry_msgs::Point32& currPoint(laser_point_cloud_1_.points[i]);
//
// 				float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;
//
// 				if ( (dist_sqr > 0.05) && (dist_sqr < 20) ){
//
// 					if ( (currPoint.x < 0.0f) && (dist_sqr < 0.2f)){
// 						continue;
// 					}
//
// 					tf::Vector3 pointPosBaseFrame(laserTransform_1 * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));
//
// 					double center_dis = sqrt(pointPosBaseFrame.x()*pointPosBaseFrame.x()+ pointPosBaseFrame.y()*pointPosBaseFrame.y());
//
// 					float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos_1.z();
//
// 					if (pointPosLaserFrameZ > -1 && pointPosLaserFrameZ < 1)
// 					{
// 						Eigen::Vector3f p(pointPosBaseFrame.x(), pointPosBaseFrame.y(), pointPosBaseFrame.z());
// 						obs_container.push_back(p);
// 						fout<<p.x() <<"\t" <<p.y()  <<std::endl;
//
// 					}
// 				}
// 			}
// 			fout.close();
//
// 			std::vector<Eigen::Vector3f> obs_gap_0;
// 			std::vector<Eigen::Vector3f> obs_gap_1;
// 			std::vector<Eigen::Vector3f> obs_gap_2;
// 			std::vector<Eigen::Vector3f> obs_gap_3;
// 			std::vector<Eigen::Vector3f> obs_gap_4;
// 			std::vector<Eigen::Vector3f> all_point;
//
// 			//--------------------------------------------//
// 			if(command_OBSMode != 1){
// 				obs_region_0.clear();
// 				obs_region_1.clear();
// 				obs_region_2.clear();
// 				obs_region_3.clear();
// 				obs_region_4.clear();
// 				dwa_obs_region.clear();
// 				dwa_obs_robot_region.clear();
//
// 				DWA_ObsDetect(obs_container, dwa_obs_region, dwa_obs_robot_region);
// 				//ObsDetect(obs_container, obs_region_0, obs_region_1, obs_region_2, obs_region_3, obs_region_4,obs_gap_0,obs_gap_1,obs_gap_2,obs_gap_3,obs_gap_4,all_point);
//
// 				// ObsStateMachine(obs_region_0, obs_region_1, obs_region_2, obs_region_3, obs_region_4,obs_gap_0,obs_gap_1,obs_gap_2,obs_gap_3,obs_gap_4,all_point);
// 				obs_container.clear();
//
// 			}
//
// 		}
//
// 	}
//
//
// }

void diff_dwa::DWA_ObsDetect(std::vector<Eigen::Vector3f> &i_obs_container,
		std::vector<Eigen::Vector3f> &dwa_obs_region, std::vector<Eigen::Vector3f> &dwa_obs_robot_region)
{
		for(int i=0;i<i_obs_container.size();i++)
		{
				Eigen::Vector3f p_buf = i_obs_container[i];
				i_obs_container[i].x() = p_buf.x() *cos(-1*slam_pose_.z()) - p_buf.y() * sin(-1*slam_pose_.z());
				i_obs_container[i].y() = p_buf.x() *sin(-1*slam_pose_.z()) + p_buf.y() * cos(-1*slam_pose_.z());
				i_obs_container[i].z() = 0.0;
				if((i_obs_container[i].x() > 0 && i_obs_container[i].x() < 2.4 && fabs(i_obs_container[i].y()) < 2.4))
				{
						float dwa_obs_x, dwa_obs_y;
						tra.robot2world(i_obs_container[i], slam_pose_, dwa_obs_x, dwa_obs_y);
						Eigen::Vector3f dwa_obs_buf(dwa_obs_x, dwa_obs_y, 0.0);
						dwa_obs_region.push_back(p_buf);
						dwa_obs_robot_region.push_back(i_obs_container[i]);
				}
		}
		if(dwa_obs_region.size() < 10) dwa_obs_region.clear();
		if(dwa_obs_robot_region.size() < 10) dwa_obs_robot_region.clear();
}

void diff_dwa::ObsDetect(std::vector<Eigen::Vector3f> &i_obs_container,
		std::vector<Eigen::Vector3f> &i_obs_region_0,
		std::vector<Eigen::Vector3f> &i_obs_region_1,
		std::vector<Eigen::Vector3f> &i_obs_region_2,
		std::vector<Eigen::Vector3f> &i_obs_region_3,
		std::vector<Eigen::Vector3f> &i_obs_region_4,
		std::vector<Eigen::Vector3f> &i_obs_gap_0,
		std::vector<Eigen::Vector3f> &i_obs_gap_1,
		std::vector<Eigen::Vector3f> &i_obs_gap_2,
		std::vector<Eigen::Vector3f> &i_obs_gap_3,
		std::vector<Eigen::Vector3f> &i_obs_gap_4,
		std::vector<Eigen::Vector3f> &all_point)
{
	//遇到障礙物減速用

	std::vector<Eigen::Vector3f> i_obs_region_2_buf;


	int obs_num_limit = OBS_isObs;


	float car_warn_theta=fabs(obs_way_theta);
	if(car_warn_theta>M_PI/2)
		car_warn_theta=M_PI-car_warn_theta;


	float Car_Front = CarLength* cos(car_warn_theta)/2 +  CarWidth* sin(car_warn_theta)/2 ;
	float Car_Side = CarWidth * cos(car_warn_theta)/2 + CarLength * sin(car_warn_theta)/2;

	//進入避障參數
	float protect_OBS_Side = Car_Side + OBS_Side;
	float protect_OBS_Front = Car_Front + OBS_Front;
	float protect_OBS_LimitFront = Car_Front + OBS_Front_limit;
	float protect_OBS_Front_avoid = Car_Front + (OBS_Front+ OBS_Front_limit)/2;


	if(obs_way_theta !=0)
		for(int i=0;i<i_obs_container.size();i++)
		{
			Eigen::Vector3f p_buf = i_obs_container[i];
			i_obs_container[i].x() = p_buf.x() *cos(-1*obs_way_theta) - p_buf.y() * sin(-1*obs_way_theta);
			i_obs_container[i].y() = p_buf.x() *sin(-1*obs_way_theta) + p_buf.y() * cos(-1*obs_way_theta);

		}

	for(int i=0; i<i_obs_container.size(); i++){
		Eigen::Vector3f p = i_obs_container[i];
		//一般避障區間
		if(p.x() < protect_OBS_Front && p.x() > -1*Car_Front){
			if( fabs(p.y()) < OBS_SideMiss_Scale*protect_OBS_Side)
			{

				if(p.y() < -1*protect_OBS_Side && p.x() < Car_Front)       i_obs_region_4.push_back(p);
				else if(p.y() < -1*protect_OBS_Side && p.x() >= Car_Front)  i_obs_region_3.push_back(p);
				//else if(p.x()>0 &&  fabs(p.y()) <= protect_OBS_Side && !OBS_limilMode  ) i_obs_region_2.push_back(p);
				else if(p.x()>0 &&  fabs(p.y()) <= protect_OBS_Side )
				{
					if(OBS_limilMode || command_OBSMode==2)//再轉彎時 與 閉障結束回到正常路徑時（diff） 閉障縮短
					{

						if( p.x()<protect_OBS_LimitFront) i_obs_region_2.push_back(p);
						all_obs_dis = protect_OBS_LimitFront;
					}
					else if(isAvoidObs)// 閉障時會把閉障距離縮短
					{

						if( p.x()< protect_OBS_Front_avoid) i_obs_region_2.push_back(p);
						all_obs_dis = protect_OBS_Front_avoid;
					}
					else
					{
						i_obs_region_2.push_back(p);
						all_obs_dis = protect_OBS_Front;

					}

				}
				else if(p.y() > protect_OBS_Side && p.x() >= Car_Front)  i_obs_region_1.push_back(p);
				else if(p.y() > protect_OBS_Side && p.x() < Car_Front)  i_obs_region_0.push_back(p);

			}
		}
		//避障減速區間
		if(p.x() < (protect_OBS_Front *2) && p.x() > 0){
			if(fabs(p.y()) <= all_obs_dis   )i_obs_region_2_buf.push_back(p);
		}
		//gap觀看區間
		if(p.x() < OBS_LookGap && p.x() > 0.0)
		{
			if(fabs(p.y()) < OBS_LookGap)
			{
				if(p.y() < -1*protect_OBS_Side) i_obs_gap_4.push_back(p);
				else if (fabs(p.y()) <= protect_OBS_Side && p.x() < all_obs_dis * 2.0 ) i_obs_gap_2.push_back(p);
				else if (p.y() > protect_OBS_Side )  i_obs_gap_0.push_back(p);

			}

			if(p.x() > (Car_Front) )  all_point.push_back(p);
		}
		///////////////////////////////////跨樓層用
		if(Command_STATE == Command_Elevator_Entrance || Command_STATE == Command_Elevator_Inside)
		{
			//std::cout<<" Elevator is close "<< i_obs_gap_2.size()<<std::endl;
			if(i_obs_gap_2.size() < 15)
			{
				///std::cout<<" Elevator is open" <<std::endl;
				ElevatorGO = true;
				//td::cout<<"==obsd ElevatorGO==" <<ElevatorGO<<std::endl;
			}
			else
			{
				ElevatorGO = false;
			}
		}
		/////////////////////////////////////
	}

	if(i_obs_region_2_buf.size() >= obs_num_limit){
		decay_obs_v = OBS_Decay_V;
	}
	else
	{
		decay_obs_v = 0;
	}


	//畫出軌跡再rviz
	Eigen::Vector3f robot_pos;
	robot_pos = slam_pose_;
	float i_obs_region_2_buf_x = 0;
	float i_obs_region_2_buf_y = 0;
	std::vector<Eigen::Vector3f> look_front;
	for(int i=0;i<i_obs_region_2.size();i++)
	{
		tra.robot2world(i_obs_region_2[i],robot_pos,obs_way_theta,i_obs_region_2_buf_x,i_obs_region_2_buf_y);

		Eigen::Vector3f p ;
		p <<i_obs_region_2_buf_x , i_obs_region_2_buf_y, 1.0;
		look_front.push_back(p);
	}
	//畫出軌跡再rviz
	draw(OBSPointMarker , 1.0, 1.0, 1.0, look_front);
	//畫出軌跡再rviz


}

void diff_dwa::ObsStateMachine(std::vector<Eigen::Vector3f> &i_obs_region_0,
		std::vector<Eigen::Vector3f> &i_obs_region_1,
		std::vector<Eigen::Vector3f> &i_obs_region_2,
		std::vector<Eigen::Vector3f> &i_obs_region_3,
		std::vector<Eigen::Vector3f> &i_obs_region_4,
		std::vector<Eigen::Vector3f> &i_obs_gap_0,
		std::vector<Eigen::Vector3f> &i_obs_gap_1,
		std::vector<Eigen::Vector3f> &i_obs_gap_2,
		std::vector<Eigen::Vector3f> &i_obs_gap_3,
		std::vector<Eigen::Vector3f> &i_obs_gap_4,
		std::vector<Eigen::Vector3f> &all_point)
{
	static int cnt = 0;
	int cnt_limit = 20;

	int obs_num_limit = OBS_isObs;

	if(AvoidObs_state == P_OBS_NORMAL){
		if(i_obs_region_2.size() > obs_num_limit){
			isFindObs = true;
		}
		else{
			isFindObs = false;
		}
	}
	else if(AvoidObs_state == P_OBS_WAIT){
		if(i_obs_region_2.size() > obs_num_limit){
			isFindObs = true;
		}
		else{
			isFindObs = false;
		}
	}
	else if(AvoidObs_state == P_OBS_REPLANNING){
		isFindObs = false;
		RePlanning(last_subpath_index, i_obs_gap_0, i_obs_gap_1, i_obs_gap_2, i_obs_gap_3, i_obs_gap_4, all_point);
	}
	else if(AvoidObs_state == P_OBS_AVOID){

		if(isblind){
			isFindObs = false;
			isAvoidObs = true;

		}
		else if(i_obs_region_2.size() >= obs_num_limit){
			isFindObs = true;
			isAvoidObs = false;
			cnt = 0;

		}
		else if(avoid_way < 0 &&
				i_obs_region_0.size() < obs_num_limit &&
				i_obs_region_1.size() < obs_num_limit){

			if(cnt >= cnt_limit){
				isFindObs = false;
				isAvoidObs = false;
				cnt = 0;

			}
			else{
				isFindObs = false;
				isAvoidObs = true;

				cnt ++;
			}


		}
		else if(avoid_way > 0 &&
				i_obs_region_3.size() < obs_num_limit &&
				i_obs_region_4.size() < obs_num_limit){
			std::cout<<"==============avoid_way >0==============  "<<std::endl;
			if(cnt >= cnt_limit){
				isFindObs = false;
				isAvoidObs = false;
				cnt = 0;

			}
			else{
				isFindObs = false;
				isAvoidObs = true;


				cnt ++;
			}
		}
		else{
			isFindObs = false;
			isAvoidObs = true;
			cnt = 0;
		}



	}
	else if(AvoidObs_state == P_OBS_DEADLOCK){
		if( i_obs_region_2.size() > obs_num_limit ){
			isFindObs = true;
		}
		else{
			isFindObs = false;
		}
	}


}
bool diff_dwa::WaitObsLeave()
{

	static bool isInitial = true;
	static bool isStop = false;
	static int Acceleration_count = 0;

	static int delay_count_1 = 0;
	static int delay_count_2 = 0;

	static float start_v = 0;
	static float start_w = 0;
	static float decay_v = 0;


	float Acceleration_time = OBS_Stop_Sec; //s
	int Acceleration_count_limit = int(Acceleration_time/time_sample);


	float delay_time = OBS_Wait_Sec; //s
	int delay_count_limit = int(delay_time/time_sample);

	if(isInitial){
		start_v = global_v;
		decay_v = -1*start_v/Acceleration_count_limit;
		isInitial = false;
		isStop = false;
	}
	else{

		if(!isStop){


			Acceleration_count += 1;

			if(Acceleration_count < Acceleration_count_limit){
				start_v += decay_v;

			}
			else{
				start_v = 0;
				isStop = true;
				Acceleration_count = 0;
			}

		}
		else{

			if(isFindObs || isAvoidObs){

				delay_count_2 = 0;

				if(delay_count_1 < delay_count_limit){
					delay_count_1 += 1;
				}
				else{
					isInitial = true;
					isStop = false;
					Acceleration_count = 0;
					delay_count_1 = 0;
					delay_count_2 = 0;


					start_v = 0;
					start_w = 0;
					decay_v = 0;

					p_state_ = P_STATE_AVOID_REPLANNING;
					AvoidObs_state = P_OBS_REPLANNING;

					return true;
				}

			}
			else{

				delay_count_1 = 0;

				if(delay_count_2 < delay_count_limit){
					delay_count_2 += 1;
				}
				else{
					isInitial = true;
					isStop = false;
					Acceleration_count = 0;
					delay_count_1 = 0;
					delay_count_2 = 0;

					start_v = 0;
					start_w = 0;
					decay_v = 0;


					p_state_ = P_STATE_MOVE;
					AvoidObs_state = P_OBS_NORMAL;
					return true;
				}
			}

		}

		global_v = start_v;


		std::vector<unsigned char> command;


			// sendreceive.Package_Diff_encoder(start_v,start_w,
			// 		command
			// 		);
		sendreceive.Package_publicWheel_encoder(start_v, 0, start_w, 0, command);

		SendPackage(command);


	}

	return false;

}
void diff_dwa::RePlanning(int &subpath_index,
		std::vector<Eigen::Vector3f> &i_obs_region_0,
		std::vector<Eigen::Vector3f> &i_obs_region_1,
		std::vector<Eigen::Vector3f> &i_obs_region_2,
		std::vector<Eigen::Vector3f> &i_obs_region_3,
		std::vector<Eigen::Vector3f> &i_obs_region_4,
		std::vector<Eigen::Vector3f> &all_point)
{


	ROS_INFO("-------------------");
	ROS_INFO("RePlanning");

	//這是最終佔點，用來告知到底目前位置能不能避障
	Eigen::Vector3f target_pos = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint.size()-1];
	Eigen::Vector3f robot_pos = slam_pose_;

	//找到最近得障礙物點並向左或右延伸的點，用來與目標點做新路徑
	Eigen::Vector3f Key_Point;

	float p_mobile_width = CarWidth;
	float p_mobile_length = CarLength;
	float dis_min = sqrt(p_mobile_width*p_mobile_width + p_mobile_length*p_mobile_length)/2.0;

	float x_error = target_pos.x() - robot_pos.x();
	float y_error = target_pos.y() - robot_pos.y();
	float dis_error = sqrt(x_error*x_error + y_error*y_error);

	bool isWorkable = false;

	if(dis_error > dis_min){

		int now_index = 0;
		float odom_v = 0.8;

		//得到前視距離點
		int target_ind = calc_target_index_obs(robot_pos, odom_v, A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint, now_index);

		target_pos = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[target_ind];

		//如果是二次避障階段 要使用閉障之路徑再規劃
		if(obs_avoid_flag)
		{
			target_ind = calc_target_index_obs(robot_pos, v_buf, avoid_path, now_index);
			target_pos = avoid_path[target_ind];


		}

		isWorkable = FindKeyPoint(target_pos, Key_Point, i_obs_region_0, i_obs_region_2, i_obs_region_4 ,all_point);

		if(isWorkable){
			CreateNewPath(target_pos, Key_Point, subpath_index, avoid_path);

			ROS_INFO("----------------");
			//ROS_INFO("NEW target_pos: %f, %f", target_pos.x(), target_pos.y());
			ROS_INFO("NEW Key_Point: %f, %f", Key_Point.x(), Key_Point.y());


			p_state_ = P_STATE_AVOID_OBS;
			AvoidObs_state = P_OBS_AVOID;
			isblind = true;
			isAvoidObs = true;
			std::cout<<"==isblind==rep== "<<isblind<<std::endl;
		}
		else{
			//找不到key_point dead
			p_state_ = P_STATE_AVOID_DEADLOCK;
			AvoidObs_state = P_OBS_DEADLOCK;
			ROS_INFO(" DEAD");
		}

	}
	else{
		//離站點太近了 會影響停車，dead
		p_state_ = P_STATE_AVOID_DEADLOCK;
		AvoidObs_state = P_OBS_DEADLOCK;
		ROS_INFO(" DEAD");
	}


	ROS_INFO("RePlanning OFF");

}

bool diff_dwa::FindKeyPoint(Eigen::Vector3f& target, Eigen::Vector3f& key_p,
		std::vector<Eigen::Vector3f> &i_obs_region_0,
		std::vector<Eigen::Vector3f> &i_obs_region_2,
		std::vector<Eigen::Vector3f> &i_obs_region_4,
		std::vector<Eigen::Vector3f> &all_point)
{
	std::cout<<"find"<<std::endl;
	//含雷達的長寬
	float p_mobile_width = CarWidth;
	float p_mobile_length = CarLength;

	//float scalar = 1.0;

	//基本淘汰完全不必算之部份
	float gap_limit = p_mobile_width + OBS_Side;


	std::vector<float> gap_array;
	std::vector<Eigen::Vector3f> point_array;
	Eigen::Vector3f robot_pos = slam_pose_;


	Eigen::Vector3f target_robot;

	Eigen::Vector3f _robot;
	_robot<<0.0 ,0.0 ,1.0;
	Eigen::Vector3f move_line;
	Eigen::Vector3f obs_line;
	Eigen::Vector3f vertical_line;
	std::vector<Eigen::Vector3f> obs_line_point;



	bool flag_right = false;
	bool flag_left = false;
	bool isWorkable = false;

	int obs_num_limit = OBS_isObs;

	float x=0.0;
	float y=0.0;

	tra.world2robot(target, robot_pos,  x,  y);
	target_robot.x() = x;
	target_robot.y() = y;
	target_robot.z() = 1.0;


	move_line = _robot.cross(target_robot);
	std::cout<<"see 1 "<<std::endl;
	//判斷我要閉障時，本身左右有沒有障礙物
	//右邊
	int count_dis =0;
	for(int i=0;i<i_obs_region_0.size() ;i++)
	{
		if(i_obs_region_0.size() >= obs_num_limit)
		{
			if(i_obs_region_0[i].x()< target_robot.x()/2)
			{
				float obs_dis = fabs(move_line.x() *i_obs_region_0[i].x() +move_line.y() *i_obs_region_0[i].y() +move_line.z())/sqrt(move_line.x() * move_line.x() +move_line.y()*move_line.y());
				if(obs_dis < OBS_Side*2)
				{
					count_dis++;
					std::cout<<"count_dis  ri "<<count_dis<<std::endl;
				}
			}
		}
	}
	if(count_dis > obs_num_limit)
	{
		flag_right = true;
	}
	std::cout<<"see 2 "<<std::endl;
	//左邊
	count_dis =0;
	for(int i=0;i<i_obs_region_4.size() ;i++)
	{
		if(i_obs_region_4.size() >= obs_num_limit)
		{
			if(i_obs_region_4[i].x()< target_robot.x()/2)
			{
				float obs_dis = fabs(move_line.x() * i_obs_region_4[i].x() +move_line.y() *i_obs_region_4[i].y() +move_line.z())/sqrt(move_line.x() * move_line.x() +move_line.y()*move_line.y());
				if(obs_dis < OBS_Side*2)
				{
					count_dis++;
					std::cout<<"count_dis  le "<<count_dis<<std::endl;
				}
			}
		}
	}
	if(count_dis > obs_num_limit)
	{
		flag_left = true;
	}
	count_dis =0;


	std::cout<<"see 3 "<<std::endl;
	int tan_min_i =0 ;

	//左右都無障礙物
	if(!flag_left || !flag_right)
	{
		//float obs_dis_limit=10000.0;

		//找離我閉障方向最靠近正前方的點
		float tan_min = 1.57;
		for(int i=0;i<i_obs_region_2.size() ;i++)
		{
			if(i_obs_region_2[i].x()> -1*p_mobile_length)
			{
				float tan = fabs(atan2(i_obs_region_2[i].y(),i_obs_region_2[i].x()));
				if(tan<tan_min)
				{
					tan_min = tan;
					tan_min_i=i;
				}
			}
		}


		//障礙物線的計算
		i_obs_region_2[tan_min_i].z()=1.0;
		Eigen::Vector3f obs_i_obs_region_2;
		obs_i_obs_region_2<<i_obs_region_2[tan_min_i].x() ,i_obs_region_2[tan_min_i].y()+0.1 ,1.0;
		obs_line = obs_i_obs_region_2.cross(i_obs_region_2[tan_min_i]);

		float scale = sqrt(obs_line.x() * obs_line.x() + obs_line.y() * obs_line.y());
		Eigen::Vector3f normalize_obs_line_Line;
		normalize_obs_line_Line << obs_line.x()/scale,obs_line.y()/scale,obs_line.z()/scale;

		//畫obsline的線
		std::vector<Eigen::Vector3f> drawline_;
		Eigen::Vector3f linepoint;
		linepoint << 0, 0, 0;

		float y = 8.0;
		float x = (-normalize_obs_line_Line.z() - normalize_obs_line_Line.y()*y)/normalize_obs_line_Line.x();
		linepoint << x, y, 1.0;
		drawline_.push_back(linepoint);


		x = (-normalize_obs_line_Line.z() + normalize_obs_line_Line.y()*y)/normalize_obs_line_Line.x();
		linepoint << x, -y, 1.0;
		drawline_.push_back(linepoint);

		for(int i = 0;i<drawline_.size();i++)
		{
			tra.robot2world(drawline_[i],robot_pos,obs_way_theta,x,y);
			drawline_[i].x() = x;
			drawline_[i].y() = y;

		}

		drawLine( obs_LineMarker, 0.0, 1.0, 0.0, drawline_);
		//畫obsline的線



		std::cout<<"see 4 "<<std::endl;
		//找平行於車子的閉障線，把侷限於障礙區區域畫出來

		std::vector<Eigen::Vector3f> all_point_world;
		std::vector<Eigen::Vector3f> obs_line_point_world;

		for(int i=0;i<all_point.size() ;i++)
		{
			//if(all_point[i].x()> 0 && all_point[i].x() < all_obs_dis )
			{	all_point[i].z() = 1.0;
				//float obs_dis = fabs(obs_line.x() *all_point[i].x() +obs_line.y() *all_point[i].y() +obs_line.z())/sqrt(obs_line.x() * obs_line.x() +obs_line.y()*obs_line.y());
				float obs_dis = fabs(normalize_obs_line_Line.transpose() * all_point[i]);
				if(obs_dis < 0.6)
				{
					obs_line_point.push_back(all_point[i]);

				}

			}
			//畫出軌跡再rviz
			float all_point_buf_x = 0;
			float all_point_buf_y = 0;
			tra.robot2world(all_point[i],robot_pos,obs_way_theta,all_point_buf_x,all_point_buf_y);

			Eigen::Vector3f p ;
			p <<all_point_buf_x , all_point_buf_y, 1.0;
			all_point_world.push_back(p);


		}


		// //垂直障礙物線的計算
		Eigen::Vector3f vertical_p;
		//vertical_p <<robot_pos.x() ,robot_pos.y() ,1.0;
		vertical_p <<i_obs_region_2[tan_min_i].x()+0.1 ,i_obs_region_2[tan_min_i].y() ,1.0;
		vertical_line = vertical_p.cross(i_obs_region_2[tan_min_i]);

		scale = sqrt(vertical_line.x() * vertical_line.x() + vertical_line.y() * vertical_line.y());
		Eigen::Vector3f normalize_vertical_line;
		normalize_vertical_line << vertical_line.x()/scale,vertical_line.y()/scale,vertical_line.z()/scale;

		int right_cnt = 0;
		int left_cnt = 0;

		for(int i = 0;i <obs_line_point.size() ;i++)
		{
			obs_line_point[i].z() = 1.0;
			float obs_dis = normalize_vertical_line.transpose() * obs_line_point[i];
			if(obs_dis > 0)
				right_cnt +=1;
			else
				left_cnt +=1;
		}

		//如果發現都無障礙物，會直製造無線遠的點讓他知道能通過
		std::cout<<"right_cnt  "<<right_cnt<<std::endl;
		std::cout<<"left_cnt  "<<left_cnt<<std::endl;
		if(right_cnt < 1)
		{
			Eigen::Vector3f obs_right;
			obs_right<<i_obs_region_2[tan_min_i].x() ,i_obs_region_2[tan_min_i].y()- 10.0 ,1.0;
			obs_line_point.push_back(obs_right);
		}
		if(left_cnt < 1)
		{
			Eigen::Vector3f obs_left;
			obs_left<<i_obs_region_2[tan_min_i].x() ,i_obs_region_2[tan_min_i].y()+ 10.0 ,1.0;
			obs_line_point.push_back(obs_left);
		}


		//畫vertical_line的線
		std::vector<Eigen::Vector3f> drawline_vertical;
		Eigen::Vector3f linepoint_vertical;
		linepoint_vertical << 0, 0, 0;

		y = (-normalize_vertical_line.z()-normalize_vertical_line.z())/normalize_vertical_line.y();
		x = 5.0;
		linepoint_vertical << x, y, 1.0;
		drawline_vertical.push_back(linepoint_vertical);


		y = (-normalize_vertical_line.z()-normalize_vertical_line.z()*-1)/normalize_vertical_line.y();
		linepoint_vertical << -x, y, 1.0;
		drawline_vertical.push_back(linepoint_vertical);

		for(int i = 0;i<drawline_vertical.size();i++)
		{
			tra.robot2world(drawline_vertical[i],robot_pos,obs_way_theta,x,y);
			drawline_vertical[i].x() = x;
			drawline_vertical[i].y() = y;

		}

		drawLine( vertical_lineMarker, 0.0, 0.0, 1.0, drawline_vertical);


		for(int i = 0;i <obs_line_point.size() ;i++)
		{
			float obs_line_point_buf_x = 0;
			float obs_line_point_buf_y = 0;
			tra.robot2world(obs_line_point[i],robot_pos,obs_way_theta,obs_line_point_buf_x,obs_line_point_buf_y);

			Eigen::Vector3f p ;
			p <<obs_line_point_buf_x , obs_line_point_buf_y, 1.0;
			obs_line_point_world.push_back(p);
		}
		draw(ALLOBSPointMarker , 0.0, 0.0, 1.0, all_point_world);
		draw(obs_line_pointMarker , 1.0, 1.0, 0.0, obs_line_point_world);
		//畫出軌跡再rviz





		std::cout<<"obs_line_point   "<< obs_line_point.size() <<std::endl;
		std::cout<<"see 5 "<<std::endl;
		//排序
		if(obs_line_point.size() > 1 )
			for(int i=0;i<obs_line_point.size()-1 ;i++)
			{
				for(int j=i+1;j<obs_line_point.size() ;j++)
				{
					if(obs_line_point[i].y() < obs_line_point[j].y())
					{
						Eigen::Vector3f buf;
						buf = obs_line_point[i];
						obs_line_point[i] = obs_line_point[j];
						obs_line_point[j] = buf;
						//std::cout<<"obs_line_point=============="<<std::endl;
					}
				}
			}


		std::cout<<"see 6 "<<std::endl;
		//找點
		float obs_dis=0.0;
		if(obs_line_point.size() > 1)
			for(int i=0;i<obs_line_point.size()-1 ;i++)
			{
				obs_dis = obs_line_point[i].y() - obs_line_point[i+1].y();
				if(obs_dis>=gap_limit)
				{
					gap_array.push_back(obs_dis);
					point_array.push_back(obs_line_point[i]);
					point_array.push_back(obs_line_point[i+1]);
				}

			}


	}


	std::cout<<"see 7 "<<std::endl;
	ROS_INFO("--------------");
	ROS_INFO("FindKeyPoint");
	for(int i=0; i<gap_array.size(); i++)
		ROS_INFO("gap_array: %f", gap_array[i]);
	ROS_INFO("gap_limit: %f", gap_limit);



	//找最大gap
	if(gap_array.size() > 0){


		Eigen::Vector3f obs_pos;

		int max_index = 0;
		float max_dis = gap_array[0];
		for(int i=1; i<gap_array.size(); i++){
			if(gap_array[i] > max_dis){
				max_index = i;
				max_dis = gap_array[i];
			}
		}

		ROS_INFO("max_dis: %f", max_dis);

		//gap比我認為的大 視為可過
		if(max_dis >= gap_limit){
			//gap的i跟point的i換算
			int index = max_index*2 + 1;
			Eigen::Vector3f p0 = point_array[index-1];
			Eigen::Vector3f p1 = point_array[index];
			Eigen::Vector3f p2 = Eigen::Vector3f( (p0.x()+p1.x())/2,  (p0.y()+p1.y())/2, 0.0);




			//判斷左邊右邊（這裡存的座標都是相對於車中心（因為雷射來的））
			float region_theta = atan2(p2.y(), p2.x());
			if(region_theta >= 0) avoid_way = 1;
			else avoid_way = -1;

			// std::cout<<"avoid_way avoid_way  "<< avoid_way<<std::endl;

			// std::cout<<"region_theta  "<< region_theta<<std::endl;

			Eigen::Vector3f local_key_p = Eigen::Vector3f(0.0, 0.0, 0.0);
			float dx,dy;

			//計算要往哪邊位移 （左右）dx都為0

			if(avoid_way > 0){
				dx = (gap_limit/2.0)*cos(M_PI/2);
				dy = (gap_limit/2.0)*sin(M_PI/2);
				ROS_INFO("--------left--------");
				if(flag_left)
				{
					max_dis=0;
				}
				//找我尋找障礙物的末端點是哪0
				float p0_robot = p0.y();
				float p1_robot = p1.y();
				if(p0_robot<0 && p1_robot<0)
				{
					if(p0_robot > p1_robot)
					{
						obs_pos = p1;
					}
					else{
						obs_pos = p0;
					}
				}
				else{
					if(p0_robot < 0)
						obs_pos = p0;
					else
						obs_pos = p1;
				}
			}
			else{
				dx = (gap_limit/2.0)*cos(-1*M_PI/2);
				dy = (gap_limit/2.0)*sin(-1*M_PI/2);
				ROS_INFO("--------right--------");
				if(flag_right)
				{
					max_dis=0;
				}
				//找我尋找障礙物的末端點是哪0
				float p0_robot = p0.y();
				float p1_robot = p1.y();
				if(p0_robot>0 && p1_robot>0)
				{
					if(p0_robot > p1_robot)
					{
						obs_pos = p0;
					}
					else{
						obs_pos = p1;
					}
				}
				else{
					if(p0_robot > 0)
						obs_pos = p0;
					else
						obs_pos = p1;
				}
			}


			// ROS_INFO("----------------------");
			// ROS_INFO("p0: %f, %f", p0.x(), p0.y());
			// ROS_INFO("p1: %f, %f", p1.x(), p1.y());
			// ROS_INFO("p2: %f, %f", p2.x(), p2.y());
			// ROS_INFO("obs_pos: %f, %f", obs_pos.x(), obs_pos.y());
			// ROS_INFO("dx, dy: %f, %f", dx, dy);

			//末端點加上一半車身距離
			local_key_p[0] = obs_pos.x() + dx;
			local_key_p[1] = obs_pos.y() + dy;

			// ROS_INFO("local_key_p: %f, %f", local_key_p.x(), local_key_p.y());


			//變成世界座標
			float obs_pos_world_x=0.0, obs_pos_world_y=0.0;
			tra.robot2world( local_key_p,  robot_pos,  obs_way_theta, dx, dy);
			tra.robot2world( obs_pos,  robot_pos,  obs_way_theta, obs_pos_world_x, obs_pos_world_y);

			key_p = Eigen::Vector3f(dx, dy, robot_pos.z()  + obs_way_theta);

			// std::cout<<"obs_pos_world_x  "<<obs_pos_world_x<<std::endl;
			// std::cout<<"obs_pos_world_y  "<<obs_pos_world_y<<std::endl;
			// std::cout<<"key_p  "<<key_p<<std::endl;
			// std::cout<<"robot_pos  "<<robot_pos<<std::endl;

			//剛剛gap用最小來算，當我們確定位移方向後，判斷車身姿態改變gap_limit大小

			float x_error_world = key_p.x() - obs_pos_world_x;
			float y_error_world = key_p.y() - obs_pos_world_y;
			//            float x_error_robot = x_error_world*cos(-1*robot_pos.z() - obs_way_theta) - y_error_world*sin(-1*robot_pos.z() - obs_way_theta);
			//            float y_error_robot = x_error_world*sin(-1*robot_pos.z() - obs_way_theta) + y_error_world*cos(-1*robot_pos.z() - obs_way_theta);

			float way_theta = atan2(y_error_world, x_error_world);
			//float warn_theta = fabs(way_theta - robot_pos.z());

			float warn_theta = (way_theta - robot_pos.z() + 2*M_PI) ;

			if(warn_theta > (2*M_PI)) warn_theta = warn_theta - 2*M_PI;

			if(warn_theta > M_PI) warn_theta = 2*M_PI - warn_theta;

			if(warn_theta>M_PI/2)
				warn_theta=M_PI-warn_theta;

			float Sca = OBS_SideMiss_Scale *0.9 ;
			if(Sca <1.2)
				Sca = 1.2;

			gap_limit = p_mobile_width * sin(warn_theta) + p_mobile_length * cos(warn_theta) + 2*OBS_Side *Sca;


			//std::cout<<"warn_theta  "<<warn_theta<<std::endl;

			if(max_dis >= gap_limit)
			{
				//重新計算新的key_p點 因為要避障的長度改變

				//計算要往哪邊位移 （左右）dx都為0
				if(avoid_way > 0){
					dx = (gap_limit/2.0)*cos(M_PI/2);
					dy = (gap_limit/2.0)*sin(M_PI/2);
					ROS_INFO("--------left--------");
				}
				else{
					dx = (gap_limit/2.0)*cos(-1*M_PI/2);
					dy = (gap_limit/2.0)*sin(-1*M_PI/2);
					ROS_INFO("--------right--------");
				}

				// std::cout<<"==================see==================="<<std::endl;
				// std::cout<<"gap_limit  "<<gap_limit<<std::endl;
				// std::cout<<"dx  "<<dx<<std::endl;
				// std::cout<<"dy  "<<dx<<std::endl;
				// std::cout<<"==================see==================="<<std::endl;

				//末端點加上一半車身距離
				local_key_p[0] = obs_pos.x() + dx;
				local_key_p[1] = obs_pos.y() + dy;

				//std::cout<<"local_key_p  "<<local_key_p<<std::endl;
				//變成世界座標
				tra.robot2world( local_key_p,  robot_pos,  obs_way_theta, dx, dy);

				key_p = Eigen::Vector3f( dx, dy, 0.0);

				//std::cout<<"key_p  "<<key_p<<std::endl;

				isWorkable = true;
			}
			//=============change================

		}
	}


	obs_line_point.clear();
	gap_array.clear();
	point_array.clear();


	return isWorkable;


}
void diff_dwa::CreateNewPath(Eigen::Vector3f& target, Eigen::Vector3f& key_point, int subpath_index, std::vector<Eigen::Vector3f> &new_path)
{

	float x_error = key_point.x() - target.x();
	float y_error = key_point.y() - target.y();


	// std::fstream fout;
	//     fout.open("avoid_LINE",std::fstream::out);
	if(obs_avoid_flag)
	{
		std::vector<Eigen::Vector3f> path;
		for(int i=0;i<avoid_path.size();i++){
		    Eigen::Vector3f path_buf;
		    path_buf.x() = avoid_path[i].x() + x_error;
		    path_buf.y() = avoid_path[i].y() + y_error;
		    path_buf.z() = 1.0;

		    path.push_back(path_buf);
		}
		avoid_path.clear();
		new_path = path;


		//測試中
		//畫出軌跡再rviz
		draw(AvoidLineMarker , 1.0, 0.1, 0.1, avoid_path);
		//畫出軌跡再rviz


	}
	else{

		for(int i=0;i<A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint.size();i++){
			Eigen::Vector3f path_buf;
			path_buf.x() = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[i].x() + x_error;
			path_buf.y() = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[i].y() + y_error;
			path_buf.z() = 1.0;


			new_path.push_back(path_buf);
		}
		draw(AvoidLineMarker , 1.0, 0.1, 0.1, new_path);
	}




}
bool diff_dwa::AvoidObs()
{

	//-----------------------------------
	float angular_velocity_kp = tracking_kp;
	float angular_velocity_kd = tracking_kd;
	float angular_velocity_error=0;
	float  angular_velocity_p_error = 0.0, angular_velocity_d_error = 0.0;
	float cmd_angular_velocity = 0.0;
	static float pre_angular_velocity_error = 0;


	static int target_ind = 0;
	static bool isInitial = true;
	float V_target = 0.2;//m/s
	float W_rw = 0;
	float cmd_velocity = 0.0;

	Eigen::Vector3f robot_pos, target_pos;

	if(isAvoidObs == true && isFindObs == false){

		robot_pos = slam_pose_;

		// std::fstream fout;
		// fout.open("obs_robot_pos",std::fstream::out);
		// fout<<robot_pos.x() <<"\t" <<robot_pos.y() <<std::endl;

		int now_index;


		target_ind = calc_target_index_obs(robot_pos, V_target, avoid_path, now_index);
		target_pos = avoid_path[target_ind];

		Eigen::Vector3f now_pos = avoid_path[now_index];


		float x_error_robot = 0.0 ;
		float y_error_robot = 0.0 ;
		tra.world2robot( target_pos,  robot_pos, x_error_robot, y_error_robot);


		// float way_theta = atan2(y_error_robot, x_error_robot);
		cmd_velocity = V_target;


		angular_velocity_error = atan2((target_pos.y() - robot_pos.y()) , (target_pos.x() - robot_pos.x())) - robot_pos.z();
				if(fabs(angular_velocity_error) > M_PI){
					if(angular_velocity_error > 0) angular_velocity_error = angular_velocity_error - 2*M_PI ;
					else                           angular_velocity_error = angular_velocity_error + 2*M_PI;
				}
				angular_velocity_p_error = angular_velocity_error;
				angular_velocity_d_error = angular_velocity_error - pre_angular_velocity_error;
				pre_angular_velocity_error = angular_velocity_error;
				cmd_angular_velocity = angular_velocity_kp*angular_velocity_p_error + angular_velocity_kd*angular_velocity_d_error;
		W_rw = cmd_angular_velocity;


		// static float time_count_avoid = 1;
		// float th_err = way_theta - Rev_odom_t1;


		//避障移動到avoid的路徑時會致盲
		if(isInitial){
			float m_error = 0.0;
			tra.closeline( now_pos, target_pos, robot_pos, m_error);


			if(fabs(m_error) < 0.03 )
			{

				isblind = false;
				isInitial = false;
				//time_count_avoid = 1;
			}
		}
		Car.two_wheel_Kinematics(cmd_velocity, W_rw, vl,  vr );

		std::vector<unsigned char> command;

		// sendreceive.Package_Diff_encoder(cmd_velocity,W_rw,
		// 	command);
    sendreceive.Package_publicWheel_encoder(cmd_velocity, 0, W_rw, 0, command);
		SendPackage(command);





	}
	else if(isAvoidObs == false && isFindObs == true){
		target_ind = 0;
		isInitial = true;
		obs_avoid_flag =true;


		p_state_ = P_STATE_AVOID_REPLANNING;
		AvoidObs_state = P_OBS_REPLANNING;

		pre_angular_velocity_error = 0.0;



	}
	else if(isAvoidObs == false && isFindObs == false){
		target_ind = 0;
		isInitial = true;
		obs_avoid_flag =false;

		std::vector<unsigned char> command;


		// sendreceive.Package_Diff_encoder(0,0,
		// 	command);
		sendreceive.Package_publicWheel_encoder(0, 0, 0, 0, command);

		SendPackage(command);

		p_state_ = P_STATE_MOVE;
		AvoidObs_state = P_OBS_NORMAL;
		obs_return = true;

		pre_angular_velocity_error = 0.0;


		//測試中
		//畫出軌跡再rviz
		std::vector<Eigen::Vector3f> NULL_buf;
		draw(AvoidLineMarker , 1.0, 0.0, 0.0, NULL_buf);
		//畫出軌跡再rviz
	}


	return false;


}

bool diff_dwa::WaitDeadlock()
{

	static int cnt = 0;
	float delay_time = 2.0;
	int cnt_limit = int(delay_time/time_sample);


	static int cnt_re = 0;
	float delay_time_re = 2.0;
	int cnt_limit_re = int(delay_time_re/time_sample);


	static int cnt_error = 0;

	//如果再死區狀態障礙物排除時
	if(!isFindObs){
		cnt += 1;

		if(cnt >= cnt_limit){
			cnt = 0;
			p_state_ = P_STATE_MOVE;
			AvoidObs_state = P_OBS_NORMAL;

			cnt_error = 0;

			return true;
		}
	}
	//障礙物無法完全排除 但能清出可行之閉障路徑
	else{
		cnt = 0;
		cnt_re +=1;
		if(cnt_re >= cnt_limit_re)
		{
			p_state_ = P_STATE_AVOID_WAIT;
			AvoidObs_state = P_OBS_WAIT;
			cnt_re = 0;
			cnt_error +=1;

			//請上層重新派任                   ////發佈error給上層需要人工排除障礙
			if(cnt_error > 10)
			{
				std_msgs::Int8 msg;

				msg.data = 0;

				ReMissionState_Publisher_.publish(msg);
			}


		}
	}
	global_v = 0;
	std::vector<unsigned char> command;

	// sendreceive.Package_Diff_encoder(0,0,
	// 		command);
  sendreceive.Package_publicWheel_encoder(0, 0, 0, 0, command);

	SendPackage(command);

	return false;
}
void diff_dwa::joystickCallback(const move_robot::joystick& joystick)
{


	if(PUSE_BUTTON_RB != joystick.btn_id && PUSE_BUTTON_START!= joystick.btn_id)
		btn_id = joystick.btn_id;

	float vx = joystick.x * JOYSTICK_SCALAR;
	float vy = joystick.y * JOYSTICK_SCALAR;

	// joystick_v = sqrt(pow(vy, 2) + pow(vx, 2));
    // joystick_theta = atan2(vy, vx);

	joystick_v = vx;
	joystick_theta = joystick.y * 3;

	isReveice_joystick = true;

}
void diff_dwa::joystick_move()
{
	if(btn_id == PUSE_BUTTON_A)
	{
		//joystick_v = joystick_v/60;// 0.33m/s
		joystick_v = joystick_v;
		//std::cout<<"joystick_v0 = "<<joystick_v<<std::endl;

		float us = joystick_theta;

		if(us>M_PI/2)
		{   us-=M_PI;
		}
		else if(us<-M_PI/2)
		{   us+=M_PI;
		}


		//L is the distance of both the wheel
		float L = LRWheeldis; //meter

		//W_rw
		//float W_rw = (sin(us)/L)*joystick_v;
		float W_rw = joystick_theta;   // Darren Style Movement 

		if(fabs(W_rw) > move_w_max){
			if(W_rw > 0)
				W_rw = move_w_max;
			else
				W_rw = -1.0*move_w_max;
		}

		//==============change=============
		/////////////////////////////////
		// if(fabs(joystick_v - fabs(global_v)) > 0.2)
		// {
		// 		if(joystick_v > fabs(global_v))
		// 		{
		// 				joystick_v = fabs(global_v) + fabs(joystick_v - fabs(global_v)) * 0.2;
		// 		}
		// 		else
		// 		{
		// 				joystick_v = fabs(global_v) - fabs(joystick_v - fabs(global_v)) * 0.2;
		// 		}
		// }
		// if(joystick_v > JOYSTICK_SCALAR)
		// {
		// 		joystick_v = JOYSTICK_SCALAR;
		// }
		/////////////////////////////////
		//std::cout<<"joystick_v1 = "<<joystick_v<<std::endl;
		float V_avg = joystick_v;

		// if(V_avg*cos(joystick_theta)<0)
		// {
		// 	V_avg = -1*V_avg;
		// }

		if(fabs(V_avg) < 0.05)
			V_avg = 0;
		if(fabs(W_rw) < 0.02)
			W_rw = 0;



		float Vx = V_avg;
		float Vy = 0;

		//Car.two_wheel_Kinematics(V_avg, W_rw, vl,  vr );

		global_v = V_avg;


		std::vector<unsigned char> command;

		// sendreceive.Package_Diff_encoder(V_avg,W_rw,
		// 		command
		// 		);
    sendreceive.Package_publicWheel_encoder(V_avg, 0, W_rw, 0, command);

		SendPackage(command);

		ROS_INFO("============diff================");



	}
	else if(btn_id == PUSE_BUTTON_X)
	{ROS_INFO("turn_left");

		Car.two_wheel_Kinematics(0, stop_w_max, vl,  vr );

		std::cout<<"vl  "<<vl<<std::endl;
		std::cout<<"vr  "<<vr<<std::endl;

		global_v = 0;

		std::vector<unsigned char> command;


			// sendreceive.Package_Diff_encoder(0,stop_w_max,
			// 		command
			// 		);
		sendreceive.Package_publicWheel_encoder(0, 0, stop_w_max, 0, command);


		SendPackage(command);

	}
	else if(btn_id == PUSE_BUTTON_Y)
	{ROS_INFO("turn_right");

		Car.two_wheel_Kinematics(0, -1.0*stop_w_max, vl,  vr );

		global_v = 0;

		std::vector<unsigned char> command;


			// sendreceive.Package_Diff_encoder(0,-1.0*stop_w_max,
			// 		command
			// 		);
		sendreceive.Package_publicWheel_encoder(0, 0, -1.0*stop_w_max, 0, command);

		SendPackage(command);

	}


}
void diff_dwa::Calculate_odom()
{
	// static float last_v = 0.0;
	// Rev_a = (Rev_V - last_v)*10;
	//
	// geometry_msgs::PoseStamped odom_odometry;
	// odom_odometry.header.frame_id = "base_link";
	// odom_odometry.pose.position.x = Rev_V;
	// odom_odometry.pose.position.y = 0;
	// odom_odometry.pose.orientation.w = Rev_W;
	// odom_odometry.pose.orientation.z = 0;
	// odometry_Publisher_.publish(odom_odometry);
	//
	// //std::cout<<"Rev_V = "<<Rev_V<<" "<<" Rev_W =  "<<Rev_W<<" Rev_a =  "<<Rev_a<<std::endl;
	//
	// last_v = Rev_V;

	// nav_msgs::Odometry odom_odometry;
	// odom_odometry.header.stamp = ros::Time::now();
	// odom_odometry.header.frame_id = "odom";
	// odom_odometry.pose.pose.position.x = Rev_odom_onewheel_v;
	// odom_odometry.pose.pose.position.y = 0;
	// odom_odometry.pose.pose.orientation.w = Rev_odom_onewheel_w;
	// odom_odometry.pose.pose.orientation.z = 0;
	// odometry_Publisher_.publish(odom_odometry);
	current_time = ros::Time::now();
	float th = Rev_th / 180.0 * M_PI;
	std::cout<<"Rev_th = "<<Rev_th<<std::endl;
	std::cout<<"Rev_V = "<<Rev_V<<" Rev_W =  "<<Rev_W<<" Rev_x =  "<<Rev_x<<" Rev_y = "<<Rev_y<<"Rev_th = "<<th<<std::endl;
	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = Rev_x;
	odom_trans.transform.translation.y = Rev_y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = Rev_x;
	odom.pose.pose.position.y = Rev_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = Rev_V;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = Rev_W;

	//publish the message
	odom_Publisher_.publish(odom);
	//std::cout<<"===============local_x "<<x<<"local_y "<<y<<" local_z "<<th<<std::endl;
	///////////////////////////////////////////////////for amcl
	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans_amcl;
	odom_trans_amcl.header.stamp = current_time;
	odom_trans_amcl.header.frame_id = "odom_amcl";
	odom_trans_amcl.child_frame_id = "base_link_amcl";

	odom_trans_amcl.transform.translation.x = Rev_x;
	odom_trans_amcl.transform.translation.y = Rev_y;
	odom_trans_amcl.transform.translation.z = 0.0;
	odom_trans_amcl.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans_amcl);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom_amcl;
	odom_amcl.header.stamp = current_time;
	odom_amcl.header.frame_id = "odom_amcl";

	//set the position
	odom_amcl.pose.pose.position.x = Rev_x;
	odom_amcl.pose.pose.position.y = Rev_y;
	odom_amcl.pose.pose.position.z = 0.0;
	odom_amcl.pose.pose.orientation = odom_quat;

	//set the velocity
	odom_amcl.child_frame_id = "base_link_amcl";
	odom_amcl.twist.twist.linear.x = Rev_V;
	odom_amcl.twist.twist.linear.y = 0.0;
	odom_amcl.twist.twist.angular.z = Rev_W;

	//publish the message
	amcl_odom_Publisher_.publish(odom_amcl);
	///////////////////////////////////////for amcl
	////////////////////////////////////////計算轉換
	// if(ismapping)
	// {
	// 		mapping_count++;
	// }
	// if(mapping_count == 1)
	// {
	// 	std::vector<unsigned char> command;
	// 	sendreceive.Package_publicWheel_encoder(0,0,0,2,command);
	// 	SendPackage(command);
	// 		// origin_localization = slam_pose_;
	// 		// origin_odom = Eigen::Vector3f(Rev_x, Rev_y, th);
	// 		// transfer_odom = true;
	// 		// std::cout<<"origin_odom = "<<origin_odom<<std::endl;
	// }
	// else if(mapping_count == 51)
	// {
	// 		origin_localization = slam_pose_;
	// 		origin_odom = Eigen::Vector3f(Rev_x,Rev_y,th);
	// 		transfer_odom = true;
	// 		std::cout<<"origin_odom = "<<origin_odom<<std::endl;
	//
	// }
	// else if(mapping_count>51){
	// 		mapping_count = 52;
	// }

	if(use_hallway_odom)
	{
			hallway_odom_count++;
	}
	if(hallway_odom_count == 1)
	{
			std::vector<unsigned char> command;
			sendreceive.Package_publicWheel_encoder(0,0,0,2,command);
			SendPackage(command);
	}
	else if(hallway_odom_count == 10)
	{
			origin_localization = slam_pose_;
			origin_odom = Eigen::Vector3f(Rev_x,Rev_y,th);
			transfer_odom = true;
			std::cout<<"origin_odom = "<<origin_odom<<std::endl;
	}
	else if(hallway_odom_count>10)
	{
			hallway_odom_count = 0;
			use_hallway_odom = false;
	}

	if(use_nonhallway_relocalization)
	{
			use_nonhallway_relocalization = false;
			std::cout<<"=============relocalization!!============"<<std::endl;
			geometry_msgs::Pose initial_pose;
			initial_pose.position.x = transfer_odom_pose.x();
			initial_pose.position.y = transfer_odom_pose.y();
			initial_pose.position.z = 0.0;
			geometry_msgs::Quaternion relocalization_quat = tf::createQuaternionMsgFromYaw(transfer_odom_pose.z());
			initial_pose.orientation = relocalization_quat;
			carto_location(initial_pose);
	}

	if(transfer_odom)
	{
			Eigen::Vector3f real_odom;
			float x_error_world = Rev_x - origin_odom.x();
			float y_error_world = Rev_y - origin_odom.y();
			real_odom.x() = x_error_world*cos(-1*origin_odom.z()) - y_error_world*sin(-1*origin_odom.z());
			real_odom.y() = x_error_world*sin(-1*origin_odom.z()) + y_error_world*cos(-1*origin_odom.z());
			// real_odom.x() = Rev_x - origin_odom.x();
			// real_odom.y() = Rev_y - origin_odom.y();
			real_odom.z() = th - origin_odom.z();
			// std::cout<<"real_x = "<<real_odom.x()<<" real_y = "<<real_odom.y()<<" real_z = "<<real_odom.z()<<std::endl;

			if(real_odom.z()>M_PI)
				real_odom.z()=real_odom.z()-2*M_PI;
			else if(real_odom.z()<-M_PI)
				real_odom.z()=real_odom.z()+2*M_PI;

			float real_x,real_y,real_z;
			tra.odom2world(real_odom, origin_localization, real_x, real_y);
			real_z = real_odom.z() + origin_localization.z();

			if(real_z>M_PI)
				real_z=real_z-2*M_PI;
			else if(real_z<-M_PI)
				real_z=real_z+2*M_PI;

			std::cout<<"real_odom.x = "<<real_odom.x()<<" real_odom.y = "<<real_odom.y()<<" real_odom.z = "<<real_odom.z()<<std::endl;
			std::cout<<"real_x = "<<real_x<<" real_y = "<<real_y<<" real_z = "<<real_z<<std::endl;
			std::cout<<"slam_pose_.x = "<<slam_pose_.x()<<" slam_pose_.y = "<<slam_pose_.y()<<" slam_pose_.z = "<<slam_pose_.z()<<std::endl;
			transfer_odom_pose = Eigen::Vector3f(real_x, real_y, real_z);

			// outFile << real_odom.x() << ',' << real_odom.y() << ',' << real_odom.z()<< ',' << " " << ','
			// << real_x << ',' << real_y << ',' << real_z<< ',' << " "<< ','
			//  << slam_pose_.x() << ',' << slam_pose_.y() << ',' << slam_pose_.z()<<std::endl;
	}
  ////////////////////////////////////////計算轉換

}

void diff_dwa::RevPacketDecorder(std::vector<unsigned char> &buff, unsigned int bufferSize, std::vector<float> &vPacket)
{
	for (int i = 0; i < bufferSize; ++i)
	{
		int HighByte_integer = buff[i * 5 + 3];
		int LowByte_integer = buff[i * 5 + 4];
		int HighByte_float = buff[i * 5 + 5];
		int LowByte_float = buff[i * 5 + 6];

		float packet_value = HighByte_integer * 256 + LowByte_integer + (HighByte_float * 256 + LowByte_float) / 1000.0;
		if (buff[i * 5 + 2] > 0)
			packet_value *= -1;
		// cout << "packet_value : " << i << " " << packet_value << endl;

		vPacket.push_back(packet_value);
	}
	// cout << "========== Next =======" << endl;
}

void diff_dwa::RevProcess(double receive_period)
{
	int Receive_Package_Size = 256;

	ros::Rate r_receive(1.0 / receive_period);
	std::vector<unsigned char> rev_buf;
	while (1)
	{
		// std::cout << "=========LOSS encoder=========" << std::endl;
		if (mySerial.serial_ok == true)
		{
			unsigned char buff[Receive_Package_Size];
			int readByte = 0;
			readByte = read(mySerial.fd, buff, 29);
			if (readByte > 0)
			{
				if (buff[0] != 'E' && buff[readByte - 1] != 'E')
				{
					rev_buf.clear();
				}
				else
				{
					if (buff[1] == 'C' && buff[readByte - 2] == 'C')
					{
						for (int i = 0; i < readByte; i++)
						{
							// std::cout << buff[i] << std::endl;
							rev_buf.push_back(buff[i]);
						}
						// sendreceive.Setmode(1);
						// sendreceive.RevProcess_two_wheel_encoder(rev_buf, Rev_V, Rev_W);

						std::vector<float> vPacket;
						int packet_count = (readByte - 4) / 5;
						vPacket.reserve(packet_count);
						RevPacketDecorder(rev_buf, packet_count, vPacket);

						Rev_V = vPacket[0];
						Rev_W = vPacket[1];
						Rev_x = vPacket[2];
						Rev_y = vPacket[3];
						Rev_th = vPacket[4];

						// for (int i = 0; i < vPacket.size(); ++i)
						// 	std::cout << "vPacket: " << vPacket[i] << std::endl;
					}
					else
					{
						rev_buf.clear();
					}
				}
			}
			rev_buf.clear();
		}
	}
	r_receive.sleep();
}

void diff_dwa::Stop()
{

	//正常來說需要速度回授做一個減速停止的動作

	global_v = 0;

	std::vector<unsigned char> command;

	// sendreceive.Package_Diff_encoder(0,0,
	// 				command
	// 				);
	sendreceive.Package_publicWheel_encoder(0, 0, 0, 0, command);
	SendPackage(command);

}
