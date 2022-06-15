#include <vector>
#include <math.h>
#include <stdio.h>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>


#define pi 3.1415926535897932384626433832795
//下标宏定义 状态[x(m), y(m), yaw(Rad), v(m / s), w(rad / s)]
#define POSE_X          0    // 坐标 X
#define POSE_Y          1    // 坐标 Y
#define YAW_ANGLE       2    // 机器人航向角
#define V_SPD           3    // 机器人速度
#define W_ANGLE_SPD     4    // 机器人角速度
//定义Kinematic的下标含义
#define MD_MAX_V        0    // 最高速度m / s]
#define MD_MAX_W        1    // 最高旋转速度[rad / s]
#define MD_ACC          2    // 加速度[m / ss]
#define MD_VW           3    // 旋转加速度[rad / ss]
#define MD_V_RESOLUTION 4    // 速度分辨率[m / s]
#define MD_W_RESOLUTION 5    // 转速分辨率[rad / s]]

struct state{
	float x;
  float y;
  float yaw;
  float velocity;
  float angular;
};

struct controlU{
  float vt;
  float wt;
};

struct maxmotion{
  float minvel;
  float maxvel;
  float minang;
  float maxang;
};

struct eval_db{
  float vt;
  float wt;
  float heading;
  float dist;
  float vel;
	float goal_dist;
	float oscillation;
	float avoid_obs;
  float feval;
	state pose;
};

float dt = 0.1;//时间[s]`在这里插入代码片`

class DWA
{
		public:
			int map_width, map_height;
			bool arrival_goal;
			float origin_x, origin_y, m_resolution;
			controlU DynamicWindowApproach(state cx, std::vector<state> *TrajDb, float *model, Eigen::Vector3f goal, float * evalParam, std::vector<Eigen::Vector3f> ob, std::vector<Eigen::Vector3f> robot_ob, float R, float goal_distance, int** oscillation_cost, std::vector<Eigen::Vector3f> &dwa_predict_pose, Eigen::Vector3f &dwa_choose_pose);
			state CarState(state cx, controlU u);
			state GenerateTrajectory(state cx, std::vector<state> *traj, float vt, float wt, float predict_dis, float *model, float goal_distance);
			state robot_GenerateTrajectory(state cx, float vt, float wt, float predict_dis, float *model, float goal_distance);
			maxmotion CalcDynamicWindow(state cx, float *model);
			float CalcHeadingEval(state cx, Eigen::Vector3f goal);
			float CalcDistEval(state cx, std::vector<Eigen::Vector3f> ob, float R, int &ob_idx);
			float Calcgoal_distance(state cx, Eigen::Vector3f goal);
			int Calculate_Uobs(state cx, std::vector<Eigen::Vector3f> ob);
			int history_number(int pixel_x, int pixel_y, int range_R, int** oscillation_cost);
			float CalcBreakingDist(float vel, float mdacc);
			float	DegreeToRadian(float degree);
			float	RadianToDegree(float radian);
			void Evaluation(state cx, std::vector<eval_db> *EvalDb, std::vector<state> *TrajDb, maxmotion Vr, Eigen::Vector3f goal, std::vector<Eigen::Vector3f> ob, std::vector<Eigen::Vector3f> robot_ob, float R, float *model, float predict_dis, float goal_distance, int** oscillation_cost);
			void NormalizeEval(std::vector<eval_db> *EvalDb);
			void robot2world(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float &x, float &y);
			void world2robot(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float &x, float &y);
			void get_mapinfo(int width, int height, float x, float y, float resolution);
};

controlU DWA::DynamicWindowApproach(state cx, std::vector<state> *TrajDb, float *model, Eigen::Vector3f goal, float * evalParam, std::vector<Eigen::Vector3f> ob, std::vector<Eigen::Vector3f> robot_ob,float R, float goal_distance, int** oscillation_cost, std::vector<Eigen::Vector3f> &dwa_predict_pose, Eigen::Vector3f &dwa_choose_pose)
{
		controlU u;
		std::vector<eval_db> EvalDb;
		// Dynamic Window [vmin,vmax,wmin,wmax] 最小速度 最大速度 最小角速度 最大角速度速度
		maxmotion cvr = CalcDynamicWindow(cx, model); //根据当前状态 和 运动模型 计算当前的参数允许范围
		if(cvr.minvel > cvr.maxvel)
		{
				float ve = cvr.minvel;
				cvr.minvel = cvr.maxvel;
				cvr.maxvel = ve;
		}
		//std::cout<<"cvr_v = "<<cvr.minvel<<" cvr_v = "<<cvr.maxvel<<" cvr_w = "<<cvr.minang<<" cvr_w = "<<cvr.maxang<<std::endl;
		//评价函数的计算 evalDB N*5  每行一组可用参数 分别为 速度、角速度、航向得分、距离得分、速度得分
		//trajDB      每5行一条轨迹 每条轨迹都有状态x点串组成
		//Evaluation(state cx, vector<state> *traj, vector<eval_db> *EvalDb, vector<state> *TrajDb, maxmotion Vr, Point goal, vector<Point> ob, float R, float *model, float predict_dis)
		Evaluation(cx, &EvalDb, TrajDb, cvr, goal, ob, robot_ob, R, model, evalParam[6], goal_distance, oscillation_cost);  //predict_dis = evalParam(4) 评价函数参数[heading, dist, velocity, predictDistance]
		if (EvalDb.empty())
		{
				std::cout << "no path to goal!!" << std::endl;
				u.vt = 0;
				u.wt = 0;
				return u;
		}
		NormalizeEval(&EvalDb);//各评价函数正则化
		//最终评价函数的计算float heading;float dist;float vel;
		for (int i = 0; i < EvalDb.size(); ++i)
		{
				EvalDb.at(i).feval = evalParam[0] * EvalDb.at(i).heading + evalParam[1] * EvalDb.at(i).dist + evalParam[2] * EvalDb.at(i).vel + evalParam[3] * EvalDb.at(i).goal_dist + evalParam[4] * EvalDb.at(i).oscillation
				+ evalParam[5] * EvalDb.at(i).avoid_obs;//根据评价函数参数 前三个参数分配的权重 计算每一组可用的路径参数信息的得分

				Eigen::Vector3f pose_buf(EvalDb[i].pose.x, EvalDb[i].pose.y, 0.0);
				dwa_predict_pose.push_back(pose_buf);
		}

		float maxheading = EvalDb.at(0).feval;
		float idx = 0;
		for (int i = 0; i < EvalDb.size(); ++i)
		{
				if (maxheading < EvalDb.at(i).feval)
				{
						maxheading = EvalDb.at(i).feval;
						idx = i;
				}
		}
	  u.vt = EvalDb.at(idx).vt;
		u.wt = EvalDb.at(idx).wt;
		/////////////////////////存取選擇的點
		dwa_choose_pose.x() = EvalDb[idx].pose.x;
		dwa_choose_pose.y() = EvalDb[idx].pose.y;
		dwa_choose_pose.z() = 0.0;
		/////////////////////////存取選擇的點
		EvalDb.clear();
		return u;
}

void DWA::Evaluation(state cx, std::vector<eval_db> *EvalDb, std::vector<state> *TrajDb, maxmotion Vr, Eigen::Vector3f goal, std::vector<Eigen::Vector3f> ob, std::vector<Eigen::Vector3f> robot_ob, float R, float *model, float predict_dis, float goal_distance, int** oscillation_cost)
{
    EvalDb->clear();
    TrajDb->clear();
    std::vector<state> traj;
    for (float vt = Vr.minvel; vt <= Vr.maxvel; vt = vt + model[4])//根据速度分辨率遍历所有可用速度： 最小速度和最大速度 之间 速度分辨率 递增
    {
      	for (float wt = -0.5; wt <= 0.5; wt = wt + model[5])//根据角度分辨率遍历所有可用角速度： 最小角速度和最大角速度 之间 角度分辨率 递增
        {
        		//轨迹推测; 得到 xt : 机器人向前运动后的预测位姿; traj: 当前时刻 到 预测时刻之间的轨迹（由轨迹点组成）
        		state xt = GenerateTrajectory(cx, &traj, vt, wt, predict_dis, model, goal_distance); //predict_dis = evalParam(4), 前向模拟距
						state robot_xt = robot_GenerateTrajectory(cx, vt, wt, predict_dis, model, goal_distance);
						//各评价函数的计算
        		float heading = CalcHeadingEval(xt, goal);//前项预测终点的航向得分  偏差越小分数越高 w1
						int ob_idx = 0;
        		float dist = CalcDistEval(xt, ob, R, ob_idx);//前项预测终点 距离最近障碍物的间隙得分 距离越远分数越高 w2
						//double wt_ = Vr.maxang - 0.5 * (vt/Vr.maxvel) * fabs(wt);
        		//float vel = fabs(vt) + fabs(wt_);//速度得分 速度越快分越高 w3
						float vel = fabs(vt);//速度得分 速度越快分越高 w3
						float goal_dist = Calcgoal_distance(xt, goal);//離終點越進分數越高 w4
						int pixel_x = fabs(xt.x - origin_x) / m_resolution;
						int pixel_y = map_height - fabs(xt.y - origin_y) / m_resolution;
						float oscillation = 0.0;//歷史訊息 w5
						if(pixel_x >= 0 && pixel_x <= map_width && pixel_y >= 0 && pixel_y <= map_height)
						{
								float tra_num = history_number(pixel_x, pixel_y, 20, oscillation_cost);
								oscillation = (1 - oscillation_cost[pixel_x][pixel_y]) + (1 - (tra_num / (int)(20 * 20 * M_PI)));
						}
						else oscillation = 0.0;
						float avoid_obs = 7 - Calculate_Uobs(xt, ob);//避開障礙物 w6
        		//float stopDist = CalcBreakingDist(vel, model[MD_ACC]); //制动距离的计算
						//std::cout<<" vt = "<<vt<<" wt = "<<wt<<" cx.x = "<<cx.x<<" cx.y = "<<cx.y<<" cx.yaw = "<<cx.yaw<<" xt.x = "<<xt.x<<" xt.y = "<<xt.y<<" goal.x = "<<goal.x()<<" goal.y = "<<goal.y()<<" heading = "<<heading<<std::endl;
						float feval = 0.0;
						eval_db db = { vt,wt,heading,dist,vel,goal_dist,oscillation,avoid_obs,feval,xt};
						if(!arrival_goal)
						{
								if (dist > 1.2 * R)
								{
										EvalDb->push_back(db);
								}
						}
        		else EvalDb->push_back(db);
        }
    }
		traj.clear();
}

void DWA::NormalizeEval(std::vector<eval_db> *EvalDb)
{
    //评价函数正则化
    float sum3 = 0, sum4 = 0, sum5 = 0, sum6 = 0, sum7 = 0, sum8 = 0;
    for (int i = 0; i < EvalDb->size(); ++i)
    {
        sum3 += EvalDb->at(i).heading;
        sum4 += EvalDb->at(i).dist;
        sum5 += EvalDb->at(i).vel;
				sum6 += EvalDb->at(i).goal_dist;
        sum7 += EvalDb->at(i).oscillation;
        sum8 += EvalDb->at(i).avoid_obs;
    }
    if (sum3 != 0)
    {
        for (int i = 0; i < EvalDb->size(); ++i)
        	EvalDb->at(i).heading = EvalDb->at(i).heading / sum3;
    }
    if (sum4 != 0)
    {
        for (int i = 0; i < EvalDb->size(); ++i)
        	EvalDb->at(i).dist = EvalDb->at(i).dist / sum4;
    }
    if (sum5 != 0)
    {
        for (int i = 0; i < EvalDb->size(); ++i)
        	EvalDb->at(i).vel = EvalDb->at(i).vel / sum5;
    }
		if (sum6 != 0)
    {
        for (int i = 0; i < EvalDb->size(); ++i)
        	EvalDb->at(i).goal_dist = EvalDb->at(i).goal_dist / sum6;
    }
		if (sum7 != 0)
    {
        for (int i = 0; i < EvalDb->size(); ++i)
        	EvalDb->at(i).oscillation = EvalDb->at(i).oscillation / sum7;
    }
		if (sum8 != 0)
    {
        for (int i = 0; i < EvalDb->size(); ++i)
        	EvalDb->at(i).avoid_obs = EvalDb->at(i).avoid_obs / sum8;
    }
}

state DWA::robot_GenerateTrajectory(state cx, float vt, float wt, float predict_dis, float *model, float goal_distance)
{
	float time = 0.0;
	controlU u = { vt, wt };
	state px = { 0 , 0, cx.yaw, cx.velocity, cx.angular};
	if(goal_distance < predict_dis)predict_dis = goal_distance;

	float predict_time = predict_dis / vt;

	if(predict_time > 3)predict_time = 3;
	else if(predict_time < 1)predict_time = 1;

	while ((int)time < (int)predict_time*10)
	{
			time = time + 10*dt;
			px = CarState(px, u);
	}
	return px;
}

state DWA::GenerateTrajectory(state cx, std::vector<state> *traj, float vt, float wt, float predict_dis, float *model, float goal_distance)
{
    float time = 0.0;
    controlU u = { vt, wt };
    traj->clear();
    traj->push_back(cx);
    state px = cx;
		if(goal_distance < predict_dis)predict_dis = goal_distance;

		float predict_time = predict_dis / vt;

		if(predict_time > 3)predict_time = 3;
		else if(predict_time < 1)predict_time = 1;

    while ((int)time < (int)predict_time*10)
    {
        time = time + 10*dt;
        px = CarState(px, u);
        traj->push_back(px);
    }
    return px;
}

float DWA::DegreeToRadian(float degree)
{
    return degree / 180 * pi;
}

float DWA::RadianToDegree(float radian)
{
    return radian / pi * 180;
}

state DWA::CarState(state cx, controlU u)
{
    state result;
		// Eigen::Vector3f robot_pose(cx.x, cx.y, cx.yaw);
		// Eigen::Vector3f add_pose(dt * cos(cx.yaw)*u.vt, dt * sin(cx.yaw)*u.vt, 0.0);
		// robot2world(add_pose, robot_pose, result.x, result.y);
		result.x = cx.x + dt * cos(cx.yaw)*u.vt;
		result.y = cx.y + dt * sin(cx.yaw)*u.vt;
    result.yaw = cx.yaw + dt * u.wt;
    result.velocity = u.vt;
    result.angular = u.wt;
    return result;
}

maxmotion DWA::CalcDynamicWindow(state cx, float *model)
{
    maxmotion V_r;
    //车子速度的最大最小范围 依次为：最小速度 最大速度 最小角速度 最大角速度速度
    //float Vs[4] = { 0, model[MD_MAX_V], -model[MD_MAX_W], model[MD_MAX_W] };
    //根据当前速度以及加速度限制计算的动态窗口  依次为：最小速度 最大速度 最小角速度 最大角速度速度
    //float Vd[4] = { cx.velocity - model[MD_ACC]*dt, cx.velocity + model[MD_ACC]*dt, cx.angular - model[MD_VW]*dt, cx.angular + model[MD_VW]*dt };
    //最终的Dynamic Window
    //float Vtmp[2 * 4];// 2 X 4  每一列依次为：最小速度 最大速度 最小角速度 最大角速度速度
    //memcpy(&Vtmp,&Vs,4*sizeof(float));
    //memcpy(&Vtmp[4], &Vd, 4 * sizeof(float));
    //V_r.minvel = max(Vs[0], Vd[0]);
    //V_r.maxvel = min(Vs[1], Vd[1]);
    //V_r.minang = max(Vs[3], Vd[3]);
    //V_r.maxang = min(Vs[4], Vd[4]);

    //V_r.minvel = std::max(0.0f, float(cx.velocity - 0.05));
		V_r.minvel = std::max(0.0f, cx.velocity - model[MD_ACC] * dt);
    V_r.maxvel = std::min(model[MD_MAX_V], cx.velocity + model[MD_ACC] * dt);
    V_r.minang = std::max(-model[MD_MAX_W], cx.angular - model[MD_VW] * dt);
    V_r.maxang = std::min(model[MD_MAX_W], cx.angular + model[MD_VW] * dt);
		// V_r.minang = -0.4;
		// V_r.maxang = 0.4;
    return V_r;
}

float DWA::CalcHeadingEval(state cx, Eigen::Vector3f goal)
{
    float theta = cx.yaw; //机器人朝向
		float goalTheta = atan2(goal.y() - cx.y, goal.x() - cx.x);//目标点相对于机器人本身的方位
    float targetTheta;
		targetTheta = fabs(goalTheta - theta);
    return 3.14-targetTheta;
}

float DWA::CalcDistEval(state cx, std::vector<Eigen::Vector3f> ob, float R, int &ob_idx)
{
    float dist = 100.0;
		int idx = 0;
    for (int i = 0; i < ob.size(); ++i)
    {
        //到第i个障碍物的距离 - 障碍物半径
        float disttmp = sqrt((ob[i].x() - cx.x)*(ob[i].x() - cx.x) + (ob[i].y() - cx.y)*(ob[i].y() - cx.y));
        if (dist > disttmp)//大于最小值 则选择最小值
				{
						dist = disttmp;
						idx = i;
				}
    }
		ob_idx = idx;
    // if (dist >= 2.5 * R)
    //    dist = 2.5 * R;
    return dist;
}

float DWA::Calcgoal_distance(state cx, Eigen::Vector3f goal)
{
		float distance = sqrt((goal.x() - cx.x) * (goal.x() - cx.x) + (goal.y() - cx.y) * (goal.y() - cx.y));
		if(distance < 2.0) return 5 - distance;
		else return 0;
}

int DWA::Calculate_Uobs(state cx, std::vector<Eigen::Vector3f> ob)
{
		float theta = cx.yaw;//機器人朝向
		bool range_90 = false;
		bool range_60 = false;
		bool range_30 = false;
		bool range_0 = false;
		bool range_n30 = false;
		bool range_n60 = false;
		bool range_n90 = false;
		int region_count = 0;
		for(int i = 0;i < ob.size();++i)
		{
				double obsTheta = atan2(ob[i].y() - cx.y, ob[i].x() - cx.x);
				double Obs_theta = obsTheta - theta;
				if(Obs_theta > M_PI) Obs_theta = Obs_theta - 2 * M_PI;
				else if(Obs_theta < -M_PI) Obs_theta = Obs_theta + 2 * M_PI;

				if(Obs_theta > DegreeToRadian(85) && Obs_theta < DegreeToRadian(95)) range_90 = true;
				else if(Obs_theta > DegreeToRadian(55) && Obs_theta < DegreeToRadian(65)) range_60 = true;
				else if(Obs_theta > DegreeToRadian(25) && Obs_theta < DegreeToRadian(35)) range_30 = true;
				else if(Obs_theta > DegreeToRadian(-5) && Obs_theta < DegreeToRadian(5)) range_0 = true;
				else if(Obs_theta > DegreeToRadian(-35) && Obs_theta < DegreeToRadian(-25)) range_n30 = true;
				else if(Obs_theta > DegreeToRadian(-65) && Obs_theta < DegreeToRadian(-55)) range_n60 = true;
				else if(Obs_theta > DegreeToRadian(-90) && Obs_theta < DegreeToRadian(-80)) range_n90 = true;
		}
		if(range_90 == true) region_count++;
		if(range_60 == true) region_count++;
		if(range_30 == true) region_count++;
		if(range_0 == true) region_count++;
		if(range_n30 == true) region_count++;
		if(range_n60 == true) region_count++;
		if(range_n90 == true) region_count++;
		return region_count;
}

int DWA::history_number(int pixel_x, int pixel_y, int range_R, int** oscillation_cost)
{
		int count = 0;
		for(int i=-range_R;i<range_R;++i)
		{
				for(int j=-range_R;j<range_R;++j)
				{
						if(pixel_x + i >= 0 && pixel_x + i <= map_width && pixel_y + j >= 0 && pixel_y + j <= map_height)
						{
								float distance = sqrt(i*i + j*j);
								if(distance < range_R)
								{
										if(oscillation_cost[pixel_x + i][pixel_y + j] != 0)count++;
								}
						}
				}
		}
		return count;
}

void DWA::get_mapinfo(int width, int height, float x, float y, float resolution)
{
		map_width = width;
		map_height = height;
		origin_x = x;
		origin_y = y;
		m_resolution = resolution;
}

float DWA::CalcBreakingDist(float vel, float mdacc)
{
    float stopDist = 0;
    while (vel > 0)//给定加速度的条件下 速度减到0所走的距离
    {
        stopDist = stopDist + vel * dt; //制动距离的计算
        vel = vel - mdacc*dt;
    }
    return stopDist;
}

void DWA::robot2world(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float &x, float &y)
{
		float x_world = target_pos.x()*cos(robot_pos.z() ) - target_pos.y()*sin(robot_pos.z() );
	  float y_world = target_pos.x()*sin(robot_pos.z() ) + target_pos.y()*cos(robot_pos.z() );
		x = x_world + robot_pos.x();
		y = y_world + robot_pos.y();
}

void DWA::world2robot(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float &x, float &y)
{
		float x_error_world = target_pos.x() - robot_pos.x();
	  float y_error_world = target_pos.y() - robot_pos.y();
	  float x_error_robot = x_error_world*cos(-1*robot_pos.z()) - y_error_world*sin(-1*robot_pos.z());
	  float y_error_robot = x_error_world*sin(-1*robot_pos.z()) + y_error_world*cos(-1*robot_pos.z());
		x = x_error_robot;
		y = y_error_robot;
}
