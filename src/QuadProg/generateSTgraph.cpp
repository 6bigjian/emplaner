#include "QuadProg/smooth.h"
#include "obstacle/obstacle.h"
#include "global/global_planning.h"

void SmoLine::prediMovObsTraj()
{
  /*设定对向动态障碍物的代价区域*/
  for(uint16_t i = 0; i < ObsProj::Move_Obstacle.size(); i++)
  {
    ObsProj::Move_Obstacle[i].OBSSTRG = NULLIFY;
    if(ObsProj::Move_Obstacle[i].obs_flag == false) continue;

    int16_t matchIndex = Global_Plan::Search_Match_Point(ObsProj::Move_Obstacle[i].x_set, ObsProj::Move_Obstacle[i].y_set,
                                    reference_path.referenceline, 0, reference_path.referenceline.poses.size()-1);

    if(cos(reference_path.theta[matchIndex] - ObsProj::Move_Obstacle[i].MotionDirect) > 0){
        if(ObsProj::Move_Obstacle[i].V_set >= lon_speed){
          ObsProj::Move_Obstacle[i].obs_flag = false;
          continue;
        }

        //计算车辆中心与障碍物中心的相遇时间
        double overlap_t = (ObsProj::Move_Obstacle[i].s_set - L_limit[0][0])/
                           (lon_speed - ObsProj::Move_Obstacle[i].V_set);

        if(overlap_t >= arraysize*ds/lon_speed){//障碍物超出界限
          ObsProj::Move_Obstacle[i].obs_flag = false;
          continue;
        }else{
          //如果相遇时间在规定时间内，将车速换成超车速度
          overlap_t = (ObsProj::Move_Obstacle[i].s_set - L_limit[0][0])/
                      (overtake_speed - ObsProj::Move_Obstacle[i].V_set);
        }
        
        double encount_t = ((ObsProj::Move_Obstacle[i].s_set - ObsProj::Move_Obstacle[i].length/2) - 
                          (L_limit[0][0] + this->vehicleLength/2)) /
                          (overtake_speed - ObsProj::Move_Obstacle[i].V_set);
        //车辆后角点与障碍物前角点的相遇时间
        double deviate_t = ((ObsProj::Move_Obstacle[i].s_set + ObsProj::Move_Obstacle[i].length/2) - 
                          (L_limit[0][0] - this->vehicleLength/2)) /
                          (overtake_speed - ObsProj::Move_Obstacle[i].V_set);

        double obs_s_min = ObsProj::Move_Obstacle[i].s_set + ObsProj::Move_Obstacle[i].V_set * encount_t - ObsProj::Move_Obstacle[i].length/2;
        double obs_s_mid = ObsProj::Move_Obstacle[i].s_set + ObsProj::Move_Obstacle[i].V_set * overlap_t;
        double obs_s_max = ObsProj::Move_Obstacle[i].s_set + ObsProj::Move_Obstacle[i].V_set * deviate_t + ObsProj::Move_Obstacle[i].length/2;

        int16_t start_index = FindObjIndex(obs_s_min, MINPOINT);
        int16_t end_index = FindObjIndex(obs_s_max, MAXPOINT);
        int16_t centre_index = FindObjIndex(obs_s_mid, MIDPOINT);
        if(centre_index == LINEBACK) centre_index = 0;
        else if(centre_index == LINEFRONT) centre_index = arraysize-1;

        if(start_index == LINEFRONT)//障碍物超出界限
        {
          ObsProj::Move_Obstacle[i].obs_flag = false;
          continue;
        }
        else if(end_index == LINEBACK)//障碍物在车后面
        {
          ObsProj::Move_Obstacle[i].obs_flag = false;
          if(L_limit[0][0] - ObsProj::Move_Obstacle[i].s_set < global_date::identifyDist){//后方障碍物在识别范围内
            ObsProj::Move_Obstacle[i].OBSSTRG = FORWARD;
            ObsProj::Move_Obstacle[i].obs_flag = true;
            ObsProj::Move_Obstacle[i].backIndex = 0;
            ObsProj::Move_Obstacle[i].frontIndex = 0;
          }
          continue;
        }

        ObsProj::Move_Obstacle[i].backIndex = start_index;
        ObsProj::Move_Obstacle[i].frontIndex = end_index;

        if(start_index == LINEBACK){//动态障碍物的前段在车后面，
          start_index = 0;
          ObsProj::Move_Obstacle[i].backIndex = -1;
        }

        if(end_index == LINEFRONT){//后段超出规划
          end_index = arraysize - 1;
          ObsProj::Move_Obstacle[i].frontIndex = arraysize;
        }

        int16_t carLengthIndex = FindObjIndex(L_limit[0][0] + 5 * this->vehicleLength, MIDPOINT);//计算车长会占据多少个间隔点

        bool ub_flag = true;
        bool lb_flag = true;

        for(int16_t j = std::max(start_index - carLengthIndex, 0); j <= std::min(end_index + carLengthIndex, arraysize - 1); j++)
        {
          if(L_limit[j][1] - ObsProj::Move_Obstacle[i].l_set - 
            ObsProj::Move_Obstacle[i].width/2 < this->vehicleWidth)
                ub_flag = false;

          if(ObsProj::Move_Obstacle[i].l_set - ObsProj::Move_Obstacle[i].width/2 -
            L_limit[j][2] < this->vehicleWidth) 
                lb_flag = false;
        }

        ObsProj::Move_Obstacle[i].OBSSTRG = OVERTAKE;
        if(ub_flag == true && lb_flag == true)//上下的宽度都能通过
        {
          if(dynamic_frenet[FindNearIndex(ObsProj::Move_Obstacle[i].s_set)].l > 
            ObsProj::Move_Obstacle[i].l_set) //从上方绕
            for(int16_t j = start_index; j <= end_index; j++)
                L_limit[j][2] = ObsProj::Move_Obstacle[i].l_set + ObsProj::Move_Obstacle[i].width/2;

          else //从下方绕
            for(int16_t j = start_index; j <= end_index; j++)
                L_limit[j][1] = ObsProj::Move_Obstacle[i].l_set - ObsProj::Move_Obstacle[i].width/2;
        }
        else if(ub_flag == true) //从上方绕
        {
          for(int16_t j = start_index; j <= end_index; j++)
                L_limit[j][2] = ObsProj::Move_Obstacle[i].l_set + ObsProj::Move_Obstacle[i].width/2;
        }
        else if(lb_flag == true)//从下方绕
        {
            for(int16_t j = start_index; j <= end_index; j++)
                L_limit[j][1] = ObsProj::Move_Obstacle[i].l_set - ObsProj::Move_Obstacle[i].width/2;
        }
        else   ObsProj::Move_Obstacle[i].OBSSTRG = FOLLOW;
    }else{//对象来障碍物处理
        if(ObsProj::Move_Obstacle[i].s_set - L_limit[0][0] > arraysize * ds)
          continue;//太远了

        if(ObsProj::Move_Obstacle[i].s_set + ObsProj::Move_Obstacle[i].length/2 - L_limit[0][0] <= 0)
           continue;//在车后面不用搭理

        ObsProj::Move_Obstacle[i].OBSSTRG = AVOIDANCE;

        //车头与障碍物头部相遇时间
        double encount_t = ((ObsProj::Move_Obstacle[i].s_set - ObsProj::Move_Obstacle[i].length/2) - 
                            (L_limit[0][0] + this->vehicleLength/2)) /
                            (lon_speed + ObsProj::Move_Obstacle[i].V_set);
        //车尾与障碍物尾部相遇时间
        double deviate_t = ((ObsProj::Move_Obstacle[i].s_set + ObsProj::Move_Obstacle[i].length/2) - 
                            (L_limit[0][0] - this->vehicleLength/2)) /
                            (lon_speed + ObsProj::Move_Obstacle[i].V_set);

        double obs_s_min = ObsProj::Move_Obstacle[i].s_set - ObsProj::Move_Obstacle[i].V_set * encount_t - ObsProj::Move_Obstacle[i].length/2;
        double obs_s_max = ObsProj::Move_Obstacle[i].s_set - ObsProj::Move_Obstacle[i].V_set * deviate_t + ObsProj::Move_Obstacle[i].length/2;

        int16_t start_index = FindObjIndex(obs_s_min, MINPOINT);
        int16_t end_index = FindObjIndex(obs_s_max, MAXPOINT);

        if(start_index == LINEBACK){
          start_index = 0;
        }

        if(end_index == LINEFRONT){
          end_index = arraysize-1;
        }

        ObsProj::Move_Obstacle[i].frontIndex = start_index;
        ObsProj::Move_Obstacle[i].backIndex = end_index;

        bool ub_flag = true;
        bool lb_flag = true;

        for(int16_t j = start_index; j <= end_index; j++){
          if(L_limit[j][1] - ObsProj::Move_Obstacle[i].l_set - 
            ObsProj::Move_Obstacle[i].width/2 < this->vehicleWidth)
                ub_flag = false;

          if(ObsProj::Move_Obstacle[i].l_set - ObsProj::Move_Obstacle[i].width/2 -
            L_limit[j][2] < this->vehicleWidth) 
                lb_flag = false;
        }

        if(lb_flag == true){//优先从右边走
            for(int16_t j = start_index; j <= end_index; j++)
                L_limit[j][1] = ObsProj::Move_Obstacle[i].l_set - ObsProj::Move_Obstacle[i].width/2 - this->vehicleWidth/2;
        }else if(ub_flag == true){
            for(int16_t j = start_index; j <= end_index; j++)
                L_limit[j][2] = ObsProj::Move_Obstacle[i].l_set + ObsProj::Move_Obstacle[i].width/2 + this->vehicleWidth/2;
        }
    }
  }
}



void SmoLine::genST()
{
  //生成最终平滑轨迹的s
  trajline_s[0] = 0.0;
  for(int16_t i = 1; i < arraysize; i++)
    trajline_s[i] = trajline_s[i - 1] + 
                    sqrt(pow(trajline_point.poses[i].pose.position.x - trajline_point.poses[i-1].pose.position.x, 2) +
                         pow(trajline_point.poses[i].pose.position.y - trajline_point.poses[i-1].pose.position.y, 2));

  STub[0] = STlb[0] = 0.0;
  for(int16_t T = 1; T < divisionTime_num; T++)
  {
    STub[T] = 2*trajline_s[arraysize-1];
    STlb[T] = 0.0;
  }
  
  int16_t backIndex;
  int16_t frontIndex;
  for(int16_t i = 0; i < ObsProj::Move_Obstacle.size(); i++)
  {
    if(ObsProj::Move_Obstacle[i].obs_flag == false) continue;
    if(ObsProj::Move_Obstacle[i].OBSSTRG == NULLIFY) continue;

    double predMovObs_s;//障碍物中心
    double predMovback_s;//后角点
    double predMovfront_s;//前角点

    if(ObsProj::Move_Obstacle[i].OBSSTRG == AVOIDANCE){
      backIndex = arraysize - 1;
      frontIndex = arraysize - 1;
    }else{
      backIndex = 0;
      frontIndex = 0;
    }

    
    for(int16_t T = 1; T < divisionTime_num; T++)
    {
      if(ObsProj::Move_Obstacle[i].OBSSTRG == AVOIDANCE){//对象来车
        predMovObs_s = ObsProj::Move_Obstacle[i].s_set - ObsProj::Move_Obstacle[i].V_set * dt * T;
        predMovback_s = predMovObs_s + ObsProj::Move_Obstacle[i].length/2;
        predMovfront_s = predMovObs_s - ObsProj::Move_Obstacle[i].length/2;

        if(predMovback_s < L_limit[0][0]) break;

        while(predMovback_s < L_limit[backIndex][0] && backIndex > 0)
          backIndex--;
        
        while(predMovfront_s < L_limit[frontIndex][0] && frontIndex > 0)
          frontIndex--;

      }else{
        predMovObs_s = ObsProj::Move_Obstacle[i].s_set + ObsProj::Move_Obstacle[i].V_set * dt * T;
        predMovback_s = predMovObs_s - ObsProj::Move_Obstacle[i].length/2;
        predMovfront_s = predMovObs_s + ObsProj::Move_Obstacle[i].length/2;

        if(predMovfront_s > L_limit[arraysize-1][0]) break;

        while(predMovback_s > L_limit[backIndex][0] && backIndex < arraysize)
          backIndex++;
        
        while(predMovfront_s > L_limit[frontIndex][0] && frontIndex < arraysize)
          frontIndex++;

        if(backIndex >= arraysize) backIndex = arraysize - 1;
        if(frontIndex >= arraysize) frontIndex = arraysize - 1;
      }

      if(ObsProj::Move_Obstacle[i].OBSSTRG == OVERTAKE){ 
        //设置上边界，在到达超车区域前不要超车
        if((backIndex < ObsProj::Move_Obstacle[i].backIndex) && (predMovback_s >= L_limit[0][0]))
          if(predMovback_s - L_limit[0][0] + this->vehicleLength/2 < STub[T])
            STub[T] = predMovback_s - L_limit[0][0] - this->vehicleLength/2;
        //设置下边界，超车完成后，要保证不被后车追上
        if((frontIndex > ObsProj::Move_Obstacle[i].frontIndex) && (predMovfront_s >= L_limit[0][0]))
          if(predMovfront_s - L_limit[0][0] - this->vehicleLength/2> STlb[T])
            STlb[T] = predMovfront_s - L_limit[0][0] + this->vehicleLength/2;
      }else if(ObsProj::Move_Obstacle[i].OBSSTRG == FORWARD){
        if((frontIndex > ObsProj::Move_Obstacle[i].frontIndex) && (predMovfront_s >= L_limit[0][0]))
          if(predMovfront_s - L_limit[0][0] - this->vehicleLength/2> STlb[T])
            STlb[T] = predMovfront_s - L_limit[0][0] + this->vehicleLength/2;
      }else if(ObsProj::Move_Obstacle[i].OBSSTRG == FOLLOW){
        if(predMovback_s >= L_limit[0][0])
          if((predMovback_s - L_limit[0][0]) - 2 * global_date::fllowDist - this->vehicleLength/2 < STub[T])
            STub[T] = (predMovback_s - L_limit[0][0]) - 2 * global_date::fllowDist - this->vehicleLength/2;
      }else if(ObsProj::Move_Obstacle[i].OBSSTRG == AVOIDANCE){
        if((frontIndex > ObsProj::Move_Obstacle[i].frontIndex) && (predMovfront_s >= L_limit[0][0]))
          if(predMovfront_s - L_limit[0][0] < STub[T])
            STub[T] = predMovfront_s - L_limit[0][0];

        if((backIndex < ObsProj::Move_Obstacle[i].backIndex) && (predMovback_s >= L_limit[0][0]))
          if(predMovback_s - L_limit[0][0] > STlb[T])
            STlb[T] = predMovback_s - L_limit[0][0];
      }

      if(STlb[T-1] > STlb[T]) STlb[T] = STlb[T-1];
    }
  }

  //对于有多段上边界的，反方向完善上边界
  for(int16_t T = divisionTime_num-1; T > 0; T--)
    if(STub[T] < STub[T-1]) STub[T-1] = STub[T];

  // std::cout<<"\rn"<<std::endl;
  // for(int16_t i = 0; i < divisionTime_num; i++){
  //   ROS_WARN("(%f, %f, %f)",dt*i, STlb[i], STub[i]);
  // }
}