#include "QuadProg/smooth.h"
#include "obstacle/obstacle.h"


void SmoLine::prediMovObsTraj()
{
  /*设定对向动态障碍物的代价区域*/
  for(uint16_t i = 0; i < ObsProj::Move_Obstacle.size(); i++)
  {
    ObsProj::Move_Obstacle[i].OBSSTRG = NULLIFY;
    if(ObsProj::Move_Obstacle[i].obs_flag == false) continue;
    else if(ObsProj::Move_Obstacle[i].V_set >= this->lon_speed)
    {
      ObsProj::Move_Obstacle[i].obs_flag = false;
      continue;
    }

    double encount_t = (ObsProj::Move_Obstacle[i].s_set - L_limit[0][0])/
                       (this->lon_speed - ObsProj::Move_Obstacle[i].V_set);//相遇时间差

    if(encount_t > 30)//障碍物超出界限
    {
      ObsProj::Move_Obstacle[i].obs_flag = false;
      continue;
    }

    double encount_s = ObsProj::Move_Obstacle[i].s_set + encount_t*ObsProj::Move_Obstacle[i].V_set;

    double obs_s_min = encount_s - ObsProj::Move_Obstacle[i].length/2 - this->vehicleLength;
    double obs_s_max = encount_s + ObsProj::Move_Obstacle[i].length/2 + this->vehicleLength;

    int16_t start_index = FindObjIndex(obs_s_min, MINPOINT);
    int16_t end_index = FindObjIndex(obs_s_max, MAXPOINT);
    int16_t centre_index = FindObjIndex(encount_s, MIDPOINT);
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
      continue;
    }

    if(start_index == LINEBACK) continue; //表示动态障碍物之间的距离不足以超越
    else if(end_index == LINEFRONT)  //表示障碍物一部分超出二次规划范围，一部分在范围内
      end_index = arraysize;

    if(centre_index == LINEFRONT)
    {
      ObsProj::Move_Obstacle[i].obs_flag = false;
      continue;//障碍物超出界限
    }

    ObsProj::Move_Obstacle[i].backIndex = start_index;
    ObsProj::Move_Obstacle[i].frontIndex = end_index;

    bool ub_flag = true;
    bool lb_flag = true;
    for(int16_t j = start_index; j <= end_index; j++)
    {
      if(L_limit[j][1] - ObsProj::Move_Obstacle[i].l_set - 
         ObsProj::Move_Obstacle[i].width/2 < this->vehicleWidth) ub_flag = false;

      if(ObsProj::Move_Obstacle[i].l_set - ObsProj::Move_Obstacle[i].width/2 -
         L_limit[j][2] < this->vehicleWidth) lb_flag = false;
    }

    ObsProj::Move_Obstacle[i].OBSSTRG = OVERTAKE;
    if(ub_flag == true && lb_flag == true)//上下的宽度都能通过
    {
      if(dynamic_frenet[FindNearIndex(ObsProj::Move_Obstacle[i].s_set)].l > 
         ObsProj::Move_Obstacle[i].l_set) //从上方绕
        for(int16_t j = start_index; j < end_index; j++)
            L_limit[j][2] = ObsProj::Move_Obstacle[i].l_set + ObsProj::Move_Obstacle[i].width/2;

      else //从下方绕
        for(int16_t j = start_index; j < end_index; j++)
            L_limit[j][1] = ObsProj::Move_Obstacle[i].l_set - ObsProj::Move_Obstacle[i].width/2;
    }
    else if(ub_flag == true) //从上方绕
    {
      for(int16_t j = start_index; j < end_index; j++)
            L_limit[j][2] = ObsProj::Move_Obstacle[i].l_set + ObsProj::Move_Obstacle[i].width/2;
    }
    else if(lb_flag == true)//从下方绕
    {
        for(int16_t j = start_index; j < end_index; j++)
            L_limit[j][1] = ObsProj::Move_Obstacle[i].l_set - ObsProj::Move_Obstacle[i].width/2;
    }
    else   ObsProj::Move_Obstacle[i].OBSSTRG = FOLLOW;

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
  for(int16_t T = 1; T < divisionTime_num ;T++)
  {
    STub[T] = trajline_s[arraysize-1];
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
    backIndex = 0;
    frontIndex = 0;
    
    for(int16_t T = 1; T < divisionTime_num; T++)
    {
      predMovObs_s = ObsProj::Move_Obstacle[i].s_set + ObsProj::Move_Obstacle[i].V_set * dt*T;
      predMovback_s = predMovObs_s - ObsProj::Move_Obstacle[i].length/2;
      predMovfront_s = predMovObs_s + ObsProj::Move_Obstacle[i].length/2;

      if(predMovfront_s > L_limit[arraysize-1][0]) break;

      while(predMovback_s > L_limit[backIndex][0])
        backIndex++;
      
      while(predMovfront_s > L_limit[frontIndex][0])
        frontIndex++;

      if(ObsProj::Move_Obstacle[i].OBSSTRG == OVERTAKE)
      { //设置上边界，在到达超车区域前不要超车
        if((backIndex <= ObsProj::Move_Obstacle[i].backIndex) && (predMovback_s >= L_limit[0][0]))
          if(abs(ObsProj::Move_Obstacle[i].l_set - L_limit[backIndex][3]) < 
            ObsProj::Move_Obstacle[i].width/2 + this->vehicleWidth/2)
            if(trajline_s[backIndex] < STub[T])
              STub[T] = trajline_s[backIndex];
        //设置下边界，超车完成后，要保证不被后车追上
        if((frontIndex >= ObsProj::Move_Obstacle[i].frontIndex) && (predMovfront_s >= L_limit[0][0]))
        {
          if(abs(ObsProj::Move_Obstacle[i].l_set - L_limit[frontIndex][3]) <
            ObsProj::Move_Obstacle[i].width/2 + this->vehicleWidth/2)
          {
            if(trajline_s[frontIndex] > STlb[T])
              STlb[T] = trajline_s[frontIndex];
          }
          else STlb[T] = STlb[T-1];
        }
        else STlb[T] = STlb[T-1];
      }
      // std::cout<<"("<<STlb[T]<<","<<STub[T]<<","<<L_limit[backIndex][3]<<")"<<std::endl;
    }
  }

  //对于有多段上边界的，反方向完善上边界
  for(int16_t T = divisionTime_num-1; T > 0; T--)
    if(STub[T] < STub[T-1]) STub[T-1] = STub[T];

  //ST坐标轴反函数转换
  int16_t T1 = 1, T2 = 1;
  TSub[0] = TSlb[0] = 0.0;
  for(int16_t j = 1; j < arraysize; j++)
  {
    TSub[j] = divisionTime_num*dt;//上限时间等于细分时间个数*时间间隔
    TSlb[j] = 0.0;

    for(; T1 < divisionTime_num; T1++)
      if(trajline_s[j] <= STlb[T1])
      {
        TSub[j] = (double)(T1-1)*0.5;
        break;
      }

    for(; T2 < divisionTime_num; T2++)
      if(trajline_s[j] <= STub[T2])
      {
        TSlb[j] = (double)(T2-1)*0.5;
        break;
      }
  }
}