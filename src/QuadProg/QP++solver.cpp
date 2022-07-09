#include "QuadProg/smooth.h"
#include <QuadProg++/QuadProg++.hh>


//平滑动态规划得出的SL曲线
void SmoLine::QuadProg_solver()
{
  clock_t startTime = clock();
  //生成Aeq_sub
  Eigen::MatrixXd Aeq_sub(2, 6);
  Aeq_sub << 1,  ds,     ds*ds/3,  -1,   0,    ds*ds/6,
             0,  1,      ds/2,     0,    -1,   ds/2;

  //生成A_sub
  Eigen::MatrixXd A_sub(8, 3);
  A_sub <<-1, -this->vehicleLength/2,  0,
          -1, -this->vehicleLength/2,  0,
          -1,  this->vehicleLength/2,  0,
          -1,  this->vehicleLength/2,  0,
           1,  this->vehicleLength/2,  0,
           1,  this->vehicleLength/2,  0,
           1, -this->vehicleLength/2,  0,
           1, -this->vehicleLength/2,  0;

  //生成Aeq
  /*这里要把Aeq转置一下*/
  quadprogpp::Matrix<double> Aeq(0.0, 3*arraysize, 2*(arraysize-1)+retaintraj_num-vehTotraj_projIndex_);
  for(int i = 0; i < arraysize-1; i++)
    for(int j = 0; j < 6; j++)
    {
      Aeq[j + 3*i][2*i + 0] = Aeq_sub(0, j);
      Aeq[j + 3*i][2*i + 1] = Aeq_sub(1, j);
    }

  if(retraj == false)
  {
    for(int16_t i = 0; i < retaintraj_num-vehTotraj_projIndex_; i++)
       Aeq[3*i][2*(arraysize-1) + i] = 1;
  }
  else
  {
    for(int16_t i = 0; i < retaintraj_num-vehTotraj_projIndex_; i++)
      Aeq[i][2*(arraysize-1) + i] = 1;
  }


  //生成A
  /*这里要把A转置一下*/
  quadprogpp::Matrix<double> A(0.0, 3*arraysize, 8*arraysize);
  for(int i = 0; i < arraysize; i++)
    for(int j = 0; j < 8; j++)
    {
      A[0 + 3*i][8*i + j] = A_sub(j, 0);
      A[1 + 3*i][8*i + j] = A_sub(j, 1);
    }

  //生成b_eq
  quadprogpp::Vector<double> b_eq(0.0, 2*(arraysize-1)+retaintraj_num-vehTotraj_projIndex_);
  if(retraj == false)
  {
    for(int16_t i = 0; i < retaintraj_num-vehTotraj_projIndex_; i++)
      b_eq[2*(arraysize-1) + i] = -L_limit[i][3];
  }
  else
  {
      b_eq[2*(arraysize-1) + 0] =  -plan_start_mags.l;
      b_eq[2*(arraysize-1) + 1] =  -plan_start_mags.dl;
      b_eq[2*(arraysize-1) + 2] =  -plan_start_mags.ddl;
  }

  //生成b
  quadprogpp::Vector<double> b(8*arraysize);
  int maxIndex;
  int minIndex;
  for(int i = 0; i < arraysize; i++)
  {
    maxIndex = i+1; if(maxIndex >= arraysize) maxIndex = arraysize-1;
    minIndex = i-1; if(minIndex <= 0) minIndex = 0;
    b[8*i + 0] =-this->vehicleWidth/2 + this->L_limit[maxIndex][1];
    b[8*i + 1] =+this->vehicleWidth/2 + this->L_limit[maxIndex][1];
    b[8*i + 2] =-this->vehicleWidth/2 + this->L_limit[minIndex][1];
    b[8*i + 3] =+this->vehicleWidth/2 + this->L_limit[minIndex][1];
    b[8*i + 4] =+this->vehicleWidth/2 - this->L_limit[maxIndex][2];
    b[8*i + 5] =-this->vehicleWidth/2 - this->L_limit[maxIndex][2];
    b[8*i + 6] =+this->vehicleWidth/2 - this->L_limit[minIndex][2];
    b[8*i + 7] =-this->vehicleWidth/2 - this->L_limit[minIndex][2];
  }

  //生成H
  quadprogpp::Matrix<double> H(0.0, 3*arraysize, 3*arraysize);
  for(int i = 0; i < arraysize; i++)
  {
    H[3*i + 0][3*i + 0] = 2*this->w_cost_centre;
    H[3*i + 1][3*i + 1] = 2*this->w_cost_dl;
    H[3*i + 2][3*i + 2] = 2*this->w_cost_ddl;
  }

  //生成f
  quadprogpp::Vector<double> f(0.0, 3*arraysize);
  for(int i = 0; i < arraysize; i++)
  {
    f[3*i] = -2*this->w_cost_centre*this->dynamic_frenet[i + plan_startIndex].l;
  }

  quadprogpp::Vector<double> x(3*arraysize);

  clock_t endTime = clock();

  ROS_WARN("time is: %f",double(endTime - startTime)/CLOCKS_PER_SEC);

  double resort_data = quadprogpp::solve_quadprog(H, f, Aeq, b_eq, A, b, x);

  ROS_WARN("QP min cost is:%f",resort_data);

  bool showflag=false;

  if(resort_data == std::numeric_limits<double>::infinity() || resort_data == 1.0E300) showflag = true;
  else{
    for(int i = 0; i < arraysize; i++)
      if(x[3*i] > 1.5*borderLimit || x[3*i] < -1.5*borderLimit) //会出现玄学bug，规划的轨迹混乱，疑似二次规划代码的原因。判断出现玄学，就把这个结果抛弃。
      {
        showflag = true;
        break;
      }
  }


  if(showflag == false)//计算结果正常，输出路径信息
  {
    for(int i = 0; i < arrayCapacity; i++)
    {
      save_line[i][0] = L_limit[i][0];
      save_line[i][1] = L_limit[i][1];
      save_line[i][2] = L_limit[i][2];
      save_line[i][3] = L_limit[i][3] = x[3*i];
    }

    Frenet_trans_Cartesian();

    // if(pow(New_car_pose.position.x - trajline_point.poses[0].pose.position.x, 2) + 
    //   pow(New_car_pose.position.y - trajline_point.poses[0].pose.position.y, 2) > 1)
    // {
    //   if(retraj == false) std::cout<<"retraj is false"<<std::endl;
    //   else std::cout<<"retraj is ture"<<std::endl;
    //   throw std::runtime_error("line is error");
    // }
  }
  else//计算结果异常，抛弃求解结果，把上一次求解的结果当成此次结果
  {
    for(int i = 0; i < arrayCapacity ;i++)
    {
      L_limit[i][0] = save_line[i][0];
      L_limit[i][1] = save_line[i][1];
      L_limit[i][2] = save_line[i][2];
      L_limit[i][3] = save_line[i][3];
    }

    std::cout <<"resort error"<<std::endl;
  }


  // for(int16_t i =0; i < arraysize; i++)
  //   if(L_limit[i][0] < dynamic_frenet[0].s - 2)
  //   {
  //     ROS_WARN("error line");
  //     for(int16_t j = 0; j < arraysize ;j++)
  //       std::cout << L_limit[j][0] << std::endl;

  //     if(retraj == true) ROS_WARN("retraj");
  //     else ROS_WARN("no retraj");

  //     ROS_WARN("plan_startIndex is:%d",plan_startIndex);

  //     throw std::runtime_error("line is error");
  //   }

  // if(showflag == true)
  // { 
  //   ROS_WARN("arraysize is:%d",arraysize);

  //   ROS_WARN("error line:");
  //   for(int i = 0; i < arraysize; i++)
  //     std::cout << x[3*i] << std::endl;

  //   ROS_WARN("b is:");
  //   for(int i = 0; i<8*arraysize; i++)
  //     std::cout << b[i] <<std::endl;

  //   ROS_WARN("bu deng si jie guo:");
  //   for(int i = 0; i < arraysize; i++)
  //     for(int j = 0; j < 8; j++)
  //     {
  //       std::cout << A[0 + 3*i][8*i + j]*x[3*i+0] + A[1 + 3*i][8*i + j]*x[3*i+1] <<std::endl;
  //     }

  //   ROS_WARN("vehTotraj_projIndex is:%d",vehTotraj_projIndex_);
  //   throw std::runtime_error("line is error");
  // }
}