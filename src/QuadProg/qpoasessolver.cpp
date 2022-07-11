#include "QuadProg/smooth.h"
#include <qpOASES/qpOASES.hpp>

#if qpsolver == qpoasessolver

/*SL坐标系平滑矩阵*/
qpOASES::real_t* H_SL_f;
qpOASES::real_t* f_SL_f;
qpOASES::real_t* A_SL_f;
qpOASES::real_t* lbA_SL_f;
qpOASES::real_t* ubA_SL_f;
qpOASES::real_t* lb_SL_f;
qpOASES::real_t* ub_SL_f;
qpOASES::QProblem sl_fulloasesslover(3*arrayCapacity, 2*(arrayCapacity - 1));
/**************/
/*笛卡尔坐标系平滑矩阵*/
qpOASES::real_t* H_XY_f;
qpOASES::real_t* f_XY_f;
qpOASES::real_t* A_XY_f;
qpOASES::real_t* lbA_XY_f;
qpOASES::real_t* ubA_XY_f;
qpOASES::real_t* lb_XY_f;
qpOASES::real_t* ub_XY_f;
qpOASES::QProblem xy_fulloasesslover(2*arrayCapacity, 0);
/******************/


void SmoLine::qpOASES_init()
{
  clock_t startTime = clock();

  qpOASES_SLinit();
  qpOASES_XYinit();

  clock_t endTime = clock();
  ROS_WARN("qpOASES init time is: %f ms",double(endTime - startTime)*1000/CLOCKS_PER_SEC);
}

void SmoLine::qpOASES_burn()
{
  qpOASES_SLburn();
  qpOASES_XYburn();
}

/*所求的点为全满，利用qpOASES求解器，求解二次规划*/
void SmoLine::qpOASES_solver()
{
  clock_t startTime = clock();

  qpOASES_SLsolver();
  qpOASES_XYsolver();

  clock_t endTime = clock();
  ROS_WARN("hotstart time is: %f ms",double(endTime - startTime)*1000/CLOCKS_PER_SEC);
}


/*所求的点不是全满，利用qpOASES求解器，求解二次规划*/
void SmoLine::qpOASES_solver(const uint16_t pointsize)
{
  clock_t startTime = clock();
  qpOASES_SLsolver(pointsize);
  clock_t endTime = clock();

  ROS_WARN("time is: %f ms",double(endTime - startTime)*1000/CLOCKS_PER_SEC);
}



void SmoLine::qpOASES_SLinit()
{
  H_SL_f = new qpOASES::real_t[3*arrayCapacity * 3*arrayCapacity];
  f_SL_f = new qpOASES::real_t[3*arrayCapacity * 1];
  A_SL_f = new qpOASES::real_t[2*(arrayCapacity - 1) * (3*arrayCapacity)];
  lbA_SL_f = new qpOASES::real_t[2*(arrayCapacity - 1)];
  ubA_SL_f = new qpOASES::real_t[2*(arrayCapacity - 1)];
  lb_SL_f = new qpOASES::real_t[3*arrayCapacity];
  ub_SL_f = new qpOASES::real_t[3*arrayCapacity];

  /*生成二次规划的代价矩阵 0.5x'Hx + x'f*/
  for(uint16_t i = 0; i < 9*arrayCapacity*arrayCapacity; i++) H_SL_f[i] = 0.0;

  for(int16_t i = 0; i < arrayCapacity; i++)
  {
    H_SL_f[(3*i+0)*3*arrayCapacity + (3*i+0)] = 2*this->w_cost_centre;
    H_SL_f[(3*i+1)*3*arrayCapacity + (3*i+1)] = 2*this->w_cost_dl;
    H_SL_f[(3*i+2)*3*arrayCapacity + (3*i+2)] = 2*this->w_cost_ddl;
  }
  for(int16_t i = 0; i < arrayCapacity; i++)
  {
    f_SL_f[3*i + 0] = 0.0;
    f_SL_f[3*i + 1] = 0.0;
    f_SL_f[3*i + 2] = 0.0;
  }
  /********************************************************/

  /* 生成不等式矩阵 lbA <= Ax <= ubA
     A中包含（2n-2）个等式约束
     等式约束上下限一致
  */
  for(u_int16_t i = 0; i < (2*arrayCapacity - 2)*(3*arrayCapacity); i++)
    A_SL_f[i] = 0.0;

  Eigen::MatrixXd Aeq_sub(2, 6);//生成等式约束的矩阵,生成Aeq_sub
  Aeq_sub << 1,  ds,     ds*ds/3,  -1,   0,    ds*ds/6,
             0,  1,      ds/2,      0,  -1,    ds/2;

  for(uint16_t i = 0; i < arrayCapacity - 1; i++) //循环导入等式矩阵
    for(uint16_t j = 0; j < 6; j++)
    {
      A_SL_f[(2*i + 0)*(3*arrayCapacity) + (3*i + j)] = Aeq_sub(0, j);
      A_SL_f[(2*i + 1)*(3*arrayCapacity) + (3*i + j)] = Aeq_sub(1, j);
    }
  for(uint16_t i = 0; i < (2*arrayCapacity - 2); i++) //生成等式约束
    ubA_SL_f[i] = lbA_SL_f[i] = 0.0;
  /**********************************************************/

  /*生成x的上下边界lb,ub*/
  for(uint16_t i = 0; i < arrayCapacity; i++)
  {
    lb_SL_f[3*i + 0] = -this->borderLimit + this->vehicleWidth/2 + this->safety_distance;
    lb_SL_f[3*i + 1] = -this->dlLimit;
    lb_SL_f[3*i + 2] = -this->ddlLimit;

    ub_SL_f[3*i + 0] =  this->borderLimit - this->vehicleWidth/2 - this->safety_distance;
    ub_SL_f[3*i + 1] =  this->dlLimit;
    ub_SL_f[3*i + 2] =  this->ddlLimit;
  }
  /*************************************/

  sl_fulloasesslover.setPrintLevel(qpOASES::PL_NONE);

  qpOASES::int_t nWSR = 1000;
  qpOASES::returnValue smoresult;
  smoresult = sl_fulloasesslover.init(H_SL_f,f_SL_f,A_SL_f,lb_SL_f,ub_SL_f,lbA_SL_f,ubA_SL_f,nWSR);

  if(smoresult == qpOASES::SUCCESSFUL_RETURN)
    ROS_WARN("SL slover has successful init");
  else
  {
    ROS_WARN("error code %d",smoresult);
    throw std::runtime_error("SL slover fault init");
  }
}

void SmoLine::qpOASES_SLburn()
{
  delete[] H_SL_f;
  delete[] f_SL_f;
  delete[] A_SL_f;
  delete[] lbA_SL_f;
  delete[] ubA_SL_f;
  delete[] lb_SL_f;
  delete[] ub_SL_f;
}

void SmoLine::qpOASES_SLsolver()
{
  for(int16_t i = 0; i < arrayCapacity; i++)
  {
    f_SL_f[3*i + 0] = -2 * this->w_cost_centre * (L_limit[i][1] + L_limit[i][2])/2;
    if((L_limit[i][1] - L_limit[i][2]) >= 0.4 * this->w_cost_centre)      //宽度占比 0.8~1.0时，中线权重
      f_SL_f[3*i + 0] *= 1.0;
    else if((L_limit[i][1] - L_limit[i][2]) >= 0.3 * this->w_cost_centre) //宽度占比 0.6~0.8时，中线权重
      f_SL_f[3*i + 0] *= 2.0;
    else if((L_limit[i][1] - L_limit[i][2]) >= 0.2 * this->w_cost_centre) //宽度占比 0.4~0.6时，中线权重
      f_SL_f[3*i + 0] *= 3.0;
    else if((L_limit[i][1] - L_limit[i][2]) >= 0.1 * this->w_cost_centre) //宽度占比 0.2~0.4时，中线权重
      f_SL_f[3*i + 0] *= 4.0;
    else if((L_limit[i][1] - L_limit[i][2]) >= 0.0 * this->w_cost_centre) //宽度占比 0.0~0.2时，中线权重
      f_SL_f[3*i + 0] *= 5.0;

    f_SL_f[3*i + 1] = 0.0;
    f_SL_f[3*i + 2] = 0.0;
  }

  for(uint16_t i = 0; i < arrayCapacity; i++)
  {
    lb_SL_f[3*i + 0] =  L_limit[i][2] + this->vehicleWidth/2 + this->safety_distance;
    lb_SL_f[3*i + 1] = -this->dlLimit;
    lb_SL_f[3*i + 2] = -this->ddlLimit;

    ub_SL_f[3*i + 0] =  L_limit[i][1] - this->vehicleWidth/2 - this->safety_distance;
    ub_SL_f[3*i + 1] =  this->dlLimit;
    ub_SL_f[3*i + 2] =  this->ddlLimit;
  }

  if(this->retraj == false)
  {
    for(uint16_t i = 0; i < retaintraj_num-vehTotraj_projIndex_; i++)
      ub_SL_f[3*i] = lb_SL_f[3*i] = L_limit[i][3];
  }
  else
  {
      ub_SL_f[0] = lb_SL_f[0] = plan_start_mags.l;
      ub_SL_f[1] = lb_SL_f[1] = plan_start_mags.dl;
      ub_SL_f[2] = lb_SL_f[2] = plan_start_mags.ddl;
  }

  qpOASES::int_t nWSR = 1000;
  qpOASES::returnValue smoresult;

  smoresult = sl_fulloasesslover.hotstart(f_SL_f,lb_SL_f,ub_SL_f,lbA_SL_f,ubA_SL_f,nWSR);

  
  
  if(smoresult == qpOASES::SUCCESSFUL_RETURN)
  {
    ROS_WARN("SUCCESSFUL hotstart,nWSR is:%d",nWSR);
    qpOASES::real_t xOpt[3*arrayCapacity];
    sl_fulloasesslover.getPrimalSolution(xOpt);

    for(int i = 0; i < arrayCapacity; i++)
    {
      save_line[i][0] = L_limit[i][0];
      save_line[i][1] = L_limit[i][1];
      save_line[i][2] = L_limit[i][2];
      save_line[i][3] = L_limit[i][3] = xOpt[3*i];
    }

    Frenet_trans_Cartesian();
  }
  else
  {
    ROS_WARN("fail return %d",smoresult);
    for(int i = 0; i < arrayCapacity ;i++)
    {
      L_limit[i][0] = save_line[i][0];
      L_limit[i][1] = save_line[i][1];
      L_limit[i][2] = save_line[i][2];
      L_limit[i][3] = save_line[i][3];
    }
  }
}

void SmoLine::qpOASES_SLsolver(const uint16_t pointsize)
{
  /*生成二次规划的代价矩阵 0.5x'Hx + x'f*/
  qpOASES::real_t H[3*pointsize * 3*pointsize];
  for(uint16_t i = 0; i < 3*pointsize * 3*pointsize; i++)
    H[i] = 0.0;

  for(int16_t i = 0; i < pointsize; i++)
  {
    H[(3*i+0)*3*pointsize + (3*i+0)] = 2*this->w_cost_centre;
    H[(3*i+1)*3*pointsize + (3*i+1)] = 2*this->w_cost_dl;
    H[(3*i+2)*3*pointsize + (3*i+2)] = 2*this->w_cost_ddl;
  }

  qpOASES::real_t f[3*pointsize * 1];
  for(int16_t i = 0; i < pointsize; i++)
  {
    f_SL_f[3*i + 0] = -2 * this->w_cost_centre * (L_limit[i][1] + L_limit[i][2])/2;
    if((L_limit[i][1] - L_limit[i][2]) >= 0.4 * this->w_cost_centre)      //宽度占比 0.8~1.0时，中线权重
      f_SL_f[3*i + 0] *= 1.0;
    else if((L_limit[i][1] - L_limit[i][2]) >= 0.3 * this->w_cost_centre) //宽度占比 0.6~0.8时，中线权重
      f_SL_f[3*i + 0] *= 2.0;
    else if((L_limit[i][1] - L_limit[i][2]) >= 0.2 * this->w_cost_centre) //宽度占比 0.4~0.6时，中线权重
      f_SL_f[3*i + 0] *= 3.0;
    else if((L_limit[i][1] - L_limit[i][2]) >= 0.1 * this->w_cost_centre) //宽度占比 0.2~0.4时，中线权重
      f_SL_f[3*i + 0] *= 4.0;
    else if((L_limit[i][1] - L_limit[i][2]) >= 0.0 * this->w_cost_centre) //宽度占比 0.0~0.2时，中线权重
      f_SL_f[3*i + 0] *= 5.0;

    f[3*i + 1] = 0.0;
    f[3*i + 2] = 0.0;
  }
    
  /************************************************************************************/

  /* 生成不等式矩阵 lbA <= Ax <= ubA
     A中包含（2n-2）个等式约束和（4n）个不等式约束
     等式约束上下限一致

     修改暂时只需要（2n-2） 个等式约束，不等式约束，用上下界简化约束
  */
  // real_t A[(2*pointsize - 2)*(3*pointsize) + (4*pointsize)*(3*pointsize)];
  // for(u_int16_t i = 0; i < (2*pointsize - 2)*(3*pointsize) + (4*pointsize)*(3*pointsize); i++)
  //   A[i] = 0.0;
  qpOASES::real_t A[(2*pointsize - 2)*(3*pointsize)];
  for(u_int16_t i = 0; i < (2*pointsize - 2)*(3*pointsize); i++)
    A[i] = 0.0;

  Eigen::MatrixXd Aeq_sub(2, 6);//生成等式约束的矩阵,生成Aeq_sub
  Aeq_sub << 1,  ds,     ds*ds/3,  -1,   0,    ds*ds/6,
             0,  1,      ds/2,     0,    -1,   ds/2;

  for(uint16_t i = 0; i < pointsize - 1; i++) //循环导入等式矩阵
    for(uint16_t j = 0; j < 6; j++)
    {
      A[(2*i + 0)*(3*pointsize) + (3*i + j)] = Aeq_sub(0, j);
      A[(2*i + 1)*(3*pointsize) + (3*i + j)] = Aeq_sub(1, j);
    }

  // Eigen::MatrixXd A_sub(4, 3);//生成不等式矩阵，生成A_sub
  // A_sub << 1,  this->vehicleLength/2,  0,
  //          1,  this->vehicleLength/2,  0,
  //          1, -this->vehicleLength/2,  0,
  //          1, -this->vehicleLength/2,  0;
  
  // for(uint16_t i = 0; i < pointsize; i++) //循环导入不等式矩阵
  //   for(uint16_t j = 0; j < 4; j++)
  //   {
  //     A[(2*pointsize - 2)*(3*pointsize) + (4*i + j)*(3*pointsize) + (3*i + 0)] = A_sub(j, 0);
  //     A[(2*pointsize - 2)*(3*pointsize) + (4*i + j)*(3*pointsize) + (3*i + 1)] = A_sub(j, 1);
  //   }
  
  // real_t lbA[(2*pointsize - 2) + (4*pointsize)];//生成约束下边界
  // real_t ubA[(2*pointsize - 2) + (4*pointsize)];//生成约束上边界
  qpOASES::real_t lbA[2*pointsize - 2];
  qpOASES::real_t ubA[2*pointsize - 2];//生成约束上边界
  for(uint16_t i = 0; i < (2*pointsize - 2); i++) //生成等式约束
    ubA[i] = lbA[i] = 0.0;


  // int maxIndex;
  // int minIndex;
  // for(uint16_t i = 0; i < pointsize; i++)//生成不等式约束下边界
  // {
  //   maxIndex = i+1; if(maxIndex >= pointsize) maxIndex = pointsize-1;
  //   minIndex = i-1; if(minIndex <= 0) minIndex = 0;
  //   lbA[(2*pointsize - 2) + (4*i + 0)] =-this->vehicleWidth/2 + this->L_limit[maxIndex][2];
  //   lbA[(2*pointsize - 2) + (4*i + 1)] =+this->vehicleWidth/2 + this->L_limit[maxIndex][2];
  //   lbA[(2*pointsize - 2) + (4*i + 2)] =-this->vehicleWidth/2 + this->L_limit[minIndex][2];
  //   lbA[(2*pointsize - 2) + (4*i + 3)] =+this->vehicleWidth/2 + this->L_limit[minIndex][2];
  // }

  // for(uint16_t i = 0; i < pointsize; i++)//生成不等式约束上边界
  // {
  //   maxIndex = i+1; if(maxIndex >= pointsize) maxIndex = pointsize-1;
  //   minIndex = i-1; if(minIndex <= 0) minIndex = 0;
  //   ubA[(2*pointsize - 2) + (4*i + 0)] =-this->vehicleWidth/2 + this->L_limit[maxIndex][1];
  //   ubA[(2*pointsize - 2) + (4*i + 1)] =+this->vehicleWidth/2 + this->L_limit[maxIndex][1];
  //   ubA[(2*pointsize - 2) + (4*i + 2)] =-this->vehicleWidth/2 + this->L_limit[minIndex][1];
  //   ubA[(2*pointsize - 2) + (4*i + 3)] =+this->vehicleWidth/2 + this->L_limit[minIndex][1];
  // }
  /*****************************************************************/

  /*生成x的上下边界lb,ub*/
  qpOASES::real_t lb[3*pointsize];
  qpOASES::real_t ub[3*pointsize];
  for(uint16_t i = 0; i < pointsize; i++)
  {
    lb[3*i + 0] =  L_limit[i][2] + this->vehicleWidth/2 + this->safety_distance;
    lb[3*i + 1] = -this->dlLimit;
    lb[3*i + 2] = -this->ddlLimit;

    ub[3*i + 0] =  L_limit[i][1] - this->vehicleWidth/2 - this->safety_distance;
    ub[3*i + 1] =  this->dlLimit;
    ub[3*i + 2] =  this->ddlLimit;
  }

  if(this->retraj == false)
  {
    for(uint16_t i = 0; i < retaintraj_num-vehTotraj_projIndex_; i++)
      ub[3*i] = lb[3*i] = L_limit[i][3];
  }
  else
  {
      ub[0] = lb[0] = plan_start_mags.l;
      ub[1] = lb[1] = plan_start_mags.dl;
      ub[2] = lb[2] = plan_start_mags.ddl;
  }

  qpOASES::QProblem qpoasesslove(3*pointsize, (2*pointsize - 2));
  qpoasesslove.setPrintLevel(qpOASES::PL_NONE);

  qpOASES::int_t nWSR = 1000;
  qpOASES::returnValue smoresult;
  smoresult = qpoasesslove.init(H,f,A,lb,ub,lbA,ubA,nWSR);
  
  if(smoresult == qpOASES::SUCCESSFUL_RETURN)
  {
    ROS_WARN("SUCCESSFUL RETURN,nWSR is:%d",nWSR);
    qpOASES::real_t xOpt[3*pointsize];
    qpoasesslove.getPrimalSolution(xOpt);

    for(int i = 0; i < pointsize; i++)
    {
      save_line[i][0] = L_limit[i][0];
      save_line[i][1] = L_limit[i][1];
      save_line[i][2] = L_limit[i][2];
      save_line[i][3] = L_limit[i][3] = xOpt[3*i];
    }

    Frenet_trans_Cartesian();
  }
  else
  {
    ROS_WARN("fail return %d",smoresult);
    for(int i = 0; i < arrayCapacity ;i++)
    {
      L_limit[i][0] = save_line[i][0];
      L_limit[i][1] = save_line[i][1];
      L_limit[i][2] = save_line[i][2];
      L_limit[i][3] = save_line[i][3];
    }
  }
}

void SmoLine::qpOASES_XYinit()
{
  /***********代价矩阵初始化****************/
  H_XY_f = new qpOASES::real_t[2*arrayCapacity * 2*arrayCapacity];
  for(uint16_t i = 0; i < 4*arrayCapacity*arrayCapacity; i++) H_XY_f[i] = 0.0;
  f_XY_f = new qpOASES::real_t[2*arrayCapacity * 1];
  for(uint16_t i = 0; i < 2*arrayCapacity; i++) f_XY_f[i] = 0.0;

  Eigen::MatrixXd A1(6, 6);//生成平滑代价矩阵
  A1 << 1,  0, -2,  0,  1,  0,
        0,  1,  0, -2,  0,  1,
       -2,  0,  4,  0, -2,  0,
        0, -2,  0,  4,  0, -2,
        1,  0, -2,  0,  1,  0,
        0,  1,  0, -2,  0,  1;
  for(int16_t i = 0; i < arrayCapacity-2; i++)
    for(int16_t j = 0; j < 6; j++)
    {
      //每3个点(6个参数)作为变量去计算代价；因此每换一个点就需要偏移两行两列
      //(2*arrayCapacity)*2i 向下偏移2i行；(2*i)是列偏移，向右偏移两列
      //(2*arrayCapacity)*j是在当前的行偏移下再向下偏移，因为A1有6行，所以需要偏移6次；
      H_XY_f[(2*arrayCapacity)*(2*i + j) + 2*i + 0] += A1(j, 0) * 2*this->w_cost_smooth;
      H_XY_f[(2*arrayCapacity)*(2*i + j) + 2*i + 1] += A1(j, 1) * 2*this->w_cost_smooth;
      H_XY_f[(2*arrayCapacity)*(2*i + j) + 2*i + 2] += A1(j, 2) * 2*this->w_cost_smooth;
      H_XY_f[(2*arrayCapacity)*(2*i + j) + 2*i + 3] += A1(j, 3) * 2*this->w_cost_smooth;
      H_XY_f[(2*arrayCapacity)*(2*i + j) + 2*i + 4] += A1(j, 4) * 2*this->w_cost_smooth;
      H_XY_f[(2*arrayCapacity)*(2*i + j) + 2*i + 5] += A1(j, 5) * 2*this->w_cost_smooth;
    }

  for(uint16_t i = 0; i < 2*arrayCapacity; i++)//生成相似代价矩阵
    H_XY_f[(2*arrayCapacity)*i + i] += 2*this->w_cost_ref;


  // Eigen::MatrixXd A2(4, 4);//生成紧凑代价矩阵
  // A2 << 1,  0, -1,  0,
  //       0,  1,  0, -1,
  //      -1,  0,  1,  0,
  //       0, -1,  0,  1;
  // for(int16_t i = 0; i < arrayCapacity-1; i++)
  //   for(int16_t j = 0; j < 4; j++)
  //   {
  //     H_XY_f[(2*arrayCapacity)*(2*i + j) + 2*i + 0] += A2(j, 0) * this->w_cost_length;
  //     H_XY_f[(2*arrayCapacity)*(2*i + j) + 2*i + 1] += A2(j, 1) * this->w_cost_length;
  //     H_XY_f[(2*arrayCapacity)*(2*i + j) + 2*i + 2] += A2(j, 2) * this->w_cost_length;
  //     H_XY_f[(2*arrayCapacity)*(2*i + j) + 2*i + 3] += A2(j, 3) * this->w_cost_length;
  //   }

  xy_fulloasesslover.setPrintLevel(qpOASES::PL_NONE);
  qpOASES::int_t nWSR = 1000;
  qpOASES::returnValue smoresult;
  smoresult = xy_fulloasesslover.init(H_XY_f,f_XY_f,NULL,NULL,NULL,NULL,NULL,nWSR);
  if(smoresult == qpOASES::SUCCESSFUL_RETURN)
    ROS_WARN("XY slover has successful init");
  else
  {
    ROS_WARN("error code %d",smoresult);
    throw std::runtime_error("XY slover fault init");
  }
}


void SmoLine::qpOASES_XYburn()
{
  delete[] H_XY_f;
  delete[] f_XY_f;
  // delete[] A_XY_f;
  // delete[] lbA_XY_f;
  // delete[] ubA_XY_f;
  // delete[] lb_XY_f;
  // delete[] ub_XY_f;
}


void SmoLine::qpOASES_XYsolver()
{
  for(uint16_t i = 0; i < arrayCapacity; i++)
  {
    f_XY_f[2*i + 0] = -2 * this->trajline_SLpoint.poses[i].pose.position.x * this->w_cost_ref;
    f_XY_f[2*i + 1] = -2 * this->trajline_SLpoint.poses[i].pose.position.y * this->w_cost_ref;
  }

  qpOASES::int_t nWSR = 1000;
  qpOASES::returnValue smoresult;

  smoresult = xy_fulloasesslover.hotstart(f_XY_f,NULL,NULL,NULL,NULL,nWSR);

  if(smoresult == qpOASES::SUCCESSFUL_RETURN)
  {
    trajline_point.poses.clear();
    trajline_point.header.frame_id = Frame_id;
    trajline_point.header.stamp = ros::Time::now();
    /*显示到rivz的图像*/
    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.header.frame_id = Frame_id;
    pose_stamp.header.stamp = ros::Time::now();

    visualization_msgs::Marker peak;
    peak.header.frame_id =  "planning_odom";
    peak.header.stamp = ros::Time::now();
    peak.ns = "points_and_line";
    peak.action = visualization_msgs::Marker::ADD;
    peak.pose.orientation.w = 1.0;
    peak.id = 3;
    peak.type = visualization_msgs::Marker::POINTS;
    peak.scale.x = 0.1;
    peak.scale.y = 0.1;
    peak.color.g = 1.0;
    peak.color.a = 1.0;
    /********************/
    geometry_msgs::Point p;

    ROS_WARN("SUCCESSFUL hotstart,nWSR is:%d",nWSR);
    qpOASES::real_t xOpt[2*arrayCapacity];
    xy_fulloasesslover.getPrimalSolution(xOpt);
    for(int16_t i = 0; i < arrayCapacity; i++)
    {
      p.x = xOpt[2*i + 0];
      p.y = xOpt[2*i + 1];
      peak.points.push_back(p);
      pose_stamp.pose.position = p;
      trajline_point.poses.push_back(pose_stamp);
    }
    // this->marker_pub_.publish(peak);
    this->trajline_pub_.publish(trajline_point);
    calc_traj_theta();
  }
}


















#endif