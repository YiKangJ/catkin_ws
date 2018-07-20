/*
 * inverse_kinematics.cpp
 *
 *  Created on: 11-July-2018
 *      Author: YiKangJ
 */

#include <mrx_t4_arm_kinematics/inverse_kinematics.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>

#define PI 3.141592653589793
#define INVALID 500
#define THETA -90 // 末端关节角度（度） 90：竖直向上，-90：竖直向下，0：水平
#define DISLIM 0.01 // disLim

using namespace mrx_t4_arm_kinematics;


const double error = 1e-7;
const double l1 = 0.257 ;
const double l2 = 0.255 ;
const double l3 = 0.250 ;
const double l4 = 0.150 ;
const double l5 = 0 ;
const double l6 = 0 ;


InverseKinematics::InverseKinematics(
        const std::vector<double> &min_angles,
        const std::vector<double> &max_angles,
        Logger &logger) : logger_(logger)
{
    min_angles_ = min_angles;
    min_angles_[1] += PI/2;
    min_angles_[2] -= PI/2;
    min_angles_[3] -= PI/2;
    max_angles_ = max_angles;
    max_angles_[1] += PI/2;
    max_angles_[2] -= PI/2;
    max_angles_[3] -= PI/2;

}

InverseKinematics::~InverseKinematics()
{
}

int InverseKinematics::CartToJnt(const KDL::JntArray &q_init,
        const KDL::Frame &p_in,
        std::vector<KDL::JntArray> &q_out)
{	

    std::vector<KDL::JntArray> solution(2);
    /*    
	for (unsigned int i=0;i<solution.size();i++)
        for (unsigned int j=0;j<solution[i].rows();j++) 
        {
            solution[i](j) = INVALID;
        }
    */

    // there are no solutions available yet
    q_out.clear();

    // iterate over all redundant solutions
    solution = ik(p_in);
    for (unsigned int i=0;i<2;i++)
    {
        if (isSolutionValid(solution[i])) q_out.push_back(solution[i]);
    }

    if (q_out.size() > 0) {
        logger_.write("Inverse kinematics found a solution",
                __FILE__, __LINE__);
        return 1;
    } else {
        logger_.write("Inverse kinematics found no solution",
                __FILE__, __LINE__);

        return -1;
    }
}

std::vector<KDL::JntArray> InverseKinematics::ik(const KDL::Frame& g0)
{
	
    std::vector<KDL::JntArray> solution(2, KDL::JntArray(4));
    double x,y,z,w ;
	KDL::Frame goal = g0 ;
    std::stringstream sstr;

    sstr << std::setprecision(15)
            << "Goal Position is: "
    		<< goal.p[0] << ","
    		<< goal.p[1] << ","
			<< goal.p[2]
			<< std::endl ;

    goal.M.GetQuaternion(x,y,z,w) ;
    sstr << std::setprecision(15)
                << "Quat is: "
        		<< x << ","
        		<< y << ","
    			<< z << ","
                << w
    			<< std::endl ;

	// Joint 1
	KDL::Vector pos_d = goal.p ;
    KDL::Vector pos_ox;
    KDL::Vector pos_oy;
    KDL::Vector pos_oz;

	// init
    for (unsigned int i=0;i<solution.size();i++)
        for (unsigned int j=0;j<solution[i].rows();j++) 
        {
            solution[i](j) = INVALID;
        }
    
    if (calFixPosMatr(pos_d, pos_ox, pos_oy, pos_oz) != 1) 
    {   
        sstr << "生成位姿矩阵失败！" << std::endl;
    	logger_.write(sstr.str(), __FILE__, __LINE__);
        //ROS_INFO("%s", sstr.str().c_str());
        return solution;
    }

	double posIn14 = pos_d[0] ;
	double posIn24 = pos_d[1] ;
	double posIn34 = pos_d[2] ;
    double posIn11 = pos_ox[0];
    double posIn21 = pos_ox[1];
    double posIn31 = pos_ox[2];
    double posIn12 = pos_oy[0];
    double posIn22 = pos_oy[1];
    double posIn32 = pos_oy[2];
    double posIn13 = pos_oz[0];
    double posIn23 = pos_oz[1];
    double posIn33 = pos_oz[2];
    double posOrg11 = 0;
    double posOrg12 = 0;
    double posOrg21 = 0;
    double posOrg22 = 0;
    double posOrg31 = 0;
    double posOrg32 = 0;
    double posOrg41 = 0;
    double posOrg42 = 0;
    
    double bx,by,bz,c1,s1,c2,s2,A,B,C,det,s3,c3,px,py,nx,ny;
    bool flag,flag1,flag2;
    bool mark[2] = {true, true};

    
    // new added
    s1 = posIn13;
    c1 = -posIn23;
    px = posIn14;
    py = posIn24;
    nx = posIn11;
    ny = posIn21;
    if (!((fabs((px - s1*l5)*s1 - (py + c1*l5)*c1) <= error) && (fabs(nx*s1 - ny*c1) <= 1e-5)))
    {
        sstr << "Input pose matrix is invalid." << std::endl ;
    	logger_.write(sstr.str(), __FILE__, __LINE__);
        //ROS_INFO("%s", sstr.str().c_str());
        return solution;
    }

	if (fabs(posIn23) == 0) posOrg11 = atan2(posIn13, fabs(posIn23));
	else posOrg11 = atan2(posIn13, -posIn23);
    posOrg12 = posOrg11;
    c1 = cos(posOrg11);
    s1 = sin(posOrg11);

    bx = posIn14 - c1*l4*posIn32 - l5*s1 + c1*l6*posIn31;
    by = posIn24 - s1*l4*posIn32 + l5*c1 + s1*l6*posIn31;
    bz = posIn34 - l4*posIn31 - l6*posIn32;
    
    if (fabs(c1)>0.1)
    {
        A = pow(c1,2)*(pow(bz - l1,2) - pow(l2,2) - pow(l3,2)) + pow(bx,2);
        B = 2*pow(c1,2)*l2*l3;
    } else
    {
        A = pow(s1,2)*(pow(bz - l1,2) - pow(l2,2) - pow(l3,2)) + pow(by,2);
        B = 2*pow(s1,2)*l2*l3; 
    }

    det = A/B;
    
    std::vector<double> lim(2);
    lim[0] = -1;
    lim[1] = 1;

    flag = judgeNum(det,lim,error);
    
    if (flag)
    {
        posOrg31 = acos(det);
        posOrg32 = -acos(det);
        if (fabs(posOrg31-posOrg32)<error)
        {
            posOrg32 = posOrg31;
        }
    } else 
    {
		sstr << "Argument for j3 is out of range. No solution exists:Point is unreachable" << std::endl ;
    	logger_.write(sstr.str(), __FILE__, __LINE__);
        //ROS_INFO("%s", sstr.str().c_str());
        return solution;
    }
   
	//  solve theta2 and theta4

    s3 = sin(posOrg31);
    c3 = cos(posOrg31);

    if (fabs(c1)>0.1)
    {
        A = -bx*s3*l3 + (bz - l1)*c1*(l2 + c3*l3);
        B = (l2 + c3*l3)*bx + c1*s3*l3*(bz - l1);
        C = c1*(pow(l2,2) + 2*c3*l2*l3 + pow(l3,2));
    } else
    {
        A = -by*s3*l3 + (bz - l1)*s1*(l2 + c3*l3);
        B = (l2 + c3*l3)*by + s1*s3*l3*(bz - l1);
        C = s1*(pow(l2,2) + 2*c3*l2*l3 + pow(l3,2));
    }
    s2 = A/C;
    c2 = B/C;
    flag1 = judgeNum(s2, lim, error);
    flag2 = judgeNum(c2, lim, error);

    if (!(flag1 || flag2))
    {
        sstr << "Argument for j2 is out of range. No solution exists:Point i    s unreachable" << std::endl ;
        mark[0] = false;
    } else if (flag1 && flag2)
    {
        if (fabs(c2==0)) posOrg21 = atan2(s2,fabs(c2));
		else posOrg21 = atan2(s2,fabs(c2));
		if (fabs(posIn32) == 0) posOrg41 = standard(atan2(posIn31,fabs(posIn32)) - posOrg21 - posOrg31); 
		else posOrg41 = standard(atan2(posIn31,posIn32) - posOrg21 - posOrg31); 
       
       // solution[0](0) = posOrg11;
       // solution[0](1) = posOrg21;
       // solution[0](2) = posOrg31;
       // solution[0](3) = posOrg41;
      
     }
    
    s3 = sin(posOrg32);
    c3 = cos(posOrg32);

    if (fabs(c1)>0.1)
    {
        A = -bx*s3*l3 + (bz - l1)*c1*(l2 + c3*l3);
        B = (l2 + c3*l3)*bx + c1*s3*l3*(bz - l1);
        C = c1*(pow(l2,2) + 2*c3*l2*l3 + pow(l3,2));
    } else
    {
        A = -by*s3*l3 + (bz - l1)*s1*(l2 + c3*l3);
        B = (l2 + c3*l3)*by + s1*s3*l3*(bz - l1);
        C = s1*(pow(l2,2) + 2*c3*l2*l3 + pow(l3,2));
    }
    s2 = A/C;
    c2 = B/C;
    flag1 = judgeNum(s2, lim, error);
    flag2 = judgeNum(c2, lim, error);

    if (!(flag1 || flag2))
    {
        sstr << "Argument for j2_2 is out of range. No solution exists:Point i    s unreachable" << std::endl ;
        mark[1] = false;
    } else if (flag1 && flag2)
    {
       	if (fabs(c2) == 0) posOrg22 = atan2(s2, fabs(c2));
	   	else  posOrg22 = atan2(s2,c2);
        if (fabs(posIn32) == 0) posOrg42 = standard(atan2(posIn31, fabs(posIn32)) - posOrg22 - posOrg32); 
		else posOrg42 = standard(atan2(posIn31,posIn32) - posOrg22 - posOrg32); 
        
        //solution[1](0) = posOrg12;
        //solution[1](1) = posOrg22;
        //solution[1](2) = posOrg32;
        //solution[1](3) = posOrg42;
       
    }
    if (!(mark[0] || mark[1]))
    {
        sstr << "No valid solution:Point is unreachable." << std::endl ;
        logger_.write(sstr.str(), __FILE__, __LINE__);
        //ROS_INFO("%s", sstr.str().c_str());
        return solution;
    }
    
    std::vector<std::vector<double> > pos(2,std::vector<double>(4,0));
    pos[0][0] = posOrg11;
    pos[0][1] = posOrg21;
    pos[0][2] = posOrg31;
    pos[0][3] = posOrg41;
    pos[1][0] = posOrg12;
    pos[1][1] = posOrg22;
    pos[1][2] = posOrg32;
    pos[1][3] = posOrg42;

    unsigned int temp = 0;
    for (unsigned int i=0;i<2;i++)
    {
        if (mark[i] && judgePos(pos[i], min_angles_, max_angles_)) temp += i+1;
    }
    
    if (temp == 0)
    {	/*
		sstr << "posIn:" << std::endl;
		sstr << posIn11 << "," << posIn12 << "," << posIn13 << std::endl;
		sstr << posIn21 << "," << posIn22 << "," << posIn23 << std::endl;
		sstr << posIn31 << "," << posIn32 << "," << posIn33 << std::endl;
		sstr << "pos[0,0]: " << pos[0][0] << std::endl;
		sstr << "pos[1,0]: " << pos[1][0] << std::endl;
		sstr << "pos[0,1]: " << pos[0][1] << std::endl;
		sstr << "pos[1,1]: " << pos[1][1] << std::endl;
		sstr << "pos[0,2]: " << pos[0][2] << std::endl;
		sstr << "pos[1,2]: " << pos[1][2] << std::endl;
		sstr << "pos[0,3]: " << pos[0][3] << std::endl;
		sstr << "pos[1,3]: " << pos[1][3] << std::endl;
        */
		sstr << "Can't find a reasonable solution. No solution exists:Point is unreachable." << std::endl ;
    	logger_.write(sstr.str(), __FILE__, __LINE__);
        //ROS_INFO("%s",sstr.str().c_str());
        return solution;
    } else if (temp>0)
    {
        if (temp == 3)
        {
            solution[0](0) = posOrg11;
        	solution[0](1) = posOrg21;
            solution[0](2) = posOrg31;
            solution[0](3) = posOrg41;
            solution[1](0) = posOrg12;
            solution[1](1) = posOrg22;
            solution[1](2) = posOrg32;
            solution[1](3) = posOrg42;
        }	else if (temp == 1)
        {
            solution[0](0) = posOrg11;
            solution[0](1) = posOrg21;
            solution[0](2) = posOrg31;
            solution[0](3) = posOrg41;
            solution[1](0) = posOrg11;
            solution[1](1) = posOrg21;
            solution[1](2) = posOrg31;
            solution[1](3) = posOrg41;
        } else if (temp ==2)
        {
            solution[0](0) = posOrg12;
            solution[0](1) = posOrg22;
            solution[0](2) = posOrg32;
            solution[0](3) = posOrg42;
            solution[1](0) = posOrg12;
            solution[1](1) = posOrg22;
            solution[1](2) = posOrg32;
            solution[1](3) = posOrg42;
        }
    }

    solution[0](1) -= PI/2;
    solution[0](2) += PI/2;
    solution[0](3) += PI/2;

    solution[1](1) -= PI/2;
    solution[1](2) += PI/2;
    solution[1](3) += PI/2;
    
    sstr << "j1: " << solution[0](0) << std::endl ;
	sstr << "j2: " << solution[0](1) << std::endl ;
	sstr << "j3: " << solution[0](2) << std::endl ;
	sstr << "j4: " << solution[0](3) << std::endl ;
	sstr << "j1_2: " << solution[1](0) << std::endl ;
	sstr << "j2_2: " << solution[1](1) << std::endl ;
	sstr << "j3_2: " << solution[1](2) << std::endl ;
	sstr << "j4_2: " << solution[1](3) << std::endl ;


	logger_.write(sstr.str(), __FILE__, __LINE__);

    //ROS_INFO("%s",sstr.str().c_str());
	return solution ;

}


bool mrx_t4_arm_kinematics::judgeNum(double &target, std::vector<double> lim,double error)
{
    bool flag=true;
    double temp;
    if (lim[0]>lim[1])
        {
            temp = lim[0];
            lim[0] = lim[1];
            lim[1] = temp;
        }  

    if (target<(lim[0]-fabs(error)))
    {
        flag = false;
    } else if(target>(lim[1]+fabs(error))) 
    {
        flag = false;
    }
    
    if (target<lim[0]) target = lim[0];
    if (target>lim[1]) target = lim[1];

    return flag;
}

double mrx_t4_arm_kinematics::standard(double target)
{
    while (target>=PI) target -= 2*PI;
    while (target<-PI) target += 2*PI;

    return target;
}

bool mrx_t4_arm_kinematics::checkLim(double tar, double min_ang, double max_ang)
{
    double max_angle, min_angle, target;
    max_angle = standard(max_ang);
    min_angle = standard(min_ang);
    target = standard(tar);
    if (min_angle >= max_angle)
    {
        if (target <= max_angle || target >= min_angle) return true;
    } else
    {
        if (target >= min_angle && target <= max_angle) return true;
    }
    
    return false;
}
bool InverseKinematics::isSolutionValid(const KDL::JntArray &solution) const
{
    bool valid = false;

    if (solution.rows() != 4) return false;

    for (unsigned int i = 0; i < solution.rows(); i++) 
    {
        if (solution(i)!= INVALID) valid = true;
    }

    /*
    for (unsigned int i = 0; i < solution.rows(); i++) 
    {
        if (!(checkLim(solution(i),min_angles_[i], max_angles_[i]))) valid = false;
    }
    */
    
    return valid;
}


bool mrx_t4_arm_kinematics::judgePos(std::vector<double> pos, std::vector<double> min_angles, std::vector<double> max_angles)
{
    for (unsigned int i=0;i<pos.size();i++)
    {
        if (checkLim(pos[i], min_angles[i], max_angles[i]) == false) return false;
    }
    return true;
}

int mrx_t4_arm_kinematics::calFixPosMatr(KDL::Vector pos_d, KDL::Vector &pos_ox, KDL::Vector &pos_oy, KDL::Vector &pos_oz)
{
    
    double d,theta,px,py,disLim,th2a3a4,t11,t12,temp,K,t1,r1,r2,px1,py1,px2,py2;
    int flag;
    
    disLim = DISLIM;
    th2a3a4 = (double)THETA*PI/180;
    d = l5;
    px = pos_d[0];
    py = pos_d[1];
    flag = 1;
    
    if (disLim < fabs(l5)) disLim = disLim + fabs(l5); // 必须保证disLim >fabs(l5)
    temp = sqrt(px*px + py*py);
    if (fabs(temp) < disLim)  // 末端在xoy上投影与坐标原点距离过近
    {
        flag = -1;
        return flag;  // 返回：无法确定位姿
    } else  // 在工作区域内，一定满足 d <= temp
    {
        K = d/temp;
        if (fabs(K) > (1 - error))
        {
            flag = 0;
            return flag; // 返回：不在工作区间内
         }
            
         if (fabs(px) == 0) t11 = atan2(py, fabs(px)) - asin(-K);
         else t11 = atan2(py, px) - asin(-K);
         if (fabs(px) == 0) t12 = atan2(-py, fabs(px)) - asin(K);
         else t12 = atan2(-py, -px) - asin(K);

         /*
          ** 利用末端轴向偏移量为零的的点来判断关节1角度 **
         */
         px1 = px - sin(t11) * l5 + l6 * cos(t11) * sin(th2a3a4); // 偏移为0处的坐标x1
         py1 = py + cos(t11) * l5 + l6 * sin(t11) * sin(th2a3a4); // 偏移为0处的坐标y1
         px2 = px - sin(t12) * l5 + l6 * cos(t12) * sin(th2a3a4); // 偏移为0处的坐标x2
         py2 = py + cos(t12) * l5 + l6 * sin(t12) * sin(th2a3a4); // 偏移为0处的坐标y2
    }
    
    /*
     ** 选取关节1角度，以约束末端点坐标姿态 **
    */
    if (fabs(px1) < error && fabs(py1) < error) t1 = t11;
    else if (fabs(px2) < error && fabs(py2) < error) t1 = t12;
    else
    {
        r1 = px1 * px + py1 * py;
        r2 = px2 * px + py2 * py;

        if (r1 < 0 ) t1 = t11;
        else if (r2 < 0) t1 = t12;
        else
        {
            theta = atan2(py1, px1);
            if (standard(theta) - standard(t11) < 1e-3) t1 = t11;
            else t1 = t12;
        }
    }
   
    pos_ox[0] = cos(t1) * cos(th2a3a4);
    pos_ox[1] = sin(t1) * cos(th2a3a4);
    pos_ox[2] = sin(th2a3a4);
    pos_oy[0] = -cos(t1) * sin(th2a3a4);
    pos_oy[1] = -sin(t1) * sin(th2a3a4);
    pos_oy[2] = cos(th2a3a4);
    pos_oz[0] = sin(t1);
    pos_oz[1] = -cos(t1);
    pos_oz[2] = 0;

    return flag;
}
