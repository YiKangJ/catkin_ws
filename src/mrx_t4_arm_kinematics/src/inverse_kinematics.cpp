/*
 * inverse_kinematics.cpp
 *
 *  Created on: 31-May-2018
 *      Author: YiKangJ
 */

#include <mrx_t4_arm_kinematics/inverse_kinematics.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>

#define PI 3.141592653589793
#define INVALID 500

using namespace mrx_t4_arm_kinematics;

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

const double InverseKinematics::ALMOST_PLUS_ONE = 0.9999999;
const double InverseKinematics::ALMOST_MINUS_ONE = -0.9999999;
const double error = 1e-7;

std::stringstream sstr;

InverseKinematics::InverseKinematics(
        const std::vector<double> &min_angles,
        const std::vector<double> &max_angles,
        Logger &logger) : logger_(logger)
{
    min_angles_ = min_angles;
    max_angles_ = max_angles;
}

InverseKinematics::~InverseKinematics()
{
}

int InverseKinematics::CartToJnt(const KDL::JntArray &q_init,
        const KDL::Frame &p_in,
        std::vector<KDL::JntArray> &q_out)
{	

    std::vector<KDL::JntArray> solution(2);
    
	for (unsigned int i=0;i<solution.size();i++)
        for (unsigned int j=0;j<solution[i].rows();j++) 
        {
            solution[i](j) = INVALID;
        }
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
	double l1 = 0.257 ;
	double l2 = 0.255 ;
	double l3 = 0.250 ;
	double l4 = 0.150 ;
	double l5 = 0 ;
	double l6 = 0 ;
	
    std::vector<KDL::JntArray> solution(2, KDL::JntArray(4));
    double R,P,Y ;
	KDL::Frame goal = g0 ;

    sstr << "Goal Position is: "
    		<< goal.p[0] << ","
    		<< goal.p[1] << ","
			<< goal.p[2]
			<< std::endl ;

    goal.M.GetRPY(R,P,Y) ;
    sstr << "RPY is: "
        		<< R << ","
        		<< P << ","
    			<< Y
    			<< std::endl ;

	// Joint 1
	KDL::Vector pos_d = goal.p ;
    KDL::Vector pos_ox = goal.M.UnitX();
    KDL::Vector pos_oy = goal.M.UnitY();
    KDL::Vector pos_oz = goal.M.UnitZ();

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
    
    double bx,by,bz,c1,s1,c2,s2,A,B,C,det,s3,c3;
    bool flag,flag1,flag2;
    bool mark[2] = {true, true};

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
        sstr << "No valid solution:Point is unreachable.**************************************************************************************************************************************************************************************************************************" << std::endl ;
        logger_.write(sstr.str(), __FILE__, __LINE__);
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
		sstr << "Can't find a reasonable solution. No solution exists:Point is unreachable. *******************************************************************************************************************************************************************************************" << std::endl ;
    	logger_.write(sstr.str(), __FILE__, __LINE__);
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

    sstr << "j1: " << solution[0](0) << std::endl ;
	sstr << "j2: " << solution[0](1) << std::endl ;
	sstr << "j3: " << solution[0](2) << std::endl ;
	sstr << "j4: " << solution[0](3) << std::endl ;
	sstr << "j1_2: " << solution[1](0) << std::endl ;
	sstr << "j2_2: " << solution[1](1) << std::endl ;
	sstr << "j3_2: " << solution[1](2) << std::endl ;
	sstr << "j4_2: " << solution[1](3) << std::endl ;


	logger_.write(sstr.str(), __FILE__, __LINE__);

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
