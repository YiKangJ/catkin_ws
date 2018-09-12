/**
 * @file test_kdl.cpp
 * @brief 求解器核心测试
 * 
 * 仅仅从inverse_kinematics.cpp抽取核心做测试用，不影响求解器即插件的使用
 *  
 * @author YikangJ
 *
 * @date 07/20 2018
 */


#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <stdio.h>
#include <iostream>
#include <iomanip>

#define PI 3.141592653589793

using namespace std;

std::stringstream sstr;

std::vector<double> min_angles,max_angles;

bool checkLim(double tar, double min_ang, double max_ang);

bool judgeNum(double &target, std::vector<double> lim,double error);

double standard(double target);

bool judgePos(std::vector<double> pos, std::vector<double> min_angles, std::vector<double> max_angles);

bool isSolutionValid(const std::vector<double> &solution);

int main()
{
	double l1 = 0.257 ;
	double l2 = 0.255 ;
	double l3 = 0.250 ;
	double l4 = 0.150 ;
	double l5 = 0 ;
	double l6 = 0 ;
    double error = 1e-7;
	min_angles.resize(4,0.0) ;
	min_angles[0] = -2.757620 ;
	min_angles[1] =  0 ;
	min_angles[2] = -2.5830873;
	min_angles[3] =	-3.14159 ;

	max_angles.resize(4,0.0);
	max_angles[0] = 2.757620 ;
	max_angles[1] = 2.05948852 ;
	max_angles[2] = 1.5358897418 ;
	max_angles[3] =	3.14159 ;

	double R,P,Y, x, y, z, w ;
    KDL::Frame goal;
    KDL::Rotation rot;

    goal.p.x(0.45);
    goal.p.y(-0.234);
    goal.p.z(0.42);
    const KDL::Vector  UX(0.88721680123, -0.461352737, 0);
    const KDL::Vector  UY(0,0,1);
    const KDL::Vector  UZ(-0.4613527367, -0.88721680123, 0); 
    goal.M = KDL::Rotation(UX,UY,UZ);
	goal.M.GetQuaternion(x, y, z, w);
/*
    goal.p.x(0.4);
    goal.p.y(0);
    goal.p.z(0.4);
   
    goal.M = rot.Quaternion(0.707107, -1.29868e-6, -1.29867e-6, 0.707107);

	goal.M.GetQuaternion(x, y, z, w);
 
    goal.p.x(0.132938660994);
    goal.p.y(-0.0955394659395);
    goal.p.z(0.818050660634);
    
    goal.M = rot.Quaternion(0.568943188752, -0.419889964574, 0.00894212272965, 0.707047455312);
*/ 
    //goal.p.x(5.11445297456e-05);
    //goal.p.y(2.2722001058e-06);
    //goal.p.z(0.761999997366);
    
    //goal.M = rot.Quaternion(-1.83892000533e-06, 0.000109189189136, 1.77042505601e-05, 0.99999999388);

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
    std::vector<std::vector<double> > posOut(2, std::vector<double>(4,0));
 	 sstr << setprecision(15) << "Quaternion: " << x << " " << y << " " << z << " " << w << endl;
  	sstr << setprecision(15) << posIn11 << " " << posIn12 << " " << posIn13 << " " << posIn14 << endl;
  	sstr << setprecision(15) << posIn21 << " " << posIn22 << " " << posIn23 << " " << posIn24 << endl;
 	 sstr << setprecision(15) << posIn31 << " " << posIn32 << " " << posIn33 << " " << posIn34 << endl; 
    
    double bx,by,bz,c1,s1,c2,s2,A,B,C,det,s3,c3,nx,ny,px,py;
    bool flag,flag1,flag2;
    bool mark[2]={true, true};
sstr << "atan2: " << atan2(posIn13, posIn23) << endl;
	double a = 0;
    // new added
    s1 = posIn13;
    c1 = -posIn23;
    px = posIn14;
    py = posIn24;
    nx = posIn11;
    ny = posIn21;
    if (!((fabs((px - s1*l5)*s1 - (py + c1*l5)*c1) <= 1e-5) && (fabs(nx*s1 - ny*c1) <= 1e-5)))
    {
        sstr << "Input pose matrix is unvalid." << std::endl ;
        sstr << "px*s1:" << px << "," << s1 << "," << px*s1 << endl;
        sstr << "py*c1:" << py << ',' << c1 << "," << py*c1 << endl;
        sstr << (fabs((px - s1*l5)*s1 - (py + c1*l5)*c1)) <<std::endl;
        sstr << (fabs(nx*s1 - ny*c1)) << endl;
        cout << sstr.str() << endl;
        return -1;
    }
    if (fabs(posIn23) == 0) posOrg11 = atan2(posIn13, fabs(posIn23));
	else posOrg11 = atan2(posIn13,-posIn23);
    posOrg12 = posOrg11;
    c1 = cos(posOrg11);
    s1 = sin(posOrg11);
sstr << "s1:" << s1 << endl;
sstr << "c1:" << c1 << endl;
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

  sstr << "bx:" << bx << endl; // det 1
  sstr << "by:" << by << endl; // det 1
  sstr << "bz:" << bz << endl; // det 1
  sstr << "B:" << B << endl; // det 1
    det = A/B;
    
    std::vector<double> lim(2);
    lim[0] = -1;
    lim[1] = 1;

    flag = judgeNum(det,lim,error);//changed

  sstr << "det:" << det << endl; // det 1
  sstr << "flag:" << flag << endl; // det 1
    if (flag)
    {
        posOrg31 = acos(det);
        posOrg32 = -acos(det);
        if ((fabs(posOrg31-posOrg32))<error)
        {
            posOrg32 = posOrg31; //changed
        }
    } else
    {
		sstr << "Argument for j3 is out of range. No solution exists:Point is unreachable" << std::endl ;
        cout << sstr.str() << endl;
        return -1; // changed
    } 

    /**
     * solve theta2 and theta4
     */
    
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

  sstr << "det:" << s2 << endl; // det 2
    flag2 = judgeNum(c2, lim, error);
  sstr << "det:" << c2 << endl; // det 3
    
    if (!(flag1 || flag2)) 
    {
		sstr << "Argument for j2 is out of range. No solution exists:Point is unreachable" << std::endl ;
        mark[0] = false;
    } else if (flag1 && flag2)
    {
        if (fabs(c2==0)) posOrg21 = atan2(s2,fabs(c2));
        else posOrg21 = atan2(s2,c2);
        if (fabs(posIn32) == 0)posOrg41 = standard(atan2(posIn31,fabs(posIn32)) - posOrg21 - posOrg31); 
        else posOrg41 = standard(atan2(posIn31,posIn32) - posOrg21 - posOrg31); 
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
  sstr << "det:" << s2 << endl; // det 4
    flag2 = judgeNum(c2, lim, error);
  sstr << "det:" << c2 << endl; // det 5
    
    if (!(flag1 || flag2)) 
    {
		sstr << "Argument for j2 is out of range. No solution exists:Point is unreachable" << std::endl ;
        mark[1] = false;
    }else if (flag1 && flag2) //changed
    {
        if (fabs(c2==0)) posOrg22 = atan2(s2,fabs(c2));
        else posOrg22 = atan2(s2,c2);
        if (fabs(posIn32) == 0)posOrg42 = standard(atan2(posIn31,fabs(posIn32)) - posOrg22 - posOrg32); 
        else posOrg42 = standard(atan2(posIn31,posIn32) - posOrg22 - posOrg32); 
    }
    if (!(mark[0] || mark[1])) 
    {
        cout << sstr.str() << endl;
        return -1;
    }
    
    std::vector<std::vector<double> > pos(2,std::vector<double>(4,0));
sstr << "posOrg31: " << posOrg31 << endl;
sstr << "posOrg41: " << posOrg41 << endl;

    pos[0][0] = posOrg11;
    pos[0][1] = posOrg21;
    pos[0][2] = posOrg31;
    pos[0][3] = posOrg41;
    pos[1][0] = posOrg12;
    pos[1][1] = posOrg22;
    pos[1][2] = posOrg32;
    pos[1][3] = posOrg42;
    sstr << "posOrg11: " << posOrg11 << endl;
	sstr << "posOrg12: " << posOrg12 << endl; 
    unsigned int temp = 0;
    for (unsigned int i=0;i<2;i++)
    {
        if (mark[i] && judgePos(pos[i], min_angles, max_angles)) temp += i+1; 
    }
	sstr << "judgePos: " << judgePos(pos[0], min_angles, max_angles) << "---" << judgePos(pos[1], min_angles, max_angles) << endl; 
	sstr << "mark: " << mark[0] << " " << mark[1] << endl;
   sstr << "temp" << temp << endl; 
    if (temp == 0) 
    {
		sstr << "Can't find a reasonable solution. No solution exists:Point is unreachable" << std::endl ;
        cout << sstr.str() << endl;
        return -1;
    } else if (temp> 0) 
    {
        if (temp == 3)
        {
            posOut[0][0] = posOrg11;
            posOut[0][1] = posOrg21;
            posOut[0][2] = posOrg31;
            posOut[0][3] = posOrg41;
            posOut[1][0] = posOrg12;
            posOut[1][1] = posOrg22;
            posOut[1][2] = posOrg32;
            posOut[1][3] = posOrg42;

        }
        else if (temp == 1) 
        {
            posOut[0][0] = posOrg11;
            posOut[0][1] = posOrg21;
            posOut[0][2] = posOrg31;
            posOut[0][3] = posOrg41;
            posOut[1][0] = posOrg11;
            posOut[1][1] = posOrg21;
            posOut[1][2] = posOrg31;
            posOut[1][3] = posOrg41;

        } else if (temp == 2)
        {
            posOut[0][0] = posOrg12;
            posOut[0][1] = posOrg22;
            posOut[0][2] = posOrg32;
            posOut[0][3] = posOrg42;
            posOut[1][0] = posOrg12;
            posOut[1][1] = posOrg22;
            posOut[1][2] = posOrg32;
            posOut[1][3] = posOrg42;
        }
    }

	sstr << "j1: " << posOut[0][0] << std::endl ;
	sstr << "j2: " << posOut[0][1] << std::endl ;
	sstr << "j2_2: " << posOut[1][1] << std::endl ;
	sstr << "j3: " << posOut[0][2] << std::endl ;
	sstr << "j3_2: " << posOut[1][2] << std::endl ;
	sstr << "j4_1: " << posOut[0][3] << std::endl ;
	sstr << "j4_2: " << posOut[1][3] << std::endl ;

    KDL::JntArray solution(4);
	solution(0) = posOut[0][0];
	solution(1) = posOut[0][1];
	solution(2) = posOut[0][2];
	solution(3) = posOut[0][3];

    std::cout << sstr.str() <<endl;

    return 0;
}

bool judgeNum(double &target, std::vector<double> lim,double error)
{
    bool flag=false;
    double temp;
    if (lim[0]>lim[1])
    {
        temp = lim[1];
        lim[1] = lim[0];
        lim[0] = temp;
    }  
sstr << "target:" << target << endl;

sstr << "test" << (lim[1]) << endl;
sstr << "p" << (target-lim[1]<fabs(error)) << endl;
    if (target<(lim[0]-fabs(error)))
    {
        flag = false;
    } else if(target>(lim[1]+fabs(error))) 
    {
        flag = false;
    } else
    {
        flag = true;  //changed
    }

    if (target<lim[0]) 
    {
        target = lim[0]; 
    } else if (target>lim[1]) 
    {
        target = lim[1];
    }
sstr << "flag:" << flag << endl;

    return flag;
}

double standard(double target)
{
    while (target>=PI) target -= 2*PI;
    while (target<-PI) target += 2*PI;

    return target;
}

bool isSolutionValid(const std::vector<double> &solution)
{
    bool valid = true;

    if (solution.size() != 4) return false;

    for (unsigned int i = 0; i < solution.size(); i++) 
    {
        if (checkLim(solution[i],min_angles[i], max_angles[i])) return true;
    }
    
    return valid;
}

bool checkLim(double tar, double min_ang, double max_ang)
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

bool judgePos(std::vector<double> pos, std::vector<double> min_angles, std::vector<double> max_angles)
{
    for (unsigned int i=0;i<pos.size();i++)
    {
        if (checkLim(pos[i], min_angles[i], max_angles[i]) == false) 
		{
			sstr << "pos_i: " << pos[i] << endl;
			return false;
		}
    }
    return true;
}
