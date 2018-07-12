#encoding=utf-8

from math import pow, sqrt, fabs, atan2, sin, cos, asin, pi

'''
##################    点坐标生成位姿矩阵    #################

输入：机构参数，末端点坐标，关节2、3 、4之和，末端点在xoy投影与原点最小距离限制，误差值

输出：标志位，异常点，末端位姿矩阵
'''
def calFixPosMatr(L, pos, th2a3a4, disLim, err):
    
    out = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]] # 初始化位姿矩阵
    
    d = L[4];
    px = pos[0]
    py = pos[1]

    # flag = 1 # 初始化标志位

    if (disLim < fabs(L[4])):
        disLim = disLim + fabs(L[4]) # 必须保证disLim > fabs(L[4])

    temp = sqrt(pow(px, 2) + pow(py, 2))

    if (fabs(temp) < disLim): # 末端在xoy上投影与坐标原点距离过近
        # flag = -1
        errPos = pos  #返回异常位置
        return None # 返回：无法确定位姿

    else:    # 在工作区域内，一定满足d <= temp
        K = d/temp
        if (fabs(K) > (1 - err)):
            errPos = pos  # 返回异常位置
            # flag = 0      # 不在工作区间内
            return None   # 返回：不在工作区间内
        
        t11 = atan2(py, px) - asin(-K)
        t12 = atan2(-py, -px) - asin(K)

        #########  利用末端轴向偏移量为0的点来判断关节1角度  #########
        px1 = px - sin(t11) * L[4] + L[5] * cos(t11) * sin(th2a3a4) # 偏移为0处的坐标x1
        py1 = py + cos(t11) * L[4] + L[5] * sin(t11) * sin(th2a3a4) # 偏移为0处的坐标y1
        px2 = px - sin(t12) * L[4] + L[5] * cos(t12) * sin(th2a3a4) # 偏移为0处的坐标x2
        py2 = py + cos(t12) * L[4] + L[5] * sin(t12) * sin(th2a3a4) # 偏移为0处的坐标y2

    ######  选取关节1角度，以约束末端点坐标姿态 ######
    if (fabs(px1) < err and fabs(py1) < err):  # 若最后一个关节在原点，则取此时关节1角度
        t1 = t11
    elif ((fabs(px2) < err) and (fabs(py2) < err)):  # 若最后一个关节在原点，则取此时关节1角度
        t1 = t12
    else: 
        r1 = px1 * px + py1 * py  # 计算矢量乘法，用于判断夹角是否为锐角
        r2 = px2 * px + py2 * py  # 计算矢量乘法，用于判断夹角是否为锐角

        if (r1 < 0):   # 如果夹角为锐角，则取此时关节1角度
            t1 = t11
        elif (r2 < 0):   # 如果夹角为锐角，则取此时关节1角度
            t1 = t12
        else:   # 如果夹角都不为锐角，则取与最后一个关节位置一致的关节1角度
            theta = atan2(py1, px1)
            if (compareAngle(theta, t11, 1e-3, 0) == 1):  # 角度相等
                t1 = t11
            else:
                t1 = t12
    
    #####  计算位姿矩阵  ####
    out[0][0] = cos(t1) * cos(th2a3a4)
    out[0][1] = -cos(t1) * sin(th2a3a4)
    out[0][2] = sin(t1) 
    out[0][3] = pos[0]

    out[1][0] = sin(t1) * cos(th2a3a4)
    out[1][1] = -sin(t1) * sin(th2a3a4)
    out[1][2] = -cos(t1)
    out[1][3] = pos[1]

    out[2][0] = sin(th2a3a4)
    out[2][1] = cos(th2a3a4)
    out[2][2] = 0
    out[2][3] = pos[2]

    return out



'''
##################    比较两角度值是否相等或差值    #################

求解参数：两角度值，误差，模式（0/1：判断相等/计算差值）

输出参数：标志位（0/1：不等/相等）或角度之差（angle1-angle2）

'''
def compareAngle(angle1, angle2, err, mode):
    angle1 = standard(angle1) # 标准化角度值
    angle2 = standard(angle2) # 标准化角度值
    det = fabs(angle1 - angle2)
    if (det > pi): # 角度偏差最大为pi
        det = fabs(det - 2*pi)
    
    if (mode == 1): # 计算角度差
        return det
    else:           # 判断角度是否相等
        if (det < err):
            return 1
        else: 
            return 0


'''
##################   限制角度范围为[-pi,pi]    #################

输入：角度值矩阵

输出：[-pi,pi)范围内角度值矩阵

'''
def standard(target):
    while (target>=pi):
        target = target-2*pi
    while (target<-pi):
        target = target+2*pi

    return target


if __name__ == '__main__':
    L = [0.257, 0.255, 0.250, 0.150, 0, 0]
    pos = [0.367, -0.421, -0.567]
    ans = calFixPosMatr(L, pos, -90, 0.01, 1e-7)
    if ans == None:
        print "None"
    else:
        print ans

    print atan2(1, 0)
    print atan2(0, 1)
    print atan2(0, -1)
    print atan2(-0, 1)
    print atan2(-1, 0)
    print atan2(1, -0)
    print atan2(0, 0)
