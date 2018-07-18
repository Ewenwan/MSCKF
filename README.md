# MSCKF  多状态约束卡尔曼滤波器 matlab
This code is written by someone else, I just add chinese commit in it.

# 参考文献 
    参考文献1：
    The Battle for Filter Supremacy: A Comparative Study of the Multi-State Constraint Kalman Filter and the Sliding Window Filter  多状态约束KF 滑动窗口滤波器
[多状态约束KF MSCKF & 滑动窗口滤波器SWF](https://leeclem.net/assets/docs/crv2015_battle_paper.pdf)
    
    
    参考文献2：Visual Inertial Odometry for Mobile Robotics


# 程序说明
********************程序中主要变量说明************************

## 参数说明：
    numLandmarks                     每帧图像特征点个数(地标数量)
    传感器物理器件参数：
    camera.c_u                       光心横坐标[u pixels]
    camera.c_v                       光心列坐标 [v pixels]
    camera.f_u                       焦距 [u pixels]
    camera.f_v                       焦距 [v pixels]
    传感器安装相关参数：
    camera.b                         双目基线距 [m]
    camera.q_CI                      4x1 IMU坐标系 到 Camera坐标系 位姿变换 T = [R,t]  中旋转矩阵 R，的四元素表示 q
    camera.p_C_I                     3x1 IMU坐标系 到 Camera坐标系 位姿变换 T = [R,t]  中平移变量p = t
    算法相关参数
    msckfParams.minTrackLength        特征点最少 相机 观测数目
    msckfParams.maxTrackLength        特征点最大 相机 观测数目
    msckfParams.maxGNCostNorm         高斯牛顿 优化求解 三角化特征点 的迭代误差上限
    msckfParams.minRCOND              矩阵条件数
    msckfParams.doNullSpaceTrick      是否做零空间映射
    msckfParams.doQRdecomp            是否做QR分解

## IMU状态：
    imuStates{k}.q_IG                 4x1 Global全局坐标系 到 IMU坐标系 的位姿变换 T = [R,t]  中旋转矩阵 R，的四元素表示 q
    imuStates{k}.p_I_G                3x1 IMU 在Global坐标系下位置( Global坐标系 到 IMU坐标系 的平移变量p)
    imuStates{k}.b_g                  3x1 陀螺仪(三轴旋转角速度)零偏
    imuStates{k}.b_v                  3x1 速度(三轴速度)零偏
    imuStates{k}.covar                12x12 IMU 状态协方差

## 相机状态：
    camStates{k}.q_CG                 4x1 Global全局坐标系 到 camera 坐标系 的位姿变换 T = [R,t] 中旋转矩阵 R，的四元素表示 q
    camStates{k}.p_C_G                3x1 Global坐标系下的 camera 位置( Global坐标系 到 camera 坐标系 的平移变量p )
    camStates{k}.trackedFeatureIds    1xM 当前相机可观测到的 特征的ID序列
    camStates{k}.state_k              当前相机ID(系统状态id)

## MSCKF状态：
    msckfState.imuState     IMU 状态
    msckfState.imuCovar     IMU-IMU 协方差矩阵块
    msckfState.camCovar     camera-camera 协方差矩阵块
    msckfState.imuCamCovar  IMU-camera 协方差
    msckfState.camStates    相机状态

## 特征跟踪列表：
    featureTracks        正在追踪的特征点（特征点坐标，能观测到这些特征点的相机，特征点ID）
    trackedFeatureIds    正在追踪特征点的ID号  
    
# 程序框架
********************程序步骤***********************************  

    步骤1：加载数据
    步骤2：初始化MSCKF中imu测量协方差与预测协方差
    步骤3：导入测量数据与参考数据
    步骤4：初始化MSCKF
          a.将第一个参考值的四元数与位置初始化msckf中IMU的状态
          b.目前MSCKF状态中只有IMU相关的状态，没有camera相关的状态
          c.msckf中不将特征点作为状态，但也会记录跟踪的特征点。初始化时认为第一帧所有特征点都被跟踪上。
    步骤5：状态与协方差预测更新。从步骤5到步骤10循环进行。
          a.状态预测更新。
               a1.IMU状态更新（角速度更新四元数，速度积分更新位置，陀螺仪零偏和速度零偏不变）
               a2.camera状态更新（保持和上一次相同）
          b.协方差预测更新
               b1.IMU-IMU状态的协方差更新，并对角化。（imuCover := P * imuCover * P' + G * Q_imu * G'）
               b2.IMU-camera状态的协方差更新。（imuCamCovar := P * imuCamCovar）
               b3.camera-camera状态的协方差更新。（camCovar:=camCovar）
    步骤6：状态增广，在msckf状态中增广相机状态
          a.由IMU以及IMU与camera的固连关系得到相机的位置和姿态
          b.增广雅克比。增广状态以后，需要得到增广状态（相机位置、相机四元数）对msckf状态（增广前以后的状态）的雅克比
          c.增广预测协方差矩阵，更新增广后的协方差矩阵
    步骤7：遍历当前帧所有特征点，更新featureTracks
          说明：msckf中用featureTracks记录了目前被跟踪到的特征点。
               featureTracks中包含每个特征点ID和观测值（即在所有能观测到该特征点的相机坐标系下的齐次坐标）
          a.遍历当前帧所有特征点，判断是否属于featureTracks
          b.如果该特征点在视野范围内：将特征点在相机坐标系下的齐次坐标添加到featureTracks中对应特征的观测中。
          c.如果该特征点超出视野范围或者能够观测到该特征的相机数目大于上限值
                c1.从所有相机状态中剔除该特征点，并将涉及到的相机状态添加到状态待优化列表中
                c2.如果待优化的相机状态超过最小跟踪长度（10），则将其添加到列表中用于优化
                c3.若已使用完给特征，则从featureTracks中剔除该特征点
          d.如果该帧检测的特征点视野范围内（但是不属于featureTracks），则将其添加至featureTracks中
    步骤8：MSCKF测量更新。遍历所有用于优化的特征点,构造观测模型（特征点重投影误差），更新MSCKF状态
          a.通过特征点所有观测估计出该特征点的3D空间坐标位置
          b.通过特征3D坐标与相机匹配特征点之间的重投影残差构造观测模型，包括重投影误差对MSCKF状态量的雅克比矩阵的求解 
          c.计算卡尔曼增益，更新误差状态
          d.根据误差状态更新MSCKF状态，x_true := x_nominal + detx
          e.更新MSCKF测量协方差矩阵
    步骤9：历史状态更新。从MSCKF状态中更新IMU的历史状态，通过相机的状态更新对应时刻imu的位姿状态
           说明：重投影误差只和相机状态有关，msckf测量更新只能更新相机有关的状态，因此在步骤8测量更新后需要通过相机状态更新IMU状态
    步骤10：剔除MSCKF中需要被删除的状态和对应的协方差矩阵块
           如果相机不能观测到featureTracks中的任一特征（对重投影误差无贡献），则MSCKF状态中剔除该相机状态
