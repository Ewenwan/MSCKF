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
    步骤2：初始化MSCKF中  imu测量协方差 与 预测协方差
    步骤3：导入 测量数据 与 参考数据
    步骤4：初始化MSCKF
          a. 将第一个 参考值的 四元数(位姿) 与 位置平移) 初始化为msckf中IMU的状态
          b. 目前MSCKF状态中只有 IMU相关的状态，没有camera相关的状态
          c. msckf中不将特征点作为状态，但也会记录跟踪的特征点。
             初始化时认为 第一帧所有特征点 都被跟踪上。
             
    从步骤5到步骤10循环进行：          
    步骤5：系统状态 与 协方差 预测更新。
          a. 状态预测更新
               a1. IMU状态更新（角速度更新四元数(位姿)，速度积分更新位置，陀螺仪零偏 和 速度零偏不变）
               a2. camera状态更新（保持和上一次相同）
               
          b. 协方差预测更新
               b1. IMU-IMU 状态的协方差更新，并对角化。（imuCover := P * imuCover * P' + G * Q_imu * G'）
               b2. IMU-camera 状态的协方差更新。（imuCamCovar := P * imuCamCovar）
               b3. camera-camera 状态的协方差更新。（camCovar := camCovar）
               
    步骤6：状态增广，在msckf状态 中 增广 相机状态
          a. 由IMU以及 IMU与camera的 固连关系 得到相机的位置和姿态。
          b. 增广雅克比。
             增广状态以后，需要得到 增广状态（相机位置、相机四元数）对 msckf状态（增广前以后的状态）的雅克比。
          c. 增广预测协方差矩阵，更新增广后的协方差矩阵
          
    步骤7：遍历当前帧所有特征点，更新featureTracks
          说明：msckf 中用 featureTracks 记录了 目前 被跟踪到的特征点。
               featureTracks 中包含每个特征点ID 和 观测值（即在所有能观测到该 特征点在相机坐标系下的齐次坐标）
          a. 遍历当前帧上所有特征点，判断是否属于featureTracks
          b. 如果该特征点 在视野范围内：
             将特征点在相机坐标系下的 齐次坐标 添加到 featureTracks 中对应特征的观测中。
          c. 如果该特征点超出视野范围 或者 能够观测到该特征的相机数目大于上限值
                c1. 从所有 相机状态 中 剔除该特征点，并将涉及到的相机状态添加到 状态待优化列表 中。
                c2. 如果待优化的相机状态超过最小跟踪长度（10），则将其添加到列表中用于优化。
                c3. 若已使用完给特征，则从 featureTracks 中剔除该特征点
          d. 如果该帧 检测的特征点在视野范围内，但是不属于featureTracks，则将其添加至featureTracks中。
          
    步骤8：MSCKF 测量更新。
         遍历所有用于优化的特征点, 构造观测模型（特征点 重投影误差），更新 MSCKF状态
          a. 通过特征点 所有 观测 估计出该特征点的 3D空间坐标位置。
          b. 通过 特征3D坐标 与 相机匹配特征点 之间的 重投影残差构造观测模型，包括 重投影误差 对 MSCKF状态量 的 雅克比矩阵的求解。
          c. 计算卡尔曼增益K，更新误差状态.=========================
          d. 根据 误差状态 更新MSCKF状态，x_true := x_nominal + detx
          e. 更新MSCKF 测量协方差 矩阵.
          
    步骤9：历史状态更新。
           从MSCKF状态中更新 IMU 的历史状态，通过相机的 状态 更新对应时刻imu的位姿状态.
           说明：重投影误差 只和 相机状态有关，
                msckf测量更新 只能 更新 相机有关的状态，
                因此在步骤8 测量更新后需要通过相机状态更新IMU状态.
           
    步骤10：剔除MSCKF中 需要被删除的 状态 和 对应的 协方差矩阵块.
           如果相机 不能观测到 featureTracks 中的任一特征（对重投影误差无贡献），则MSCKF状态中剔除该相机状态.
