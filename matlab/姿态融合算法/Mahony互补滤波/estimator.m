% 功能：姿态仿真测试
% 作者：across
% 说明：Eulerdata xsens 50hz取得 用其角度作为准确性参考

clear
close all
clc

load data
magx=[(MAG_0(:,2)-MAG_0(1,2))*(1e-6),MAG_0(:,4)];
magy=[(MAG_0(:,2)-MAG_0(1,2))*(1e-6),MAG_0(:,5)];
magz=[(MAG_0(:,2)-MAG_0(1,2))*(1e-6),MAG_0(:,6)];
xacc=[(IMU_0(:,2)-IMU_0(1,2))*(1e-6),IMU_0(:,7)];
yacc=[(IMU_0(:,2)-IMU_0(1,2))*(1e-6),IMU_0(:,8)];
zacc=[(IMU_0(:,2)-IMU_0(1,2))*(1e-6),IMU_0(:,9)];
rollrate=[(IMU_0(:,2)-IMU_0(1,2))*(1e-6),IMU_0(:,4)];
pitchrate=[(IMU_0(:,2)-IMU_0(1,2))*(1e-6),IMU_0(:,5)];
yawrate=[(IMU_0(:,2)-IMU_0(1,2))*(1e-6),IMU_0(:,6)];
roll=[(ATT(:,2)-ATT(1,2))*(1e-6),ATT(:,4)];
pitch=[(ATT(:,2)-ATT(1,2))*(1e-6),ATT(:,6)];
yaw=[(ATT(:,2)-ATT(1,2))*(1e-6),ATT(:,8)];

% magx=[(MAG_0(:,2)-MAG_0(1,2)),MAG_0(:,4)];
% magy=[(MAG_0(:,2)-MAG_0(1,2)),MAG_0(:,5)];
% magz=[(MAG_0(:,2)-MAG_0(1,2)),MAG_0(:,6)];
% xacc=[(IMU_0(:,2)-IMU_0(1,2)),IMU_0(:,7)];
% yacc=[(IMU_0(:,2)-IMU_0(1,2)),IMU_0(:,8)];
% zacc=[(IMU_0(:,2)-IMU_0(1,2)),IMU_0(:,9)];
% rollrate=[(IMU_0(:,2)-IMU_0(1,2)),IMU_0(:,4)];
% pitchrate=[(IMU_0(:,2)-IMU_0(1,2)),IMU_0(:,5)];
% yawrate=[(IMU_0(:,2)-IMU_0(1,2)),IMU_0(:,6)];
% roll=[(ATT(:,2)-ATT(1,2)),ATT(:,4)];
% pitch=[(ATT(:,2)-ATT(1,2)),ATT(:,6)];
% yaw=[(ATT(:,2)-ATT(1,2)),ATT(:,8)];

roll(:,2) = roll(:,2) * pi/180;
pitch(:,2) = pitch(:,2) * pi/180;
yaw(:,2) = yaw(:,2) * pi/180;


% dt1 = 0.04; %高速率任务，IMU(有些问题）
% dt2 = 20e-3; %低速率任务，ATT、MAG

% [m,n]=size(xacc);
% t1 = (0:(m-1))*dt1;
% t1_fin = t1(end);
% t2_fin = t2(end);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% part 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%原始数据观测
figure('color',[1 1 1],'NumberTitle','on','Name','参考角度')
subplot(311)
plot(roll(:,1),roll(:,2),'r','linewidth',1.3);xlabel('Time(s)');
ylabel('roll');
grid;
subplot(312)
plot(pitch(:,1),pitch(:,2),'r','linewidth',1.3);
ylabel('pitch');
xlabel('Time(s)');
grid;
subplot(313)
plot(yaw(:,1),yaw(:,2),'r','linewidth',1.3);
ylabel('yaw');
xlabel('Time(s)');
grid;

figure('color',[1 1 1],'NumberTitle','on','Name','原始数据')
subplot(331)
plot(rollrate(:,1),rollrate(:,2),'r','linewidth',1.3);
ylabel('RollRate');
xlabel('Time(s)');
grid;
subplot(332)
plot(pitchrate(:,1),pitchrate(:,2),'r','linewidth',1.3);
ylabel('PitchRate');
xlabel('Time(s)');
grid;
subplot(333)
plot(yawrate(:,1),yawrate(:,2),'r','linewidth',1.3);
ylabel('YawRate');
xlabel('Time(s)');
grid;
subplot(334)
plot(xacc(:,1),xacc(:,2),'r','linewidth',1.3);
ylabel('Xacc');
xlabel('Time(s)');
grid;
subplot(335)
plot(yacc(:,1),yacc(:,2),'r','linewidth',1.3);
ylabel('Yacc');
xlabel('Time(s)');
grid;
subplot(336)
plot(zacc(:,1),zacc(:,2),'r','linewidth',1.3);
ylabel('Zacc');
xlabel('Time(s)');
grid;
subplot(337)
plot(magx(:,1),magx(:,2),'r','linewidth',1.3);
ylabel('MagX');
xlabel('Time(s)');
grid;
subplot(338)
plot(magy(:,1),magy(:,2),'r','linewidth',1.3);
ylabel('MagY');
xlabel('Time(s)');
grid;
subplot(339)
plot(magz(:,1),magz(:,2),'r','linewidth',1.3);
ylabel('MagZ');
xlabel('Time(s)');
grid;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% part 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%坐标系对齐
%转换到前右下  rollrate对应x轴  pitchrate对应y轴
gx = rollrate;
gy = pitchrate;
gz = yawrate;
ax = xacc;
ay = yacc;
az = zacc;
mx = magx;
my = magy;
mz = magz;

% gx = rollrate;
% gy = pitchrate;
% gz = yawrate;
% ax = xacc;
% ay = yacc;
% az = -zacc;
% mx = magx;
% my = magy;
% mz = -magz;


%%
%数据滤波
[b,a]=butter(2,10/50);
XAcc=filter(b,a,ax(:,2));
YAcc=filter(b,a,ay(:,2));
ZAcc=filter(b,a,az(:,2));

figure('color',[1 1 1],'NumberTitle','on','Name','滤波效果')
subplot(311)
plot(ax(:,1),ax(:,2),'r',ax(:,1),XAcc,'b:','linewidth',1.3);
legend('ax','XAcc');
title('Xacc');xlabel('Time(s)');grid;
subplot(312)
plot(ay(:,1),ay(:,2),'r',ay(:,1),YAcc,'b:','linewidth',1.3);
legend('ay','YAcc');
title('Yacc');xlabel('Time(s)');grid;
subplot(313)
plot(az(:,1),az(:,2),'r',az(:,1),ZAcc,'b:','linewidth',1.3);
legend('zx','ZAcc');
title('Zacc');xlabel('Time(s)');grid;

% ax = XAcc;
% ay = YAcc;
% az = ZAcc;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% part 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%对比角速度积分与加速度得到的姿态角之间的区别
%角速度积分得到的角度会随时间漂移
%加速度得到的角度会噪声很大


[m,n]=size(gx);
roll_gyro = zeros(m,1);
roll_acc = zeros(m,1);
for i = 1:m
    roll_acc(i) = atan2(-ay(i,2),-az(i,2));
    % roll_acc(i) = atan2(ay(i,2),-az(i,2));
    if i==1
        roll_gyro(1) = roll_acc(1);%将acc求得的角度作为积分初始值
    else
        dt1=gx(i,1)-gx(i-1,1);
        roll_gyro(i) = roll_gyro(i-1) + gx(i,2) * dt1;
    end
end
figure('color',[1 1 1],'NumberTitle','on','Name','角速度、加速度推roll对比')
plot(gx(:,1),roll_gyro,'r',ax(:,1),roll_acc,'b:',roll(:,1),roll(:,2),'g-.','linewidth',1.3);
legend('roll_gyro','roll_acc','roll');
title('roll 角速度、加速度推角度、参考角度对比');xlabel('Time(s)');grid;

pitch_gyro = zeros(m,1);
pitch_acc = zeros(m,1);
for i = 1:m
    % pitch_acc(i) = atan2(ax(i,2),-az(i,2));
    pitch_acc(i) = atan2(ax(i,2),sqrt(ay(i,2)^2+az(i,2)^2));
    % pitch_acc(i) = atan2(-ax(i,2),sqrt(ay(i,2)^2+az(i,2)^2));
    if i==1
        pitch_gyro(1) = pitch_acc(1);
    else
        dt1=gy(i,1)-gy(i-1,1);
        pitch_gyro(i) = pitch_gyro(i-1) + gy(i,2) * dt1;
    end
end

figure('color',[1 1 1],'NumberTitle','on','Name','角速度、加速度推pitch对比')
plot(gy(:,1),pitch_gyro,'r',ay(:,1),pitch_acc,'b:',pitch(:,1),pitch(:,2),'g-.','linewidth',1.3);
legend('pitch_gyro','pitch_acc','pitch');
title('pitch 角速度、加速度推角度、参考角度对比');xlabel('Time(s)');grid;


% ------------------ 处理磁力计采样低于IMU采样的情况 ------------------
% 假设：gx,gy,gz,ax,ay,az 已经和之前一样：第1列时间，第2列测量值
% 假设：mx,my,mz 第1列时间，第2列测量值
m = size(gx,1);
yaw_acc  = zeros(m,1);
yaw_gyro = zeros(m,1);

% 磁力计索引与样本数
mag_idx = 1;
num_mag = size(mx,1);

% 如果没有磁力计样本则直接返回（避免越界）
if num_mag == 0
    error('没有磁力计数据: mx/my/mz 为空');
end

% 用第一帧磁力计初始化 yaw_acc（使用第一个可用的mag样本）
% 注意：roll_acc/pitch_acc 已与 gx 时间轴对齐，使用 i=1 的 roll/pitch
r = roll_acc(1);
p = pitch_acc(1);
Mx = mx(1,2); My = my(1,2); Mz = mz(1,2);
% 倾斜补偿（常见形式）：
% mx2 =  Mx * cos(p) + Mz * sin(p);
% my2 =  Mx * sin(r) * sin(p) + My * cos(r) - Mz * sin(r) * cos(p);
mx2 =  Mx * cos(p) * sin(p) + My * sin(r) + Mz * cos(r) * sin(p);
my2 =  My * cos(r) - Mz * sin(r);
% 常用航向定义：北为0，顺时针为正 -> atan2(-my2, mx2)
yaw_acc(1)  = atan2(my2, mx2);
% 初始化陀螺积分 yaw（使用同一初值）
yaw_gyro(1) = yaw_acc(1);

% 主循环：在每个高频 IMU 时间点更新 gyro 积分，
% 仅当有新的磁力计样本到达（时间推进）时重新计算 yaw_acc
for i = 2:m
    % dt 由高频陀螺时间戳决定
    dt = gx(i,1) - gx(i-1,1);
    if dt <= 0
        dt = 1e-3;
    end

    % 更新磁力计索引到最新不大于当前时间的样本
    % （与你 Mahony 实现一致的同步策略）
    while (mag_idx < num_mag) && (mx(mag_idx+1,1) <= gx(i,1))
        mag_idx = mag_idx + 1;
    end

    % 如果磁力计时间刚好推进到一个新的样本（即比上次 mag_idx 变大），
    % 则用当前 roll_acc(i) 和 pitch_acc(i) 计算新的 yaw_acc。
    % 注意：如果 roll_acc/pitch_acc 的时间轴不是与 gx 同步，需要先插值到 gx 时间轴。
    if mag_idx > 1 && (mx(mag_idx,1) == gx(i,1) || mx(mag_idx,1) > mx(mag_idx-1,1) && mx(mag_idx,1) <= gx(i,1) ...
            && mx(mag_idx,1) > gx(max(1,i-1),1)) 
        % 这里的判定意在确认我们确实“跨过”了一个新的 mag 样本
        % 直接使用当前 mag_idx 的磁力计样本来计算 yaw_acc
        r = roll_acc(i);
        p = pitch_acc(i);
        Mx = mx(mag_idx,2);
        My = my(mag_idx,2);
        Mz = mz(mag_idx,2);

        % 倾斜补偿（和上面初始化一致）
        mx2 =  Mx * cos(p) + Mz * sin(p);
        my2 =  Mx * sin(r) * sin(p) + My * cos(r) - Mz * sin(r) * cos(p);

        yaw_acc(i) = atan2(-my2, mx2);
    else
        % 否则复用上一次的 yaw_acc（保持不变）
        yaw_acc(i) = yaw_acc(i-1);
    end

    % ---------------- 陀螺积分得到 yaw_gyro ----------------
    gz_i = gz(i,2); % 假设 gz 单位为 rad/s；若为 deg/s 请先转换：gz_i = gz(i,2)*pi/180;
    yaw_gyro(i) = yaw_gyro(i-1) + gz_i * dt;

    % 可选：当新的磁力计样本到达时，可以重置/校准陀螺积分（简单置位），
    % 或者用于后续融合（例如互补）：
    % if (mag_idx_changed)
    %     yaw_gyro(i) = yaw_acc(i); % 简单纠正漂移（非滤波，仅示例）
    % end

    % 规范化到 (-pi, pi]
    % yaw_acc(i)  = atan2( sin(yaw_acc(i)),  cos(yaw_acc(i)) );
    % yaw_gyro(i) = atan2( sin(yaw_gyro(i)), cos(yaw_gyro(i)) );
end

% % 画图对比
% figure('color',[1 1 1],'NumberTitle','on','Name','yaw 低频mag处理对比')
% plot(gx(:,1), yaw_gyro, 'r', gx(:,1), yaw_acc, 'b:', 'linewidth', 1.3);
% legend('yaw\_gyro','yaw\_acc (use latest mag sample only)');
% title('yaw: 陀螺积分 vs 磁力计（低频mag仅在新样本到达时更新）');
% xlabel('Time(s)'); grid on;

% 首先将所有角度从弧度转换为度
% yaw_acc_deg  = yaw_acc * 180/pi;
% pitchmahony_deg = pitchmahony * 180/pi;
yaw_acc_deg   = yaw_acc * 180/pi;
yaw_gyro_deg   = yaw_gyro * 180/pi;

% [新增] 调整Yaw的角度范围以匹配参考数据
% 将所有负角度值加上360度
yaw_acc_deg(yaw_acc_deg < 0) = yaw_acc_deg(yaw_acc_deg < 0) + 360;
yaw_gyro_deg(yaw_gyro_deg < 0) = yaw_gyro_deg(yaw_gyro_deg < 0) + 360;

figure('color',[1 1 1],'NumberTitle','on','Name','角速度、加速度推yaw对比')
plot(gz(:,1),yaw_gyro_deg,'r',az(:,1),yaw_acc_deg,'b:',yaw(:,1),yaw(:,2)* 180/pi,'g-.','linewidth',1.3);
legend('pitch_gyro','pitch_acc','pitch');
title('yaw 角速度、加速度推角度、参考角度对比');xlabel('Time(s)');grid;



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% part 4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %互补滤波 单轴
% complementation_filtered = zeros(m,1);
% adaption_complementation_filtered = zeros(m,1);
% 
% coeff = 0.9;
% 
% for i=2:m
%     dt=gx(i,1)-gx(i-1,1);
%     % complementation_filtered(i) = (complementation_filtered(i-1) + gx(i)*dt1    ) * coeff + (1 - coeff) * roll_acc(i);
%     complementation_filtered(i) = (complementation_filtered(i-1) + gx(i,2)*dt) * coeff + (1 - coeff) * roll_acc(i);
%     if abs(gx(i,2)) > 0.3
%         adaption_complementation_filtered(i) = (adaption_complementation_filtered(i-1) + gx(i,2)*dt) * coeff + (1 - coeff) * roll_acc(i);
%     else
%         adaption_complementation_filtered(i) = (adaption_complementation_filtered(i-1) + gx(i,2)*dt) * 0.005 + (1 - 0.005) * roll_acc(i);
%     end
%     % if abs(gy(i)) > 0.3
%     %     adaption_complementation_filtered(i) = (adaption_complementation_filtered(i-1) + gy(i)*dt1) * coeff + (1 - coeff) * roll_acc(i);
%     % else
%     %     adaption_complementation_filtered(i) = (adaption_complementation_filtered(i-1) + gy(i)*dt1) * 0.005 + (1 - 0.005) * roll_acc(i);
%     % end
% end
% 
% figure('color',[1 1 1],'NumberTitle','on','Name','互补滤波roll对比')
% subplot(411)
% title('滚转');
% plot(gx(:,1),complementation_filtered,'r',gx(:,1),roll_gyro,'b:','linewidth',1.3);grid;
% legend('c_filter','roll_gyro')
% subplot(412)
% plot(gx(:,1),complementation_filtered,'r',gx(:,1),roll_acc,'b:','linewidth',1.3);grid;
% legend('c_filter','roll_acc')
% subplot(413)
% plot(gx(:,1),complementation_filtered,'r',roll(:,1),roll(:,2),'b:','linewidth',1.3);grid;
% legend('c_filter','roll')
% subplot(414)
% plot(roll(:,1),roll(:,2),'r',gx(:,1),adaption_complementation_filtered,'b:','linewidth',1.3);grid;
% legend('roll','c_filter')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% part 5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 单轴kalman 模型1 (已修改以支持非均匀采样)

% --- 1. 设置卡尔曼滤波器参数 ---

% 原始参数值 (可以根据你的数据进行调整)
Q_angle = 0.023; % 角度的过程噪声方差
Q_bias  = 0.01;  % 陀螺仪偏置的过程噪声方差
R_angle = 1;     % 加速度计测量值的噪声方差

% 假设以上Q值是为某个典型时间步长（例如0.02s）调整的。
% 我们将其转换为“噪声速率”（单位时间的方差），以适应非均匀采样。
% 如果你的IMU平均采样周期是40ms，可以改为0.04。
assumed_dt = 0.02; 
Q_angle_rate = Q_angle / assumed_dt;
Q_bias_rate  = Q_bias / assumed_dt;

% --- 2. 初始化状态变量 ---
[m, ~] = size(gy); % 获取数据点数量

pitchKF1 = zeros(m,1);      % 存储卡尔曼滤波后的俯仰角
bias = zeros(m,1);          % 存储估计的陀螺仪偏置
pitchMeasure1 = zeros(m,1); % 存储由加速度计计算的俯仰角

% 初始化状态协方差矩阵 P 和测量矩阵 C
P = [1 0; 0 1]; % P可以从一个非零值开始，表示初始不确定性
C = [1 0];      % 测量矩阵，因为我们只测量角度

% 使用第一个加速度计读数来初始化滤波器的初始角度
pitchMeasure1(1) = atan2(ax(1,2), sqrt(ay(1,2)^2 + az(1,2)^2));
pitchKF1(1) = pitchMeasure1(1);
bias(1) = 0; % 假设初始偏置为0

% --- 3. 执行卡尔曼滤波循环 ---
for i = 2:m

    % 动态计算当前步的时间间隔 dt_step
    dt_step = gy(i, 1) - gy(i-1, 1);

    % 安全检查：如果dt_step为0或负数，则跳过本次迭代
    if dt_step <= 0
        pitchKF1(i) = pitchKF1(i-1);
        bias(i) = bias(i-1);
        pitchMeasure1(i) = pitchMeasure1(i-1);
        continue;
    end

    % I. 预测步骤
    % -----------------
    % 1. 预测状态估计 x_hat_minus = A * x_hat_plus_prev + B * u
    %    u 是陀螺仪读数 gy(i,2)
    angle_pred = pitchKF1(i-1) + dt_step * (gy(i,2) - bias(i-1));
    bias_pred = bias(i-1);

    % 2. 根据 dt_step 更新状态转移矩阵 A 和过程噪声协方差 Q
    A = [1 -dt_step; 0 1];
    Q_step = [Q_angle_rate * dt_step, 0; 0, Q_bias_rate * dt_step];

    % 3. 预测协方差 P_minus = A * P_plus_prev * A' + Q
    P = A * P * A' + Q_step;

    % II. 更新步骤
    % -----------------
    % 1. 获取当前测量值
    pitchMeasure1(i) = atan2(ax(i,2), sqrt(ay(i,2)^2 + az(i,2)^2));

    % 2. 计算新息 (测量残差) y = z - C * x_hat_minus
    y = pitchMeasure1(i) - angle_pred;

    % 3. 计算新息协方差 S = C * P_minus * C' + R
    S = C * P * C' + R_angle;

    % 4. 计算最优卡尔曼增益 K = P_minus * C' * inv(S)
    K = P * C' / S;

    % 5. 更新状态估计 x_hat_plus = x_hat_minus + K * y
    pitchKF1(i) = angle_pred + K(1) * y;
    bias(i) = bias_pred + K(2) * y;

    % 6. 更新协方差 P_plus = (I - K * C) * P_minus
    P = (eye(2) - K * C) * P;
end

% --- 4. 绘图验证结果 ---
figure('color',[1 1 1],'NumberTitle','on','Name','单轴卡尔曼(Pitch)融合结果')
plot(gy(:,1), pitchKF1, 'b', pitch(:,1), pitch(:,2), 'r--', 'linewidth', 1.3);
title('单轴卡尔曼模型(非均匀采样)结果验证');
xlabel('Time(s)');
ylabel('Pitch Angle (rad)');
grid on;
legend('单轴卡尔曼估计值', '参考角度');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% part 6 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %两轴kalman  模型2
% pitchKF2 = zeros(m,1);
% rollKF2 = zeros(m,1);
% bp = zeros(m,1);
% bq = zeros(m,1);
% pitchMeasure2 = zeros(m,1);
% rollMeasure2 = zeros(m,1);
% P_out = zeros(m,4,4);
%
% Q_pitch = 0.001; %Calculated from data
% Q_roll = 0.001; %Calculated from data
% Q_bp = 0.1;
% Q_bq = 0.1;
% R_pitch = 5; %Accel variance, calculated from data
% R_roll = 5; %Accel variance, calculated from data
%
% Q = [Q_pitch 0     0    0
%      0     Q_roll 0    0
%      0     0     Q_bp 0
%      0     0     0    Q_bq];
%
% R = [R_pitch 0     0 0
%      0     R_roll 0 0
%      0     0     0 0
%      0     0     0 0];
%
% P = [0 0 0 0
%      0 0 0 0
%      0 0 10000 0
%      0 0 0 10000];
%
% H = [1 0 0 0
%      0 1 0 0
%      0 0 0 0
%      0 0 0 0];
%
% I = [1 0 0 0
%      0 1 0 0
%      0 0 1 0
%      0 0 0 1];
%
% x = [gy(1)
%      gx(1)
%      0
%      0];
% for i = 1:m
%
%      pitchMeasure2(i) = atan2(-ax(i),-az(i));
%      rollMeasure2(i) = atan2(ay(i),-az(i));
%
%      F = [1 0 -dt 0
%          0 1 0 -dt
%          0 0 1  0
%          0 0 0  1];
%
%     B = [dt 0  0 0
%          0  dt 0 0
%          0  0  0 0
%          0  0  0 0];
%
%     u = [gy(i)
%          gx(i)
%          0
%          0];
%
%     z = [pitchMeasure2(i)
%          rollMeasure2(i)
%          0
%          0];
%
%     %Predicted state estimate
%     x = F*x + B*u;
%
%     %Predicted estimate covariance
%     P = F*P*F' + dt*Q;
%
%     %Measurement residual
%     y = z - H*x;
%
%     %Residual covariance
%     S = H*P*H' + R;
%
%     %Optimal Kalman gain
%     K = P*H'*pinv(S);
%
%     %Updated state estimate
%     x = x + K*y;
%
%     %Updated estimate covariance
%     P = (I - K*H)*P;
%
%     pitchKF2(i) = x(1);
%     rollKF2(i) = x(2);
%     bp(i) = x(3);
%     bq(i) = x(4);
%     P_out(i,1:4,1:4) = P(1:4,1:4);
% end
%
%
% figure(8)
% subplot(211)
% plot(t1,rollKF2,t1,roll,'r');title('roll 角度对比');xlabel('Time(s)');grid;
% legend('两轴卡尔曼','参考角度')
% subplot(212)
% plot(t1,pitchKF2,t1,pitch,'r');title('pitch 角度对比');xlabel('Time(s)');grid;
% legend('两轴卡尔曼','参考角度')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% part 7 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% =========================================================================
% Mahony AHRS 姿态解算算法 (已适配ArduPilot非均匀、异步数据)
% =========================================================================

% --- 算法参数 ---
Kp = 2;  % 比例增益，处理加速度计/磁力计的修正
Ki = 0.2;  % 积分增益，处理陀螺仪的零偏

% --- 变量初始化 ---
% 沿用您之前定义的IMU数据长度 m
q = zeros(m, 4); % 使用一个Nx4的矩阵存储四元数 [q0, q1, q2, q3]
exInt = 0;       % 误差积分项初始化
eyInt = 0;
ezInt = 0;

rollmahony  = zeros(m, 1);
pitchmahony = zeros(m, 1);
yawmahony   = zeros(m, 1);

% --- 使用第一帧数据进行姿态初始化 ---
% 1. 从加速度计计算初始Roll和Pitch
roll_init = atan2(-ay(1,2), -az(1,2));
pitch_init = atan2(ax(1,2), sqrt(ay(1,2)^2 + az(1,2)^2));

% 2. 使用初始Roll/Pitch和磁力计数据计算初始Yaw
% 首先将磁力计读数从机体坐标系转换到水平坐标系
mx_h = mx(1,2)*cos(pitch_init) + my(1,2)*sin(pitch_init)*sin(roll_init) + mz(1,2)*sin(pitch_init)*cos(roll_init);
my_h = my(1,2)*cos(roll_init) - mz(1,2)*sin(roll_init);
yaw_init = atan2(-my_h, mx_h);

% 3. 将初始欧拉角转换为初始四元数
cy = cos(yaw_init * 0.5);
sy = sin(yaw_init * 0.5);
cp = cos(pitch_init * 0.5);
sp = sin(pitch_init * 0.5);
cr = cos(roll_init * 0.5);
sr = sin(roll_init * 0.5);

q(1,1) = cr * cp * cy + sr * sp * sy; % q0
q(1,2) = sr * cp * cy - cr * sp * sy; % q1
q(1,3) = cr * sp * cy + sr * cp * sy; % q2
q(1,4) = cr * cp * sy - sr * sp * cy; % q3

% 磁力计数据索引
mag_idx = 1;
num_mag_samples = size(mx, 1);

% --- 主循环 ---
for i = 2:m
    % 动态计算时间间隔 dt
    dt = gx(i,1) - gx(i-1,1);
    if dt <= 0 % 防止时间戳异常
        dt = 1e-3; 
    end

    % --- 同步磁力计数据 ---
    % 找到当前IMU时间点(gx(i,1))对应的最新磁力计数据
    while (mag_idx < num_mag_samples) && (mx(mag_idx+1, 1) <= gx(i,1))
        mag_idx = mag_idx + 1;
    end

    % 获取当前时刻的传感器读数
    gx_i = gx(i,2); gy_i = gy(i,2); gz_i = gz(i,2);
    ax_i = ax(i,2); ay_i = ay(i,2); az_i = az(i,2);
    mx_i = mx(mag_idx,2); my_i = my(mag_idx,2); mz_i = mz(mag_idx,2);

    % --- Mahony算法核心 ---

    % 获取上一时刻的四元数
    q0 = q(i-1,1); q1 = q(i-1,2); q2 = q(i-1,3); q3 = q(i-1,4);

    % 1. 加速度计和磁力计数据归一化
    norm_acc = sqrt(ax_i*ax_i + ay_i*ay_i + az_i*az_i);
    if norm_acc > 0
        ax_i = ax_i / norm_acc;
        ay_i = ay_i / norm_acc;
        az_i = az_i / norm_acc;
    end

    norm_mag = sqrt(mx_i*mx_i + my_i*my_i + mz_i*mz_i);
    if norm_mag > 0
        mx_i = mx_i / norm_mag;
        my_i = my_i / norm_mag;
        mz_i = mz_i / norm_mag;
    end

    % 2. 将磁力计数据从机体坐标系转换到世界坐标系（地理坐标系）
    hx = 2*mx_i*(0.5 - q2*q2 - q3*q3) + 2*my_i*(q1*q2 - q0*q3) + 2*mz_i*(q1*q3 + q0*q2);
    hy = 2*mx_i*(q1*q2 + q0*q3) + 2*my_i*(0.5 - q1*q1 - q3*q3) + 2*mz_i*(q2*q3 - q0*q1);
    % 在水平面上的投影，并以此计算世界坐标系下的磁场方向
    bx = sqrt(hx*hx + hy*hy);
    bz = 2*mx_i*(q1*q3 - q0*q2) + 2*my_i*(q2*q3 + q0*q1) + 2*mz_i*(0.5 - q1*q1 - q2*q2);

    % 3. 计算重力和磁场的参考方向向量（在机体坐标系下）
    vx = -2*(q1*q3 - q0*q2);
    vy = -2*(q0*q1 + q2*q3);
    vz = -(q0*q0 - q1*q1 - q2*q2 + q3*q3);

    wx = 2*bx*(0.5 - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2);
    wy = 2*bx*(q1*q2 - q0*q3) + 2*bz*(q0*q1 + q2*q3);
    wz = 2*bx*(q0*q2 + q1*q3) + 2*bz*(0.5 - q1*q1 - q2*q2); 

    % 4. 计算加速度计和磁力计的误差
    % 误差等于测量值与参考方向的叉积
    ex = (ay_i*vz - az_i*vy) + (my_i*wz - mz_i*wy);
    ey = (az_i*vx - ax_i*vz) + (mz_i*wx - mx_i*wz);
    ez = (ax_i*vy - ay_i*vx) + (mx_i*wy - my_i*wx);
    % --- 只使用加速度计误差 ---
    % ex = (ay_i*vz - az_i*vy); % + (my_i*wz - mz_i*wy);  <-- 注释掉磁力计部分
    % ey = (az_i*vx - ax_i*vz); % + (mz_i*wx - mx_i*wz);  <-- 注释掉磁力计部分
    % ez = (ax_i*vy - ay_i*vx); % + (mx_i*wy - my_i*wx);  <-- 注释掉磁力计部分

    % 5. 误差积分
    if Ki > 0
        exInt = exInt + ex * Ki * dt;
        eyInt = eyInt + ey * Ki * dt;
        ezInt = ezInt + ez * Ki * dt;
    end

    % 6. 修正陀螺仪读数
    gx_i = gx_i + Kp*ex + exInt;
    gy_i = gy_i + Kp*ey + eyInt;
    gz_i = gz_i + Kp*ez + ezInt;

    % 7. 更新四元数 (一阶龙格-库塔法)
    halfT = 0.5 * dt;
    q0_new = q0 + (-q1*gx_i - q2*gy_i - q3*gz_i) * halfT;
    q1_new = q1 + ( q0*gx_i + q2*gz_i - q3*gy_i) * halfT;
    q2_new = q2 + ( q0*gy_i - q1*gz_i + q3*gx_i) * halfT;
    q3_new = q3 + ( q0*gz_i + q1*gy_i - q2*gx_i) * halfT;

    % 8. 归一化四元数
    norm_q = sqrt(q0_new*q0_new + q1_new*q1_new + q2_new*q2_new + q3_new*q3_new);
    q(i,1) = q0_new / norm_q;
    q(i,2) = q1_new / norm_q;
    q(i,3) = q2_new / norm_q;
    q(i,4) = q3_new / norm_q;
end

% --- 将四元数转换为欧拉角，用于绘图 ---
q0_vec = q(:,1); q1_vec = q(:,2); q2_vec = q(:,3); q3_vec = q(:,4);
rollmahony  = atan2(2*(q0_vec.*q1_vec + q2_vec.*q3_vec), 1 - 2*(q1_vec.^2 + q2_vec.^2));
pitchmahony = asin(2*(q0_vec.*q2_vec - q3_vec.*q1_vec));
yawmahony   = atan2(2*(q0_vec.*q3_vec + q1_vec.*q2_vec), 1 - 2*(q2_vec.^2 + q3_vec.^2));

% --- 绘图对比 ---
% --- 绘图对比 ---

% 首先将所有角度从弧度转换为度
rollmahony_deg  = rollmahony * 180/pi;
pitchmahony_deg = pitchmahony * 180/pi;
yawmahony_deg   = yawmahony * 180/pi;

% [新增] 调整Yaw的角度范围以匹配参考数据
% 将所有负角度值加上360度
yawmahony_deg(yawmahony_deg < 0) = yawmahony_deg(yawmahony_deg < 0) + 360;

figure('Name', 'Mahony AHRS vs ArduPilot ATT')
subplot(311)
plot(gx(:,1), rollmahony_deg, 'b','linewidth',1.3); % 使用转换后的变量
hold on;
plot(roll(:,1), roll(:,2) * 180/pi, 'r--','linewidth',1.3); 
hold on;
plot(gx(:,1),roll_gyro * 180/pi,'g-','linewidth',1.3);
hold on;
plot(gx(:,1),roll_acc * 180/pi,'m:','linewidth',1.3);
title('Roll 角度对比');
xlabel('Time(s)');
ylabel('Angle (deg)');
grid on;
legend('Mahony解算', 'ArduPilot参考',"陀螺仪","加速度计");

subplot(312)
plot(gy(:,1), pitchmahony_deg, 'b','linewidth',1.3); % 使用转换后的变量
hold on;
plot(pitch(:,1), pitch(:,2) * 180/pi, 'r--','linewidth',1.3);
hold on;
plot(gy(:,1),pitch_gyro * 180/pi,'g-','linewidth',1.3);
hold on;
plot(gy(:,1),pitch_acc * 180/pi,'m:','linewidth',1.3);
title('Pitch 角度对比');
xlabel('Time(s)');
ylabel('Angle (deg)');
grid on;
legend('Mahony解算', 'ArduPilot参考',"陀螺仪","加速度计");

subplot(313)
plot(gz(:,1), yawmahony_deg, 'b'); % 使用转换并调整范围后的变量
hold on;
plot(yaw(:,1), yaw(:,2) * 180/pi, 'r--');
title('Yaw 角度对比');
xlabel('Time(s)');
ylabel('Angle (deg)');
grid on;
legend('Mahony解算', 'ArduPilot参考');
