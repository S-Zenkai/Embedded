% 功能：姿态仿真测试
% 作者：across
% 说明：Eulerdata xsens 50hz取得 用其角度作为准确性参考

clear all
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

roll(:,2) = roll(:,2) * 0.0174;
pitch(:,2) = pitch(:,2) * 0.0174;
yaw(:,2) = yaw(:,2) * 0.0174;


% dt1 = 0.04; %高速率任务，IMU(有些问题）
% dt2 = 20e-3; %低速率任务，ATT、MAG

[m,n]=size(xacc);
% t1 = (0:(m-1))*dt1;
% t1_fin = t1(end);
% t2_fin = t2(end);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% part 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%原始数据观测

figure(1)
subplot(311)
plot(roll(:,1),roll(:,2));title('roll');xlabel('Time(s)');grid;
subplot(312)
plot(pitch(:,1),pitch(:,2));title('pitch');xlabel('Time(s)');grid;
subplot(313)
plot(yaw(:,1),yaw(:,2));title('yaw');xlabel('Time(s)');grid;

figure(2)
subplot(331)
plot(rollrate(:,1),rollrate(:,2));title('RollRate');xlabel('Time(s)');grid;
subplot(332)
plot(pitchrate(:,1),pitchrate(:,2));title('PitchRate');xlabel('Time(s)');grid;
subplot(333)
plot(yawrate(:,1),yawrate(:,2));title('YawRate');xlabel('Time(s)');grid;
subplot(334)
plot(xacc(:,1),xacc(:,2));title('Xacc');xlabel('Time(s)');grid;
subplot(335)
plot(yacc(:,1),yacc(:,2));title('Yacc');xlabel('Time(s)');grid;
subplot(336)
plot(zacc(:,1),zacc(:,2));title('Zacc');xlabel('Time(s)');grid;
subplot(337)
plot(magx(:,1),magx(:,2));title('MagX');xlabel('Time(s)');grid;
subplot(338)
plot(magy(:,1),magy(:,2));title('MagY');xlabel('Time(s)');grid;
subplot(339)
plot(magz(:,1),magz(:,2));title('MagZ');xlabel('Time(s)');grid;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% part 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%坐标系对齐
%转换到前右下  rollrate对应x轴  pitchrate对应y轴
% gx = rollrate;
% gy = pitchrate;
% gz = yawrate;
% ax = xacc(:,2);
% ay = yacc(:,2);
% az = -zacc(:,2);
% gx = pitchrate(:,2);
% gy = -rollrate(:,2);
% gz = yawrate(:,2);
% ax = yacc(:,2);
% ay = -xacc(:,2);
% az = zacc(:,2);
% mx = -magy(:,2);
% my = magx(:,2);
% mz = magz(:,2);
gx = pitchrate;
gy = -rollrate;
gz = yawrate;
ax = yacc;
ay = -xacc;
az = zacc;
mx = -magy;
my = magx;
mz = magz;
gx(:,2) = pitchrate(:,2)* 0.0174;
gy(:,2) = -rollrate(:,2)* 0.0174;
gz(:,2) = yawrate(:,2)* 0.0174;


%%
%数据滤波
[b,a]=butter(2,10/50);
XAcc=filter(b,a,ax(:,2));
YAcc=filter(b,a,ay(:,2));
ZAcc=filter(b,a,az(:,2));
figure(3)
subplot(311)
plot(ax(:,1),ax(:,2),ax(:,1),XAcc,'r');title('Xacc');xlabel('Time(s)');grid;
subplot(312)
plot(ay(:,1),ay(:,2),ay(:,1),YAcc,'r');title('Yacc');xlabel('Time(s)');grid;
subplot(313)
plot(az(:,1),az(:,2),az(:,1),ZAcc,'r');title('Zacc');xlabel('Time(s)');grid;

% ax = XAcc;
% ay = YAcc;
% az = ZAcc;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% part 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%对比角速度积分与加速度得到的姿态角之间的区别
%角速度积分得到的角度会随时间漂移
%加速度得到的角度会噪声很大

roll_gyro = zeros(m,1);
roll_acc = zeros(m,1);
dt1=0.01;
for i = 2:m

    if i==1
        roll_gyro(1) = gx(1,2) * dt1;
    else
        dt1=gx(i,1)-gx(i-1,1);
        roll_gyro(i) = roll_gyro(i-1) + gx(i,2) * dt1;
    end

    % roll_acc(i) = atan2(ay(i,2),-az(i,2));
    roll_acc(i) = atan2(ay(i,2),az(i,2));

end

figure(4)
plot(gx(:,1),roll_gyro,ax(:,1),roll_acc,'r',roll(:,1),roll(:,2));title('roll 角速度、加速度推角度、参考角度对比');xlabel('Time(s)');grid;

pitch_gyro = zeros(m,1);
pitch_acc = zeros(m,1);

for i = 2:m

    if i==1
        pitch_gyro(1) = gy(1,2) * dt1;
    else
        dt1=gy(i,1)-gy(i-1,1);
        pitch_gyro(i) = pitch_gyro(i-1) + gy(i,2) * dt1;
    end

    % pitch_acc(i) = atan2(-ax(i,2),-az(i,2));
    pitch_acc(i) = atan2(-ax(i,2),az(i,2));

end

figure(5)
plot(gy(:,1),pitch_gyro,ay(:,1),pitch_acc,'r',pitch(:,1),pitch(:,2));title('pitch 角速度、加速度推角度、参考角度对比');xlabel('Time(s)');grid;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% part 4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %互补滤波 单轴
% complementation_filtered = zeros(m,1);
% adaption_complementation_filtered = zeros(m,1);
% 
% coeff = 0.95;
% 
% for i=2:m
%     dt1=pitchrate(i,1)-pitchrate(i-1,1);
%     complementation_filtered(i) = (complementation_filtered(i-1) + gx(i)*dt1) * coeff + (1 - coeff) * roll_acc(i);
%     % complementation_filtered(i) = (complementation_filtered(i-1) + gy(i)*dt1) * coeff + (1 - coeff) * roll_acc(i);
%     if abs(gx(i)) > 0.3
%         adaption_complementation_filtered(i) = (adaption_complementation_filtered(i-1) + gx(i)*dt1) * coeff + (1 - coeff) * roll_acc(i);
%     else
%         adaption_complementation_filtered(i) = (adaption_complementation_filtered(i-1) + gx(i)*dt1) * 0.005 + (1 - 0.005) * roll_acc(i);
%     end
%     % if abs(gy(i)) > 0.3
%     %     adaption_complementation_filtered(i) = (adaption_complementation_filtered(i-1) + gy(i)*dt1) * coeff + (1 - coeff) * roll_acc(i);
%     % else
%     %     adaption_complementation_filtered(i) = (adaption_complementation_filtered(i-1) + gy(i)*dt1) * 0.005 + (1 - 0.005) * roll_acc(i);
%     % end
% end
% 
% figure(6)
% subplot(411)
% title('滚转');
% plot(pitchrate(:,1),complementation_filtered,'r',pitchrate(:,1),roll_gyro);grid;
% legend('互补滤波','角速度积分')
% subplot(412)
% plot(pitchrate(:,1),complementation_filtered,'r',pitchrate(:,1),roll_acc);grid;
% legend('互补滤波','加速度参考')
% subplot(413)
% plot(pitchrate(:,1),complementation_filtered,'r',roll(:,1),roll(:,2));grid;
% legend('互补滤波','参考角度')
% subplot(414)
% plot(roll(:,1),roll(:,2),'r',pitchrate(:,1),adaption_complementation_filtered);grid;
% legend('互补滤波','自适应互补')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% part 5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %单轴kalman  模型1
% Q_angle = 0.023; %Calculated from data
% Q_bias  = 0.01;
% R_angle = 1;     %Accel variance, calculated from data
% 
% Q = [Q_angle 0; 0 Q_bias];
% 
% pitchKF1 = zeros(m,1);
% pitchMeasure1 = zeros(m,1);
% bias = zeros(m,1);
% 
% P = [0 0
%      0 0];
%  
% C = [1 0]; 
% 
% 
% for i = 2:m
%     
%     pitchMeasure1(i) = atan2(-ax(i),-az(i));
% 
%     A = [1 -dt; 0 1];
% 
%     %Predicted state estimate x = F.x + B.u
%     pitchKF1(i) = pitchKF1(i-1) + dt * (gy(i) - bias(i-1));
%     bias(i) = bias(i-1);
% 
%     %Predicted estimate covariance P = F.P.F' + Q
%     P = A*P*A' + Q;
% 
%     %Innovation y = z - H.x
%     y = pitchMeasure1(i) - pitchKF1(i);
% 
%     %Innovation covariance S = H.P.H' + R
%     S = C*P*C' + R_angle;
% 
%     %Optimal kalman gain K = P.H'/S
%     K = P * C' / S;
% 
%     %Updated state estimate x=x + K.y
%     pitchKF1(i) = pitchKF1(i) + K(1)*y;
%     bias(i) = bias(i) + K(2)*y;
% 
%     %Updated estimate covariance P = (I - K.H).P
%     P = ([1 0; 0 1] - [K(1); K(2)] * C) * P;
% end
% 
% figure(7)
% plot(t1,pitchKF1,t1,pitch,'r');title('单轴卡尔曼模型结果验证');xlabel('Time(s)');grid;
% legend('单轴卡尔曼','参考角度')



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
% %mahony
% halfT = 0.5 * dt;
% Kp = 0.25;
% Ki = 0.0;
% 
% exInt = zeros(m,1);							 
% eyInt = zeros(m,1);
% ezInt = zeros(m,1); 
% 
% q0 = zeros(m,1);
% q1 = zeros(m,1);
% q2 = zeros(m,1);
% q3 = zeros(m,1);
% q0q0 = zeros(m,1);
% q0q1 = zeros(m,1);
% q0q2 = zeros(m,1);
% q0q3 = zeros(m,1);
% q1q1 = zeros(m,1);
% q1q2 = zeros(m,1);
% q1q3 = zeros(m,1);
% q2q2 = zeros(m,1);
% q2q3 = zeros(m,1);
% q3q3 = zeros(m,1);
% 
% q0(1) = 1;
% 
% rollmahony = zeros(m,1);
% pitchmahony = zeros(m,1);
% yawmahony = zeros(m,1);
% 
% az = -az;
% 
% for i=2:m
% 
%     q0q0(i)=q0(i-1)*q0(i-1);
%     q0q1(i)=q0(i-1)*q1(i-1);
%     q0q2(i)=q0(i-1)*q2(i-1);
%     q0q3(i)=q0(i-1)*q3(i-1);
%     q1q1(i)=q1(i-1)*q1(i-1);
%     q1q2(i)=q1(i-1)*q2(i-1);
%     q1q3(i)=q1(i-1)*q3(i-1);
%     q2q2(i)=q2(i-1)*q2(i-1);
%     q2q3(i)=q2(i-1)*q3(i-1);
%     q3q3(i)=q3(i-1)*q3(i-1);
% 
%     norm = sqrt(mx(i)*mx(i)+my(i)*my(i)+mz(i)*mz(i));
%     mx(i)=mx(i)/norm;
%     my(i)=my(i)/norm;
%     mz(i)=mz(i)/norm;
% 
%     hx = 2.0 * (mx(i) * (0.5 - q2q2(i) - q3q3(i)) + my(i) * (q1q2(i) - q0q3(i)) + mz(i) * (q1q3(i) + q0q2(i)));  
%     hy = 2.0 * (mx(i) * (q1q2(i) + q0q3(i)) + my(i) * (0.5 - q1q1(i) - q3q3(i)) + mz(i) * (q2q3(i) - q0q1(i)));  
%     bx = sqrt(hx * hx + hy * hy);  
%     ez_ef = -hy * bx;
% 
%     mag_ex = 2.0 * (q1q3(i) - q0q2(i)) * ez_ef;
%     mag_ey = 2.0 * (q2q3(i) + q0q1(i)) * ez_ef;
%     mag_ez = (1.0 - 2.0 * q1q1(i) - 2.0 * q2q2(i)) * ez_ef;
% 
%     norm=sqrt(ax(i)*ax(i)+ay(i)*ay(i)+az(i)*az(i));%把加计的三维向量转成单位向量
%     ax(i)=ax(i)/norm;
%     ay(i)=ay(i)/norm;
%     az(i)=az(i)/norm;
% 
%     halfex = (ay(i)*(1.0 - 2.0*q1q1(i) - 2.0*q2q2(i)) - az(i)*(2.0*(q2q3(i)+q0q1(i))));                          					
%     halfey = (az(i)*(2.0*(q1q3(i)-q0q2(i))) - ax(i)*(1.0 - 2.0*q1q1(i) - 2.0*q2q2(i)));
%     halfez = (ax(i)*(2.0*(q2q3(i)+q0q1(i))) - ay(i)*(2.0*(q1q3(i)-q0q2(i))));
% 
%     ex = halfex;% + mag_ex;
%     ey = halfey;% + mag_ey;
%     ez = halfez;% + mag_ez;
% 
%     exInt(i) = exInt(i) + ex* Ki* dt;								 
%     eyInt(i) = eyInt(i) + ey* Ki* dt;
%     ezInt(i) = ezInt(i) + ez* Ki* dt; 
% 
%     gx(i) = (gx(i) + Kp*ex + exInt(i))*halfT;					   							
%     gy(i) = (gy(i) + Kp*ey + eyInt(i))*halfT;
%     gz(i) = (gz(i) + Kp*ez + ezInt(i))*halfT;
% 
%     q0(i) = q0(i-1) + (-q1(i-1)*gx(i)  - q2(i-1)*gy(i) - q3(i-1)*gz(i));%四元数微分方程
%     q1(i) = q1(i-1) + ( q0(i-1)*gx(i) + q2(i-1)*gz(i) - q3(i-1)*gy(i));
%     q2(i) = q2(i-1) + ( q0(i-1)*gy(i)  - q1(i-1)*gz(i) + q3(i-1)*gx(i));
%     q3(i) = q3(i-1) + ( q0(i-1)*gz(i) + q1(i-1)*gy(i)  - q2(i-1)*gx(i));
% 
%     norm = sqrt(q0(i)*q0(i) + q1(i)*q1(i) + q2(i)*q2(i) + q3(i)*q3(i));%四元数规范化
%     q0(i) = q0(i) / norm;
%     q1(i) = q1(i) / norm;
%     q2(i) = q2(i) / norm;
%     q3(i) = q3(i) / norm;
% 
%     rollmahony(i)   = atan2(2 * q2(i) * q3(i) + 2 * q0(i) * q1(i), -2 * q1(i) * q1(i) - 2 *q2(i)* q2(i) + 1); %roll
%     pitchmahony(i)  = (asin(2 * q0(i) * q2(i) - 2 * q1(i)* q3(i))); % pitch
%     yawmahony(i)    = atan2(2 * q1(i) * q2(i) + 2 * q0(i) * q3(i) , -2 *q2(i) * q2(i) - 2 * q3(i) * q3(i) +1);
% end
% 
% 
% figure(9)
% subplot(311)
% plot(t1,rollmahony,t1,roll,'r');title('roll 角度对比');xlabel('Time(s)');grid;
% legend('mahony','参考角度')
% subplot(312)
% plot(t1,pitchmahony,t1,pitch,'r');title('pitch 角度对比');xlabel('Time(s)');grid;
% legend('mahony','参考角度')
% subplot(313)
% plot(t1,yawmahony,t1,yaw,'r');title('yaw 角度对比');xlabel('Time(s)');grid;
% legend('mahony','参考角度')




