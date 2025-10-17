
close all;

%%% 更新: 绘制姿态对比图 %%%
figure('Name', 'Attitude Comparison');
legend_items = {};
has_att_data = false;

% 绘制 Roll
subplot(3, 1, 1);
hold on;
if ~isempty(logData.attitude)
    time_ms = [logData.attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.attitude.roll], 'r-', 'LineWidth', 1.5);
    legend_items{end+1} = 'Roll (主融合)';
    has_att_data = true;
end
if ~isempty(logData.acc_mag_attitude)
    time_ms = [logData.acc_mag_attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.acc_mag_attitude.roll], 'g--', 'LineWidth', 1.2);
    legend_items{end+1} = 'Roll (加计/磁力计)';
    has_att_data = true;
end
if ~isempty(logData.gyro_attitude)
    time_ms = [logData.gyro_attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.gyro_attitude.roll], 'b:', 'LineWidth', 1.2);
    legend_items{end+1} = 'Roll (陀螺仪)';
    has_att_data = true;
end
title('横滚角 (Roll) 对比'); ylabel('角度 (度)'); grid on;
if ~isempty(legend_items); legend(legend_items, 'Location', 'best'); end
hold off;

% 绘制 Pitch
subplot(3, 1, 2);
hold on;
legend_items = {};
if ~isempty(logData.attitude)
    time_ms = [logData.attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.attitude.pitch], 'r-', 'LineWidth', 1.5);
    legend_items{end+1} = 'Pitch (主融合)';
end
if ~isempty(logData.acc_mag_attitude)
    time_ms = [logData.acc_mag_attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.acc_mag_attitude.pitch], 'g--', 'LineWidth', 1.2);
    legend_items{end+1} = 'Pitch (加计/磁力计)';
end
if ~isempty(logData.gyro_attitude)
    time_ms = [logData.gyro_attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.gyro_attitude.pitch], 'b:', 'LineWidth', 1.2);
    legend_items{end+1} = 'Pitch (陀螺仪)';
end
title('俯仰角 (Pitch) 对比'); ylabel('角度 (度)'); grid on;
if ~isempty(legend_items); legend(legend_items, 'Location', 'best'); end
hold off;

% 绘制 Yaw
subplot(3, 1, 3);
hold on;
legend_items = {};
if ~isempty(logData.attitude)
    time_ms = [logData.attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.attitude.yaw], 'r-', 'LineWidth', 1.5);
    legend_items{end+1} = 'Yaw (主融合)';
end
if ~isempty(logData.acc_mag_attitude)
    time_ms = [logData.acc_mag_attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.acc_mag_attitude.yaw], 'g--', 'LineWidth', 1.2);
    legend_items{end+1} = 'Yaw (加计/磁力计)';
end
if ~isempty(logData.gyro_attitude)
    time_ms = [logData.gyro_attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.gyro_attitude.yaw], 'b:', 'LineWidth', 1.2);
    legend_items{end+1} = 'Yaw (陀螺仪)';
end
title('偏航角 (Yaw) 对比'); xlabel('时间 (s)'); ylabel('角度 (度)'); grid on;
if ~isempty(legend_items); legend(legend_items, 'Location', 'best'); end
hold off;

if ~has_att_data
    close('Attitude Comparison'); % 如果没有任何姿态数据，则关闭空图
end












% 
% 
% acc_cal_x = [logData.imu_cal.acc_x];
% acc_cal_y = [logData.imu_cal.acc_y];
% acc_cal_z = [logData.imu_cal.acc_z];
% 
% gyro_cal_x = [logData.imu_cal.gyro_x];
% gyro_cal_y = [logData.imu_cal.gyro_y];
% gyro_cal_z = [logData.imu_cal.gyro_z];
% % acc_cal_x = [logData.imu_raw.acc_x];
% % acc_cal_y = [logData.imu_raw.acc_y];
% % acc_cal_z = [logData.imu_raw.acc_z];
% % 
% % gyro_cal_x = [logData.imu_raw.gyro_x];
% % gyro_cal_y = [logData.imu_raw.gyro_y];
% % gyro_cal_z = [logData.imu_raw.gyro_z];
% 
% acc_filter_x = [logData.imu_filter.acc_x];
% acc_filter_y = [logData.imu_filter.acc_y];
% acc_filter_z = [logData.imu_filter.acc_z];
% 
% gyro_filter_x = [logData.imu_filter.gyro_x];
% gyro_filter_y = [logData.imu_filter.gyro_y];
% gyro_filter_z = [logData.imu_filter.gyro_z];
% 
% %%acc_x
% figure('color',[1 1 1],'NumberTitle','on','Name','acc-x');
% plot(time_s,acc_cal_x,'r',time_s,acc_filter_x,'b:','linewidth',1.3);
% legend('acc-cal-x','acc-filter-x');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('acc-x','FontSize',12,'Fontname','Time New Roman');
% %%acc_y
% figure('color',[1 1 1],'NumberTitle','on','Name','acc-y');
% plot(time_s,acc_cal_y,'r',time_s,acc_filter_y,'b:','linewidth',1.3);
% legend('acc-cal-y','acc-filter-y');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('acc-y','FontSize',12,'Fontname','Time New Roman');
% %%acc_y
% figure('color',[1 1 1],'NumberTitle','on','Name','acc-z');
% plot(time_s,acc_cal_z,'r',time_s,acc_filter_z,'b:','linewidth',1.3);
% legend('acc-cal-z','acc-filter-z');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('acc-z','FontSize',12,'Fontname','Time New Roman');
% %%gyro_x
% figure('color',[1 1 1],'NumberTitle','on','Name','gyro-x');
% plot(time_s,gyro_cal_x,'r',time_s,gyro_filter_x,'b:','linewidth',1.3);
% legend('gyro-cal-x','gyro-filter-x');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('gyro-x','FontSize',12,'Fontname','Time New Roman');
% %%gyro_y
% figure('color',[1 1 1],'NumberTitle','on','Name','gyro-y');
% plot(time_s,gyro_cal_y,'r',time_s,gyro_filter_y,'b:','linewidth',1.3);
% legend('gyro-cal-y','gyro-filter-y');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('gyro-y','FontSize',12,'Fontname','Time New Roman');
% %%gyro_x
% figure('color',[1 1 1],'NumberTitle','on','Name','gyro-z');
% plot(time_s,gyro_cal_z,'r',time_s,gyro_filter_z,'b:','linewidth',1.3);
% legend('gyro-cal-z','gyro-filter-z');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('gyro-z','FontSize',12,'Fontname','Time New Roman');



% %%qBv、qDv.姿态跟踪
% figure('color',[1 1 1],'NumberTitle','on','Name','Attitude tracking');
% subplot(3,1,1);
% plot(qBv(:,1),qBv(:,2),'r',qDv(:,1),qDv(:,2),'b:','linewidth',1.3);
% legend('qBv1','qDv1');
% subplot(3,1,2);
% plot(qBv(:,1),qBv(:,3),'r',qDv(:,1),qDv(:,3),'b:','linewidth',1.3);
% legend('qBv2','qDv2');
% subplot(3,1,3);
% plot(qBv(:,1),qBv(:,4),'r',qDv(:,1),qDv(:,4),'b:','linewidth',1.3);
% legend('qBv3','qDv3');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('四元数','FontSize',12,'Fontname','Time New Roman');
% 
% %%omegaB_B、omegaD_D角速度跟踪
% figure('color',[1 1 1],'NumberTitle','on','Name','Angular velocity tracking');
% subplot(3,1,1);
% plot(omegaB_B(:,1),omegaB_B(:,2),'r',omegaD_D(:,1),omegaD_D(:,2),'b:','linewidth',1.3);
% legend('omega_B_B1','omega_D_D1');
% subplot(3,1,2);
% plot(omegaB_B(:,1),omegaB_B(:,3),'r',omegaD_D(:,1),omegaD_D(:,3),'b:','linewidth',1.3);
% legend('omega_B_B2','omega_D_D2');
% subplot(3,1,3);
% plot(omegaB_B(:,1),omegaB_B(:,4),'r',omegaD_D(:,1),omegaD_D(:,4),'b:','linewidth',1.3);
% legend('omega_B_B3','omega_D_D3');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('角速度','FontSize',12,'Fontname','Time New Roman');
% 
% %%姿态跟踪误差
% figure('color',[1 1 1],'NumberTitle','on','Name','qev');
% plot(qev(:,1),qev(:,2),'r',qev(:,1),qev(:,3),'b:',qev(:,1),qev(:,4),'g-.','linewidth',1.3);
% % legend('qev1','qev2','qev3');
% hold on;
% plot(rol(:,1),rol(:,2),'k--',rol(:,1),-rol(:,2),'k--','linewidth',0.5)
% legend('qev1','qev2','qev3','rol');
% hold off;
% grid on;
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('四元数','FontSize',12,'Fontname','Time New Roman');
% 
% %%角速度跟踪误差
% figure('color',[1 1 1],'NumberTitle','on','Name','omegae');
% plot(omegae(:,1),omegae(:,2),'r',omegae(:,1),omegae(:,3),'b:',omegae(:,1),omegae(:,4),'g-.','linewidth',1.3);
% legend('ome1','ome1','ome3');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('omegae','FontSize',12,'Fontname','Time New Roman');
% 
% 
% %%角速度跟踪误差
% figure('color',[1 1 1],'NumberTitle','on','Name','omegae');
% plot(omegae(3000:end,1),omegae(3000:end,2),'r',omegae(3000:end,1),omegae(3000:end,3),'b:',omegae(3000:end,1),omegae(3000:end,4),'g-.','linewidth',1.3);
% legend('ome1','ome1','ome3');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('omegae','FontSize',12,'Fontname','Time New Roman');
% 
% %%原始输入力矩力矩tau_B
% figure('color',[1 1 1],'NumberTitle','on','Name','tau_B');
% plot(tauc(:,1),tauc(:,2),'r',tauc(:,1),tauc(:,3),'b:',tauc(:,1),tauc(:,4),'g-.','linewidth',1.3);
% legend('tc1','tc2','tc3');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('tau_B','FontSize',12,'Fontname','Time New Roman');
% 
% %%量化后的输入力矩tau_B
% figure('color',[1 1 1],'NumberTitle','on','Name','tau_B');
% plot(tau_B(:,1),tau_B(:,2),'r',tau_B(:,1),tau_B(:,3),'b:',tau_B(:,1),tau_B(:,4),'g-.','linewidth',1.3);
% legend('ta1','ta2','ta3');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('tau_B','FontSize',12,'Fontname','Time New Roman');
% 
% %%干扰估计
% figure('color',[1 1 1],'NumberTitle','on','Name','Angular velocity tracking');
% subplot(3,1,1);
% plot(d(:,1),d(:,2),'r',dg(:,1),dg(:,2),'b:','linewidth',1.3);
% legend('d1','g1');
% subplot(3,1,2);
% plot(d(:,1),d(:,3),'r',dg(:,1),dg(:,3),'b:','linewidth',1.3);
% legend('d2','g2');
% subplot(3,1,3);
% plot(d(:,1),d(:,4),'r',dg(:,1),dg(:,4),'b:','linewidth',1.3);
% legend('d3','g3');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('干扰','FontSize',12,'Fontname','Time New Roman');
% 
% 
% %%干扰估计
% figure('color',[1 1 1],'NumberTitle','on','Name','Angular velocity tracking');
% subplot(3,1,1);
% plot(de(:,1),de(:,2),'r','linewidth',1.3);
% legend('de1');
% subplot(3,1,2);
% plot(de(:,1),de(:,3),'r','linewidth',1.3);
% legend('de2');
% subplot(3,1,3);
% plot(de(:,1),de(:,4),'r','linewidth',1.3);
% legend('de3');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('干扰','FontSize',12,'Fontname','Time New Roman');
% 
% de1=d;
% de1(:,2:4)=d(:,2:4)-dg(:,2:4);
% figure('color',[1 1 1],'NumberTitle','on','Name','Angular velocity tracking');
% subplot(3,1,1);
% plot(de1(:,1),de1(:,2),'r','linewidth',1.3);
% legend('de1');
% subplot(3,1,2);
% plot(de1(:,1),de1(:,3),'r','linewidth',1.3);
% legend('de2');
% subplot(3,1,3);
% plot(de1(:,1),de1(:,4),'r','linewidth',1.3);
% legend('de3');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('干扰','FontSize',12,'Fontname','Time New Roman');
% 
% 
% 
% %%量化后的输入力矩tau_B
% figure('color',[1 1 1],'NumberTitle','on','Name','tau_B');
% plot(tau_B(:,1),tau_B(:,2),'r',tauc(:,1),tauc(:,2),'b:','linewidth',1.3);
% legend('ta1','ta2');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('tau_B','FontSize',12,'Fontname','Time New Roman');
% 
% 
% %不同参数量化误差对比
% figure('color',[1 1 1],'NumberTitle','on','Name','De_norm');
% plot(De_norm_10(:,1),De_norm_10(:,2),'r',De_norm_100(:,1),De_norm_100(:,2),'b:','linewidth',1.3);
% legend('k_phi=10','k_phi=100');
% xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
% ylabel('tau_B','FontSize',12,'Fontname','Time New Roman');





