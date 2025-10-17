
close all;
%%qBv、qDv.姿态跟踪
figure('color',[1 1 1],'NumberTitle','on','Name','Attitude tracking');
subplot(3,1,1);
plot(qBv(:,1),qBv(:,2),'r',qDv(:,1),qDv(:,2),'b:','linewidth',1.3);
legend('qBv1','qDv1');
subplot(3,1,2);
plot(qBv(:,1),qBv(:,3),'r',qDv(:,1),qDv(:,3),'b:','linewidth',1.3);
legend('qBv2','qDv2');
subplot(3,1,3);
plot(qBv(:,1),qBv(:,4),'r',qDv(:,1),qDv(:,4),'b:','linewidth',1.3);
legend('qBv3','qDv3');
xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
ylabel('四元数','FontSize',12,'Fontname','Time New Roman');

%%omegaB_B、omegaD_D角速度跟踪
figure('color',[1 1 1],'NumberTitle','on','Name','Angular velocity tracking');
subplot(3,1,1);
plot(omegaB_B(:,1),omegaB_B(:,2),'r',omegaD_D(:,1),omegaD_D(:,2),'b:','linewidth',1.3);
legend('omega_B_B1','omega_D_D1');
subplot(3,1,2);
plot(omegaB_B(:,1),omegaB_B(:,3),'r',omegaD_D(:,1),omegaD_D(:,3),'b:','linewidth',1.3);
legend('omega_B_B2','omega_D_D2');
subplot(3,1,3);
plot(omegaB_B(:,1),omegaB_B(:,4),'r',omegaD_D(:,1),omegaD_D(:,4),'b:','linewidth',1.3);
legend('omega_B_B3','omega_D_D3');
xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
ylabel('角速度','FontSize',12,'Fontname','Time New Roman');

%%姿态跟踪误差
figure('color',[1 1 1],'NumberTitle','on','Name','qev');
plot(qev(:,1),qev(:,2),'r',qev(:,1),qev(:,3),'b:',qev(:,1),qev(:,4),'g-.','linewidth',1.3);
% legend('qev1','qev2','qev3');
hold on;
plot(rol(:,1),rol(:,2),'k--',rol(:,1),-rol(:,2),'k--','linewidth',0.5)
legend('qev1','qev2','qev3','rol');
hold off;
grid on;
xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
ylabel('四元数','FontSize',12,'Fontname','Time New Roman');

%%角速度跟踪误差
figure('color',[1 1 1],'NumberTitle','on','Name','omegae');
plot(omegae(:,1),omegae(:,2),'r',omegae(:,1),omegae(:,3),'b:',omegae(:,1),omegae(:,4),'g-.','linewidth',1.3);
legend('ome1','ome1','ome3');
xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
ylabel('omegae','FontSize',12,'Fontname','Time New Roman');

%%原始输入力矩力矩tau_B
figure('color',[1 1 1],'NumberTitle','on','Name','tau_B');
plot(tauc(:,1),tauc(:,2),'r',tauc(:,1),tauc(:,3),'b:',tauc(:,1),tauc(:,4),'g-.','linewidth',1.3);
legend('tc1','tc2','tc3');
xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
ylabel('tau_B','FontSize',12,'Fontname','Time New Roman');

%%量化后的输入力矩tau_B
figure('color',[1 1 1],'NumberTitle','on','Name','tau_B');
plot(tau_B(:,1),tau_B(:,2),'r',tau_B(:,1),tau_B(:,3),'b:',tau_B(:,1),tau_B(:,4),'g-.','linewidth',1.3);
legend('ta1','ta2','ta3');
xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
ylabel('tau_B','FontSize',12,'Fontname','Time New Roman');

%%干扰估计
figure('color',[1 1 1],'NumberTitle','on','Name','Angular velocity tracking');
subplot(3,1,1);
plot(d(:,1),d(:,2),'r',dg(:,1),dg(:,2),'b:','linewidth',1.3);
legend('d1','g1');
subplot(3,1,2);
plot(d(:,1),d(:,3),'r',dg(:,1),dg(:,3),'b:','linewidth',1.3);
legend('d2','g2');
subplot(3,1,3);
plot(d(:,1),d(:,4),'r',dg(:,1),dg(:,4),'b:','linewidth',1.3);
legend('d3','g3');
xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
ylabel('干扰','FontSize',12,'Fontname','Time New Roman');


%%干扰估计
figure('color',[1 1 1],'NumberTitle','on','Name','Angular velocity tracking');
subplot(3,1,1);
plot(de(:,1),de(:,2),'r','linewidth',1.3);
legend('de1');
subplot(3,1,2);
plot(de(:,1),de(:,3),'r','linewidth',1.3);
legend('de2');
subplot(3,1,3);
plot(de(:,1),de(:,4),'r','linewidth',1.3);
legend('de3');
xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
ylabel('干扰','FontSize',12,'Fontname','Time New Roman');



%%量化后的输入力矩tau_B
figure('color',[1 1 1],'NumberTitle','on','Name','tau_B');
plot(tau_B(:,1),tau_B(:,2),'r',tauc(:,1),tauc(:,2),'b:','linewidth',1.3);
legend('ta1','ta2');
xlabel('Time(s)','FontSize',12,'Fontname','Times New Roman');
ylabel('tau_B','FontSize',12,'Fontname','Time New Roman');
