clear, clc 
close all 
load('reaction_data_ki_1.mat')
figure()
plot(t_s,theta,'LineWidth',1.5)
xlabel('Time','interpreter','latex')
ylabel('$\theta$ response, $^\circ$','interpreter','latex')
title('Arm Angle','interpreter','latex')
figure()
plot(t_s,theta_dot*180/pi,'LineWidth',1.5)
xlabel('Time','interpreter','latex')
ylabel('$\dot{\theta}$ response, $\frac{^\circ}{s}$','interpreter','latex')
%legend('Commanded','Actual','interpreter','latex')
title('Arm Angular Velocity','interpreter','latex')
figure()
plot(t_s,Vm,'LineWidth',1.5)
xlabel('Time','interpreter','latex')
ylabel('$V_m$, V','interpreter','latex')
title('Experimental Control Voltage','interpreter','latex')
ylim([-12 12])

%% Determine the following control specs: 
%E_SS 
%max overshoot
%Settling Time (2%) 
%Rise time
E_SS = theta(end) - 0; %Difference from the zero point at the end of the data 
Max_Overshoot = max(theta) - 0; %maximum overshoot of the system vs zero point 
Step_range = 545:609; 
Step_data = theta(Step_range);
Step_Time = t_s(Step_range);
figure() 
plot(Step_Time,Step_data,'LineWidth',1.5)
xlabel('Time','interpreter','latex')
ylabel('$\theta$ response, $^\circ$','interpreter','latex')
title('Selected Step','interpreter','latex')
Results = stepinfo(Step_data,Step_Time);

%Rise time happens within a single IMU data cycle, so 
t_rise = .05;% seconds

% Print system performance info 
fprintf('--- System Performance --- \n \n')
fprintf('Steady state error of %4.2f deg \n',E_SS)
fprintf('Rise time of %4.2f s \n',t_rise)
fprintf('Maximum Overshoot of %4.2f deg \n',Max_Overshoot)
fprintf('Settling time of %4.2f s \n',Results.SettlingTime)
