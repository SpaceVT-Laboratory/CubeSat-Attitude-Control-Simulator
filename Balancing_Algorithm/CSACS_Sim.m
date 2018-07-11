clear all; close all; clc;
format long

%% CSACS_Sim: Main Program
%  This program simulates the dynamics of the CubeSat Attitude Control 
%  Simulator (CSACS), given initial values for the state vector.

% Order of the state vector
% y = [psi,theta,phi,wx,wy,wz,rx,ry,rz,rm_x,rm_y]

% Get initial values from user
pa = user_inp;

% Specify number of iterations 
runs = 4;

% Memory preallocation
CM       = zeros(3,runs);  % CM vectors
T_pitch  = zeros(1,runs);  % periods of oscillation for pitch state time-histories 
T_roll   = zeros(1,runs);  % periods of oscillation for roll state time-histories
A_pitch  = zeros(1,runs);  % maximum amplitudes for pitch state time-histories
A_roll   = zeros(1,runs);  % maximum amplitudes for roll state time-histories

% Simulation parameters
t0  = 0;       % initial time: sec
dt  = 0.25;    % data acquisition time step
tf  = pa(1);   % final time: sec
r2d = 180/pi;  % radian to degree conversion

% CSACS physical properties
I     = [0.07916853     0.00944042    0.00214993;
         0.00944042     0.07766567   -0.00161408;
         0.00214993    -0.00161408    0.13642287]; % inertia matrix about center of gravity [kg*m^2]
m     = 7.75345629;                                % total mass of table [kg]
m_mmu = 0.043;                                     % mass of stepper motors [kg]
g = 9.81;                                          % acceleration due to gravity [m/s^2]

% Initial MMU displacements
% two MMUs (x- and y-axis)
r_mmu = [0; 0];

% three MMUs (x-, y-, and z-axis)
% r_mmu = [0; 0; 0];

% initial conditions
y(:,1) = [pa(2) pa(3) pa(4) pa(5) pa(6) pa(7) pa(8) pa(9) pa(10) r_mmu(1) r_mmu(2)]';

% Check maximum correctable offset before simulation
rm_max     = 0.055;                   % maximum linear distance for each MMU [m]

CM_off_max = (m_mmu/m)*rm_max;        % maximum correctable CM offset [m]
CM_off     = sqrt(pa(8)^2+pa(9)^2);   % magnitude of rx and ry [m]

if CM_off > CM_off_max
    error(['\nAutomatic balancing cannot be achieved.', ...
        '\nThe CSACS must be manually balanced to ',...
        'within the maximum correctable CM offset of %0.3f mm.'],CM_off_max*1000)
end

%% Specify method for which you would like to solve system of ODEs:
%   'Case 1' = ode45 Solver
%   'Case 2' = Euler Explicit 
%   'Case 3' = Two-Stage Runge-Kutta (Heun's Method)
%   'Case 4' = Four-Stage Runge-Kutta (RK4)
caseNum   = 'Case 4';

iter = 0;
% Loop for repeating simulation after changing C.M.
while iter < runs
    %% Main Loop
    tc = 0;
    tt = tc;
    %tic;
    for t=t0:dt:tf
        tc = tc + 1; % Current time-vector counter
        
        if strcmp(caseNum,'Case 1') == 1
            method = 'ODE45 Solver';
            %% ode45 "Black-Box" solution
            % set ODE solver tolerance (optional)
            options = odeset('RelTol',1e-5);
            [t1,y_dt] = ode45('csacseom',[t,dt+t],y(:,tc),options);
            y(:,tc+1) = y_dt(size(y_dt,1),:)';
            tt(tc+1) = t;
        elseif strcmp(caseNum,'Case 2') == 1
            method = 'Euler Explicit Method';
            %% Euler Explicit
            y(:,tc+1) = y(:,tc) + dt*csacseom(tc, y(:,tc));
            tt(tc+1) = t;
        elseif strcmp(caseNum,'Case 3') == 1
            method = 'Heun''s Method';
            %% Two-Stage Runge-Kutta (Heun's Method)
            k1 = csacseom(tc, y(:,tc));
            k2 = csacseom(tc+dt,y(:,tc)+dt*k1);

            y(:,tc+1) = y(:,tc) + (dt/2)*(k1+k2);
            tt(tc+1) = t;
        else
            method = 'RK4 Method';
            %% Four-Stage Runge-Kutta (RK4)
            k1 = dt*csacseom(tc, y(:,tc));
            k2 = dt*csacseom(tc+dt/2,y(:,tc)+k1/2);
            k3 = dt*csacseom(tc+dt/2,y(:,tc)+k2/2);
            k4 = dt*csacseom(tc+dt,y(:,tc)+k3);

            y(:,tc+1) = y(:,tc) + (1/6)*(k1+2*k2+2*k3+k4);
            tt(tc+1) = t;
        end
    end
    %toc;

    % Calculate CM offset and store test number data for presentation
    r_hat        = r_est(y,t0,dt,tf,I)
    CM(:,iter+1) = r_hat;
    
    % Difference between known CM offset and calculated CM offset
    r_diff       = r_hat-[pa(8);pa(9);pa(10)]
    
    %% Plot Results
    if runs == 4
        line = {'k:', 'k--', 'k-.', 'k-'};
    elseif runs == 3
        line = {'k:', 'k-.', 'k-'};
    elseif runs == 2
        line = {'k-.', 'k-'};
    else
        line = {'k-'};
    end
    
    %% Seperate Figures
%     figure(1)
%     hold on
%     plot(tt,y(1,:)*r2d,line{iter+1},'LineWidth',1)
%     grid on
%     xl = xlabel('Length of Simulation (sec)');
%     yl = ylabel('$\psi$ (deg)');
%     set([xl yl],'interpreter','Latex','fontsize',14)
%     set(gca,'TickLabelInterpreter','Latex','FontSize',12)
%     axis tight
%         
%     figure(2)
%     hold on
%     plot(tt,y(2,:)*r2d,line{iter+1},'LineWidth',1)
%     grid on
%     xl = xlabel('Length of Simulation (sec)');
%     yl = ylabel('$\theta$ (deg)');
%     set([xl yl],'interpreter','Latex','fontsize',14)
%     set(gca,'TickLabelInterpreter','Latex','FontSize',12)
%     axis tight
%     
%     figure(3)
%     hold on
%     plot(tt,y(3,:)*r2d,line{iter+1},'LineWidth',1)
%     grid on
%     xl = xlabel('Length of Simulation (sec)');
%     yl = ylabel('$\phi$ (deg)');
%     set([xl yl],'interpreter','Latex','fontsize',14)
%     set(gca,'TickLabelInterpreter','Latex','FontSize',12)
%     axis tight
%         
%     figure(4)
%     hold on
%     plot(tt,y(4,:),line{iter+1},'LineWidth',1)
%     grid on
%     xl = xlabel('Length of Simulation (sec)');
%     yl = ylabel('$\omega_x$ (rad/sec)');
%     set([xl yl],'interpreter','Latex','fontsize',14)
%     set(gca,'TickLabelInterpreter','Latex','FontSize',12)
%     axis tight
%         
%     figure(5)
%     hold on
%     plot(tt,y(5,:),line{iter+1},'LineWidth',1)
%     grid on
%     xl = xlabel('Length of Simulation (sec)');
%     yl = ylabel('$\omega_y$ (rad/sec)');
%     set([xl yl],'interpreter','Latex','fontsize',14)
%     set(gca,'TickLabelInterpreter','Latex','FontSize',12)
%     axis tight
%     
%     figure(6)
%     hold on
%     plot(tt,y(6,:),line{iter+1},'LineWidth',1)
%     grid on
%     xl = xlabel('Length of Simulation (sec)');
%     yl = ylabel('$\omega_z$ (rad/sec)');
%     set([xl yl],'interpreter','Latex','fontsize',14)
%     set(gca,'TickLabelInterpreter','Latex','FontSize',12)
%     axis tight
    
    %% Combined Figure
    subplot(3,2,1)
    hold on
    plot(tt,y(1,:)*r2d,line{iter+1},'LineWidth',1)
    grid on
    xlabel('\textbf{(a)} yaw state time-history','Interpreter','Latex','FontSize',14,...
        'VerticalAlignment','top');
    ylabel('$\psi$ (deg)','Interpreter','Latex','FontSize',15);
    set(gca,'TickLabelInterpreter','Latex','FontSize',12)
    axis tight

    subplot(3,2,3)
    hold on
    plot(tt,y(2,:)*r2d,line{iter+1},'LineWidth',1)
    grid on
    xlabel('\textbf{(c)} pitch state time-history','Interpreter','Latex','FontSize',14,...
        'VerticalAlignment','top');
    ylabel('$\theta$ (deg)','Interpreter','Latex','FontSize',15);
    set(gca,'TickLabelInterpreter','Latex','FontSize',12)
    axis tight

    subplot(3,2,5)
    hold on
    plot(tt,y(3,:)*r2d,line{iter+1},'LineWidth',1)
    grid on
    xlbl = xlabel('\begin{tabular}{c} \textbf{(e)} roll state time-history \\ \\ Length of Simulation (sec)\end{tabular}');
    set(xlbl,'Interpreter','Latex','FontSize',14,'VerticalAlignment','top');
    ylabel('$\phi$ (deg)','Interpreter','Latex','FontSize',15);
    set(gca,'TickLabelInterpreter','Latex','FontSize',12)
    axis tight

    subplot(3,2,2)
    hold on
    plot(tt,y(4,:),line{iter+1},'LineWidth',1)
    grid on
    xlabel('\textbf{(b)} x-angular velocity state time-history','Interpreter','Latex','FontSize',14,...
        'VerticalAlignment','top');
    ylabel('$\omega_x$ ($\frac{\rm{rad}}{\rm{sec}}$)','Interpreter','Latex','FontSize',15);
    set(gca,'TickLabelInterpreter','Latex','FontSize',12)
    axis tight

    subplot(3,2,4)
    hold on
    plot(tt,y(5,:),line{iter+1},'LineWidth',1)
    grid on
    xlabel('\textbf{(d)} y-angular velocity state time-history','Interpreter','Latex','FontSize',14,...
        'VerticalAlignment','top');
    ylabel('$\omega_y$ ($\frac{\rm{rad}}{\rm{sec}}$)','Interpreter','Latex','FontSize',15);
    set(gca,'TickLabelInterpreter','Latex','FontSize',12)
    axis tight

    subplot(3,2,6)
    hold on
    plot(tt,y(6,:),line{iter+1},'LineWidth',1)
    grid on
    xlbl = xlabel('\begin{tabular}{c} \textbf{(f)} z-angular velocity state time-history \\ \\ Length of Simulation (sec)\end{tabular}');
    set(xlbl,'Interpreter','Latex','FontSize',14,'VerticalAlignment','top');
    ylabel('$\omega_z$ ($\frac{\rm{rad}}{\rm{sec}}$)','Interpreter','Latex','FontSize',15);
    set(gca,'TickLabelInterpreter','Latex','FontSize',12)
    axis tight

    %title = suptitle(sprintf('CSACS Nonlinear Dynamics Solution via %s',method));
    %set(title,'Interpreter','Latex')
    
    % Calculating periods of oscillation for pitch and roll
    ac_pitch        = xcorr(y(2,:),y(2,:));
    [~,locs_pitch]  = findpeaks(ac_pitch);
    T_pitch(iter+1) = mean(diff(locs_pitch)*dt); % [seconds]
    
    ac_roll         = xcorr(y(3,:),y(3,:));
    [~,locs_roll]   = findpeaks(ac_roll);
    T_roll(iter+1)  = mean(diff(locs_roll)*dt);  % [seconds]
    
    % Calculating peak amplitudes for pitch and roll
    A_pitch(iter+1) = max([max((y(2,:)*r2d)) abs(min(y(2,:)*r2d))]);
    A_roll(iter+1)  = max([max((y(3,:)*r2d)) abs(min(y(3,:)*r2d))]);

    % Calculate MMU move distance
    dr_mmu = rm_est(r_hat(1:end-1),m,m_mmu)
    r_mmu  = r_mmu+dr_mmu
    
    iter = iter + 1;
    
    % prints state vector
    y(:,1) = [pa(2) pa(3) pa(4) pa(5) pa(6) pa(7) pa(8) pa(9) pa(10) r_mmu(1) r_mmu(2)]';
end

% Center of mass location measurements
if runs == 1
    return
else
    figure(7)
    plot(1:runs,CM(1,:)*1000,'ko-','LineWidth',1)
    hold on
    plot(1:runs,CM(2,:)*1000,'ko--','LineWidth',1)
    plot(1:runs,CM(3,:)*1000,'ko:','LineWidth',1)
    xlabel('Test Number','Interpreter','Latex','FontSize',14);
    ylabel('CM Location (mm)','Interpreter','Latex','FontSize',14);
    xticks(1:runs)
    set(gca,'TickLabelInterpreter','Latex','FontSize',12)
    grid on

    figure(8)
    plot(1:runs,CM(1,:)*1000,'ko-','LineWidth',1)
    hold on
    plot(1:runs,CM(2,:)*1000,'ko--','LineWidth',1)
    xlabel('Test Number','Interpreter','Latex','FontSize',14);
    ylabel('CM Location (mm)','Interpreter','Latex','FontSize',14);
    xticks(1:runs)
    set(gca,'TickLabelInterpreter','Latex','FontSize',12)
    grid on
end