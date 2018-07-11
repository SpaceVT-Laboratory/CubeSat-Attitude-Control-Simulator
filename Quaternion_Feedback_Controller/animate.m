close all;
%*********************************************************************
% animate.m
% 3D animation display of the CSACS dynamics simulations
%*********************************************************************
%
% NOTE: USER MUST FIRST RUN RWA_Controller.slx TO GENERATE 
%       TRAJECTORY AND MOTOR SPEED DATA FOR animate.m
%
%*********************************************************************
% Generate CSACS Geometry
%*********************************************************************
CSACS_Geometry;

%*********************************************************************
% Initialize Displays
%*********************************************************************
% PLOT INITIAL POSITION:
clf;
subplot(4,4,[1:12]);
hFig = figure(1); 
set(hFig, 'Position', [250 75 1100 700]); % dependent on screen size
colormap bone;

plot3(0,0,0); hold on;
% set(gca,'Zdir','reverse')
for n=1:numel(Face)
    p(n) = patch(Face(n).bcf(1,:),Face(n).bcf(2,:),Face(n).bcf(3,:),.65);
end

for w = 1:4
    for f = 1:wfaces
        wp((w-1)*wfaces+f) = patch(Wheel(w).Face(f).bcf(1,:),Wheel(w).Face(f).bcf(2,:),...
            Wheel(w).Face(f).bcf(3,:),.65,'LineStyle','none');
    end
end

% PLOT UNIT VECTORS:
unitvecs = [10 0 0;
            0 10 0;
            0 0 10];
bcfunitvecs(1) = plot3([0 unitvecs(1,1)],[0 unitvecs(2,1)],[0 unitvecs(3,1)],...
                       'Color',[0.9290 0.6940 0.1250],'LineWidth',3);
bcfunitvecs(2) = plot3([0 unitvecs(1,2)],[0 unitvecs(2,2)],[0 unitvecs(3,2)],...
                       'Color',[0.8500 0.3250 0.0980],'LineWidth',3);
bcfunitvecs(3) = plot3([0 unitvecs(1,3)],[0 unitvecs(2,3)],[0 unitvecs(3,3)],...
                       'Color',[0 0.4470 0.7410],'LineWidth',3);

% FORMAT AXES:
view(45,5);
camlight(-135,60);
set(gcf,'Renderer','openGL'); lighting gouraud;
% set(gcf,'Renderer','zbuffer'); lighting phong;
material([.4 .1 .8 .8 .1])
grid on;
axis equal;
axis([-1 1 -1 1 -1 1]*10);
set(gca,'xticklabel',[])
set(gca,'yticklabel',[])
set(gca,'zticklabel',[])

% SET UP WHEEL RATE GAUGES:
for n=1:4
    subplot(4,4,13);
    rw(n) = patch([-1 -1 1 1]*.4+n,[0 50 50 0],[.8 .1 .2]);
end
set(gca,'XTick',[1:4]);
ymin_gauge = min(min(rxnwheel_speed.Data(:,1:4)))*(30/pi);
ymax_gauge = max(max(rxnwheel_speed.Data(:,1:4)))*(30/pi);
len_gauge  = length(ymin_gauge:ymax_gauge);
% set(gca,'YTick',[ymin_gauge:ymax_gauge]);
set(gca,'YGrid','on');
xlabel('Reaction Wheel');
ylabel('Rate (rpm)');
axis([.5 4.5 ymin_gauge ymax_gauge]);

% SET UP TILT LIMIT PLOT:
%   In this plot, radius is proportional to the angle between the K body
%   vector and the K eci vector (tilt), and direction (theta) is simply the
%   x-y direction of the tilt. Think of this plot as a top-down view of the
%   travel of the body "up" direction from inertial "up".
subplot(4,4,16);
tiltplotmax = 20;
tiltmax = 15;
theta = linspace(0,2*pi)';
patch(tiltplotmax*cos(theta),tiltplotmax*sin(theta),[1 .8 .8],'EdgeColor',[1 .8 .8]); hold on;
patch(tiltmax*cos(theta),tiltmax*sin(theta),[.8 1 .8],'EdgeColor',[.8 1 .8]);
plot(0,0,'ko');
for tilt=0:5:tiltplotmax
    plot(tilt*cos(theta),tilt*sin(theta),'k:');
end

for theta=0:45:360
    plot([0 cosd(theta)*tiltplotmax],[0 sind(theta)*tiltplotmax],'k:');
end
axis([-1 1 -1 1]*(tiltplotmax));
tiltmarker = plot(0,0,'.','Color',[0 .3 0],'MarkerSize',15);
set(gca,'XColor',[.8 .8 .8],'YColor',[.8 .8 .8],'XTick',[],'YTick',[],'XTickLabel',[],...
    'YTickLabel',[],'Color',[.8 .8 .8])
axis equal;

% INITIALIZE DATA:
yprhist  = zeros(1,3);
thist    = zeros(1,1);

% SET UP EULER ANGLE GEOMETRY:
subplot(4,4,[14:15]);
yprhistplot = plot(thist,yprhist,'LineWidth',1);
xlabel('time (sec)');
ylabel('Attitude (deg)');
legend('\psi','\theta','\phi','Location','best');
% legend('boxoff');
ymin_euler = min(min(euler.Data(:,1:3)));
ymax_euler = max(max(euler.Data(:,1:3)));
xlim([0 time.Time(end)]);
ylim([ymin_euler ymax_euler]);
grid on

% FINISHED SETUP:
drawnow;
subplot(4,4,[1:12]);

%*********************************************************************
% Update Geometry
%*********************************************************************
disp('updating...');
ii = 1;

% tic;
while(1)
    thist(ii)     = time.Time(ii);
    yprhist(ii,:) = [euler.Data(ii,1), euler.Data(ii,2), euler.Data(ii,3)];
    %q(ii,:)       = euler2q([euler.Data(ii,1) euler.Data(ii,2) euler.Data(ii,3)])';
    %disp(['q: ', num2str(q(ii,:),4)]);
    
    % ROTATION MATRIX FOR GEOMETRY ROTATIONS
    C = R1(-euler.Data(ii,3)*(pi/180))*...
        R2(-euler.Data(ii,2)*(pi/180))*...
        R3(-euler.Data(ii,1)*(pi/180));
    
    % UPDATE PLATFORM:
    for n = 1:numel(Face)
        for m = 1:size(Face(n).bcf,2)
            Face(n).icf(:,m) = C*Face(n).bcf(:,m);
        end
        set(p(n),'Vertices',Face(n).icf');
    end

    % UPDATE WHEEL SIDES:
    for w = 1:4
        for f = 1:wfaces
            for m = 1:5
                Wheel(w).Face(f).icf(:,m) = C*Wheel(w).Face(f).bcf(:,m);
            end
            set(wp((w-1)*wfaces+f),'Vertices',Wheel(w).Face(f).icf')
        end
    end
    
    % UPDATE UNIT VECTORS:
    for n = 1:3
        eciunitvecs(:,n) = C*unitvecs(:,n);
        set(bcfunitvecs(n),'XData',[0 eciunitvecs(1,n)],'YData',...
            [0 eciunitvecs(2,n)],'ZData',[0 eciunitvecs(3,n)]);
    end

    % UPDATE WHEEL GUAGES:
    for n = 1:4
        set(rw(n),'YData',[0 1 1 0].*rxnwheel_speed.Data(ii,n)*(30/pi));
    end

    % UPDATE TILT POLAR & CHECK VALUE:
    subplot(4,4,16)
    K = eciunitvecs(:,3);
    theta = atan2(K(2),K(1));
    tilt = atan2(sqrt(K(1)^2+K(2)^2),K(3))*180/pi;
    
    set(tiltmarker,'XData',tilt*cos(theta),'YData',tilt*sin(theta));
    if tilt > tiltmax
        set(tiltmarker,'Color',[.8 0 0]);
    else
        set(tiltmarker,'Color',[0 .3 0]);
    end
    
    % UPDATE EULER ANGLE HISTORY PLOT:
    for n=1:3
        set(yprhistplot(n),'XData',thist,'YData',yprhist(:,n));
    end
    
    drawnow;
    % UNCOMMENT TO SAVE SIMULATION TO VIDEO FILE:
    % F(ii) = getframe(gcf);

    if time.Time(ii) == time.Time(end)
        break;
    end
    ii = ii+1;
end
% toc
disp('Simulation complete!');

% UNCOMMENT TO SAVE SIMULATION TO VIDEO FILE:
% video = VideoWriter('RWAControl_Case5.avi','Motion JPEG AVI');
% open(video)
% writeVideo(video,F);
% close(video)