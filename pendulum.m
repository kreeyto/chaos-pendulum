clc; clearvars; close all % housecleaning

%-------------------------------------------------------------------------%
% INITIAL PARAMETERS AND CONDITIONS %

m1 = 1; % first pendulum mass
m2 = 1; % second pendulum mass

l1 = 1; % first pendulum length
l2 = 1; % second pendulum length

th1 = pi; % first pendulum initial position
th2 = pi; % second pendulum initial position

w1 = 0; % first pendulum initial velocity
w2 = 0; % second pendulum initial velocity

%-------------------------------------------------------------------------%
while true
    vt = input('Enter the simulation time (in seconds): ','s');
    vt = str2double(vt);
    if ~isnan(vt)
        break
    end
    disp('Error: Input value must be numeric!')
    disp(' ')
end

g = 9.8; % gravitational acceleration
m = [m1 m2]; % masses of the pendulums
l = [l1 l2]; % lengths of the pendulums
th = [th1 th2]; % initial positions of the pendulums
w = [w1 w2]; % initial velocities of the pendulums

fprintf('Animation Time: %.2f seconds \n',vt)
tspan = [0 vt]; 

% start differential equations
warning('off', 'MATLAB:ode45:IntegrationTolNotMet')
y0 = [th w]; % initial conditions
[t,y] = ode45(@(t,y)pendulum(t,y,m,l,g),tspan,y0); %#ok<ASGLU>

x = zeros(length(t),2);
y_pos = zeros(length(t),2);
x(:,1) = l(1)*sin(y(:,1));
y_pos(:,1) = -l(1)*cos(y(:,1));
x(:,2) = x(:,1) + l(2)*sin(y(:,2));
y_pos(:,2) = y_pos(:,1) - l(2)*cos(y(:,2));

h = figure;
set(gcf,'Color','k')
i=1;
disp('Animating...')
traj_x = zeros(2,0);
traj_y = zeros(2,0);

while ishandle(h)
    screen_size = get(0, 'ScreenSize');
    screen_width = screen_size(3);
    screen_height = screen_size(4);
    figure_width = 1200;
    figure_height = 600;
    figure_left = (screen_width - figure_width) / 2;
    figure_bottom = (screen_height - figure_height) / 2;
    set(gcf,'Position',[figure_left figure_bottom figure_width figure_height]);
    plot([0 x(i,:)],[0 y_pos(i,:)],'o-','MarkerFaceColor','w','MarkerSize',...
         10,'LineWidth',2,'Color','w','DisplayName','Pendulums');
    traj_x = [traj_x x(i,:)']; %#ok<AGROW>
    traj_y = [traj_y y_pos(i,:)']; %#ok<AGROW>
    hold on
    colors = ['m' 'r'];
    for j = 1:2
        plot(traj_x(j,:), traj_y(j,:), colors(mod(j-1,length(colors))+1), 'LineWidth', 1, 'HandleVisibility', 'off');
    end
    hold off
    axis equal
    axis([-2.5 2.5 -2.5 2.5]) 
    set(gca,'color','k','xcolor','w','ycolor','w')
    set(gcf,'Name','Pendulums Simulation');
    title(sprintf('Time: %.2f s',t(i)),'Color','w')
    drawnow
    i=i+1;   
    if i>length(t)
        break        
    end
end

if i > length(t)
    disp('End of simulation.')
    disp('Plotting full trajectory...')
    hold on
    colors = ['m' 'r'];
    for j = 1:2
        plot(x(:,j),y_pos(:,j),colors(mod(j-1,length(colors))+1),'DisplayName',['Pendulum ' num2str(j) ' Trajectory'],'LineWidth',1);
    end
    set(gcf,'Color','k');
    set(gca,'XColor','w','YColor','w');
    set(gcf,'Name','Full Trajectory');
    main = sprintf('Full Trajectory');
    sub = sprintf('Time: %.2f s',vt);
    title({sub,main},'Color','w')
    legend('TextColor','white','Location','southoutside')
else
    disp('Premature ending of the simulation!')
    disp('Plotting partial and potential trajectory...')   
    screen_size = get(0, 'ScreenSize');
    screen_width = screen_size(3);
    screen_height = screen_size(4);
    figure_width = 1200;    
    figure_height = 600;
    figure_left = (screen_width - figure_width) / 2;
    figure_bottom = (screen_height - figure_height) / 2;
    figure('Position', [figure_left figure_bottom figure_width figure_height]);
    subplot(1,2,1)
    axis equal
    axis([-2.5 2.5 -2.5 2.5]) 
    set(gca,'Color','k')
    set(gcf,'Color','k');
    set(gca,'XColor','w','YColor','w');
    main_title = sprintf('Time: %.2f s',t(i));
    subtitle = sprintf('Trajectory Traveled');
    title({main_title,subtitle},'Color','w')
    hold on
    colors = ['m' 'r'];
    for j = 1:2
        plot(x(1:i-1,j),y_pos(1:i-1,j),colors(mod(j-1,length(colors))+1),'DisplayName',['Pendulum ' num2str(j) ' Trajectory'],'LineWidth',1);
    end
    set(gca,'Color','k');
    set(gcf,'Color','k');
    set(gca,'XColor','w','YColor','w');
    set(gcf,'Name','Partial and Potential Trajectory');
    legend('TextColor','white','Location','southoutside')
    subplot(1,2,2)
    [t_full,y_full] = ode45(@(t,y)pendulum(t,y,m,l,g),[0 vt],y0); %#ok<ASGLU>
    x_full = zeros(length(t_full),2);
    y_full_pos = zeros(length(t_full),2);
    x_full(:,1) = l(1)*sin(y_full(:,1));
    y_full_pos(:,1) = -l(1)*cos(y_full(:,1));
    x_full(:,2) = x_full(:,1) + l(2)*sin(y_full(:,2));
    y_full_pos(:,2) = y_full_pos(:,1) - l(2)*cos(y_full(:,2));
    colors = ['m' 'r'];
    for j = 1:2
        plot(x_full(:,j),y_full_pos(:,j),colors(mod(j-1,length(colors))+1),'DisplayName',['Pendulum ' num2str(j) ' Trajectory'],'LineWidth',1);
    end
    axis equal
    axis([-2.5 2.5 -2.5 2.5]) 
    main_title2 = sprintf('Time: %.2f s',vt);
    subtitle2 = sprintf('Potential Trajectory');
    title({main_title2,subtitle2},'Color','w')
    set(gca,'Color','k');
    set(gcf,'Color','k');
    set(gca,'XColor','w','YColor','w');
    legend('TextColor','white','Location','southoutside')
    disp('Finished plots!')
end

function dydt = pendulum(t,y,m,l,g) %#ok<INUSD>
    th1 = y(1);
    th2 = y(2);
    w1 = y(3);
    w2 = y(4);
    dth1dt = w1;
    dth2dt = w2;

    delta = th2 - th1;
    den1 = (m(1)+m(2))*l(1) - m(2)*l(1)*cos(delta)*cos(delta);
    den2 = (l(2)/l(1))*den1;

    dw1dt = (m(2)*l(1)*w1*w1*sin(delta)*cos(delta) + m(2)*g*sin(th2)*cos(delta) + m(2)*l(2)*w2*w2*sin(delta) - (m(1)+m(2))*g*sin(th1)) / den1;
    dw2dt = (-m(2)*l(2)*w2*w2*sin(delta)*cos(delta) + (m(1)+m(2))*(g*sin(th1)*cos(delta) - l(1)*w1*w1*sin(delta) - g*sin(th2))) / den2;

    dydt = [dth1dt; dth2dt; dw1dt; dw2dt];
end
