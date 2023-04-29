clc; clearvars; close all % housecleaning
%-------------------------------------------------------------------------%
% INITIAL PARAMETERS AND CONDITIONS %

% inner to outer
% m1 = 1; % first pendulum mass
% m2 = 1; % second pendulum mass
% m3 = 1; % third pendulum mass
% 
% l1 = 1; % first pendulum length
% l2 = 1; % second pendulum length
% l3 = 1; % third pendulum length
% 
% th1 = pi; % first pendulum initial position
% th2 = pi; % second pendulum initial position
% th3 = pi; % third pendulum initial position
% 
% w1 = 0; % first pendulum v0
% w2 = 0; % second pendulum v0
% w3 = 0; % third pendulum v0

%-------------------------------------------------------------------------%
fprintf(['This code is a computer program that simulates the movement of three pendulums \n', ...
'connected to each other. Imagine two weights hanging from three ropes, one rope \n', ...
'tied to the other in sequence. When you release the weights, they start to move forward and backward. \n', ...
'This program shows how the pendulums move over time. First, the program asks for how long you \n', ...
'want the simulation to run. After that, the program solves differential equations to discover how the pendulums \n', ...
'behave and displays an animation of the pendulums moving. At the end, a graph is shown with its trajectory. \n', ...
'If the simulation is completed, a single graph is plotted with the full trajectory. On the other side, \n', ...
'if the simulation is ended prematurely, two graphs are plotted: one with the trajectory up to where the \n', ...
'simulation was ended, and one with the full potential trajectory. \n'])
disp(' ')
while true
    vt = input('Enter the simulation time (in seconds): ','s');
    vt = str2double(vt);
    if ~isnan(vt)
        break
    end
    disp('Error: Input value must be numeric!')
    disp(' ')
end

n = input('Enter the number of pendulums: ');

g = 9.8; % gravitational acceleration
m = ones(1,n); % masses of the pendulums
l = ones(1,n); % lengths of the pendulums
th = pi*ones(1,n); % initial positions of the pendulums
w = zeros(1,n); % initial velocities of the pendulums

fprintf('Animation Time: %.2f seconds \n',vt)
tspan = [0 vt]; 

% start differential equations
warning('off', 'MATLAB:ode45:IntegrationTolNotMet')
y0 = [th w]; % initial conditions
[t,y] = ode45(@(t,y)pendulum(t,y,m,l,g,n),tspan,y0); %#ok<ASGLU>

x = zeros(length(t),n);
y = zeros(length(t),n);
for i = 1:n
    if i == 1
        x(:,i) = l(i)*sin(y(:,i));
        y(:,i) = -l(i)*cos(y(:,i));
    else
        x(:,i) = x(:,i-1) + l(i)*sin(y(:,i));
        y(:,i) = y(:,i-1) - l(i)*cos(y(:,i));
    end
end

% x1 = l1*sin(y(:,1));
% y1 = -l1*cos(y(:,1));
% x2 = x1 + l2*sin(y(:,3));
% y2 = y1 - l2*cos(y(:,3));
% x3 = x2 + l3*sin(y(:,5));
% y3 = y2 - l3*cos(y(:,5));

h = figure;
set(gcf,'Color','k')
i=1;
disp('Animating...')
traj_x = zeros(n,0);
traj_y = zeros(n,0);
while ishandle(h)
    screen_size = get(0, 'ScreenSize');
    screen_width = screen_size(3);
    screen_height = screen_size(4);
    figure_width = 1200;
    figure_height = 600;
    figure_left = (screen_width - figure_width) / 2;
    figure_bottom = (screen_height - figure_height) / 2;
    set(gcf,'Position',[figure_left figure_bottom figure_width figure_height]);
    pen = plot([0 x(i,:)],[0 y(i,:)],'o-','MarkerFaceColor','w','MarkerSize',...
         10,'LineWidth',2,'Color','w','DisplayName','Pendulums');
    traj_x = [traj_x x(i,:)']; %#ok<AGROW>
    traj_y = [traj_y y(i,:)']; %#ok<AGROW>
    hold on
    colors = ['m' 'r' 'b' 'g' 'c'];
    for j = 1:n
        plot(traj_x(j,:), traj_y(j,:), colors(mod(j-1,length(colors))+1), 'LineWidth', 3);
    end
    hold off
    axis equal
    axis([-3.5 3.5 -3.5 3.5])
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
    colors = ['m' 'r' 'b' 'g' 'c'];
    for j = 1:n
        plot(x(:,j),y(:,j),colors(mod(j-1,length(colors))+1),'DisplayName',['Pendulum ' num2str(j) ' Trajectory'],'LineWidth',3);
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
    axis([-3.5 3.5 -3.5 3.5])
    set(gca,'Color','k')
    set(gcf,'Color','k');
    set(gca,'XColor','w','YColor','w');
    main_title = sprintf('Time: %.2f s',t(i));
    subtitle = sprintf('Trajectory Traveled');
    title({main_title,subtitle},'Color','w')
    hold on
    colors = ['m' 'r' 'b' 'g' 'c'];
        for j = 1:n
            plot(x(1:i-1,j),y(1:i-1,j),colors(mod(j-1,length(colors))+1),'DisplayName',['Pendulum ' num2str(j) ' Trajectory'],'LineWidth',3);
        end
    set(gca,'Color','k');
    set(gcf,'Color','k');
    set(gca,'XColor','w','YColor','w');
    set(gcf,'Name','Partial and Potential Trajectory');
    legend('TextColor','white','Location','southoutside')
    subplot(1,2,2)
    [t_full,y_full] = ode45(@(t,y)pendulum(t,y,m,l,g,n),[0 max(tspan)],y0); %#ok<ASGLU>
    x_full = zeros(length(t_full),n);
    y_full = zeros(length(t_full),2*n-1);
    for j = 1:n
        if j == 1
            x_full(:,j) = l(j)*sin(y_full(:,j));
            y_full(:,j) = -l(j)*cos(y_full(:,j));
        else
            x_full(:,j) = x_full(:,j-1) + l(j)*sin(y_full(:,2*j-1));
            y_full(:,j) = y_full(:,j-1) - l(j)*cos(y_full(:,2*j-1));
        end
    end
    colors = ['m' 'r' 'b' 'g' 'c'];
    for j = 1:n
        plot(x_full(:,j),y_full(:,j),colors(mod(j-1,length(colors))+1),'DisplayName',['Pendulum ' num2str(j) ' Trajectory'],'LineWidth',3);
    end
    axis equal
    axis([-3.5 3.5 -3.5 3.5])
    main_title2 = sprintf('Time: %.2f s',max(tspan));
    subtitle2 = sprintf('Potential Trajectory');
    title({main_title2,subtitle2},'Color','w')
    set(gca,'Color','k');
    set(gcf,'Color','k');
    set(gca,'XColor','w','YColor','w');
    legend('TextColor','white','Location','southoutside')
    disp('Finished plots!')
end

function dydt = pendulum(t,y,m,l,g,n) %#ok<INUSL>
    th = y(1:n);
    w = y(n+1:end);
    dthdt = w;
    dwdt = zeros(n,1);
    for i = 1:n
        if i == 1
            if isempty(m(1:i+1))
                sum_m_1_i_plus_1 = 0;
            else
                sum_m_1_i_plus_1 = sum(m(1:i+1));
            end
            if isempty(m(1:i))
                sum_m_1_i = 0;
            else
                sum_m_1_i = sum(m(1:i));
            end
            dwdt(i) = (-m(i+1)*l(i)*w(i)^2*sin(th(i+1)-th(i))*cos(th(i+1)-th(i)) + m(i+1)*g*sin(th(i+1))*cos(th(i+1)-th(i)) + m(i+1)*l(i+1)*w(i+1)^2*sin(th(i+1)-th(i)) - sum_m_1_i_plus_1*g*sin(th(i))) / (sum_m_1_i*l(i) - m(i+1)*l(i)*cos(th(i+1)-th(i))^2);
        elseif i < n
            if isempty(m(1:i))
                sum_m_1_i = 0;
            else
                sum_m_1_i = sum(m(1:i));
            end
            dwdt(i) = (-m(i+1)*l(i)*w(i)^2*sin(th(i+1)-th(i))*cos(th(i+1)-th(i)) + sum_m_1_i*g*sin(th(i-1))*cos(th(i-1)-th(i)) - sum_m_1_i*l(i-1)*w(i-1)^2*sin(th(i-1)-th(i)) - sum_m_1_i*g*sin(th(i)) + m(i+1)*g*sin(th(i+1))*cos(th(i+1)-th(i)) + m(i+1)*l(i+1)*w(i+1)^2*sin(th(i+1)-th(i))) / (sum_m_1_i*l(i) - m(i+1)*l(i)*cos(th(i+1)-th(i))^2);
        else
            if isempty(m(1:i-1))
                sum_m_1_i_minus_1 = 0;
            else
                sum_m_1_i_minus_1 = sum(m(1:i-1));
            end
            if isempty(m(2:i))
                sum_m_2_i = 0;
            else
                sum_m_2_i = sum(m(2:i));
            end
            dwdt(i) = (sum_m_1_i_minus_1*g*sin(th(n-2))*cos(th(n-2)-th(n-1)) - sum_m_1_i_minus_1*l(n-2)*w(n-2)^2*sin(th(n-2)-th(n-1)) - sum_m_2_i*g*sin(th(n-1))) / (sum_m_2_i*l(n-1));
        end
    end
dydt = [dthdt; dwdt];
end

