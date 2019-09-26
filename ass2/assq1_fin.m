% =========
% ass2_q1.m
% =========
%
% This assignment will introduce you to the idea of first building an
% occupancy grid then using that grid to estimate a robot's motion using a
% particle filter.
% 
% There are two questions to complete (5 marks each):
%
%    Question 1: code occupancy mapping algorithm 
%    Question 2: see ass2_q2.m
%
% Fill in the required sections of this script with your code, run it to
% generate the requested plot/movie, then paste the plots into a short report
% that includes a few comments about what you've observed.  Append your
% version of this script to the report.  Hand in the report as a PDF file
% and the two resulting AVI files from Questions 1 and 2.
%
% requires: basic Matlab, 'gazebo.mat'
%
% T D Barfoot, January 2016
%
clear all;

% set random seed for repeatability
rng(1);

% ==========================
% load the dataset from file
% ==========================
%
%    ground truth poses: t_true x_true y_true theta_true
% odometry measurements: t_odom v_odom omega_odom
%           laser scans: t_laser y_laser
%    laser range limits: r_min_laser r_max_laser
%    laser angle limits: phi_min_laser phi_max_laser
%
load gazebo.mat;

% =======================================
% Question 1: build an occupancy grid map
% =======================================
%
% Write an occupancy grid mapping algorithm that builds the map from the
% perfect ground-truth localization.  Some of the setup is done for you
% below.  The resulting map should look like "ass2_q1_soln.png".  You can
% watch the movie "ass2_q1_soln.mp4" to see what the entire mapping process
% should look like.  At the end you will save your occupancy grid map to
% the file "occmap.mat" for use in Question 2 of this assignment.

% allocate a big 2D array for the occupancy grid
ogres = 0.05;                   % resolution of occ grid
ogxmin = -7;                    % minimum x value
ogxmax = 8;                     % maximum x value
ogymin = -3;                    % minimum y value
ogymax = 6;                     % maximum y value
ognx = (ogxmax-ogxmin)/ogres;   % number of cells in x direction
ogny = (ogymax-ogymin)/ogres;   % number of cells in y direction
oglo = zeros(ogny,ognx);        % occupancy grid in log-odds format
ogp = zeros(ogny,ognx);         % occupancy grid in probability format

% precalculate some quantities
numodom = size(t_odom,1);
npoints = size(y_laser,2);
angles = linspace(phi_min_laser, phi_max_laser,npoints);
dx = ogres*cos(angles);
dy = ogres*sin(angles);

% interpolate the noise-free ground-truth at the laser timestamps
t_interp = linspace(t_true(1),t_true(numodom),numodom);
x_interp = interp1(t_interp,x_true,t_laser);
y_interp = interp1(t_interp,y_true,t_laser);
theta_interp = interp1(t_interp,theta_true,t_laser);
omega_interp = interp1(t_interp,omega_odom,t_laser);
  
% set up the plotting/movie recording
vid = VideoWriter('ass2_q1.avi');
open(vid);
figure(1);
clf;
pcolor(ogp);
colormap(1-gray);
shading('flat');
axis equal;
axis off;
M = getframe;
writeVideo(vid,M);

% loop over laser scans (every fifth)
for i=1:5:size(t_laser,1)
    
    % ------insert your occupancy grid mapping algorithm here------
    
    x_current = y_laser(i,:) .* cos(angles);
    y_current = y_laser(i,:) .* sin(angles);
    
        
        delta = [cos(theta_interp(i)) -sin(theta_interp(i)); sin(theta_interp(i)) cos(theta_interp(i))]...
                *[0.1;0];
        x_interp(i) = x_interp(i) - delta(1);
        y_interp(i) = y_interp(i) - delta(2);
        

    laser_world_robo = [cos(theta_interp(i)) -sin(theta_interp(i)); sin(theta_interp(i)) cos(theta_interp(i))] *...
	[x_current; y_current];
    laser_world = laser_world_robo  + [x_interp(i); y_interp(i)];
    figure()
    hold on
        
    empty_grid = [];
    occupy_grid = [];
    for j=1:size(y_laser,2)
        if isnan(y_laser(i,j))
            continue;
        end
        
        %%%
        dx_dy = [cos(theta_interp(i)) -sin(theta_interp(i)); sin(theta_interp(i)) cos(theta_interp(i))]...
                *[dx(j);dy(j)];
        for k = 1:floor(abs(laser_world_robo(1,j))/ogres)  
            if laser_world_robo(1,j)<0
                empty = ([-k*ogres; -k*dx_dy(2)/dx_dy(1)*ogres] + [x_interp(i); y_interp(i)])/ogres;
                empty = floor(empty);
            else
                empty = ([k*ogres; k*dx_dy(2)/dx_dy(1)*ogres] + [x_interp(i); y_interp(i)])/ogres;
                empty = floor(empty);
            end
            empty_grid = [empty_grid empty];
        end
        %%%
            
% %%%%%
%         % find empty grids
%         ray_length = sqrt(dx(j)^2+dy(j)^2);
%         for indx=1:floor(y_laser(i,j)/ray_length)
%             
%             empty = [cos(theta_interp(i)) -sin(theta_interp(i)); sin(theta_interp(i)) cos(theta_interp(i))]...
%                 *[dx(j);dy(j)]*indx+[x_interp(i); y_interp(i)];
%             empty = floor(empty/ogres);
% %             plot(empty(1),empty(2),'.')
%             if indx == 1
%                 empty_grid = [empty_grid empty];
%             elseif empty(1)~=empty_grid(1,end) && empty(2)~=empty_grid(2,end)
%                 empty_grid = [empty_grid empty];
%             end
%         end
% %%%%
        
%         occupy_grid = [occupy_grid floor(laser_world_robo(:,j)/ogres)+robo_grid];
        occupy_grid = [occupy_grid floor(laser_world(:,j)/ogres)];
        if (occupy_grid(1,end)==empty_grid(1,end) &&  occupy_grid(2,end)==empty_grid(2,end))
            empty_grid(:,end) = [];
        end
 
    end
    
    % offset
    empty_grid(1,:) = empty_grid(1,:) + 140;
    empty_grid(2,:) = empty_grid(2,:) + 60;
    occupy_grid(1,:) = occupy_grid(1,:) + 140;
    occupy_grid(2,:) = occupy_grid(2,:) + 60;
    
    
    empty_grid = (unique(empty_grid','rows'))';
    occupy_grid = (unique(occupy_grid','rows'))';
    
%     figure
%     hold on
%     plot(empty_grid(1,:),empty_grid(2,:),'.');
%     plot(occupy_grid(1,:),occupy_grid(2,:),'*');
    %update metric map
    for m = 1:size(empty_grid,2)
        oglo(empty_grid(2,m),empty_grid(1,m)) = oglo(empty_grid(2,m),empty_grid(1,m)) - 1;
    end
    for n = 1:size(occupy_grid,2)
        oglo(occupy_grid(2,n),occupy_grid(1,n)) = oglo(occupy_grid(2,n),occupy_grid(1,n)) + 2;
    end
    ogp = exp(oglo)./(1+exp(oglo));
    
%     ogp(ogp>.5) = 1;
%     ogp(ogp<.5) = 0;
    

    
    
    % ------end of your occupancy grid mapping algorithm-------

    % draw the map
    clf;
    pcolor(ogp);
    colormap(1-gray);
    shading('flat');
    axis equal;
    axis off;
    
    % draw the robot
    hold on;
    x = (x_interp(i)-ogxmin)/ogres;
    y = (y_interp(i)-ogymin)/ogres;
    th = theta_interp(i);
    r = 0.15/ogres;
    set(rectangle( 'Position', [x-r y-r 2*r 2*r], 'Curvature', [1 1]),'LineWidth',2,'FaceColor',[0.35 0.35 0.75]);
    set(plot([x x+r*cos(th)]', [y y+r*sin(th)]', 'k-'),'LineWidth',2);
    
    % save the video frame
    M = getframe;
    writeVideo(vid,M);
    
    pause(0.1);
    
end

close(vid);
print -dpng ass2_q1.png

save occmap.mat ogres ogxmin ogxmax ogymin ogymax ognx ogny oglo ogp;

