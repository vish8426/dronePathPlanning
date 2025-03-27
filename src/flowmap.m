clear
clc

tic
%% Generate map

%map parameters
MaxWind = 1;
gridSize = 30;
dimensions = 100;
kernelWidth = 15;

%sample random fields
xs = linspace(0,dimensions,gridSize);
[X,Y] = meshgrid(xs);
K = ones(kernelWidth,kernelWidth);
U = randn(gridSize, gridSize);
V = randn(gridSize, gridSize);

%smooth out via convolution
Uc = conv2(U,K,'same');
Vc = conv2(V,K,'same');

%set maximum wind speed
mv = max(max(sqrt(Uc.^2+Vc.^2)));

Uc = Uc*MaxWind/mv;
Vc = Vc*MaxWind/mv;

figure(1);
quiver(X,Y,Uc,Vc);
title('UAV Path Visualisation');
axis equal

xlabel('x');
ylabel('y');

%% Interpolate mapwindspeed
%example on getting an arbitrary point

%xquery = 32.7;
%yquery = 61.8;
%figure(1);hold on;plot(xquery,yquery,'ro')
%hold off
%uquery = interp2(X,Y,Uc, xquery,yquery);
%vquery = interp2(X,Y,Vc, xquery,yquery);


%% System Design Specifications
% Sampling K random points on the map

%set a limit to the system for number of points (atleast two points)
n = 2:25;

%set a limit to the x & y values that can be plotted
x = dimensions;
y = dimensions;

%generate a random number of points
K = randsample(n,1);

%define two arrays to store all x and y coordinates of the points
%xarray = zeros(1,K);
%yarray = zeros(1,K);

b = 2;
flag = 0;

fprintf('total points in the system are %g\n',K);
%% Implementation of Dijkstra Algorithm & Approximating Cost of Travel
%ask the user for a end point to travel to
prompt = 'What is the desired point to travel? ';

%check for valid input from user
while flag == 0
    
    des_pt = input(prompt);
    
    if(des_pt > 1 && des_pt <= K)
    
        flag = flag + 1;
        
    else
        
        fprintf('Invalid Input\nWhat is the desired point to travel? ');
    end
end

%for loop to go through the number of points
for points = 1:K
    
    %get a random x value for each point
    xpoint = randsample(x,1);
    
    %get a random y value for each point 
    ypoint = randsample(y,1);
    
    %place the x and y coordinate in the matrix 
    xarray(1,points) = xpoint; 
    yarray(1,points) = ypoint;
    
    %plot the point onto the flowmap
    figure(1);hold on;plot(xpoint,ypoint,'ro');
    
    %estimatation of wind speed values at each point along path
    xquery = xpoint;
    yquery = ypoint;
    
    uquery(1,points) = interp2(X,Y,Uc, xquery,yquery);
    vquery(1,points) = interp2(X,Y,Vc, xquery,yquery);
    
    if(points == 1)
        
        %label a start position/point
        text(xpoint,ypoint+4,'start (1)');
        
    %elseif(points == K)
        
        %text(xpoint,ypoint+5,'end');
    else
        
        %label all the points using integers
        text(xpoint, ypoint+5,int2str(b));
        b = b + 1;
    end
    
    if(uquery(1,points) > 0 && vquery(1,points) > 0)
        
        dir_wind(1,points) = "NE";
        
    elseif(uquery(1,points) < 0 && vquery(1,points) > 0)
        
        dir_wind(1,points) = "NW";
        
   elseif(uquery(1,points) < 0 && vquery(1,points) < 0)
        
        dir_wind(1,points) = "SW"; 
        
    elseif(uquery(1,points) > 0 && vquery(1,points) < 0)
        
        dir_wind(1,points) = "SE";
    end
end

hold off;

%disp(xarray);
%disp(yarray);

xlength = size(xarray);
ylength = size(yarray);

%disp(xlength);
%disp(ylength);

a = 1;
count = 0;
des_count = 1;

i = 1;
j = 1;

%calculate an estimated wind speed 
for vel_w = 1:K
   
    cal_Vw(1,vel_w) = sqrt((uquery(1,vel_w)^2 + vquery(1,vel_w)^2));
    
    fprintf('point %g wind speed = %.2f\twith a direction %s\n',vel_w,cal_Vw(1,vel_w),dir_wind(1,vel_w));
end

fprintf('\n');

%draw a line for each point that has been created
for pre_val = 1:K
    
    i = pre_val;
    
    for val = a:K
        
        if(val < K)
            
            %plot a line from each K point to the next - creating paths
            line([xarray(1,pre_val) xarray(1,val+1)],[yarray(1,pre_val) yarray(1,val+1)],'color','b');
            
            j = val + 1;
            count = count + 1;
            
            length_line = [xarray(1,pre_val),yarray(1,pre_val);xarray(1,val+1),yarray(1,val+1)];
            vector_val(1,count) = pdist(length_line,'euclidean');
            
            %save the direction of each plotted path             
            grad_path(1,count) = (yarray(1,val+1) - yarray(1,pre_val)) ./ (xarray(1,val+1) - xarray(1,pre_val));

            if(((yarray(1,val+1) - yarray(1,pre_val)) == 0))
                if(xarray(1,val+1) > xarray(1,pre_val))

                    dir_path(1,count) = "E";
                elseif(xarray(1,val+1) < xarray(1,pre_val))

                    dir_path(1,count) = "W";
                end
            elseif(((xarray(1,val+1) - xarray(1,pre_val)) == 0))
                if(yarray(1,val+1) < yarray(1,pre_val))

                    dir_path(1,count) = "N";
                elseif(yarray(1,val+1) > yarray(1,pre_val))

                    dir_path(1,count) = "S";
                end
            end
            if((xarray(1,val+1) > xarray(1,pre_val)))
                if(yarray(1,val+1) < yarray(1,pre_val))

                    dir_path(1,count) = "SW";
                elseif(yarray(1,val+1) > yarray(1,pre_val))

                    dir_path(1,count) = "NE";
                end
            elseif((xarray(1,val+1) < xarray(1,pre_val)))
                if(yarray(1,val+1) < yarray(1,pre_val))

                    dir_path(1,count) = "SE";
                elseif(yarray(1,val+1) > yarray(1,pre_val))

                    dir_path(1,count) = "NW";
                end
            end
            
            %comparing wind directions with path direction
            if(grad_path(1,count) < 0 && (dir_wind(1,pre_val) == "NW" || dir_wind(1,pre_val) == "SE"))
                
                if(uquery(1,pre_val) < 0 && yarray(val+1) < yarray(pre_val))
                    
                    cal_Vw(1,pre_val) = cal_Vw(1,pre_val)*-1;
                    
                elseif(uquery(1,pre_val) > 0 && yarray(val+1) > yarray(pre_val))
                    
                    cal_Vw(1,pre_val) = cal_Vw(1,pre_val)*-1;
                end
                
            elseif(grad_path(1,count) > 0 && (dir_wind(1,pre_val) == "NE" || dir_wind(1,pre_val) == "SW"))
                
                if(uquery(1,pre_val) < 0 && yarray(val+1) > yarray(pre_val))
                    
                    cal_Vw(1,pre_val) = cal_Vw(1,pre_val)*-1;
                    
                elseif(uquery(1,pre_val) > 0 && yarray(val+1) < yarray(pre_val))
                    
                    cal_Vw(1,pre_val) = cal_Vw(1,pre_val)*-1;
                end
            end
            
            %calculate travel cost
            cost_travel(1,count) = vector_val(1,count) ./ (1 + (cal_Vw(1,pre_val)));
            
            %label the length of each of the lines
            fprintf('%g - %g = %.2f\twith gradient = %.2f',i,j,vector_val(1,count),grad_path(1,count));
            fprintf('\twith direction %s',dir_path(1,count));
            fprintf('\tand cost to travel = %.2f\n',cost_travel(1,count));
            
            if(des_pt == j)
                                
                des_cost_travel(1,des_count) = cost_travel(1,count);
                des_count = des_count + 1;
            end
            
            j = 0;
        end
    end
    
    if(a < K)
        
        a = a + 1;
    else
        
        a = K;
    end
end

fprintf('\n');

%calculate an estimated wind velocity 
for d = 1:K
    
    fprintf('point %g wind velocity = %.2f\n',d,cal_Vw(1,d));
end

%calculate the shortest travel cost value
min_cost_travel = min(des_cost_travel);
fprintf('the minimum cost to travel to point %g is %.2f\n',des_pt,min_cost_travel);

%% Visualisation of Optimal Path & Timing

t = toc;
fprintf('\n%.2f is the total time of execution.\n',t);

ii = 1e4;
cost = zeros(ii,1);
for i = 1:ii
    cost(i) = path_cost(cost_travel);
end

figure(2);histogram(cost,50);

%% Value Function
%plot the value function
figure(3);imshow(flipud(dimensions),'InitialMagnification',3000)
figure(3);imshow(flipud(cost_travel),'InitialMagnification',3000)
hold on;

figure(3);imshow(flipud(V)/max(max(V)),'InitialMagnification',4000,'Interpolation','bilinear')
colormap copper