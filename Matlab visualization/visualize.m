imu_sub = rossubscriber('/imu_data');
pose_sub = rossubscriber('/pose');
front_sub = rossubscriber('/sonar_front');
fleft_sub = rossubscriber('/sonar_front_left');
fright_sub = rossubscriber('/sonar_front_right');
left_sub = rossubscriber('/sonar_left');
right_sub = rossubscriber('/sonar_right');
vel_sub = rossubscriber('/cmd_vel');

close all
figure(1)
X = [0.01];
Y = [0.01];
THETA = [0];

ob_thres = 0.5;
grid()


while 1 
    vel = receive(vel_sub);
    if vel.Linear.X == 0.0
        disp('Target Reached');
    end
    
    msg = receive(pose_sub);
    targetx = msg.Orientation.X;
    targety = msg.Orientation.Y;
    scatter(targetx,targety,'X','linewidth',2)
    

    f_msg = receive(front_sub);
    fl_msg = receive(fleft_sub);
    fr_msg = receive(fright_sub);
    l_msg = receive(left_sub);
    r_msg = receive(right_sub);
   
    ob_front =  f_msg.Range_ < ob_thres;
    ob_fleft =  fl_msg.Range_ < ob_thres;
    ob_fright = fr_msg.Range_ < ob_thres;
    ob_left =   l_msg.Range_ < ob_thres;
    ob_right =  r_msg.Range_ < ob_thres;
    
    if ob_front
        scatter(X(end)+f_msg.Range_*cos(THETA(end)),Y(end)+f_msg.Range_*sin(THETA(end)),'o','fill','black')
    end
    if ob_fleft
        scatter(X(end)+fl_msg.Range_*cos(THETA(end) + pi/4),Y(end)+fl_msg.Range_*sin(THETA(end) + pi/4),'o','fill','black')
    end
    if ob_fright
        scatter(X(end)+fr_msg.Range_*cos(THETA(end) - pi/4),Y(end)+fr_msg.Range_*sin(THETA(end) - pi/4),'o','fill','black')
    end
    if ob_left
        scatter(X(end)+l_msg.Range_*cos(THETA(end) + pi/2),Y(end)+l_msg.Range_*sin(THETA(end) + pi/2),'o','fill','black')
    end
    if ob_right
        scatter(X(end)+r_msg.Range_*cos(THETA(end) - pi/2),Y(end)+r_msg.Range_*sin(THETA(end) - pi/2),'o','fill','black')
    end
    
    X = [X msg.Position.X];
    Y = [Y msg.Position.Y];
    THETA = [THETA msg.Position.Z];
    hold on
    plot(X,Y,'b-o')
    grid()

    U = cos(THETA);
    V = sin(THETA);
    quiver(X,Y,U,V,0.5,'color',[1 0 0]);
    quiver(X(end),Y(end),targetx-X(end),targety-Y(end),0,'color',[0 1 0]);

   %pause(0.1)
end