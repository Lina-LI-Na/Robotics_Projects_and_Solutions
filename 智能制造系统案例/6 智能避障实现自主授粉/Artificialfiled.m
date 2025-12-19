ob=[24 27]; %the obstacle in the path
q_start=[1 1]; %the beginning position
q_end=[44 54]; %the target of the robot
ob_R=5; %the radius of the obstacle will effect the path and the robot will conduct circumvention
k=2;   %Gravitational gain coefficient                                       
m=600; %Repulsion gain coefficient
L=1;   %length of each step                                                                            
t_max=1000; %the max step of the path.when the number of calculations is exceed, the program will stop
deta=5; %the range of the robot to arrive the target
Q=q_start; %take the begining position into the current position

for i=1:t_max
    disp(['step',num2str(i),'arriving',num2str(i/t_max*100),'%'])
    X=Q(end,:);                           
    R=sqrt((X(1)-q_end(1,1))^2+(X(2)-q_end(1,2))^2);                    
    U1=q_end-X;
    F_Y=k*R*U1/sqrt(U1(1)^2+U1(2)^2); 
    U2=ob-X;
    S=sqrt((X(1)-ob(1,1))^2+(X(2)-ob(1,2))^2);
    if S<ob_R
        F_c=m*((5-S)/5)*U2/sqrt(U2(1)^2+U2(2)^2);
    else
        F_c=[0,0]
    end   
    F_Z=F_Y-F_c;                                                                          
    X_new=X+F_Z/sqrt(F_Z(1)^2+F_Z(2)^2)*L;                                
    Q=[Q;X_new];                                                                                   
    if sqrt((q_end(1)-X_new(1))^2+(q_end(2)-X_new(2))^2)<deta    
        Q=[Q;q_end];
        plot(Q(:,1),Q(:,2),'-r');
        break
    end
    axis equal
    plot(q_start(:,1),q_start(:,2),'.b','MarkerSize',20);
    hold on
    plot(q_end(:,1),q_end(:,2),'.g','MarkerSize',20);
    hold on
    plot(ob(:,1),ob(:,2),'.k','MarkerSize',20);
    hold on
    plot(Q(:,1),Q(:,2),'-r');
    pause(0.1)
    r=ob_R;
    theta=0:pi/100:2*pi;
    x=ob(1)+r*cos(theta);
    y=ob(2)+r*sin(theta); 
    plot(x,y,'k:');
end


