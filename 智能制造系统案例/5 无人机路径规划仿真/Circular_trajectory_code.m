% 定义圆的半径和中心
r = 10; % 半径
center = [0, 0]; % 圆心

% 定义无人机的初始位置
drone_pos = [r, 0];

% 定义无人机的速度（单位：弧度/秒）
speed = 0.1;

% 创建一个新的图形窗口
figure;

% 创建一个无限循环，可以通过按Ctrl+C来停止
while true
    % 清除当前图形窗口的内容
    clf;
    
    % 绘制圆形轨迹
    theta = linspace(0, 2*pi, 100);
    x = center(1) + r*cos(theta);
    y = center(2) + r*sin(theta);
    plot(x, y);
    hold on;
    
    % 绘制无人机的当前位置
    plot(drone_pos(1), drone_pos(2), 'ro');
    
    % 更新无人机的位置
    angle = atan2(drone_pos(2) - center(2), drone_pos(1) - center(1));
    angle = angle + speed;
    drone_pos = center + r*[cos(angle), sin(angle)];
    
    % 强制Matlab立即更新图形窗口
    drawnow;
    
    % 暂停一小段时间，使得无人机的移动看起来更平滑
    pause(0.01);
end