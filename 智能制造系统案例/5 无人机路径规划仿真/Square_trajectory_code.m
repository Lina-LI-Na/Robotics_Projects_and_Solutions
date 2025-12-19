% 初始化
clear; clc; close all;

% 定义方形的四个顶点 [x1,y1; x2,y2; x3,y3; x4,y4]
vertices = [0,0; 10,0; 10,10; 0,10];

% 定义无人机的速度
v = 0.1;

% 定义时间间隔
dt = 0.01;

% 初始化无人机的位置
pos = vertices(1,:);

% 初始化无人机的轨迹
trajectory = pos;

% 无人机沿着方形轨迹飞行
for i = 1:size(vertices, 1)
    % 计算当前顶点到下一个顶点的距离
    next_vertex = vertices(mod(i, size(vertices, 1)) + 1, :);
    d = norm(next_vertex - pos);
    
    % 计算无人机需要飞行的步数
    steps = round(d / (v * dt));
    
    % 计算每一步的位置
    for j = 1:steps
        direction = (next_vertex - pos) / norm(next_vertex - pos);
        pos = pos + v * dt * direction;
        trajectory = [trajectory; pos];
    end
end

% 绘制无人机的轨迹
plot(trajectory(:,1), trajectory(:,2), 'b-');
hold on;
plot(vertices(:,1), vertices(:,2), 'ro--'); % 绘制方形顶点
grid on;
xlabel('x');
ylabel('y');
title('无人机方形轨迹');
axis equal;