%***************************************
%Author: Yiming OU   ——2021.06.21
%Filename: main_Astar.m
%***************************************
clear;
clc;

% 初始化地图
pic = imread('map.png');  %读取图像
pic_gray = rgb2gray(pic);
thresh = 0.9;  % 根据实际需求取值
map = imbinarize(pic_gray, thresh); % thresh=0.5 表示将灰度128以下的像素全部变为黑色，灰度在128以上的像素全部变为白色。
map = imcomplement(map);   %二值取反

%起点坐标和目标点坐标[x, y]，分别对应列序号和行序号
q_group = [[110,40]; [45,340]; [404,71]; [216,308]; [228,419]];
[size_q_group, ~] = size(q_group);
%% 路径规划
edges = cell(size_q_group, size_q_group);
path = cell(size_q_group, size_q_group);
vertices = cell(size_q_group, size_q_group);

tic

for ii = 1 : size_q_group
    for jj = (ii + 1) : size_q_group

        [edges{ii,jj}, path{ii,jj}, vertices{ii,jj}] = Astar(map, q_group(ii,:), q_group(jj,:));%A*算法

    end
end

toc

for ii = 2 : size_q_group
    for jj = 1 : (ii - 1)
        path{ii,jj} = path{jj,ii};
    end
end
%% 画图
imshow(int32(1 - map), []);
pathOrder = [1,2,5,4,3];
p_color = {'g*','b*','c*','m*'};
l_color = {'r.','g.','m.','c.','b.'};
title('A*');


hold on;

%Draw initial points
for  ii = 1 : size(q_group,1)

    if ii == 1 
        plot(q_group(ii,1), q_group(ii,2), 'rp', 'linewidth', 1);
    else
        plot(q_group(ii,1), q_group(ii,2), p_color{ii - 1}, 'linewidth', 1);
    end

end

%Draw paths
for ii = 1 : 4
    [pathCount, ~] = size(path{pathOrder(ii),pathOrder(ii + 1)});

    for nn = 1 : pathCount - 1
        plot(path{pathOrder(ii),pathOrder(ii + 1)}(nn, 1), path{pathOrder(ii),pathOrder(ii + 1)}(nn, 2), l_color{ii}, 'linewidth', 1);

    end
end

[pathCount, ~] = size(path{3,1});

for nn = 1 : pathCount - 1
plot(path{3,1}(nn, 1), path{3,1}(nn, 2), 'b.', 'linewidth', 1);
end