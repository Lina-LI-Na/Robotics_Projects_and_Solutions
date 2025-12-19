%***************************************
%Author: Yiming OU   ——2021.06.21
%Filename: Astar.m
%***************************************
function [edges, path, vertices] = Astar(map, q_start, q_goal)

    if nargin < 3
        error('输入参数不足，请提供地图、起点和终点信息');
    end
    
    checkInput(map, q_start, q_goal);
    
    [~,mapWidth] = size(map); %地图的宽度
    
    list = zeros(size(map)); %0代表未探索，1代表open，2代表close
    
    index_start = index(q_start, mapWidth);  %起始点序号
    index_goal = index(q_goal, mapWidth);  %目标点序号
    
    edges = int32.empty(0, 2);  %1，2列分别为子节点和父节点序号
    
    vertices = q_start;  %存放所有遍历过的节点
    
    k = numel(map);  %总节点数
    q_father = q_start;  
    
    %存放节点估计代价，各行分别为节点序号、到起点距离、到目标点曼哈顿距离、估计代价
    CostTable = [index_start; 0; Mdistance(q_start, q_goal); Mdistance(q_start, q_goal)];
    
    for ii = 1 : k
        costOfQNew = Inf;
        
        index_father = index(q_father, mapWidth);
        list(q_father(2), q_father(1)) = 2;
        
        %遍历父节点周围的可到达节点
        for i = -1 : 1
            for j = -1 : 1
                
                q_son = q_father + [i, j]; 

                index_son = index(q_son, mapWidth);      
                
                if ~isNodeFree(map, q_son) || list(q_son(2), q_son(1)) == 2
                    %子节点位于障碍物或CloseList中
                    continue;
%                 elseif isNodeTrapped(q_son, map, list)
%                     %子节点没有潜在的子节点
%                     continue;
                end
                
                if isequal(q_son, q_goal)
                    edges = [edges; [index_son, index_father]];
                    %找到目标点，生成路径
                    path = findPath(edges, index_start, mapWidth);
                    return;
                end
                
                %计算子节点经q_father的估计代价
                [EdOfQSon, MdOfQSon, costOfQSon] = cost(q_son, q_father, index_father, CostTable, q_goal);
                
                if list(q_son(2), q_son(1)) == 1
                    %子节点在OpenList中
                    [~, column] = find(CostTable(1, :) == index_son);
                    if costOfQSon >= CostTable(4, column)
                        costOfQSon = CostTable(4, column);
                    else
                        %更新父节点
                        [row, ~] = find(edges(:, 1) == index_son);
                        edges(row, :) = [];
                        edges = [edges; [index_son, index_father]];
                        
                        CostTable(:, column) = [];
                        CostTable = [CostTable, [index_son; EdOfQSon; MdOfQSon; costOfQSon]];
                    end
                else
                    %子节点未探索过，直接放入edges
                    edges = [edges; [index_son, index_father]];
                    
                    CostTable = [CostTable, [index_son; EdOfQSon; MdOfQSon; costOfQSon]];
                    
                    list(q_son(2), q_son(1)) = 1;
                    
                    vertices = [vertices; q_son];
                end
                
                if costOfQSon < costOfQNew
                    %找到所有可到达节点中估计代价最小者
                    q_new = q_son;
                    index_new = index_son;
                    
                    costOfQNew = costOfQSon;
                end
            end        
        end
        q_father = q_new;
    end
    
    error('未找到路径。');
end

% 输入参数检查
function checkInput(map, q_start, q_goal)

    [mapHeight, mapWidth] = size(map);
    
    if mapWidth < 1 || mapHeight < 1
        error('地图尺寸有误，请检查是否输入了正确的二维地图。');
    end
    
    [x, y] = size(q_start);
    if x ~= 1 || y ~= 2
        error('点坐标应为1×2矩阵，如：[20, 30]。');
    elseif q_start(1) < 0 || q_start(1) > mapWidth || q_start(2) < 0 || q_start(2) > mapHeight
        error('点坐标不能超出地图范围。');
    end
    
    [x, y] = size(q_goal);
    if x ~= 1 || y ~= 2
        error('点坐标应为1×2矩阵，如：[20, 30]。');
    elseif q_goal(1) < 0 || q_goal(1) > mapWidth || q_goal(2) < 0 || q_goal(2) > mapHeight
        error('点坐标不能超出地图范围。');
    end
end

%节点编号
function [in] = index(q, mapWidth)
    in = mapWidth*(q(2)-1)+q(1);
    in = int32(in);
end

%相邻可到达节点之间欧氏距离
function [Ed] = Edistance(q1, q2)
    q1_q2 = q1 - q2;
    if q1_q2(1) == 0 && q1_q2(2) == 0
        Ed = 0;
    elseif q1_q2(1) == 0 || q1_q2(2) == 0
        Ed = 10;
    else
        Ed = 14;
    end
    
    Ed = int32(Ed);
end

% 两节点之间曼哈顿距离
function [Md] = Mdistance(q1, q2)
    q1_q2 = q1 - q2;
    Md = abs(q1_q2(1)) + abs(q1_q2(2));
    Md = Md * 10;
    Md = int32(Md);
end

% 计算子节点的估计代价
function [di2start, Md2goal, co] = cost(q_son, q_father, index_father, CostTable, q_goal)

    [~, column] = find(CostTable(1, :) == index_father); %存放父节点代价信息列向量的列序号
    di2start = int32(CostTable(2, column)) + Edistance(q_son, q_father);
    Md2goal = Mdistance(q_son, q_goal);
    
    co = int32(di2start + Md2goal);

end

% 节点是否为自由区域，1代表是，0代表否
function [isFree] = isNodeFree(map, q)
    [mapHeight, mapWidth] = size(map); 
    if q(1) < 1 || q(2) < 1 || q(1) > mapWidth || q(2) > mapHeight
        isFree = 0;
    else
        isFree = ~map(q(2), q(1));
    end
end

%将节点序号转换为坐标
function coordinate = index2node(index, mapWidth)
    x = int32(mod(index, mapWidth));
    if x == 0
        x = mapWidth;
    end
    
    y = int32((index-x)/mapWidth + 1);
    
    coordinate = [x, y];
end


%找到组成路径的节点
function [path] = findPath(edges, index_start, mapWidth)

    [edge_num, ~] = size(edges);
    index_son = edges(end, 1);
    index_father = edges(end, 2);
    path = index2node(index_son, mapWidth);
    
    for i = 1 : edge_num

        [row, ~] = find(edges(:, 1) == index_father);
        
        index_son = index_father;
        path = [path; index2node(index_son, mapWidth)];
        
        index_father = edges(row, 2);
        
        if index_father == index_start
            path = [path; index2node(index_start, mapWidth)];
            
            break;
        end
    end
    
    path = flipud(path);
end

% 判断节点是否拥有子节点
% function [isN] = isNodeTrapped(q, map, list)
%    
%     for i = -1 : 1
%         for j = -1 : 1
%             
%             if ~i && ~j
%                 continue;
%             end
%             
%             q_son = q + [i, j]; 
%             if isNodeFree(map, q_son) && list(q_son(2), q_son(1)) ~= 2
%                 isN = 0;
%                 
%                 return;
%             end
%         end
%     end
%     
%     isN = 1;
% end
