
c = [76, 87, 60, 70, 81, 84, 93, 75, 64, 71, 74, 81, 78, 76, 62];


Aeq = [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1;
       1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0;
       0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0;
       0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0;
       0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
       0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];


beq = [25; 30; 25; 17; 18; 15; 16; 14];

vlb = zeros(15, 1);  
vub = [17; 18; 15; 16; 14; 17; 18; 15; 16; 14; 17; 18; 15; 16; 14];


[x, fval] = linprog(c, [], [], Aeq, beq, vlb, vub);


if ~isempty(x)
    fprintf('最优解找到！\n');
    fprintf('目标函数最小值: %.2f\n', fval);
    fprintf('\n决策变量值:\n');
    for i = 1:length(x)
        fprintf('x%d = %.4f\n', i, x(i));
    end
    

    fprintf('\n约束验证:\n');
    constraint_check = Aeq * x;
    for i = 1:length(beq)
        fprintf('约束 %d: 要求值 = %.1f, 实际值 = %.6f, 误差 = %.6f\n', ...
                i, beq(i), constraint_check(i), abs(constraint_check(i) - beq(i)));
    end
else
    fprintf('未找到可行解！\n');
end


figure('Position', [100, 100, 1200, 600]);


subplot(1, 2, 1);
if ~isempty(x)
    bar(x);
    title('决策变量最优值', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('变量索引');
    ylabel('变量值');
    grid on;
end


subplot(1, 2, 2);
if ~isempty(x)
    bar([beq, Aeq * x]);
    title('约束条件满足情况', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('约束索引');
    ylabel('约束值');
    legend('要求值', '实际值', 'Location', 'best');
    grid on;
end

fprintf('\n问题描述:\n');
fprintf('这是一个线性规划问题，包含:\n');
fprintf('- %d 个决策变量\n', length(c));
fprintf('- %d 个等式约束\n', length(beq));
fprintf('- 变量边界约束: 下界全部为0，上界各不相同\n');