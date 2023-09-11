% 提取数据（假设结构体数组名为 data）
n = 300;% 假设结构体数组的长度为 300
avgTime = zeros(1, n);
success = zeros(1, n);
numSuccess = 0;
numFail = 0;
numCollided = 0;

for i = 1:n
    avgTime(i) = quantStatuses{i}.AvgTime;
    if quantStatuses{i}.Success == 1
        numSuccess = numSuccess + 1;
    else
        if quantStatuses{i}.collided == 1
            numCollided = numCollided + 1;
        else
            numFail = numFail + 1;
        end
    end
end

successRate = numSuccess / 300;
collideRate = numCollided / 300;

disp(collideRate);
disp(successRate);
% 创建子图
figure;

% 第一个子图显示AvgTime
subplot(2, 1, 1);
plot(1:n, avgTime);
xlabel('Index');
ylabel('AvgTime');
title('AvgTime');


% 第二个子图：统计 Success 的柱状图
subplot(2, 1, 2); % 2 行 1 列的第 2 个子图
bar([numSuccess, numFail, numCollided]); % 绘制两个柱子，一个表示成功次数，另一个表示失败次数
xticks([1, 2, 3]);
xticklabels({'Success', 'Fail', 'Collided'});
ylabel('Count');
title('Success Count');