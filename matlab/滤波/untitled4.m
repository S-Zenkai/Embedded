% MATLAB Script to Parse STM32 Binary Flight Data Log (Corrected Version)
%
% This script reads a binary file ('datalog.txt') containing flight data
% recorded from an STM32. It uses typecasting to correctly handle data 
% alignment issues, ensuring accurate decoding.

clear; clc; close all;

%% 1. Configuration - 定义文件和数据结构
% -------------------------------------------------------------------------
inputFile = 'datalog.txt'; % 你的二进制日志文件名

% 定义数据包的总大小（字节）
% uint32_t (4 bytes) + 6 * int16_t (2 bytes) = 4 + 12 = 16 bytes
PACKET_SIZE = 16;

%% 2. File Reading - 读取二进制文件 (修正部分)
% -------------------------------------------------------------------------
fprintf('正在打开文件: %s\n', inputFile);

fileID = fopen(inputFile, 'rb'); % 只用 'rb' 即可，字节序在 typecast 中处理

if fileID == -1
    error('无法打开文件。请确保文件存在于当前MATLAB路径中。');
end

% 一次性读取整个文件的所有字节
fileBytes = fread(fileID, Inf, 'uint8'); % 'Inf' 读取到文件尾, 'uint8' 按字节读
fclose(fileID); % 马上关闭文件

% 计算总共有多少个完整的数据包
numPackets = floor(length(fileBytes) / PACKET_SIZE);

if numPackets == 0
    disp('文件中没有找到有效的完整数据包。');
    return;
end

% 截断文件末尾可能存在的不完整数据
fileBytes = fileBytes(1 : numPackets * PACKET_SIZE);

% 将一维的字节流重塑为一个矩阵
% 每一列代表一个完整的数据包
% 矩阵尺寸将是 16 x numPackets
dataMatrix = reshape(fileBytes, PACKET_SIZE, numPackets);

% 初始化结果矩阵
allData = zeros(numPackets, 7); % 7列: timestamp, accX,Y,Z, gyroX,Y,Z

% 循环处理每一个数据包（每一列）
for i = 1:numPackets
    packet = dataMatrix(:, i); % 取出当前数据包的16个字节
    
    % 使用 typecast 进行精确的类型转换 (小端序)
    % 1. 时间戳: 字节 1-4 -> uint32
    %    'L' 代表小端序 (Little-Endian)
    timestamp = typecast(uint8(packet(1:4)), 'uint32');
    
    % 2. IMU数据: 字节 5-16 -> 6个 int16
    imu_raw = typecast(uint8(packet(5:16)), 'int16');
    
    % 组合成一行数据
    allData(i, :) = [double(timestamp), double(imu_raw')];
end

fprintf('文件读取完毕。总共解析了 %d 条记录。\n\n', numPackets);

%% 3. Data Processing and Organization - 数据整理
% -------------------------------------------------------------------------
% (这部分无需修改)
if isempty(allData)
    disp('文件中没有找到有效数据。');
    return;
end

columnNames = {'Timestamp_ms', 'AccX', 'AccY', 'AccZ', 'GyroX', 'GyroY', 'GyroZ'};
flightData = array2table(allData, 'VariableNames', columnNames);

disp('数据已成功转换为MATLAB table。预览前5行:');
head(flightData, 5)

%% 4. Data Visualization - 数据可视化示例
% -------------------------------------------------------------------------
% (这部分无需修改)
disp('正在生成数据图表...');
figure('Name', 'IMU Raw Data Visualization', 'NumberTitle', 'off');
subplot(2, 1, 1);
plot(flightData.Timestamp_ms, flightData.AccX, 'r-', 'LineWidth', 1);
hold on;
plot(flightData.Timestamp_ms, flightData.AccY, 'g-', 'LineWidth', 1);
plot(flightData.Timestamp_ms, flightData.AccZ, 'b-', 'LineWidth', 1);
hold off;
title('Accelerometer Raw Data');
xlabel('Timestamp (ms)');
ylabel('Raw ADC Value');
legend('AccX', 'AccY', 'AccZ', 'Location', 'best');
grid on;

subplot(2, 1, 2);
plot(flightData.Timestamp_ms, flightData.GyroX, 'r-', 'LineWidth', 1);
hold on;
plot(flightData.Timestamp_ms, flightData.GyroY, 'g-', 'LineWidth', 1);
plot(flightData.Timestamp_ms, flightData.GyroZ, 'b-', 'LineWidth', 1);
hold off;
title('Gyroscope Raw Data');
xlabel('Timestamp (ms)');
ylabel('Raw ADC Value');
legend('GyroX', 'GyroY', 'GyroZ', 'Location', 'best');
grid on;

fprintf('\n脚本执行完毕！\n');