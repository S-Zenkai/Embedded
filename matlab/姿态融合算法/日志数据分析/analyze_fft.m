% analyze_fft_advanced.m
%
% 改进版FFT分析脚本，包含一个可重用的分析函数，
% 并能生成滤波前后的频域对比图。
%
% 要求:
%   在运行此脚本之前，必须先运行 'parse_datalog.m'，
%   确保MATLAB工作区中已存在 'logData' 结构体变量。
%
% 作者: AI
% 版本: 2.0.0
% 日期: 2023/10/28

%% --- 脚本主程序 ---

% --- 0. 初始化与环境检查 ---
clc;
% clearvars -except logData; % 清理工作区，但保留logData
close all;

fprintf('开始进行高级FFT对比分析...\n');

% 检查 'logData' 变量是否存在于工作区
if ~exist('logData', 'var')
    error(['错误: 未在工作区中找到 "logData" 变量。\n' ...
           '请先运行 "parse_datalog.m" 脚本来加载数据。']);
end

% --- 1. 用户配置 ---
% 在这里选择您想要对比分析的信号。
% IMU信号选项: 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z'
signal_field_to_analyze = 'acc_z'; 


% --- 2. 数据分析 ---
% 调用封装好的函数，分别获取滤波前后的频域数据
fprintf('\n--- 正在分析滤波前的数据 (%s) ---\n', signal_field_to_analyze);
[f_before, P1_before, Fs_before] = analyze_signal_fft(logData.imu_cal, signal_field_to_analyze);
% [f_before, P1_before, Fs_before] = analyze_signal_fft(logData.imu_raw, signal_field_to_analyze);

fprintf('\n--- 正在分析滤波后的数据 (%s) ---\n', signal_field_to_analyze);
[f_after, P1_after, Fs_after] = analyze_signal_fft(logData.imu_filter, signal_field_to_analyze);


% --- 3. 结果可视化 ---
if isempty(f_before) || isempty(f_after)
    fprintf('\n数据不足，无法生成对比图。\n');
    return;
end

fprintf('\n--- 正在生成对比图 ---\n');

% 绘图1: 子图对比 (Subplot Comparison)
figure('Name', ['FFT 对比 (子图): ' signal_field_to_analyze], 'NumberTitle', 'off');

% 上子图: 滤波前
subplot(2, 1, 1);
plot(f_before, P1_before, 'b-', 'LineWidth', 1.2);
title(['滤波前 (Calibrated) - ' strrep(signal_field_to_analyze, '_', ' ')]);
xlabel('频率 (Hz)');
ylabel('幅值');
grid on;
axis tight;
legend(sprintf('Fs = %.1f Hz', Fs_before));

% 下子图: 滤波后
subplot(2, 1, 2);
plot(f_after, P1_after, 'r-', 'LineWidth', 1.2);
title(['滤波后 (Filtered) - ' strrep(signal_field_to_analyze, '_', ' ')]);
xlabel('频率 (Hz)');
ylabel('幅值');
grid on;
axis tight;
legend(sprintf('Fs = %.1f Hz', Fs_after));


% 绘图2: 叠加对比 (Overlay Comparison)
figure('Name', ['FFT 对比 (叠加): ' signal_field_to_analyze], 'NumberTitle', 'off');
hold on;
plot(f_before, P1_before, 'b-', 'DisplayName', '滤波前 (Calibrated)');
plot(f_after, P1_after, 'r-', 'LineWidth', 1.5, 'DisplayName', '滤波后 (Filtered)');
hold off;

title(['滤波前后频谱对比 - ' strrep(signal_field_to_analyze, '_', ' ')]);
xlabel('频率 (Hz)');
ylabel('幅值');
grid on;
legend('show', 'Location', 'northeast');
axis tight;

fprintf('所有图表生成完毕！\n');


%% --- 核心分析函数 ---
function [f, P1, Fs] = analyze_signal_fft(source_data, signal_field)
    % analyze_signal_fft: 接收时域数据结构，返回频域数据
    %
    % 输入:
    %   source_data  - 包含 .timestamp 和指定 signal_field 的结构体数组
    %                  (例如 logData.imu_cal)
    %   signal_field - 要分析的信号字段名 (例如 'gyro_y')
    %
    % 输出:
    %   f            - 频率向量 (Hz)
    %   P1           - 对应于f的单边频谱幅值
    %   Fs           - 计算出的平均采样频率 (Hz)

    % --- 初始化输出变量，以防出错 ---
    f = [];
    P1 = [];
    Fs = 0;

    % --- 检查输入数据 ---
    if isempty(source_data) || ~isfield(source_data, 'timestamp') || ~isfield(source_data, signal_field)
        fprintf('警告: 输入数据为空或缺少必要的字段 ("timestamp" 或 "%s")。\n', signal_field);
        return;
    end
    
    % --- 数据准备与采样率计算 ---
    timestamps_ms = [source_data.timestamp];
    signal = [source_data.(signal_field)]; % 动态字段提取

    if length(timestamps_ms) < 2
        fprintf('警告: 样本数量不足 (<2)，无法进行FFT分析。\n');
        return;
    end
    
    time_s = double(timestamps_ms - timestamps_ms(1)) / 1000.0;
    dt_all = diff(time_s);
    dt_avg = mean(dt_all);
    Fs = 1 / dt_avg;

    % 检查和报告采样率稳定性
    dt_std_dev = std(dt_all);
    jitter_percentage = (dt_std_dev / dt_avg) * 100;
    
    fprintf('平均采样频率 (Fs): %.2f Hz\n', Fs);
    fprintf('采样周期抖动: %.2f%%\n', jitter_percentage);
    if jitter_percentage > 5
        fprintf('警告: 采样时间抖动较大 (>5%%)，FFT结果准确性可能受影响。\n');
    end

    % --- 执行FFT计算 ---
    L = length(signal);
    y = signal - mean(signal); % 移除直流分量
    
    Y = fft(y);
    P2 = abs(Y/L);
    P1 = P2(1:floor(L/2)+1);
    P1(2:end-1) = 2*P1(2:end-1);
    
    f = Fs*(0:(L/2))/L;
end