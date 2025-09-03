% parse_datalog.m
%
% MATLAB脚本，用于解析由STM32四轴飞行器记录的二进制飞行数据日志文件。
% 该脚本根据 task_datalog.h 中定义的 log_packet_t 结构进行解析。
%
% 作者: Kai (基于C代码) / AI
% 版本: 1.3.0 (适配 IMU_RAW)
% 日期: 2023/10/28

%% --- 初始化 ---
clear;
clc;
close all;

fprintf('开始解析飞行数据日志...\n');

%% --- 定义常量 (与 C 代码中的 #define 保持一致) ---
% ... (这部分不变)
LOG_PACKET_HEADER_1 = uint8(0xA5);
LOG_PACKET_HEADER_2 = uint8(0x5A);
%%% 新增 %%%
LOG_ID_IMU_RAW    = uint8(0x00); 
LOG_ID_IMU_CAL    = uint8(0x01);
LOG_ID_IMU_FILTER = uint8(0x02);
LOG_ID_ATTITUDE   = uint8(0x03);
LOG_ID_GPS        = uint8(0x04);
LOG_ID_STATUS     = uint8(0x05);
PACKET_SIZE = 33; 

%% --- 用户选择日志文件 ---
[filename, pathname] = uigetfile({'*.txt'; '*.bin'; '*.*'}, '请选择飞行数据日志文件');
if isequal(filename, 0); disp('用户取消了操作，脚本已终止。'); return; end
logFilePath = fullfile(pathname, filename);
fprintf('正在打开文件: %s\n', logFilePath);

%% --- 读取并解析文件 ---
try
    fid = fopen(logFilePath, 'r', 'l'); 
    if fid == -1; error('无法打开文件。请检查文件路径和权限。'); end
    rawData = fread(fid, inf, 'uint8=>uint8');
    fclose(fid);
    fileSize = length(rawData);
    fprintf('文件读取成功，总大小: %d 字节。\n', fileSize);
catch e
    fprintf('文件读取失败: %s\n', e.message);
    return;
end

% --- MODIFICATION START: 内存预分配 ---

% 1. 计算最大可能的数据包数量
max_packets = floor(fileSize / PACKET_SIZE);

% 2. 为每种数据类型创建一个预分配的结构体数组
%    我们先定义一个模板结构体，然后用repmat创建数组
%%% 新增 %%%
imu_raw_template = struct('timestamp', 0, 'acc_x', 0, 'acc_y', 0, 'acc_z', 0, 'gyro_x', 0, 'gyro_y', 0, 'gyro_z', 0);
imu_template = struct('timestamp', 0, 'acc_x', 0, 'acc_y', 0, 'acc_z', 0, 'gyro_x', 0, 'gyro_y', 0, 'gyro_z', 0);
att_template = struct('timestamp', 0, 'roll', 0, 'pitch', 0, 'yaw', 0);
gps_template = struct('timestamp', 0, 'latitude', 0, 'longitude', 0, 'altitude', 0, 'sats_in_view', 0);
sts_template = struct('timestamp', 0, 'flight_mode', 0, 'is_armed', 0);

logData = struct();
%%% 新增 %%%
logData.imu_raw    = repmat(imu_raw_template, max_packets, 1);
logData.imu_cal    = repmat(imu_template, max_packets, 1);
logData.imu_filter = repmat(imu_template, max_packets, 1);
logData.attitude   = repmat(att_template, max_packets, 1);
logData.gps        = repmat(gps_template, max_packets, 1);
logData.status     = repmat(sts_template, max_packets, 1);

% 3. 为每个预分配的数组创建一个索引计数器
%%% 新增 %%%
counters = struct('imu_raw', 1, 'imu_cal', 1, 'imu_filter', 1, 'attitude', 1, 'gps', 1, 'status', 1);

bad_checksum_count = 0;
lost_sync_bytes = 0;
% --- MODIFICATION END ---


% --- 循环解析数据流 ---
idx = 1;
while (idx <= fileSize - PACKET_SIZE + 1)
    
    if (rawData(idx) ~= LOG_PACKET_HEADER_1 || rawData(idx+1) ~= LOG_PACKET_HEADER_2)
        idx = idx + 1;
        lost_sync_bytes = lost_sync_bytes + 1;
        continue;
    end
    
    packetBytes = rawData(idx : idx + PACKET_SIZE - 1);
    
    received_checksum = packetBytes(end);
    calculated_checksum = 0;
    for i = 1:(PACKET_SIZE - 1)
        calculated_checksum = bitxor(calculated_checksum, packetBytes(i), 'uint8');
    end
    
    if calculated_checksum ~= received_checksum
        bad_checksum_count = bad_checksum_count + 1;
        idx = idx + 1;
        continue;
    end
    
    msg_id = packetBytes(3);
    msg_len = packetBytes(4);
    timestamp = typecast(packetBytes(5:8), 'uint32');
    payload_bytes = packetBytes(9 : 9 + 24 - 1);
    
    switch msg_id
        %%% 新增 %%%
        case LOG_ID_IMU_RAW
            if msg_len == 12
                % C类型为 int16_t，对应MATLAB的 'int16'
                data = typecast(payload_bytes(1:12), 'int16');
                s.timestamp = timestamp;
                s.acc_x = data(1); s.acc_y = data(2); s.acc_z = data(3);
                s.gyro_x = data(4); s.gyro_y = data(5); s.gyro_z = data(6);
                
                % 使用索引填充，而不是追加
                logData.imu_raw(counters.imu_raw) = s;
                counters.imu_raw = counters.imu_raw + 1;
            end

        case LOG_ID_IMU_CAL
            if msg_len == 24
                data = typecast(payload_bytes(1:24), 'single');
                s.timestamp = timestamp;
                s.acc_x = data(1); s.acc_y = data(2); s.acc_z = data(3);
                s.gyro_x = data(4); s.gyro_y = data(5); s.gyro_z = data(6);
                
                logData.imu_cal(counters.imu_cal) = s;
                counters.imu_cal = counters.imu_cal + 1;
            end
            
        case LOG_ID_IMU_FILTER
             if msg_len == 24
                data = typecast(payload_bytes(1:24), 'single');
                s.timestamp = timestamp;
                s.acc_x = data(1); s.acc_y = data(2); s.acc_z = data(3);
                s.gyro_x = data(4); s.gyro_y = data(5); s.gyro_z = data(6);
                
                logData.imu_filter(counters.imu_filter) = s;
                counters.imu_filter = counters.imu_filter + 1;
            end
            
        case LOG_ID_ATTITUDE
            if msg_len == 12
                data = typecast(payload_bytes(1:12), 'single');
                s.timestamp = timestamp;
                s.roll = data(1); s.pitch = data(2); s.yaw = data(3);

                logData.attitude(counters.attitude) = s;
                counters.attitude = counters.attitude + 1;
            end
            
        case LOG_ID_GPS
            if msg_len == 21
                s.timestamp = timestamp;
                s.latitude = typecast(payload_bytes(1:8), 'double');
                s.longitude = typecast(payload_bytes(9:16), 'double');
                s.altitude = typecast(payload_bytes(17:20), 'single');
                s.sats_in_view = payload_bytes(21);

                logData.gps(counters.gps) = s;
                counters.gps = counters.gps + 1;
            end

        case LOG_ID_STATUS
            if msg_len == 2
                s.timestamp = timestamp;
                s.flight_mode = payload_bytes(1);
                s.is_armed = payload_bytes(2);

                logData.status(counters.status) = s;
                counters.status = counters.status + 1;
            end
            
        otherwise
            fprintf('发现未知消息 ID: 0x%02X\n', msg_id);
    end
    
    idx = idx + PACKET_SIZE;
end

% --- MODIFICATION START: 裁剪未使用的预分配空间 ---
%%% 新增 %%%
logData.imu_raw    = logData.imu_raw(1:counters.imu_raw-1);
logData.imu_cal    = logData.imu_cal(1:counters.imu_cal-1);
logData.imu_filter = logData.imu_filter(1:counters.imu_filter-1);
logData.attitude   = logData.attitude(1:counters.attitude-1);
logData.gps        = logData.gps(1:counters.gps-1);
logData.status     = logData.status(1:counters.status-1);
% --- MODIFICATION END ---

fprintf('解析完成！\n\n');
fprintf('--- 解析统计 ---\n');
%%% 新增 %%%
fprintf('IMU 原始数据包: %d\n', length(logData.imu_raw));
fprintf('IMU 校准数据包: %d\n', length(logData.imu_cal));
fprintf('IMU 滤波数据包: %d\n', length(logData.imu_filter));
fprintf('姿态数据包: %d\n', length(logData.attitude));
fprintf('GPS 数据包: %d\n', length(logData.gps));
fprintf('状态数据包: %d\n', length(logData.status));
fprintf('-------------------\n');
fprintf('校验和错误包数: %d\n', bad_checksum_count);
fprintf('丢失同步字节数: %d\n', lost_sync_bytes);
fprintf('-------------------\n\n');



%% --- 数据可视化 ---
fprintf('正在生成图表...\n');

%%% 新增: 绘制 IMU 原始数据 %%%
if ~isempty(logData.imu_raw)
    % 将结构体数组转换为方便绘图的格式
    time_ms = [logData.imu_raw.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0; % 转换为以秒为单位的相对时间
    
    acc_raw_x = [logData.imu_raw.acc_x];
    acc_raw_y = [logData.imu_raw.acc_y];
    acc_raw_z = [logData.imu_raw.acc_z];
    
    gyro_raw_x = [logData.imu_raw.gyro_x];
    gyro_raw_y = [logData.imu_raw.gyro_y];
    gyro_raw_z = [logData.imu_raw.gyro_z];
    
    figure('Name', 'IMU Raw Data');
    % 绘制加速度
    subplot(2, 1, 1);
    plot(time_s, acc_raw_x, 'r-'); hold on;
    plot(time_s, acc_raw_y, 'g-');
    plot(time_s, acc_raw_z, 'b-');
    title('原始加速度计数据');
    xlabel('时间 (s)');
    ylabel('原始值 (ADC Counts)');
    legend('Acc raw X', 'Acc raw Y', 'Acc raw Z');
    grid on;
    
    % 绘制陀螺仪
    subplot(2, 1, 2);
    plot(time_s, gyro_raw_x, 'r-'); hold on;
    plot(time_s, gyro_raw_y, 'g-');
    plot(time_s, gyro_raw_z, 'b-');
    title('原始陀螺仪数据');
    xlabel('时间 (s)');
    ylabel('原始值 (ADC Counts)');
    legend('Gyro raw X', 'Gyro raw Y', 'Gyro raw Z');
    grid on;
end

% 绘制 IMU 校准后数据
if ~isempty(logData.imu_cal)
    % 将结构体数组转换为方便绘图的格式
    time_ms = [logData.imu_cal.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0; % 转换为以秒为单位的相对时间

    acc_cal_x = [logData.imu_cal.acc_x];
    acc_cal_y = [logData.imu_cal.acc_y];
    acc_cal_z = [logData.imu_cal.acc_z];

    gyro_cal_x = [logData.imu_cal.gyro_x];
    gyro_cal_y = [logData.imu_cal.gyro_y];
    gyro_cal_z = [logData.imu_cal.gyro_z];

    figure('Name', 'IMU Calibrated Data');
    % 绘制加速度
    subplot(2, 1, 1);
    plot(time_s, acc_cal_x, 'r-', 'LineWidth', 1.2); hold on;
    plot(time_s, acc_cal_y, 'g-', 'LineWidth', 1.2);
    plot(time_s, acc_cal_z, 'b-', 'LineWidth', 1.2);
    title('校准后的加速度计数据');
    xlabel('时间 (s)');
    ylabel('加速度 (m/s^2)');
    legend('Acc cal X', 'Acc cal Y', 'Acc cal Z');
    grid on;

    % 绘制陀螺仪
    subplot(2, 1, 2);
    plot(time_s, gyro_cal_x, 'r-', 'LineWidth', 1.2); hold on;
    plot(time_s, gyro_cal_y, 'g-', 'LineWidth', 1.2);
    plot(time_s, gyro_cal_z, 'b-', 'LineWidth', 1.2);
    title('校准后的陀螺仪数据');
    xlabel('时间 (s)');
    ylabel('角速度 (rad/s)');
    legend('Gyro cal X', 'Gyro cal Y', 'Gyro cal Z');
    grid on;
end

% 绘制 IMU 滤波后数据 (与上面类似)
if ~isempty(logData.imu_filter)
    time_ms = [logData.imu_filter.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    
    acc_filter_x = [logData.imu_filter.acc_x];
    acc_filter_y = [logData.imu_filter.acc_y];
    acc_filter_z = [logData.imu_filter.acc_z];
    
    gyro_filter_x = [logData.imu_filter.gyro_x];
    gyro_filter_y = [logData.imu_filter.gyro_y];
    gyro_filter_z = [logData.imu_filter.gyro_z];
    
    figure('Name', 'IMU Filtered Data');
    % 绘制加速度
    subplot(2, 1, 1);
    plot(time_s, acc_filter_x, 'r-'); hold on;
    plot(time_s, acc_filter_y, 'g-');
    plot(time_s, acc_filter_z, 'b-');
    title('滤波后的加速度计数据');
    xlabel('时间 (s)');
    ylabel('加速度 (m/s^2)');
    legend('Acc filter X', 'Acc filter Y', 'Acc filter Z');
    grid on;
    
    % 绘制陀螺仪
    subplot(2, 1, 2);
    plot(time_s, gyro_filter_x, 'r-'); hold on;
    plot(time_s, gyro_filter_y, 'g-');
    plot(time_s, gyro_filter_z, 'b-');
    title('滤波后的陀螺仪数据');
    xlabel('时间 (s)');
    ylabel('角速度 (rad/s)');
    legend('Gyro filter X', 'Gyro filter Y', 'Gyro filter Z');
    grid on;
end

% 绘制姿态数据
if ~isempty(logData.attitude)
    time_ms = [logData.attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    
    roll  = [logData.attitude.roll]  * (180/pi); % 转换为度
    pitch = [logData.attitude.pitch] * (180/pi); % 转换为度
    yaw   = [logData.attitude.yaw]   * (180/pi); % 转换为度
    
    figure('Name', 'Attitude Data');
    plot(time_s, roll, 'r-', 'LineWidth', 1.5); hold on;
    plot(time_s, pitch, 'g-', 'LineWidth', 1.5);
    plot(time_s, yaw, 'b-', 'LineWidth', 1.5);
    title('飞行姿态 (Roll, Pitch, Yaw)');
    xlabel('时间 (s)');
    ylabel('角度 (度)');
    legend('Roll', 'Pitch', 'Yaw');
    grid on;
end

% 绘制GPS轨迹 (如果存在)
if ~isempty(logData.gps) && length(logData.gps) > 1
    lat = [logData.gps.latitude];
    lon = [logData.gps.longitude];
    
    figure('Name', 'GPS Track');
    plot(lon, lat, 'b-o');
    title('GPS 轨迹');
    xlabel('经度');
    ylabel('纬度');
    grid on;
    axis equal; % 使经纬度比例正确
end

fprintf('图表生成完毕！\n');