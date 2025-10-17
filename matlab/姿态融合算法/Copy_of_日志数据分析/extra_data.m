% parse_datalog.m
%
% MATLAB脚本，用于解析由STM32四轴飞行器记录的二进制飞行数据日志文件。
% 该脚本根据最新的 task_datalog.h (V1.0.0 2025/08/05) 中定义的 log_packet_t 结构进行解析。
%
% 作者: Kai (基于C代码) / AI
% 版本: 2.0.0 (适配最新的日志协议 task_datalog.h V1.0.0 2025/08/05)
% 日期: 2024/05/23

%% --- 初始化 ---
clear;
clc;
close all;

fprintf('开始解析飞行数据日志...\n');

%% --- 定义常量 (与 C 代码中的 #define 保持一致) ---
LOG_PACKET_HEADER_1 = uint8(0xA5);
LOG_PACKET_HEADER_2 = uint8(0x5A);

% --- 根据最新的 task_datalog.h 更新ID ---
LOG_ID_IMU_RAW          = uint8(0x00); % IMU原始数据
LOG_ID_IMU_CAL          = uint8(0x01); % IMU校准数据
LOG_ID_IMU_FILTER       = uint8(0x02); % IMU滤波数据
LOG_ID_ATTITUDE         = uint8(0x03); % 姿态解算数据 (主融合)
LOG_ID_ACC_MAG_ATTITUDE = uint8(0x04); % 新增: 加速度计、磁力计解算姿态数据
LOG_ID_GYRO_ATTITUDE    = uint8(0x05); % 新增: 陀螺仪解算姿态数据
LOG_ID_GPS              = uint8(0x06); % ID从 0x04 更改
LOG_ID_STATUS           = uint8(0x07); % ID从 0x05 更改
LOG_ID_MAG_RAW          = uint8(0x08); % ID从 0x06 更改
LOG_ID_MAG_GAUSS        = uint8(0x09); % ID从 0x07 更改
LOG_ID_MAG_CAL          = uint8(0x0A); % ID从 0x08 更改

PACKET_SIZE = 33; % 包的总大小保持不变，因为payload是union，大小由最大成员决定

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

% --- 内存预分配 ---

% 1. 计算最大可能的数据包数量
max_packets = floor(fileSize / PACKET_SIZE);

% 2. 为每种数据类型创建一个预分配的结构体数组
imu_raw_template = struct('timestamp', 0, 'acc_x', 0, 'acc_y', 0, 'acc_z', 0, 'gyro_x', 0, 'gyro_y', 0, 'gyro_z', 0);
imu_template = struct('timestamp', 0, 'acc_x', 0, 'acc_y', 0, 'acc_z', 0, 'gyro_x', 0, 'gyro_y', 0, 'gyro_z', 0);
att_template = struct('timestamp', 0, 'roll', 0, 'pitch', 0, 'yaw', 0);
gps_template = struct('timestamp', 0, 'latitude', 0, 'longitude', 0, 'altitude', 0, 'sats_in_view', 0);
sts_template = struct('timestamp', 0, 'flight_mode', 0, 'is_armed', 0);
mag_raw_template = struct('timestamp', 0, 'mag_x', 0, 'mag_y', 0, 'mag_z', 0);
mag_gauss_template = struct('timestamp', 0, 'mag_x', 0, 'mag_y', 0, 'mag_z', 0);
mag_cal_template = struct('timestamp', 0, 'mag_x', 0, 'mag_y', 0, 'mag_z', 0);

logData = struct();
logData.imu_raw    = repmat(imu_raw_template, max_packets, 1);
logData.imu_cal    = repmat(imu_template, max_packets, 1);
logData.imu_filter = repmat(imu_template, max_packets, 1);
logData.attitude   = repmat(att_template, max_packets, 1);
logData.gps        = repmat(gps_template, max_packets, 1);
logData.status     = repmat(sts_template, max_packets, 1);
logData.mag_raw    = repmat(mag_raw_template, max_packets, 1);
logData.mag_gauss  = repmat(mag_gauss_template, max_packets, 1);
logData.mag_cal    = repmat(mag_cal_template, max_packets, 1);
%%% 新增: 为新的姿态数据类型预分配空间 %%%
logData.acc_mag_attitude = repmat(att_template, max_packets, 1);
logData.gyro_attitude    = repmat(att_template, max_packets, 1);

% 3. 为每个预分配的数组创建一个索引计数器
counters = struct('imu_raw', 1, 'imu_cal', 1, 'imu_filter', 1, 'attitude', 1, 'gps', 1, 'status', 1, ...
                  'mag_raw', 1, 'mag_gauss', 1, 'mag_cal', 1);
%%% 新增: 添加新姿态数据类型的计数器 %%%
counters.acc_mag_attitude = 1;
counters.gyro_attitude    = 1;

bad_checksum_count = 0;
lost_sync_bytes = 0;

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
        case LOG_ID_IMU_RAW
            if msg_len == 12
                clear s; 
                data = typecast(payload_bytes(1:12), 'int16');
                s.timestamp = timestamp;
                s.acc_x = data(1); s.acc_y = data(2); s.acc_z = data(3);
                s.gyro_x = data(4); s.gyro_y = data(5); s.gyro_z = data(6);

                logData.imu_raw(counters.imu_raw) = s;
                counters.imu_raw = counters.imu_raw + 1;
            end

        case LOG_ID_IMU_CAL
            if msg_len == 24
                clear s;
                data = typecast(payload_bytes(1:24), 'single');
                s.timestamp = timestamp;
                s.acc_x = data(1); s.acc_y = data(2); s.acc_z = data(3);
                s.gyro_x = data(4); s.gyro_y = data(5); s.gyro_z = data(6);

                logData.imu_cal(counters.imu_cal) = s;
                counters.imu_cal = counters.imu_cal + 1;
            end

        case LOG_ID_IMU_FILTER
             if msg_len == 24
                clear s;
                data = typecast(payload_bytes(1:24), 'single');
                s.timestamp = timestamp;
                s.acc_x = data(1); s.acc_y = data(2); s.acc_z = data(3);
                s.gyro_x = data(4); s.gyro_y = data(5); s.gyro_z = data(6);

                logData.imu_filter(counters.imu_filter) = s;
                counters.imu_filter = counters.imu_filter + 1;
            end

        case LOG_ID_ATTITUDE
            if msg_len == 12
                clear s;
                data = typecast(payload_bytes(1:12), 'single');
                s.timestamp = timestamp;
                s.roll = data(1); s.pitch = data(2); s.yaw = data(3);

                logData.attitude(counters.attitude) = s;
                counters.attitude = counters.attitude + 1;
            end
        
        %%% 新增: 解析 LOG_ID_ACC_MAG_ATTITUDE 消息 %%%
        case LOG_ID_ACC_MAG_ATTITUDE
            if msg_len == 12
                clear s;
                data = typecast(payload_bytes(1:12), 'single');
                s.timestamp = timestamp;
                s.roll = data(1); s.pitch = data(2); s.yaw = data(3);
                
                logData.acc_mag_attitude(counters.acc_mag_attitude) = s;
                counters.acc_mag_attitude = counters.acc_mag_attitude + 1;
            end
            
        %%% 新增: 解析 LOG_ID_GYRO_ATTITUDE 消息 %%%
        case LOG_ID_GYRO_ATTITUDE
            if msg_len == 12
                clear s;
                data = typecast(payload_bytes(1:12), 'single');
                s.timestamp = timestamp;
                s.roll = data(1); s.pitch = data(2); s.yaw = data(3);
                
                logData.gyro_attitude(counters.gyro_attitude) = s;
                counters.gyro_attitude = counters.gyro_attitude + 1;
            end

        case LOG_ID_GPS
            if msg_len == 21
                clear s;
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
                clear s;
                s.timestamp = timestamp;
                s.flight_mode = payload_bytes(1);
                s.is_armed = payload_bytes(2);

                logData.status(counters.status) = s;
                counters.status = counters.status + 1;
            end

        case LOG_ID_MAG_RAW
            if msg_len == 6 
                clear s;
                data = typecast(payload_bytes(1:6), 'int16');
                s.timestamp = timestamp;
                s.mag_x = data(1); s.mag_y = data(2); s.mag_z = data(3);

                logData.mag_raw(counters.mag_raw) = s;
                counters.mag_raw = counters.mag_raw + 1;
            end

        case LOG_ID_MAG_GAUSS
            if msg_len == 12
                clear s;
                data = typecast(payload_bytes(1:12), 'single');
                s.timestamp = timestamp;
                s.mag_x = data(1); s.mag_y = data(2); s.mag_z = data(3);

                logData.mag_gauss(counters.mag_gauss) = s;
                counters.mag_gauss = counters.mag_gauss + 1;
            end

        case LOG_ID_MAG_CAL
            if msg_len == 12
                clear s;
                data = typecast(payload_bytes(1:12), 'single');
                s.timestamp = timestamp;
                s.mag_x = data(1); s.mag_y = data(2); s.mag_z = data(3);

                logData.mag_cal(counters.mag_cal) = s;
                counters.mag_cal = counters.mag_cal + 1;
            end

        otherwise
            % fprintf('发现未知消息 ID: 0x%02X\n', msg_id);
    end

    idx = idx + PACKET_SIZE;
end

% --- 裁剪未使用的预分配空间 ---
logData.imu_raw    = logData.imu_raw(1:counters.imu_raw-1);
logData.imu_cal    = logData.imu_cal(1:counters.imu_cal-1);
logData.imu_filter = logData.imu_filter(1:counters.imu_filter-1);
logData.attitude   = logData.attitude(1:counters.attitude-1);
logData.gps        = logData.gps(1:counters.gps-1);
logData.status     = logData.status(1:counters.status-1);
logData.mag_raw    = logData.mag_raw(1:counters.mag_raw-1);
logData.mag_gauss  = logData.mag_gauss(1:counters.mag_gauss-1);
logData.mag_cal    = logData.mag_cal(1:counters.mag_cal-1);
%%% 新增: 裁剪新姿态数据 %%%
logData.acc_mag_attitude = logData.acc_mag_attitude(1:counters.acc_mag_attitude-1);
logData.gyro_attitude    = logData.gyro_attitude(1:counters.gyro_attitude-1);

fprintf('解析完成！\n\n');
fprintf('--- 解析统计 ---\n');
fprintf('IMU 原始数据包: %d\n', length(logData.imu_raw));
fprintf('IMU 校准数据包: %d\n', length(logData.imu_cal));
fprintf('IMU 滤波数据包: %d\n', length(logData.imu_filter));
fprintf('姿态数据包 (主融合): %d\n', length(logData.attitude));
%%% 新增: 新姿态数据统计信息 %%%
fprintf('姿态数据包 (加计/磁力计): %d\n', length(logData.acc_mag_attitude));
fprintf('姿态数据包 (陀螺仪): %d\n', length(logData.gyro_attitude));
fprintf('GPS 数据包: %d\n', length(logData.gps));
fprintf('状态数据包: %d\n', length(logData.status));
fprintf('磁力计原始数据包: %d\n', length(logData.mag_raw));
fprintf('磁力计高斯数据包: %d\n', length(logData.mag_gauss));
fprintf('磁力计校准数据包: %d\n', length(logData.mag_cal));
fprintf('-------------------\n');
fprintf('校验和错误包数: %d\n', bad_checksum_count);
fprintf('丢失同步字节数: %d\n', lost_sync_bytes);
fprintf('-------------------\n\n');


%% --- 数据可视化 ---
fprintf('正在生成图表...\n');

% (原有的 IMU_RAW, IMU_CAL, IMU_FILTER 绘图代码保持不变)
% ... (此处省略未修改的绘图代码以节省篇幅，实际使用时请保留)
% 绘制 IMU 原始数据
if ~isempty(logData.imu_raw)
    time_ms = [logData.imu_raw.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    figure('Name', 'IMU Raw Data');
    subplot(2, 1, 1);
    plot(time_s, [logData.imu_raw.acc_x], 'r-', time_s, [logData.imu_raw.acc_y], 'g-', time_s, [logData.imu_raw.acc_z], 'b-');
    title('原始加速度计数据'); xlabel('时间 (s)'); ylabel('原始值 (ADC Counts)'); legend('Acc raw X', 'Acc raw Y', 'Acc raw Z'); grid on;
    subplot(2, 1, 2);
    plot(time_s, [logData.imu_raw.gyro_x], 'r-', time_s, [logData.imu_raw.gyro_y], 'g-', time_s, [logData.imu_raw.gyro_z], 'b-');
    title('原始陀螺仪数据'); xlabel('时间 (s)'); ylabel('原始值 (ADC Counts)'); legend('Gyro raw X', 'Gyro raw Y', 'Gyro raw Z'); grid on;
end

% 绘制 IMU 校准后数据
if ~isempty(logData.imu_cal)
    time_ms = [logData.imu_cal.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    figure('Name', 'IMU Calibrated Data');
    subplot(2, 1, 1);
    plot(time_s, [logData.imu_cal.acc_x], 'r-', time_s, [logData.imu_cal.acc_y], 'g-', time_s, [logData.imu_cal.acc_z], 'b-');
    title('校准后的加速度计数据'); xlabel('时间 (s)'); ylabel('加速度 (m/s^2)'); legend('Acc cal X', 'Acc cal Y', 'Acc cal Z'); grid on;
    subplot(2, 1, 2);
    plot(time_s, [logData.imu_cal.gyro_x], 'r-', time_s, [logData.imu_cal.gyro_y], 'g-', time_s, [logData.imu_cal.gyro_z], 'b-');
    title('校准后的陀螺仪数据'); xlabel('时间 (s)'); ylabel('角速度 (rad/s)'); legend('Gyro cal X', 'Gyro cal Y', 'Gyro cal Z'); grid on;
end

% 绘制 IMU 滤波后数据
if ~isempty(logData.imu_filter)
    time_ms = [logData.imu_filter.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    figure('Name', 'IMU Filtered Data');
    subplot(2, 1, 1);
    plot(time_s, [logData.imu_filter.acc_x], 'r-', time_s, [logData.imu_filter.acc_y], 'g-', time_s, [logData.imu_filter.acc_z], 'b-');
    title('滤波后的加速度计数据'); xlabel('时间 (s)'); ylabel('加速度 (m/s^2)'); legend('Acc filter X', 'Acc filter Y', 'Acc filter Z'); grid on;
    subplot(2, 1, 2);
    plot(time_s, [logData.imu_filter.gyro_x], 'r-', time_s, [logData.imu_filter.gyro_y], 'g-', time_s, [logData.imu_filter.gyro_z], 'b-');
    title('滤波后的陀螺仪数据'); xlabel('时间 (s)'); ylabel('角速度 (rad/s)'); legend('Gyro filter X', 'Gyro filter Y', 'Gyro filter Z'); grid on;
end


%%% 更新: 绘制姿态对比图 %%%
figure('Name', 'Attitude Comparison');
legend_items = {};
has_att_data = false;

% 绘制 Roll
subplot(3, 1, 1);
hold on;
if ~isempty(logData.attitude)
    time_ms = [logData.attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.attitude.roll] * (180/pi), 'r-', 'LineWidth', 1.5);
    legend_items{end+1} = 'Roll (主融合)';
    has_att_data = true;
end
if ~isempty(logData.acc_mag_attitude)
    time_ms = [logData.acc_mag_attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.acc_mag_attitude.roll], 'g--', 'LineWidth', 1.2);
    legend_items{end+1} = 'Roll (加计/磁力计)';
    has_att_data = true;
end
if ~isempty(logData.gyro_attitude)
    time_ms = [logData.gyro_attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.gyro_attitude.roll], 'b:', 'LineWidth', 1.2);
    legend_items{end+1} = 'Roll (陀螺仪)';
    has_att_data = true;
end
title('横滚角 (Roll) 对比'); ylabel('角度 (度)'); grid on;
if ~isempty(legend_items); legend(legend_items, 'Location', 'best'); end
hold off;

% 绘制 Pitch
subplot(3, 1, 2);
hold on;
legend_items = {};
if ~isempty(logData.attitude)
    time_ms = [logData.attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.attitude.pitch] * (180/pi), 'r-', 'LineWidth', 1.5);
    legend_items{end+1} = 'Pitch (主融合)';
end
if ~isempty(logData.acc_mag_attitude)
    time_ms = [logData.acc_mag_attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.acc_mag_attitude.pitch], 'g--', 'LineWidth', 1.2);
    legend_items{end+1} = 'Pitch (加计/磁力计)';
end
if ~isempty(logData.gyro_attitude)
    time_ms = [logData.gyro_attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.gyro_attitude.pitch], 'b:', 'LineWidth', 1.2);
    legend_items{end+1} = 'Pitch (陀螺仪)';
end
title('俯仰角 (Pitch) 对比'); ylabel('角度 (度)'); grid on;
if ~isempty(legend_items); legend(legend_items, 'Location', 'best'); end
hold off;

% 绘制 Yaw
subplot(3, 1, 3);
hold on;
legend_items = {};
if ~isempty(logData.attitude)
    time_ms = [logData.attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.attitude.yaw] * (180/pi), 'r-', 'LineWidth', 1.5);
    legend_items{end+1} = 'Yaw (主融合)';
end
if ~isempty(logData.acc_mag_attitude)
    time_ms = [logData.acc_mag_attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.acc_mag_attitude.yaw], 'g--', 'LineWidth', 1.2);
    legend_items{end+1} = 'Yaw (加计/磁力计)';
end
if ~isempty(logData.gyro_attitude)
    time_ms = [logData.gyro_attitude.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    plot(time_s, [logData.gyro_attitude.yaw], 'b:', 'LineWidth', 1.2);
    legend_items{end+1} = 'Yaw (陀螺仪)';
end
title('偏航角 (Yaw) 对比'); xlabel('时间 (s)'); ylabel('角度 (度)'); grid on;
if ~isempty(legend_items); legend(legend_items, 'Location', 'best'); end
hold off;

if ~has_att_data
    close('Attitude Comparison'); % 如果没有任何姿态数据，则关闭空图
end

% (原有的 MAG_RAW, MAG_GAUSS, MAG_CAL, GPS 绘图代码保持不变)
% ... (此处省略未修改的绘图代码以节省篇幅，实际使用时请保留)
% 绘制磁力计原始数据
if ~isempty(logData.mag_raw)
    time_ms = [logData.mag_raw.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    figure('Name', 'Magnetometer Raw Data');
    plot(time_s, [logData.mag_raw.mag_x], 'r-', time_s, [logData.mag_raw.mag_y], 'g-', time_s, [logData.mag_raw.mag_z], 'b-');
    title('原始磁力计数据'); xlabel('时间 (s)'); ylabel('原始值 (ADC Counts)'); legend('Mag raw X', 'Mag raw Y', 'Mag raw Z'); grid on;
end

% 绘制转换后的磁力计数据 (Gauss)
if ~isempty(logData.mag_gauss)
    time_ms = [logData.mag_gauss.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    figure('Name', 'Magnetometer Gauss Data');
    plot(time_s, [logData.mag_gauss.mag_x], 'r-', time_s, [logData.mag_gauss.mag_y], 'g-', time_s, [logData.mag_gauss.mag_z], 'b-');
    title('转换后的磁力计数据 (高斯)'); xlabel('时间 (s)'); ylabel('磁场强度 (Gauss)'); legend('Mag gauss X', 'Mag gauss Y', 'Mag gauss Z'); grid on;
end

% 绘制磁力计校准后数据
if ~isempty(logData.mag_cal)
    time_ms = [logData.mag_cal.timestamp];
    time_s = double(time_ms - time_ms(1)) / 1000.0;
    figure('Name', 'Magnetometer Calibrated Data');
    plot(time_s, [logData.mag_cal.mag_x], 'r-', time_s, [logData.mag_cal.mag_y], 'g-', time_s, [logData.mag_cal.mag_z], 'b-');
    title('校准后的磁力计数据'); xlabel('时间 (s)'); ylabel('磁场强度 (uT)'); legend('Mag cal X', 'Mag cal Y', 'Mag cal Z'); grid on;
end

% 绘制GPS轨迹
if ~isempty(logData.gps) && length(logData.gps) > 1
    figure('Name', 'GPS Track');
    plot([logData.gps.longitude], [logData.gps.latitude], 'b-o');
    title('GPS 轨迹'); xlabel('经度'); ylabel('纬度'); grid on; axis equal;
end

fprintf('图表生成完毕！\n');

%% --- 3D 磁力计数据可视化与数学拟合分析 ---
% (此部分代码无需修改，因为它依赖的数据结构没有变化)
fprintf('正在生成3D磁力计散点图及拟合分析...\n');

% (此处省略未修改的 3D 磁力计绘图代码，实际使用时请保留)
% 1. 分析并绘制原始磁力计数据 (mag_raw)
if ~isempty(logData.mag_raw) && length(logData.mag_raw) > 10 % 确保有足够的数据点
    mag_raw_x = double([logData.mag_raw.mag_x]);
    mag_raw_y = double([logData.mag_raw.mag_y]);
    mag_raw_z = double([logData.mag_raw.mag_z]);
    [center_raw, radii_raw, ~] = fit_ellipsoid(mag_raw_x, mag_raw_y, mag_raw_z);
    fprintf('\n--- 原始磁力计数据拟合分析 (MAG_RAW) ---\n');
    fprintf('拟合椭球中心 (硬磁偏移): [%.2f, %.2f, %.2f]\n', center_raw(1), center_raw(2), center_raw(3));
    fprintf('归一化缩放因子 (软磁形变): [%.3f, %.3f, %.3f]\n', radii_raw / mean(radii_raw));
    fprintf('-----------------------------------\n\n');
    figure('Name', '3D Magnetometer Raw Data Analysis');
    hold on;
    scatter3(mag_raw_x, mag_raw_y, mag_raw_z, 10, 'b', 'filled', 'DisplayName', '原始数据点');
    plot3(center_raw(1), center_raw(2), center_raw(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', '拟合中心 (硬磁偏移)');
    [el_x, el_y, el_z] = ellipsoid(center_raw(1), center_raw(2), center_raw(3), radii_raw(1), radii_raw(2), radii_raw(3), 30);
    surf(el_x, el_y, el_z, 'FaceAlpha', 0.2, 'EdgeColor', 'r', 'LineStyle', ':', 'DisplayName', '拟合椭球');
    title('原始磁力计数据与椭球拟合'); xlabel('X 轴'); ylabel('Y 轴'); zlabel('Z 轴');
    grid on; axis equal; legend('show'); view(3); hold off;
end

% 2. 分析并绘制高斯磁力计数据 (mag_gauss)
if ~isempty(logData.mag_gauss) && length(logData.mag_gauss) > 10
    mag_gauss_x = double([logData.mag_gauss.mag_x]);
    mag_gauss_y = double([logData.mag_gauss.mag_y]);
    mag_gauss_z = double([logData.mag_gauss.mag_z]);
    [center_gauss, radii_gauss, ~] = fit_ellipsoid(mag_gauss_x, mag_gauss_y, mag_gauss_z);
    fprintf('--- 高斯磁力计数据拟合分析 (MAG_GAUSS) ---\n');
    fprintf('拟合椭球中心: [%.4f, %.4f, %.4f] (理想值: [0, 0, 0])\n', center_gauss(1), center_gauss(2), center_gauss(3));
    fprintf('拟合椭球半径 (软磁形变): [%.4f, %.4f, %.4f]\n', radii_gauss(1), radii_gauss(2), radii_gauss(3));
    fprintf('-----------------------------------\n\n');
    figure('Name', '3D Magnetometer Gauss Data Analysis');
    hold on;
    scatter3(mag_gauss_x, mag_gauss_y, mag_gauss_z, 10, 'm', 'filled', 'DisplayName', '高斯数据点 (硬磁校准后)');
    plot3(center_gauss(1), center_gauss(2), center_gauss(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', '拟合中心');
    [el_x, el_y, el_z] = ellipsoid(center_gauss(1), center_gauss(2), center_gauss(3), radii_gauss(1), radii_gauss(2), radii_gauss(3), 30);
    surf(el_x, el_y, el_z, 'FaceAlpha', 0.2, 'EdgeColor', 'r', 'LineStyle', ':', 'DisplayName', '拟合椭球');
    title('高斯磁力计数据与椭球拟合 (硬磁校准后)'); xlabel('X 轴 (Gauss)'); ylabel('Y 轴 (Gauss)'); zlabel('Z 轴 (Gauss)');
    grid on; axis equal; legend('show'); view(3); hold off;
end

% 3. 分析并绘制校准后磁力计数据 (mag_cal)
if ~isempty(logData.mag_cal) && length(logData.mag_cal) > 10
    mag_cal_x = double([logData.mag_cal.mag_x]);
    mag_cal_y = double([logData.mag_cal.mag_y]);
    mag_cal_z = double([logData.mag_cal.mag_z]);
    [center_cal, radii_cal, ~] = fit_ellipsoid(mag_cal_x, mag_cal_y, mag_cal_z);
    fprintf('--- 校准后磁力计数据拟合分析 (MAG_CAL) ---\n');
    fprintf('拟合球心: [%.4f, %.4f, %.4f] (理想值: [0, 0, 0])\n', center_cal(1), center_cal(2), center_cal(3));
    avg_radius_cal = mean(radii_cal);
    fprintf('平均半径: %.4f\n', avg_radius_cal);
    fprintf('半径变化率 (评估是否为正球): %.2f%%\n', (max(radii_cal) - min(radii_cal)) / avg_radius_cal * 100);
    fprintf('-----------------------------------\n\n');
    figure('Name', '3D Magnetometer Calibrated Data Analysis');
    hold on;
    scatter3(mag_cal_x, mag_cal_y, mag_cal_z, 10, 'g', 'filled', 'DisplayName', '校准后数据点');
    plot3(center_cal(1), center_cal(2), center_cal(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', '拟合中心');
    [sphere_x, sphere_y, sphere_z] = sphere(30);
    surf(sphere_x*avg_radius_cal, sphere_y*avg_radius_cal, sphere_z*avg_radius_cal, 'FaceAlpha', 0.1, 'EdgeColor', 'k', 'LineStyle', ':', 'DisplayName', '理想球面 (参考)');
    title('校准后磁力计数据与理想球面比较'); xlabel('X 轴 (uT)'); ylabel('Y 轴 (uT)'); zlabel('Z 轴 (uT)');
    grid on; axis equal; legend('show'); view(3); hold off;
end

% 注意: fit_ellipsoid 是一个用于椭球拟合的辅助函数，此脚本中未提供。
% 如果您没有这个函数，可以从网上找到开源实现，例如 Yury Petrov 在 File Exchange 上的作品。
% 如果不需要3D磁力计拟合功能，可以注释掉最后这部分代码。
% function [ center, radii, evecs ] = fit_ellipsoid( x, y, z )
%     % ... 椭球拟合算法实现 ...
% end