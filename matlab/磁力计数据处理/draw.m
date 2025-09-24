%% --- 3D 磁力计数据可视化 ---
fprintf('正在生成3D磁力计散点图...\n');

% 1. 绘制原始磁力计数据的3D散点图
if ~isempty(logData.mag_raw)
    % 从结构体数组中提取X, Y, Z数据
    mag_raw_x = [logData.mag_raw.mag_x];
    mag_raw_y = [logData.mag_raw.mag_y];
    mag_raw_z = [logData.mag_raw.mag_z];
    
    % 创建一个新的图形窗口
    figure('Name', '3D Magnetometer Raw Data Scatter Plot');
    
    % 创建一个颜色向量，让数据点根据采集顺序从蓝到黄渐变，以观察数据采集过程
    num_points = length(mag_raw_x);
    color_vector = 1:num_points;
    
    % 使用 scatter3 函数绘制3D散点图
    % scatter3(X, Y, Z, MarkerSize, ColorVector, 'filled')
    scatter3(mag_raw_x, mag_raw_y, mag_raw_z, 15, color_vector, 'filled');
    
    % --- 美化和配置图表 ---
    title('3D原始磁力计数据分布', 'FontSize', 14);
    xlabel('X 轴 (ADC Counts)');
    ylabel('Y 轴 (ADC Counts)');
    zlabel('Z 轴 (ADC Counts)');
    grid on; % 显示网格
    axis equal; % !!至关重要!! 确保各轴比例相同，否则球体会被拉伸，无法正确判断形状
    
    % 添加颜色条，用于解释颜色渐变的含义
    cb = colorbar;
    ylabel(cb, '数据点顺序 (时间进程)');
    
    % 在图上添加文本框，解释如何解读此图
    annotation('textbox', [0.15, 0.7, 0.1, 0.1], 'FitBoxToText','on', ...
        'BackgroundColor', 'white', 'String', ...
        {'如何解读:', ...
         '1. 理想状态: 数据点构成一个球面。', ...
         '2. 球心偏离原点: 存在硬磁干扰。', ...
         '3. 数据点呈椭球状: 存在软磁干扰。'});
         
    fprintf('已生成原始磁力计3D散点图。\n');
end

% 2. 绘制校准后磁力计数据的3D散点图
if ~isempty(logData.mag_cal)
    % 提取数据
    mag_cal_x = [logData.mag_cal.mag_x];
    mag_cal_y = [logData.mag_cal.mag_y];
    mag_cal_z = [logData.mag_cal.mag_z];
    
    % 创建新图形窗口
    figure('Name', '3D Magnetometer Calibrated Data Scatter Plot');
    
    % 创建颜色向量
    num_points = length(mag_cal_x);
    color_vector = 1:num_points;
    
    % 绘制3D散点图
    scatter3(mag_cal_x, mag_cal_y, mag_cal_z, 15, color_vector, 'filled');
    
    % --- 美化和配置图表 ---
    title('3D校准后磁力计数据分布', 'FontSize', 14);
    xlabel('X 轴 (uT)');
    ylabel('Y 轴 (uT)');
    zlabel('Z 轴 (uT)');
    grid on;
    axis equal; % 同样重要，用于检查校准效果
    
    % 添加颜色条
    cb = colorbar;
    ylabel(cb, '数据点顺序 (时间进程)');
    
    % 在图上添加文本框
    annotation('textbox', [0.15, 0.7, 0.1, 0.1], 'FitBoxToText','on', ...
        'BackgroundColor', 'white', 'String', ...
        {'如何解读:', ...
         '理想状态下, 所有数据点应', ...
         '分布在一个以原点(0,0,0)为中心的', ...
         '标准球面上。'});

    fprintf('已生成校准后磁力计3D散点图。\n');
end