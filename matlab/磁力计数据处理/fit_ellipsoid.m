function [center, radii, axes_rotation] = fit_ellipsoid(x, y, z)
% fit_ellipsoid: 使用最小二乘法对3D数据点进行椭球拟合
%
% 输入:
%   x, y, z: 长度相同的向量，代表3D数据点的坐标
%
% 输出:
%   center: 1x3 向量，表示椭球的中心 [cx, cy, cz]
%   radii:  1x3 向量，表示椭球的三个半轴长 [rx, ry, rz]
%   axes_rotation: 3x3 矩阵，表示椭球坐标轴相对于主坐标轴的旋转矩阵
%
% 算法基于:
% A. B. Yuryev, "Least-squares ellipsoid specific fitting," 2010.

    % 确保输入是列向量
    x = x(:);
    y = y(:);
    z = z(:);

    % 构建设计矩阵 D
    % 椭球方程: Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0
    % 这里我们求解 v = [A, B, C, D, E, F, G, H, I, J]'
    % 为了避免平凡解 v=0, 我们通常设置一个约束，比如 J=-1, 
    % 那么方程变为 Ax^2 + ... + 2Iz = 1
    D = [x.^2, y.^2, z.^2, 2*x.*y, 2*x.*z, 2*y.*z, 2*x, 2*y, 2*z];
    
    % 求解最小二乘问题 v = D \ ones(size(x))
    v = (D' * D) \ (D' * ones(size(x)));
    
    % 从解 v 中提取椭球参数
    A = v(1); B = v(2); C = v(3);
    Dxy = v(4); Exz = v(5); Fyz = v(6);
    G = v(7); H = v(8); I = v(9);
    
    % 构建代数形式的矩阵
    M = [ A,  Dxy, Exz;
         Dxy, B,   Fyz;
         Exz, Fyz, C ];
    
    b = [ G; H; I ];
    
    % 计算中心点
    center = (-M \ b)';
    
    % 计算旋转矩阵和半径
    % 对称矩阵M进行特征值分解
    [evecs, evals] = eig(M);
    
    % 半径是特征值的倒数平方根
    radii = sqrt(1 ./ diag(evals))';
    
    % 旋转矩阵就是特征向量矩阵
    axes_rotation = evecs;
    
end