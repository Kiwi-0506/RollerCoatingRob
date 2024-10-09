function normal_vector = calculate_normal_vector(x0,y0,z0,point_cloud,tolerance)
    % calculate_Normal_Vector 计算点云中特定z坐标附近的点的法向量
    %
    % 输入:
    % point_cloud - N x 3 矩阵，每一行是一个点的(x, y, z)坐标
    % z0 - 目标z坐标
    % tolerance - z坐标的容忍范围
    %
    % 输出:
    % normal_vector - 计算得到的法向量
    % nearby_points - z坐标在z0附近的点

    % 找到z坐标在z0附近的点
    z_diff = abs(point_cloud(:, 3) - z0);
    nearby_indices = z_diff < tolerance;
    nearby_points = point_cloud(nearby_indices, :);

    % 如果附近没有点，则返回空矩阵
    if isempty(nearby_points)
        normal_vector = [];
        disp("no nearby points");
        return;
    end

    % 计算法向量
    % 使用最小二乘法拟合一个平面并计算法向量
    % 使用邻近点拟合平面
    [A_coeff, B_coeff, C_coeff] = polyfit(nearby_points(:, 1), nearby_points(:, 2), nearby_points(:, 3), 1);

    % 法向量
    normal_vector = [-A_coeff, -B_coeff, 1];
    % 归一化法向量
    normal_vector = normal_vector / norm(normal_vector);
end
