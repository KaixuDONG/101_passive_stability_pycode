% close all
% 
% leader_quat_I_to_B = quat_trans_output(leader_QW, leader_QX, leader_QY, leader_QZ);
% leader_euler_I_to_B = quat_to_euler(leader_quat_I_to_B);
%  
% figure(1)
% grid on
% plot(leader_euler_I_to_B(:, 3));


% qw = [[0.96 -0.01 -0.26 -0.05];
%       [0.94 0.34 -0.05 -0.01];
%       [0.93 0.29 -0.24 -0.02];
% ];
% 
% quat2eul(qw(1, :))*57.3;

euler_follower_1 = quat_euler_Body_inInertia(follower_1_QW, follower_1_QX, follower_1_QY, follower_1_QZ);
euler_follower_2 = quat_euler_Body_inInertia(follower_2_QW, follower_2_QX, follower_2_QY, follower_2_QZ);

function quat_I_to_B = quat_trans_output(QW, QX, QY, QZ)
    data_length = length(QW);

    quat_M_to_I = [0 1 0 0];

    quat_I_to_B = zeros(data_length, 4);

    for i = 1: 1: data_length
        quat_M_to_B = [QW(i) QX(i) QY(i) QZ(i)];
        
        quat_I_to_B_temp = quatdivide(quat_M_to_B, quat_M_to_I);

        quat_I_to_B(i, 1) = quat_I_to_B_temp(1);
        quat_I_to_B(i, 2) = quat_I_to_B_temp(2);
        quat_I_to_B(i, 3) = quat_I_to_B_temp(3);
        quat_I_to_B(i, 4) = quat_I_to_B_temp(4);
    end
    
end


function euler_array = quat_to_euler(quat_array)
    quat_size = size(quat_array);
    quat_row = quat_size(1);
    euler_array = zeros(quat_row, 3);

    for i = 1: 1: quat_row
        euler_temp = quat2eul(quat_array(i, :));
        
        euler_array(i, 1) = euler_temp(1)*180/pi;
        euler_array(i, 2) = euler_temp(2)*180/pi;
        euler_array(i, 3) = euler_temp(3)*180/pi;
    end

end


function euler = quat_euler_Body_inInertia(QW, QX, QY, QZ)
    data_length = length(QW);
    
    euler = zeros(data_length, 3);

    rotm_I_to_M = [-1 0 0; 0 -1 0; 0 0 1];

    for i = 1: 1: data_length
        quat_temp = [QW(i) QX(i) QY(i) QZ(i)];

        rotm_M_to_B_temp = quat2rotm(quat_temp) * rotm_I_to_M;

        euler_temp = rotm2eul(rotm_M_to_B_temp);

        euler(i, 3) = euler_temp(1)*57.3;
        euler(i, 2) = euler_temp(2)*57.3;
        euler(i, 1) = euler_temp(3)*57.3;

    end

end