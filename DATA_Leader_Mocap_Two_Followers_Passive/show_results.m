% if exit('leader_X_pos')
%     data_length_mocap = length(leader_)
% end

% the reference line shoudl be added


close all

% follower_1

follower_1_pitch_onboard_neg = -follower_1_pitch_onboard;
follower_2_pitch_onboard_neg = -follower_2_pitch_onboard;


%% data of the follower 1
if exist('follower_1_X_pos')
    data_length_mocap = length(follower_1_X_pos);
    data_length_onboard = length(follower_1_pitch_onboard_neg);
    
    time_mocap = 0: 0.01: (data_length_mocap - 1)/100;
    time_onboard = 0: 0.01: (data_length_onboard - 1)/100;
    

    follower_1_euler = quat_euler_Body_inInertia(follower_1_QW, follower_1_QX, follower_1_QY, follower_1_QZ);
    follower_1_roll_mocap = follower_1_euler(:, 1);
    follower_1_pitch_mocap = follower_1_euler(:, 2);
    follower_1_yaw_mocap = follower_1_euler(:, 3);
    
    figure(1)
    subplot(3, 1, 1)
    plot(time_mocap, follower_1_roll_mocap)
    hold on
    plot(time_mocap, follower_1_roll_onboard(end - data_length_mocap+1: end))
    hold off
    title('roll of the follower 1')
    legend('MoCap', 'Onboard')
    grid on
    
    subplot(3, 1, 2)
    plot(time_mocap, follower_1_pitch_mocap)
    hold on
    plot(time_mocap, follower_1_pitch_onboard_neg(end - data_length_mocap+1: end))
    title('pitch of the follower 1')
    legend('MoCap', 'Onboard')
    hold off
    grid on

%     subplot(3, 1, 3)
%     plot(time_mocap, follower_1_yaw_mocap)
%     title('yaw of the follower 1 by the mocap')
%     grid on
    
%     figure(2)
%     subplot(3, 1, 1)
%     plot(time_onboard, follower_1_roll_onboard)
%     title('roll of the follower 1 from the onboard data')
%     grid on
    
%     subplot(3, 1, 2)
%     plot(time_onboard, follower_1_pitch_onboard)
%     title('pitch of the follower 1 from the onboard data')
%     grid on
    
    subplot(3, 1, 3)
    plot(time_onboard, follower_1_thrust_inNewton_onboard)
    title('thrust in Newton of follower 1 from the onboard data')
    grid on
    
end

%% data of the follower 2
if exist('follower_2_X_pos')
    data_length_mocap = length(follower_2_X_pos);
    data_length_onboard = length(follower_2_pitch_onboard_neg);
    
    time_mocap = 0: 0.01: (data_length_mocap - 1)/100;
    time_onboard = 0: 0.01: (data_length_onboard - 1)/100;
    
    follower_2_euler = quat_euler_Body_inInertia(follower_2_QW, follower_2_QX, follower_2_QY, follower_2_QZ);
    follower_2_roll_mocap = follower_2_euler(:, 1);
    follower_2_pitch_mocap = follower_2_euler(:, 2);
    follower_2_yaw_mocap = follower_2_euler(:, 3);
    
    figure(3)
    subplot(3, 1, 1)
    plot(time_mocap, follower_2_roll_mocap)
    hold on
    plot(time_onboard, follower_2_roll_onboard)
    hold off
    legend('MoCap', 'Onboard')
    title('roll of the follower 2')
    grid on
   
    subplot(3, 1, 2)
    plot(time_mocap, follower_2_pitch_mocap)
    hold on
    plot(time_onboard, follower_2_pitch_onboard_neg)
    hold off
    legend('MoCap', 'Onboard')
    title('pitch of the follower 2 ')
    grid on
    
%     subplot(3, 1, 3)
%     plot(time_mocap, follower_2_yaw_mocap)
%     title('yaw of the follower 2 by the mocap')
%     grid on
%    
%     figure(4)
%     subplot(3, 1, 1)
%     plot(time_onboard, follower_2_roll_onboard)
%     title('roll of the follower 2 from the onboard data')
%     grid on
%     
%     subplot(3, 1, 2)
%     plot(time_onboard, follower_2_pitch_onboard)
%     title('pitch of the follower 2 from the onboard data')
%     grid on
    
    subplot(3, 1, 3)
    plot(time_onboard, follower_2_thrust_inNewton_onboard)
    title('thrust in Newton of follower 2 from the onboard data')
    grid on
    
end


%% the information about the center of the payload and the followers
% center_X_pos = (payload_X_pos + follower_1_X_pos + follower_2_X_pos)/3;
% center_Y_pos = (payload_Y_pos + follower_1_Y_pos + follower_2_Y_pos)/3;


% center_Z_pos = (payload_Z_pos + follower_1_Z_pos + follower_2_Z_pos)/3;

center_X_pos = (payload_X_pos*0.225 + follower_1_X_pos*0.384 + follower_2_X_pos*0.384);
center_Y_pos = (payload_Y_pos*0.225 + follower_1_Y_pos*0.384 + follower_2_Y_pos*0.384);
center_Z_pos = (payload_Z_pos*0.225 + follower_1_Z_pos*0.384 + follower_2_Z_pos*0.384);

%% the positions about the agents
figure(5)
plot(time_mocap, leader_X_pos)
hold on
grid on
plot(time_mocap, payload_X_pos)
plot(time_mocap, follower_1_X_pos)
plot(time_mocap, follower_2_X_pos)
plot(time_mocap, center_X_pos)
hold off
legend('leader', 'payload', 'follower_1', 'follower_2', 'center')
title('X position of the agents')
grid on


figure(6)
plot(time_mocap, leader_Y_pos)
hold on
grid on
plot(time_mocap, payload_Y_pos)
plot(time_mocap, follower_1_Y_pos)
plot(time_mocap, follower_2_Y_pos)
plot(time_mocap, center_Y_pos)
hold off
legend('leader', 'payload', 'follower_1', 'follower_2', 'center')
title('Y position of the agents')
grid on

figure(7)
plot(time_mocap, leader_Z_pos)
hold on
grid on

plot(time_mocap, payload_Z_pos)
plot(time_mocap, follower_1_Z_pos)
plot(time_mocap, follower_2_Z_pos)
plot(time_mocap, center_Z_pos)
hold off
legend('leader', 'payload', 'follower_1', 'follower_2', 'center')
title('Z position of the agents')
grid on



%% to determin the relative position of the agent, including the elevation angle and azimuth angle 
alpha_0_ref = zeros(1, data_length_mocap);
alpha_1_ref = zeros(1, data_length_mocap);
alpha_2_ref = zeros(1, data_length_mocap);

beta_0_ref = zeros(1, data_length_mocap);
beta_1_ref = zeros(1, data_length_mocap);
beta_2_ref = zeros(1, data_length_mocap);

% set the reference values for the elevation and azimuth
switch_time_1 = 30;
switch_time_2 = 60;

alpha_0_ref(1: switch_time_1*100) = 45;
alpha_1_ref(1: switch_time_1*100) = 45;
alpha_2_ref(1: switch_time_1*100) = 45;

beta_0_ref(1: switch_time_1*100) = 90;
beta_1_ref(1: switch_time_1*100) = -45;
beta_2_ref(1: switch_time_1*100) = -135;

alpha_0_ref(switch_time_1*100: switch_time_2*100) = 60;
alpha_1_ref(switch_time_1*100: switch_time_2*100) = 60;
alpha_2_ref(switch_time_1*100: switch_time_2*100) = 60;

beta_0_ref(switch_time_1*100: switch_time_2*100) = 90;
beta_1_ref(switch_time_1*100: switch_time_2*100) = -30;
beta_2_ref(switch_time_1*100: switch_time_2*100) = -150;


alpha_0_ref(switch_time_2*100: end) = 45;
alpha_1_ref(switch_time_2*100: end) = 30;
alpha_2_ref(switch_time_2*100: end) = 30;

beta_0_ref(switch_time_2*100: end) = 90;
beta_1_ref(switch_time_2*100: end) = -60;
beta_2_ref(switch_time_2*100: end) = -120;

% % stage 1
% alpha_0_ref(1: switch_time*100) = 45;
% alpha_1_ref(1: switch_time*100) = 45;
% alpha_2_ref(1: switch_time*100) = 45;
% 
% beta_0_ref(1: switch_time*100) = 90;
% beta_1_ref(1: switch_time*100) = -45;
% beta_2_ref(1: switch_time*100) = -135;
% 
% % stage 2
% alpha_0_ref(switch_time*100 + 1: end) = 60;
% alpha_1_ref(switch_time*100 + 1: end) = 45;
% alpha_2_ref(switch_time*100 + 1: end) = 60;
% 
% beta_0_ref(switch_time*100 + 1: end) = 90;
% beta_1_ref(switch_time*100 + 1: end) = -30;
% beta_2_ref(switch_time*100 + 1: end) = -150;

% stage 1
% alpha_0_ref(1: end) = 45;
% alpha_1_ref(1: end) = 45;
% alpha_2_ref(1: end) = 45;
% 
% beta_0_ref(1: end) = 90;
% beta_1_ref(1: end) = -45;
% beta_2_ref(1: end) = -135;



% trans the data to the defined inertia frame (which is not aligned with the mocap inertia system)
follower_1_X_trans = -follower_1_X_pos;
follower_1_Y_trans =  follower_1_Y_pos;
follower_1_Z_trans = -follower_1_Z_pos;

follower_2_X_trans = -follower_2_X_pos;
follower_2_Y_trans =  follower_2_Y_pos;
follower_2_Z_trans = -follower_2_Z_pos;

leader_X_trans = -leader_X_pos;
leader_Y_trans =  leader_Y_pos;
leader_Z_trans = -leader_Z_pos;

payload_X_trans = -payload_X_pos;
payload_Y_trans =  payload_Y_pos;
payload_Z_trans = -payload_Z_pos;

distance_payload_leader = sqrt((payload_X_trans - leader_X_trans).^2 + (payload_Y_trans - leader_Y_trans).^2 + (payload_Z_trans - leader_Z_trans).^2);
distance_payload_follower_1 = sqrt((payload_X_trans - follower_1_X_trans).^2 + (payload_Y_trans - follower_1_Y_trans).^2 + (payload_Z_trans - follower_1_Z_trans).^2);
distance_payload_follower_2 = sqrt((payload_X_trans - follower_2_X_trans).^2 + (payload_Y_trans - follower_2_Y_trans).^2 + (payload_Z_trans - follower_2_Z_trans).^2);

alpha_0 = asin((leader_Z_pos - payload_Z_pos)./distance_payload_leader).*57.3;
alpha_1 = asin((follower_1_Z_pos - payload_Z_pos)./distance_payload_follower_1).*57.3;
alpha_2 = asin((follower_2_Z_pos - payload_Z_pos)./distance_payload_follower_2).*57.3;

beta_0 = atan2(payload_Y_trans - leader_Y_trans, payload_X_trans - leader_X_trans) * 57.3;
beta_1 = atan2(payload_Y_trans - follower_1_Y_trans, payload_X_trans - follower_1_X_trans) * 57.3;
beta_2 = atan2(payload_Y_trans - follower_2_Y_trans, payload_X_trans - follower_2_X_trans) * 57.3;



% the leader is in BLUE, the follower is in ORANGE and the follower 2 is in
% GREEN
figure(10)
plot(time_mocap, alpha_0, 'Color', '#0072BD')
hold on 
plot(time_mocap, alpha_1, 'Color', '#D95319')	
plot(time_mocap, alpha_2, 'Color', '#77AC30')
% the reference value
plot(time_mocap, alpha_0_ref, '--', 'Color', '#0072BD')
plot(time_mocap, alpha_1_ref, '--', 'Color', '#D95319')	
plot(time_mocap, alpha_2_ref, '--', 'Color', '#77AC30')
hold off
legend('leader', 'follower 1', 'follower 2', 'leader-ref', 'follower 1-ref', 'follower 2-ref')
title('the elevation angles of the leader, follower_1 and follower_2')
grid on


figure(11)
plot(time_mocap, -beta_0, 'Color', '#0072BD')
hold on 
plot(time_mocap, -beta_2, 'Color', '#D95319')
plot(time_mocap, -beta_1, 'Color', '#77AC30')
plot(time_mocap, beta_0_ref, '--', 'Color', '#0072BD')
plot(time_mocap, beta_1_ref, '--', 'Color', '#D95319')	
plot(time_mocap, beta_2_ref, '--', 'Color', '#77AC30')
hold off
legend('leader', 'follower 1', 'follower 2', 'leader-ref', 'follower 1-ref', 'follower 2-ref')
title('the azimuth angles of the leader, follower_1 and follower_2')
grid on


%% functions 
function euler = quat_euler_Body_inInertia(QW, QX, QY, QZ)
    data_length = length(QW);
    
    euler = zeros(data_length, 3);

    rotm_I_to_M = [-1 0 0; 0 -1 0; 0 0 1];

    for i = 1: 1: data_length
        quat_temp = [QW(i) QX(i) QY(i) QZ(i)];
        % quaternion to rotation matrix
        rotm_M_to_B_temp = quat2rotm(quat_temp) * rotm_I_to_M;
        
        %rotation matrix to euler angles
        euler_temp = rotm2eul(rotm_M_to_B_temp);
        
        euler(i, 3) = euler_temp(1)*57.3;
        euler(i, 2) = euler_temp(2)*57.3;
        euler(i, 1) = euler_temp(3)*57.3;
        % RETURN IN THE SEQUENCE OF ROLL PITCH YAW
    end

end