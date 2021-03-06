close all

array_length = length(thrust_actual);

sample_rate = 100;

time_series = (0: 1/sample_rate : (array_length-1)/sample_rate);

thrust_desired = 2.85*ones(1,array_length);
pitch_desired = 0 * ones(1, array_length);
roll_desired = -7.0 * ones(1, array_length);

alpha_0_desired = 60*ones(1,array_length);
alpha_1_desired = 60*ones(1,array_length);

beta_0_desired = 90*ones(1,array_length);
beta_1_desired = -90*ones(1,array_length);

time_index_1 = 30*100;
time_index_2 = 60*100;
time_index_3 = 90*100;

thrust_desired(time_index_1: time_index_2) = 2.704;
thrust_desired(time_index_2: time_index_3) = 2.893;

roll_desired(time_index_1: time_index_2) = -9.4;
roll_desired(time_index_2: time_index_3) = -12.0337;

alpha_0_desired(time_index_1: time_index_2) = 60;
alpha_0_desired(time_index_2: time_index_3) = 45;

alpha_1_desired(time_index_1: time_index_2) = 45;
alpha_1_desired(time_index_2: time_index_3) = 45;



figure(1);
plot(time_series, thrust_actual, 'r');
hold on
plot(time_series, thrust_desired, 'b');
hold off
grid on
title('thrust')

figure(2)
plot(time_series, pitch_actual, 'r');
hold on
plot(time_series, pitch_desired, 'b');
hold off
grid on
title('pitch')

figure(3)
plot(time_series, roll_actual, 'r');
hold on
plot(time_series, roll_desired, 'b');
hold off
grid on
title('roll')


% transform the XYZ
distance_leader_payload = sqrt((X_pos_leader - X_pos_payload).^2 + (Y_pos_leader - Y_pos_payload).^2 + (Z_pos_leader - Z_pos_payload).^2);
distance_follower_payload = sqrt((X_pos_follower_1 - X_pos_payload).^2 + (Y_pos_follower_1 - Y_pos_payload).^2 + (Z_pos_follower_1 - Z_pos_payload).^2);

% distance_leader_payload = 1.12;
% distance_follower_paylaod = 1.12;

X_pos_leader_trans = -(X_pos_leader - 0.01);
Y_pos_leader_trans =  Y_pos_leader;
Z_pos_leader_trans = -(Z_pos_leader - 0.03);

X_pos_follower_1_trans = -X_pos_follower_1;
Y_pos_follower_1_trans =  Y_pos_follower_1;
Z_pos_follower_1_trans = -Z_pos_follower_1;

X_pos_payload_trans = -X_pos_payload;
Y_pos_payload_trans =  Y_pos_payload;
Z_pos_payload_trans = -(Z_pos_payload - 0.015);

beta_0 = -atan2(Y_pos_payload_trans - Y_pos_leader_trans, X_pos_payload_trans - X_pos_leader_trans)*57.3;

beta_1 = -atan2(Y_pos_payload_trans - Y_pos_follower_1_trans, X_pos_payload_trans - X_pos_follower_1_trans)*57.3;

alpha_0 = asin((Z_pos_payload_trans - Z_pos_leader_trans)./distance_leader_payload)*57.3;

alpha_1 = asin((Z_pos_payload_trans - Z_pos_follower_1_trans)./distance_follower_payload)*57.3;

figure(4)
plot(time_series, beta_0, 'r')
hold on
plot(time_series, beta_1, 'g')
plot(time_series, beta_0_desired, 'r-.')
plot(time_series, beta_1_desired, 'g-.')
hold off
grid on
title('azimuth angle')
legend('beta_0', 'beta_1', 'beta_0_desired', 'beta_1_desired')

figure(5)
plot(time_series, alpha_0, 'r')
hold on 
plot(time_series, alpha_1, 'g')
plot(time_series, alpha_0_desired, 'r-.')
plot(time_series, alpha_1_desired, 'g-.')
hold off
grid on
title('elevation angle')
legend('alpha_0', 'alpha_1', 'alpha_0_desired', 'alpha_1_desired')

