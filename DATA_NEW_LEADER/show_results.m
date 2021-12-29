close all

figure(1)
plot(leader_pos_z_mocap, 'b');
hold on
plot(leader_pos_z_state, 'r--');
plot(leader_pos_z_filter, 'g.-');
hold off
grid on
title('the position feedback and state estimation')
legend('mocap', 'state', 'filter');

figure(2)
plot(leader_vel_z_mocap, 'b');
hold on
plot(leader_vel_z_state, 'r--');
plot(leader_vel_z_filter, 'g.-');
hold off
grid on
title('the veloctiy feedback and state estimation')
legend('mocap', 'state', 'filter');


