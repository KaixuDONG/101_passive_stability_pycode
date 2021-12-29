close all
figure(1)
plot(pos_z_des)
hold on
plot(pos_z_fdk_mocap)
plot(pos_z_fdk_optical)
% plot(opticalz)

hold off
grid on
legend('des', 'mocap', 'optical')
title('position')


figure(2)
plot(vel_z_fdk_mocap)
hold on
plot(vel_z_fdk_optical)
% plot(opticalvz)

hold off
grid on 
legend('mocap', 'optical')
title('velocity')