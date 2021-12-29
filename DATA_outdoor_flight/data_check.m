% close all
% 
% % if exist('pitch_cmd')
% %     figure(1)
% %     plot(pitch_cmd);
% %     grid on;
% %     title('pitch_cmd');
% % end
% 
% if exist('pos_z')
%     pos_z_des = zeros(1, length(pos_z));
%     pos_z_fdk = zeros(1, length(pos_z));
%     pos_z_err = zeros(1, length(pos_z));
%     pos_z_err_inte = zeros(1, length(pos_z));
%     
%     for i = 1: 1: length(pos_z)
%         pos_z_des(i) = pos_z{i}.des;
%         pos_z_fdk(i) = pos_z{i}.fdk;
%         pos_z_err(i) = pos_z{i}.err;
%         pos_z_err_inte(i) = pos_z{i}.err_inte;
%     end
%     
%     figure(2)
%     plot(pos_z_des)
%     hold on
%     plot(pos_z_fdk)
%     plot(pos_z_err)
%     plot(pos_z_err_inte)
%     hold off
%     grid on
%     title('pos_z')
%     legend('des', 'fdk', 'err', 'err_inte')
% end
% 
% 
% if exist('vel_z')
%     vel_z_des = zeros(1, length(vel_z));
%     vel_z_fdk = zeros(1, length(vel_z));
%     vel_z_err = zeros(1, length(vel_z));
%     vel_z_err_inte = zeros(1, length(vel_z));
%     
%     for i = 1: 1: length(vel_z)
%         vel_z_des(i) = vel_z{i}.des;
%         vel_z_fdk(i) = vel_z{i}.fdk;
%         vel_z_err(i) = vel_z{i}.err;
%         vel_z_err_inte(i) = vel_z{i}.err_inte;
%     end
%     
%     figure(3)
%     plot(vel_z_des)
%     hold on
%     plot(vel_z_fdk)
%     plot(vel_z_err)
%     plot(vel_z_err_inte)
%     hold off
%     grid on
%     title('vel_z')
%     legend('des', 'fdk', 'err', 'err_inte')
% end
% 
% 
% if exist('vel_x')
%     vel_x_des = zeros(1, length(vel_x));
%     vel_x_fdk = zeros(1, length(vel_x));
%     vel_x_err = zeros(1, length(vel_x));
%     vel_x_err_inte = zeros(1, length(vel_x));
%     
%     for i = 1: 1: length(vel_x)
%         vel_x_des(i) = vel_x{i}.des;
%         vel_x_fdk(i) = vel_x{i}.fdk;
%         vel_x_err(i) = vel_x{i}.err;
%         vel_x_err_inte(i) = vel_x{i}.err_inte;
%     end
%     
%     figure(4)
%     plot(vel_x_des)
%     hold on
%     plot(vel_x_fdk)
%     plot(vel_x_err)
%     plot(vel_x_err_inte)
%     hold off
%     grid on
%     title('vel_x')
%     legend('des', 'fdk', 'err', 'err_inte')
% end
% 
% if exist('vel_y')
%     vel_y_des = zeros(1, length(vel_y));
%     vel_y_fdk = zeros(1, length(vel_y));
%     vel_y_err = zeros(1, length(vel_y));
%     vel_y_err_inte = zeros(1, length(vel_y));
%     
%     for i = 1: 1: length(vel_y)
%         vel_y_des(i) = vel_y{i}.des;
%         vel_y_fdk(i) = vel_y{i}.fdk;
%         vel_y_err(i) = vel_y{i}.err;
%         vel_y_err_inte(i) = vel_y{i}.err_inte;
%     end
%     
%     figure(5)
%     plot(vel_y_des)
%     hold on
%     plot(vel_y_fdk)
%     plot(vel_y_err)
%     plot(vel_y_err_inte)
%     hold off
%     grid on
%     title('vel_y')
%     legend('des', 'fdk', 'err', 'err_inte')
% end
% 
% figure(6)
% plot(thrust_cmd_leader);
% hold on
% plot(roll_cmd)
% plot(pitch_cmd)
% hold off
% grid on
% legend('thrust', 'roll', 'pitch')

close all

% figure(2)
% plot(pos_z_des)
% hold on
% plot(pos_z_fdk)
% plot(pos_z_err)
% plot(pos_z_err_inte)
% hold off
% grid on
% title('pos_z')
% legend('des', 'fdk', 'err', 'err_inte')
% 
% figure(3)
% plot(vel_z_des)
% hold on
% plot(vel_z_fdk)
% plot(vel_z_err)
% plot(vel_z_err_inte)
% hold off
% grid on
% title('vel_z')
% legend('des', 'fdk', 'err', 'err_inte')
% 
% figure(4)
% plot(vel_x_des)
% hold on
% plot(vel_x_fdk)
% plot(vel_x_err)
% plot(vel_x_err_inte)
% hold off
% grid on
% title('vel_x')
% legend('des', 'fdk', 'err', 'err_inte')
% 
% figure(5)
% plot(vel_y_des)
% hold on
% plot(vel_y_fdk)
% plot(vel_y_err)
% plot(vel_y_err_inte)
% hold off
% grid on
% title('vel_y')
% legend('des', 'fdk', 'err', 'err_inte')
% 
P_gain = 15000;
I_gain = 500;
D_gain = 8000;
feedward_term = 31000;
cmd_matlab = P_gain*pos_z_err + I_gain*pos_z_err_inte + D_gain*vel_z_err + feedward_term;

% figure(6)
% plot(thrust_cmd_leader);
% hold on
% plot(roll_cmd)
% plot(pitch_cmd)
% plot(cmd_matlab)
% hold off
% grid on
% legend('thrust', 'roll', 'pitch', 'thrust_matlab')

figure(6)
plot(thrust_cmd_leader);
hold on
plot(cmd_matlab)
plot(thrust_int)
plot(abs(thrust_int))
plot(thrust_int(37:end))
hold off
grid on
legend('thrust_cmd_leader', 'cmd_matlab', 'thrust_int', 'abs_thrust_int')

figure(7)
plot(thrust_cmd_leader);
hold on
plot(rec_thrust)
hold off


