[gamma, thrust] = equilibrium_calculate(0.123, 0.227, pi/4, pi/4);


function [gamma, thrust] = equilibrium_calculate(mp, m_f_1, alpha_0, alpha_1)
    g = 9.81;
    tension = mp*g/(sin(alpha_1) + cos(alpha_1)/cos(alpha_0)*sin(alpha_0));
    % thrust in Newton
    thrust = sqrt(tension^2 + (m_f_1*g)^2 + 2*m_f_1*g*tension*sin(alpha_1));
    % in degree
    gamma = -asin(tension*cos(alpha_1)/thrust)*180/pi;
end