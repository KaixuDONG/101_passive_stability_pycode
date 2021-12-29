close all
time_stemp = (0: length(pitch)-1)/100;

figure(1)
plot(time_stemp, modeled_thrust)
title('thrust')

figure(2)
plot(time_stemp, roll)
title('roll')


figure(3)
plot(time_stemp, pitch)
title('pitch')


figure(4)
plot(time_stemp, roll_rate)
title('roll_rate')


figure(5)
plot(time_stemp, pitch_rate)
title('pitch_rate')
