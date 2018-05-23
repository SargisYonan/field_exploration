clc;
clear all;
close all;

u1 = UAV([1 1])
u1.set_radius(.1)
u1.set_destination([1 5])


u2 = UAV([8 8])
u2.set_radius(.1)
u2.set_destination([1 5])

u1.start_flight()
u2.start_flight()

figure()

plot(u1.curr_pos(1), u1.curr_pos(2), 'k^')
hold on
plot(u2.curr_pos(1), u2.curr_pos(2), 'r^')
hold on

for i = 1:1000
    u1.update()
    u2.update()
    
    plot(u1.curr_pos(1), u1.curr_pos(2), 'k^')
    hold on
    plot(u2.curr_pos(1), u2.curr_pos(2), 'r^')
    hold on
end

