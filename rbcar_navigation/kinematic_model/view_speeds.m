v_ack = 0:0.05:1;
a_ack = -[0:0.05:1];
d_ack = 1.65;

V = [];
W = [];
for v=v_ack
    [vd, wd] = ack2dif(v, a_ack, d_ack);
    V = [V; vd];
end

plot(vd, wd)
axis square;