function [v_ack, a_ack] = dif2ack(v_twist, w_twist, d_ack)

v_ack = sqrt(v_twist*v_twist + w_twist*w_twist*d_ack*d_ack);
a_ack = atan(d_ack * w_twist / v_twist) ;
