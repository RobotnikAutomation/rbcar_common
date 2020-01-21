function [v_twist, w_twist, c] = ack2dif(v_ack, a_ack, d_ack, c)

v_twist = cos(a_ack) .* v_ack;
w_twist = -sin(a_ack) .* v_ack ./ d_ack;

if nargin > 3
    if numel(c) == 1
        c = c * ones(size(v_twist));
    end
else
    c = [];
end