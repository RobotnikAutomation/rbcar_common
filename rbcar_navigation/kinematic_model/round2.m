function m = round2(n, dec)

signo = sign(n);
n = abs(n);
r = abs(rem(n, dec));
suma = r > (dec/2);

m = signo .* (n - r + suma * dec);
end