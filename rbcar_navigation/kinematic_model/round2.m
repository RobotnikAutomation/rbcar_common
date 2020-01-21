function m = round2(n, dec)

r = abs(rem(n, dec));
suma = r > (dec/2);

m = n - r + suma * dec;