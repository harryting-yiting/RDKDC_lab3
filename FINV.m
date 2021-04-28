function h_inv = FINV(h)
    h_inv = zeros(4,4);
    h_inv(4,4) = 1;
    R = h(1:3, 1:3);
    p = h(1:3, 4);
    h_inv(1:3, 1:3) = R.';
    h_inv(1:3, 4) = -R.' * p;
end