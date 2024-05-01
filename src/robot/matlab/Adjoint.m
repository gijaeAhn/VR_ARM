function ad_ret = Adjoint(T)
    [R, p] = TransToRp(T); % Assuming TransToRp returns R and p (position vector)
    zeroes = zeros(3, 3);
    ad_ret = zeros(6, 6);
    ad_ret(1:3, 1:3) = R;
    ad_ret(1:3, 4:6) = zeroes;
    ad_ret(4:6, 1:3) = VecToso3(p) * R; % Assuming VecToso3 computes the skew-symmetric matrix
    ad_ret(4:6, 4:6) = R;
end
