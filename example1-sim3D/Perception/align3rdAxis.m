function coeff_aligned = align3rdAxis(ptCloud, coeff, center)
    ptCloud_aligned = pctransform(ptCloud, rigid3d(eye(3), -center));
    ptCloud_aligned = pctransform(ptCloud_aligned, rigid3d(coeff, [0 0 0]));

    points_z = ptCloud_aligned.Location(:, 3);
    posy = points_z > 0;

    moments_pos = sum(points_z(posy) .^ 2);
    moments_neg = sum(points_z(~posy) .^ 2);

    coeff_rot = coeff;
    coeff_rot(:, 1) = -coeff(:, 1);
    coeff_rot(:, 3) = -coeff(:, 3);

    if (moments_pos > moments_neg)
        coeff_aligned = coeff;
    else
        coeff_aligned = coeff_rot;
    end
end