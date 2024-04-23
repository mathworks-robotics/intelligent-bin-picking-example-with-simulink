function [U,V,W] = makeUVWfromCoeff(coeff)
    visual_fitting = 0.05;
    % 1st principal axis
    U1 = coeff(1, 1) * visual_fitting;
    V1 = coeff(2, 1) * visual_fitting;
    W1 = coeff(3, 1) * visual_fitting;

    % 2nd principal axis
    U2 = coeff(1, 2) * visual_fitting;
    V2 = coeff(2, 2) * visual_fitting;
    W2 = coeff(3, 2) * visual_fitting;

    % 3rd principal axis
    U3 = coeff(1, 3) * visual_fitting;
    V3 = coeff(2, 3) * visual_fitting;
    W3 = coeff(3, 3) * visual_fitting;
    U = [U1;U2;U3];
    V = [V1;V2;V3];
    W = [W1;W2;W3];
end