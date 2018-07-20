function [ y_prime ] = y_ref_prime( theta )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    global p_dy
    y_prime = ppval(p_dy,theta);

end

