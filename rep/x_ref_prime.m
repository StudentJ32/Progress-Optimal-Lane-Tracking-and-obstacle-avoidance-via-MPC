function [ x_prime ] = x_ref_prime( theta )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    global p_dx
    x_prime = ppval(p_dx,theta);

end

