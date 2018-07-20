function [ dphi_dtheta ] = PHI_prime( theta )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
    %% center diff
    dphi_dtheta = (PHI(theta+0.01)-PHI(theta-0.01))/0.02;
    
end

