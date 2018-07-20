function [ phi ] = PHI( theta )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    %% trajectory
    global p_dx p_dy

    %% phi calculation
    phi = atan(ppval(p_dy,theta)/ppval(p_dx,theta));

end

