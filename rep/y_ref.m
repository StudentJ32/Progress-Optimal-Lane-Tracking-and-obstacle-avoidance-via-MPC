function [ y ] = y_ref( theta )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
global pp_y
y = ppval(pp_y,theta);
end

