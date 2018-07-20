function [ x ] = x_ref( theta )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
global pp_x
x = ppval(pp_x,theta);

end

