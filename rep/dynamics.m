function [ dxdt ] = dynamics( t,x )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    global m I_z l_f l_r C_f D_f D_r C_m1 C_m2 B_f B_r C_r C_d u
    delta = u(1,:);
    d = u(2,:);
    dxdt = [x(4)*cos(x(3))-x(5)*sin(x(3));%x1
              x(4)*sin(x(3))+x(5)*cos(x(3));%x2
              x(6);%x3
              1/m*(((C_m1-C_m2*x(4))*d-C_r-C_d*x(4)^2)-(D_r*sin(C_r*atan(B_r*(atan((x(6)*l_r-x(5))/x(4))))))*sin(delta)+m*x(5)*x(6));%x4
              1/m*((D_r*sin(C_r*atan(B_r*(atan((x(6)*l_r-x(5))/x(4))))))+(D_f*sin(C_f*atan(B_f*(-atan((x(6)*l_f+x(5))/x(4))+delta))))*cos(delta)-m*x(4)*x(6));%x5
              1/I_z*((D_f*sin(C_f*atan(B_f*(-atan((x(6)*l_f+x(5))/x(4))+delta))))*l_f*cos(delta)-(D_r*sin(C_r*atan(B_r*(atan((x(6)*l_r-x(5))/x(4))))))*l_r);];%x6
%               D_f*sin(C_f*atan(B_f*(-atan((x(6)*l_f+x(5))/x(4))+delta)));%x7
%               D_r*sin(C_r*atan(B_r*(atan((x(6)*l_r-x(5))/x(4)))));%x8
%               (C_m1-C_m2*x(4))*d-C_r-C_d*x(4)^2];%x9

