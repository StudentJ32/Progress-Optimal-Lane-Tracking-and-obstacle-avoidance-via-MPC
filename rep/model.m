clc
clear

syms X Y phi v_x v_y w F_fy F_ry F_rx %states
syms delta d %control
syms m I_z l_f l_r C_f D_f D_r C_m1 C_m2 B_f B_r C_r C_d

states = [X Y phi v_x v_y w ]; %F_fy F_ry F_rx 
control = [delta d];
transition = [v_x*cos(phi)-v_y*sin(phi);
              v_x*sin(phi)+v_y*cos(phi);
              w;
              1/m*( (C_m1-C_m2*v_x)*d-C_r-C_d*v_x^2-(D_r*sin(C_r*atan(B_r*(atan((w*l_r-v_y)/v_x)))))*sin(delta)+m*v_y*w);
              1/m*((D_r*sin(C_r*atan(B_r*(atan((w*l_r-v_y)/v_x)))))+(D_f*sin(C_f*atan(B_f*(-atan((w*l_f+v_y)/v_x)+delta))))*cos(delta)-m*v_x*w);
              1/I_z*((D_f*sin(C_f*atan(B_f*(-atan((w*l_f+v_y)/v_x)+delta))))*l_f*cos(delta)-(D_r*sin(C_r*atan(B_r*(atan((w*l_r-v_y)/v_x)))))*l_r);]
       
%               D_f*sin(C_f*atan(B_f*(-atan((w*l_f+v_y)/v_x)+delta)));
%               D_r*sin(C_r*atan(B_r*(atan((w*l_r-v_y)/v_x))));
%               (C_m1-C_m2*v_x)*d-C_r-C_d*v_x^2]; 
          
A = jacobian(transition,states)
B = jacobian(transition,control)
