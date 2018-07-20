clc
clear

global m I_z l_f l_r C_f D_f D_r C_m1 C_m2 B_f B_r C_r C_d u
state_dim = 6;
control_dim = 2;
model_dim = 13;


model = cell(model_dim);
for i = 1:length(model)
    model{i} = rand;
end
[m I_z l_f l_r C_f D_f D_r C_m1 C_m2 B_f B_r C_r C_d] = model{:};


%% trajectory information
    p = 0:0.01:6*pi;
    x_t = 5*cos(p);
    y_t = 5*sin(p);
    global pp_x pp_y p_dx p_dy
    pp_x = spline(p,x_t);
    pp_y = spline(p,y_t);
    p_dx = fnder(pp_x,p);
    p_dy = fnder(pp_y,p);
    
 global L 
 L = p(end);


%% system sampling
Ts = 0.02;
N = 40;


x = [5;0.01;rand(4,1)];
u = rand(control_dim,1);
theta_a = 0;
v = 0;
theta_ref = zeros(N,1);
%% initialize state traj
for i =1:N
    [A_d{i},B_d{i}] = Linearized_Model(x,u,Ts);
end

[ x_traj,u_traj,v_traj,theta_traj ] = MPCSolver(x,u,v,theta_a,N,Ts,A_d,B_d,theta_ref);
u = u_traj(:,1);
theta_a = theta_traj()
[~,x_sim] = ode45(@dynamics,[0 Ts],x);
x_hist(1,:) = x_sim(end,:);
% u = u_traj(:,end);
% [~,x_sim_end] = ode45(@dynamics,[0 Ts],x_sim(end,:));

for i = 1:100
    x = x_sim(end,:);
    lin_x = x_traj(:,2:end);
    lin_u = u_traj(:,2:end);
    for j =1:N
        [A_d{i},B_d{i}] = Linearized_Model(lin_x(:,j),lin_u(:,j),Ts);
    end
    theta_ref = theta_traj(2:end);
    [ x_traj,u_traj,v_traj,theta_traj ] = MPCSolver(x,u,v,theta_a,N,Ts,A_d,B_d,theta_ref);
    u = u_traj(:,1);
    [~,x_sim] = ode45(@dynamics,[0 Ts],x);
    x_hist(i+1,:) = x_sim(end,:);
end


