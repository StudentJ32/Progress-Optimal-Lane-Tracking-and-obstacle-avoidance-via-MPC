function [ x_traj,u_traj,v_traj,theta_traj ] = MPCSolver(x_0,u_0,v_0,theta_0,N,Ts,A,B,theta_ref)%[ x_traj,u_traj,v_traj,theta_traj ] = MPCSolver(x_0,u_0,v_0,theta_0,N,Ts,A,B,theta_ref)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    global L 
    wid = 0.1; %
    Q = diag([1,1,0.5,0.2,0.3,0.1,0.2,0.1,0.5]);
    c = 0.1*ones(10);
    gamma = 0.1;
    nx = 6;
    nu = 2;
    u = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));
    x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    theta_a = sdpvar(repmat(1,1,N+1),repmat(1,1,N+1));
    v = sdpvar(repmat(1,1,N+1),repmat(1,1,N+1));
    
    constraints = [];
    objective = 0;
    for k = 1:N
        tan_phi = y_ref_prime(theta_ref(k))/x_ref_prime(theta_ref(k));
        E_c = sin(PHI(theta_ref(k)))*(x{k+1}(1)-x_ref(theta_ref(k)))-cos(PHI(theta_ref(k)))*(x{k+1}(2)-y_ref(theta_ref(k)))+...
              (PHI_prime(theta_ref(k))*cos(PHI(theta_ref(k)))*(x{k+1}(1)-x_ref(theta_ref(k)))+sin(PHI(theta_ref(k)))*(-x_ref_prime(theta_ref(k)))...
              +PHI_prime(theta_ref(k))*sin(PHI(theta_ref(k)))*(x{k+1}(2)-y_ref(theta_ref(k)))-cos(PHI(theta_ref(k)))*(-y_ref_prime(theta_ref(k))))...
              *(theta_a{k+1}-theta_ref(k));
        E_l = -cos(PHI(theta_ref(k)))*(x{k+1}(1)-x_ref(theta_ref(k)))-sin(PHI(theta_ref(k)))*(x{k+1}(2)-y_ref(theta_ref(k)))+...
              (PHI_prime(theta_ref(k))*sin(PHI(theta_ref(k)))*(x{k+1}(1)-x_ref(theta_ref(k)))-cos(PHI(theta_ref(k)))*(-x_ref_prime(theta_ref(k)))...
              -PHI_prime(theta_ref(k))*cos(PHI(theta_ref(k)))*(x{k+1}(2)-y_ref(theta_ref(k)))-sin(PHI(theta_ref(k)))*(-y_ref_prime(theta_ref(k))))...
              *(theta_a{k+1}-theta_ref(k));
        E_u = (u{k+1}-u{k})'*diag([0.1,0.1])*(u{k+1}-u{k});
        E_v = (v{k+1}-v{k})*0.1*(v{k+1}-v{k});
        objective = objective + 1*E_c^2+5*E_l^2 - gamma*v{k+1}*Ts-gamma*v_0*Ts+E_u+E_v;
        hall_constrains = [-tan_phi,1; tan_phi -1]*[x{k+1}(1) x{k+1}(2)]' <= [-tan_phi*x_ref(theta_ref(k))+y_ref(theta_ref(k))+wid/(cos(pi-PHI(theta_ref(k))));...
            tan_phi*x_ref(theta_ref(k))-y_ref(theta_ref(k))-wid/(cos(pi-PHI(theta_ref(k))))];
        dynamics_constrains = [x{k+1}==A{k}*x{k}+B{k}*u{k+1},theta_a{k+1}==theta_a{k}+v{k+1}/Ts];
        state_constrains = [0<= x{k+1}(4)<=2, 0<= x{k+1}(5)<=2, 0<= theta_a{k+1}<=L,0<=v{k+1}<=2];
        constraints = [constraints, dynamics_constrains,state_constrains];
    end
    solutions_out = {[u{:}],[x{:}],[v{:}],[theta_a{:}]};
    initial_conditions = {x{1},v{1},theta_a{1},u{1}};
    controller = optimizer(constraints, objective,sdpsettings('solver','sedumi'),initial_conditions,solutions_out);
    input = {x_0,v_0,theta_0,u_0};
    [solutions, diagnal] = controller{input};
    if diagnal ==1
        error('The problem is infeasible');
    end
    
    u_traj = solutions{1};
    x_traj = solutions{2};
    v_traj = solutions{3};
    theta_traj = solutions{4};
end

