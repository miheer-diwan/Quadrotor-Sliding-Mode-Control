function x = traj(q_o,q_f,t_o,t_f)

syms t
t1 = [t_o:0.1:t_f];

for i = 1:length(q_o)

    qo = q_o(i);
    qf = q_f(i);
    
    q_traj = [1 t t.^2 t.^3 t.^4 t.^5];
    q_traj_dot = [0 1 2*t 3*t.^2 4*t.^3 5*t.^4];
    q_traj_ddot = [0 0 2 6*t 12*t.^2 20*t.^3];
    
    %% initial and final velocity and acc is 0
    qo_dot = 0;
    qo_ddot = 0;
    
    qf_dot = 0;
    qf_ddot = 0;
    
    A = [ 1 t_o t_o^2 t_o^3 t_o^4 t_o^5; 
        0 1 2*t_o 3*t_o^2 4*t_o^3 5*t_o^4;
        0 0 2 6*t_o 12*t_o^2 20*t_o^3; 
        1 t_f t_f^2 t_f^3 t_f^4 t_f^5; 
        0 1 2*t_f 3*t_f^2 4*t_f^3 5*t_f^4; 
        0 0 2 6*t_f 12*t_f^2 20*t_f^3];
    
    b = [qo ;qo_dot; qo_ddot ;qf; qf_dot; qf_ddot];
    
    a = A\b;
    
    qd = simplify(subs(q_traj * a,t,t1))
    qd_dot = subs(q_traj_dot * a,t,t1)
    qd_ddot= subs(q_traj_ddot *a, t , t1)
    %% Plotting
%     figure;
    subplot(3,1,1)
    hold on;
    plot(t1,qd,'linewidth',2,'Color','r');
    xlabel('time','FontSize',14)
    ylabel({'$q_d (rad)$'}, 'Interpreter', 'latex','FontSize',16);
    
    subplot(3,1,2)
    hold on;
    plot(t1,qd_dot,'linewidth',2,'Color','b');
    xlabel('time','FontSize',14)
    ylabel({'$\dot{q}_d (rad/s)$'}, 'Interpreter', 'latex','FontSize',16);
    
    subplot(3,1,3)
    hold on;
    plot(t1,qd_ddot,'linewidth',2,'Color','g');
    xlabel('time','FontSize',14)
    ylabel({'$\ddot{q}_d (rad/s^2)$'}, 'Interpreter', 'latex','FontSize',16);


end



