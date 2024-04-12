% TTK4135 - Helicopter lab
% Hints/template for problem 2.
% Updated spring 2018, Andreas L. Flåten

%% Initialization and model definition
init_lab4; % Change this to the init file corresponding to your helicopter

global nx N
% Continuous time system model. x = [lambda r p p_dot e e_dot]'

Ac = [0   1       0          0            0           0;
      0   0     -K_2         0            0           0;
      0   0       0          1            0           0;
      0   0  -K_1.*K_pp  -K_1.*K_pd       0           0;
      0   0       0          0            0           1;
      0   0       0          0       -K_3.*K_ep  -K_3.*K_ed];
Bc = [ 0         0;
       0         0;
       0         0;
      K_1.*K_pp  0;
       0         0;
       0     K_3.*K_ep];

% Number of states and inputs
nx = size(Ac,2); % Number of states (number of columns in A)
nu = size(Bc,2); % Number of inputs(number of columns in B)

% Discrete time system model using Forward Euler Method. x = [lambda r p p_dot e e_dot]'
delta_t	= 0.25; % sampling time
I = eye(nx); 
Ad = I + delta_t*Ac;
Bd = delta_t*Bc;

% Initial values
x1_0 = pi;                               % Lambda
x2_0 = 0;                               % r
x3_0 = 0;                               % p
x4_0 = 0;                               % p_dot
x5_0 = 0;
x6_0 = 0;
x0 = [x1_0 x2_0 x3_0 x4_0 x5_0 x6_0]';           % Initial values

% Time horizon and initialization
N  = 40;                               % Time horizon for states
M  = N;                                 % Time horizon for inputs
n = N.*nx+M.*nu;
z  = zeros(n,1);                % Initialize z for the whole horizon
z0 = z;                                 % Initial value for optimization


% Bounds
ul 	    = [-30.*pi/180   -inf]' ;                % Lower bound on control
uu 	    = [30.*pi/180   inf]';                   % Upper bound on control
xl      = -Inf*ones(nx,1);              
xu      = Inf*ones(nx,1);               
xl(3)   = ul(1);                           % Lower bound on state x3 (p)
xu(3)   = uu(1);                           % Upper bound on state x3 (p)

% Generate constraints on measurements and inputs
[vlb,vub]       = gen_constraints(N,M,xl,xu,ul,uu); % hint: gen_constraints
vlb(N*nx+M*nu)  = 0;                    % We want the last input to be zero
vub(N*nx+M*nu)  = 0;                    % We want the last input to be zero

% Generate the matrix Q and the vector c (objecitve function weights in the QP problem) 
Q1 = diag([2 0 0 0 0 0]);               % Weight on states (lambda r p p_dot e e_dot)
P1 = zeros(nu,nu);                      % Weight on input (q_1 = 1..., q_2 = q_1)
q_1 = 1; q_2 = q_1;
P1(1,1) = q_1;
P1(2,2) = q_2;
Q = gen_q(Q1,P1,N,M);                  % Generate Q, hint: gen_q
%c = zeros(n,1);                        % Generate c, this is the linear constant term in the QP

%% Generate system matrixes for linear model
Aeq = gen_aeq(Ad,Bd,N,nx,nu);             % Generate A, hint: gen_aeq
beq = zeros(size(Aeq,1),1);             % Generate b
beq(1:nx) = Ad*x0;

%% Solve SQP 
% tic
% [z,lambda] = quadprog(Q, c, [], [], Aeq, beq, vlb, vub, x0); % hint: quadprog. Type 'doc quadprog' for more info 
% t1=toc;
fun = @(z) 0.5*z'*Q*z;
nonlcon = @constraints;
options = optimoptions("fmincon","MaxFunctionEvaluations",Inf,'MaxIter', 60000);
tic
[z, fval] = fmincon(fun, z0, [], [], Aeq, beq, vlb, vub, nonlcon, options);
t1 = toc;


% Calculate objective value
phi1 = 0.0;
PhiOut = zeros(N*nx+M*nu,1);
for i=1:N*nx+M*nu
  phi1=phi1+Q(i,i)*z(i)*z(i);
  PhiOut(i) = phi1;
end

%% Extract control inputs and states
u1  = [z(N*nx+1:2:N*nx+M*nu);z(n-1)]; % Control input from solution
u2  = [z(N*nx+2:2:N*nx+M*nu);z(n)]; 

x1 = [x0(1);z(1:nx:N*nx)];              % State x1 from solution
x2 = [x0(2);z(2:nx:N*nx)];              % State x2 from solution
x3 = [x0(3);z(3:nx:N*nx)];              % State x3 from solution
x4 = [x0(4);z(4:nx:N*nx)];              % State x4 from solution
x5 = [x0(5);z(5:nx:N*nx)];              % State x5 from solution
x6 = [x0(6);z(6:nx:N*nx)];              % State x6 from solution

num_variables = 5/delta_t;
zero_padding = zeros(num_variables,1);
unit_padding  = ones(num_variables,1);

u1   = [zero_padding; u1; zero_padding];
u2   = [zero_padding; u2; zero_padding];
x1  = [pi*unit_padding; x1; zero_padding];
x2  = [zero_padding; x2; zero_padding];
x3  = [zero_padding; x3; zero_padding];
x4  = [zero_padding; x4; zero_padding];
x5  = [zero_padding; x5; zero_padding];
x6  = [zero_padding; x6; zero_padding];

% %% LQ-controller 
% %Calculate K with weighting matrices Q and R
 Q_lqr = diag([22 1 30 6 15 8]); % Penalizing the state vectors => Q = diag([lambda r p pdot])' 
 R_lqr = diag([0.1 0.1]); % R = Input pref = Vd
 [K_lqr, S, e] = dlqr(Ad , Bd, Q_lqr, R_lqr);


%% Plotting
u = [u1 u2];
t = 0:delta_t:delta_t*(length(u)-1);

p_ref = timeseries(u, t);

x_opt = timeseries([x1 x2 x3 x4 x5 x6], t);



loaded_data=importdata("output_lab4_q_1_Q_22_1_30_6_15_8_R_0.1_0.1.mat");

time_dat=loaded_data(1, :);
travel_dat=loaded_data(2, :);
travel_rate_data=loaded_data(3, :);
pitch_dat=loaded_data(4, :);
pitch_rate_dat=loaded_data(5, :);
elevation_dat=loaded_data(6, :);
elevation_rate_dat=loaded_data(7, :);

%% Test plot

figure(1)
subplot(211)
stairs(t,u(:,1)),grid
ylabel('u1')
subplot(212)
stairs(t,u(:,2)),grid
ylabel('u2')

 figure(3)
 subplot(611)
 hold on;
 plot(time_dat, travel_dat, 'linewidth', 1);
 plot(t,x1,'.',t,x1,'m'),grid
 ylabel('lambda')
 hold off;
 subplot(612)
 hold on;
 plot(time_dat, travel_rate_data, 'linewidth', 1);
 plot(t,x2,'.',t,x2','m'),grid
 ylabel('r')
 hold off;
 subplot(613)
 hold on;
 plot(time_dat, pitch_dat, 'linewidth', 1);
 plot(t,x3,'.',t,x3,'m'),grid
 ylabel('p')
 hold off;
 subplot(614)
 hold on;
 plot(time_dat, pitch_rate_dat, 'linewidth', 1);
 plot(t,x4,'.',t,x4','m'),grid
 xlabel('tid (s)'),ylabel('pdot')
 hold off;
 subplot(615)
 hold on;
 plot(time_dat, elevation_dat, 'linewidth', 1);
 plot(t,x5,'.',t,x5','m'),grid
 xlabel('tid (s)'),ylabel('e')
 hold off;
 subplot(616)
 hold on;
plot(time_dat, elevation_rate_dat, 'linewidth', 1);
 plot(t,x6,'.',t,x6','m'),grid
 xlabel('tid (s)'),ylabel('edot')
 hold off;
