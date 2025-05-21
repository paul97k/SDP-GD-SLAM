% Define symbolic variables
clc 
clear

% Define symbolic variables
syms m I_x I_y I_z u v w p q r real
syms X_u X_v X_w X_p X_q X_r Y_v Y_w Y_p Y_q Y_r Z_w Z_p Z_q Z_r K_p K_q K_r M_q M_r N_r real
syms m_xg m_yg m_zg X_w_dot Y_v_dot Z_w_dot I_xx I_yy I_zz I_xy I_xz I_yz real

syms X_uu Y_vv Z_ww K_pp M_qq N_rr real
syms W B theta phi x_g y_g z_g x_b y_b z_b real
syms   g0 tau real

% Define the Coriolis matrix C(ν)
C_nu = [0,  0,   0,    0,   m*w, -m*v;
        0,  0,   0,  -m*w,   0,   m*u;
        0,  0,   0,   m*v, -m*u,   0;
        0, -m*w,  m*v, 0,  I_z*r, -I_y*q;
        m*w,  0, -m*u, I_z*r, 0,  I_x*p;
       -m*v, m*u,  0, -I_y*q, I_x*p, 0];



% Define the Mass-Inertia Matrix M
M = [m - X_u, -X_v, -X_w, -X_p,  m_zg - X_q,  m_yg - X_r;
    -X_v, m - Y_v, -Y_w, -m_zg - Y_p, -Y_q, m_xg - Y_r;
    -X_w, -Y_w, m - Z_w,  m_yg - Z_p, -m_xg - Z_q, -Z_r;
    -X_p, -m_zg - Y_p,  m_yg - Z_p,  I_xx - K_p, -I_xy - K_q, -I_xz - K_r;
     m_zg - X_q, -Y_q, -m_xg - Z_q, -I_xy - K_q, I_yy - M_q, -I_yz - M_r;
     m_yg - X_r, m_xg - Y_r, -Z_r, -I_xz - K_r, -I_yz - M_r, I_zz - N_r];

M =diag([m, m, m, I_xx, I_yy, I_zz]);





% Define the velocity vector |ν|
v_states = [u; v; w; p; q; r];

% Define the linear damping matrix D
D_linear = -diag([X_u, Y_v, Z_w, K_p, M_q, N_r]);

% Define the quadratic damping matrix D_N(ν)
D_quadratic = -diag([X_uu*abs(u), Y_vv*abs(v), Z_ww*abs(w), K_pp*abs(p), M_qq*abs(q), N_rr*abs(r)]);

% Total damping matrix D(ν)
D_nu = D_linear + D_quadratic;





% Define the gravity vector g(η)
g_eta = [(W - B) * sin(theta);
        -(W - B) * cos(theta) * sin(phi);
        -(W - B) * cos(theta) * sin(phi);
        -(y_g*W - y_b*B) * cos(theta) * cos(phi) + (z_g*W - z_b*B) * cos(theta) * sin(phi);
         (z_g*W - z_b*B) * sin(theta) + (x_g*W - x_b*B) * cos(theta) * cos(phi);
        -(x_g*W - x_b*B) * cos(theta) * sin(phi) - (y_g*W - y_b*B) * sin(theta)];

% disp('Coriolis Matrix C(ν):');
% disp(C_nu);
% disp('Mass-Inertia Matrix M:');
% disp(M);
% disp('Damping Matrix D(ν):');
% disp(D_nu);
% disp('Gravity Vector g(η):');
% disp(g_eta);



syms   tau_u tau_v tau_w tau_p tau_q tau_r 
tau = [tau_u; tau_v; tau_w; tau_p; tau_q; tau_r];

% syms u v w p q r real

% Define state vector (velocity)
v_states = [u; v; w; p; q; r];
syms   x y z
syms phi theta psi
eta_states = [x; y; z; phi; theta; psi];

% Define the equation of motion
v_dot = inv(M) * (-C_nu * v_states - D_nu * v_states - g_eta - g0 + tau);


% % Compute the Hessian for each component of nu_dot
% Hessian_nu_dot = cell(6,1); % Store results in a cell array
% 
% disp('Hessian matrices for each component of nu_dot:');
% for i = 1:6
%     Hessian_nu_dot{i} = hessian(nu_dot(i), nu);
%     disp(Hessian_nu_dot{i});
% end
% 
% Define the rotation matrix
T_G_B_eta2 = [cos(psi)*cos(theta), -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
     sin(psi)*cos(theta),  cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi), -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
     -sin(theta),          cos(theta)*sin(phi),                              cos(theta)*cos(phi)];



% Define A^G_B(η2) matrix
A_G_B = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),           -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

zero_3x3 = zeros(3,3);
Transformation_Matrix = [T_G_B_eta2, zero_3x3;
                             zero_3x3,   A_G_B];
eta_dot = Transformation_Matrix * v_states;


disp('Equation of motion (eta_dot):');
disp(eta_dot);
disp('Equation of motion (v_dot):');
disp(v_dot);

% 



disp('jacobian X');
jacobian_X = jacobian([eta_dot; v_dot],[eta_states; v_states])

jacobian_u = jacobian([eta_dot; v_dot],tau);
% disp('jacobian u');
% disp(jacobian_u);

% Define the substitution condition
subs_conditions = {v,q, phi, psi};  % Variables to replace
% subs_conditions = {x,y,z,phi, theta, psi,  u,v,w,p,q,r};  % Variables to replace
subs_conditions = {y, z, phi, theta , psi   ,u,v,w,  p,q,r};  % Variables to replace
subs_values =     [0,-5, 0  ,0      , 0     ,1,0,0,  0,0,0];

% Generate a list of zeros of the same size
% subs_values = num2cell([1 zeros(size(subs_conditions))]);  % Convert array to cell

disp('jacobian X at opertating point');
jacobian_X_OP = subs(jacobian_X, subs_conditions, subs_values)
disp('jacobian u at opertating point');
jacobian_u_OP = subs(jacobian_u, subs_conditions, subs_values)



disp('jacobian X with assumption W=B and x_g, x_b, y_g, z_g = 0');
subs_conditions = {W,B,x_g  , x_b   , y_b   , y_g   , z_b   , z_g};  % Variables to replace
subs_values =     [1,1,0    ,0      , 0     , 0     , 0     , 0];

subs(jacobian_X_OP, subs_conditions, subs_values)
disp('jacobian u with assumption W=B and x_g, x_b, y_g, z_g = 0');
subs(jacobian_u_OP, subs_conditions, subs_values)
%%
clc
A1 = ...
[0, 0, 0,                     0,                     0, 0,                0,     0,     0,        0,        0,        0;
 0, 0, 0,                     0,                     0, 1,                0,     0,     0,        0,        0,        0;
 0, 0, 0,                     0,                    -1, 0,                0,     0,     0,        0,        0,        0;
 0, 0, 0,                     0,                     0, 0,                0,     0,     0,        0,        0,        0;
 0, 0, 0,                     0,                     0, 0,                0,     0,     0,        0,        0,        0;
 0, 0, 0,                     0,                     0, 0,                0,     0,     0,        0,        0,        0];

A2 = ...
[0, 0, 0,                     0,             (B - W)/m, 0, (X_u + 2*X_uu)/m,     0,     0,        0,        0,        0;
 0, 0, 0,            -(B - W)/m,                     0, 0,                0, Y_v/m,     0,        0,        0,       -1;
 0, 0, 0,            -(B - W)/m,                     0, 0,                0,     0, Z_w/m,        0,        1,        0;
 0, 0, 0,  (B*z_b - W*z_g)/I_xx,                     0, 0,                0,     0,     0, K_p/I_xx,        0,        0;
 0, 0, 0,                     0,  (B*z_b - W*z_g)/I_yy, 0,                0,     0,     0,        0, M_q/I_yy,        0;
 0, 0, 0, -(B*x_b - W*x_g)/I_zz, -(B*y_b - W*y_g)/I_zz, 0,                0,     0,     0,        0,        0, N_r/I_zz];
A1+A2
%%
% % Define symbolic variables
% clc 
% clear
% syms m g 
% syms X_u Y_v Z_w
% syms m_xg 
% syms M_iw M_p M_q M_w
% syms x_g y_g z_g 
% syms Z_q 
% syms I_xx I_yy I_zz
% syms K_p Y_r  N_v N_r
% 
% % Define the matrix A
% A = [
%     m - X_u , 0             , 0         , 0             ,   m * z_g       , -m * y_g;
%     0       , m - Y_v       , 0         , -m * z_g      ,   0             ,  m*x_g-Y_r;
%     0       , 0             , m - Z_w   , m * y_g       ,   -m * x_g-Z_q  , 0;
%     0       , -m * z_g      , m * y_g   , I_xx - K_p    ,    0             , 0;
%     m * z_g , 0             , -m*x_g-M_w, 0             ,   I_yy - M_q    , 0;
%     -m * y_g, m * x_g-N_v   , 0         , 0             ,   0             , I_zz - N_r];
% 
% 
% 
% 
% % Define symbolic variables for Euler angles
% syms roll pitch yaw
% 
% % Define the rotation matrix
% T_G_B_eta2 = [cos(yaw)*cos(pitch), -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll), sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll);
%      sin(yaw)*cos(pitch),  cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll), -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll);
%      -sin(pitch),          cos(pitch)*sin(roll),                              cos(pitch)*cos(roll)];
% 
% 
% 
% % Define A^G_B(η2) matrix
% A_G_B = [1, sin(roll)*tan(pitch), cos(roll)*tan(pitch);
%          0, cos(roll),           -sin(roll);
%          0, sin(roll)/cos(pitch), cos(roll)/cos(pitch)];
% 
% 
% syms m u v w p q r x_g y_g z_g X Y Z 
% syms u_ v_ w_ p_ q_ r_
% syms I_xx I_yy I_zz K M N
% 
% % Define the force equations
% eq1 = m * (u_ - v*r + w*q - x_g * (q^2 - r^2) + y_g * (p*q - r) + z_g * (p*r + q_)) == X;
% eq2 = m * (v_ - w*p + u*r - y_g * (p^2 + r^2) + z_g * (q*r - p_) + x_g * (q + r_)) == Y;
% eq3 = m * (w_ - u*q + v*p - z_g * (p^2 + q^2) + x_g * (r*p - q_) + y_g * (r + p_)) == Z;
% 
% % Define the moment equations
% eq4 = I_xx * p_ + (I_zz - I_yy) * q*r + m * (y_g * (w_ - u*q + v*p) - z_g * (v_ - w*p + u*r)) == K;
% eq5 = I_yy * q_ + (I_xx - I_zz) * r*p + m * (z_g * (u_ - v*r + w*q) - x_g * (w_ - u*q + v*p)) == M;
% eq6 = I_zz * r_ + (I_yy - I_xx) * p*q + m * (x_g * (v_ - w*p + u*r) - y_g * (u_ - v*r + w*q)) == N;
% 
% nu = [u; v; w; p; q; r];
% 
% % disp('Full Transformation Matrix:');
% % disp(Transformation_Matrix);
% 
% 
% % Compute relationship between state derivatives and velocities
% 
% 
% % Define transformation matrices
% zero_3x3 = zeros(3,3);
% 
% % Define full transformation matrix
%     Transformation_Matrix = [T_G_B_eta2, zero_3x3;
%                              zero_3x3,   A_G_B];
% eta_dot_relation = Transformation_Matrix * nu;
% disp(eta_dot_relation);


%%

% Define the substitution condition
% Define the substitution condition
subs_conditions = {p, q, roll, pitch};  % Variables to replace

% Generate a list of zeros of the same size
subs_values = num2cell(zeros(size(subs_conditions)));  % Convert array to cell
Transformation_Matrix= subs(Transformation_Matrix, subs_conditions, subs_values);
eq1           = subs(eq1, subs_conditions, subs_values);
eq2           = subs(eq2, subs_conditions, subs_values);
eq3           = subs(eq3, subs_conditions, subs_values);
eq4           = subs(eq4, subs_conditions, subs_values);
eq5           = subs(eq5, subs_conditions, subs_values);
eq6           = subs(eq6, subs_conditions, subs_values);

eta_dot_relation = Transformation_Matrix * v_states;



% tau = A_zero_values * nu;

disp('------------------------------------------------------:');
disp('Equation relating state derivatives to velocities:');
disp(eta_dot_relation);
% disp(tau)
disp(eq1)
disp(eq2)
disp(eq3)
disp(eq4)
disp(eq5)
disp(eq6)