
function [A, B, C, D] = parametric_model2(params, dt, varargin)


    % A = [params(1)  params(2)   params(3);
    %      params(4)  params(5)   params(6);
    %      params(7)  params(8)   params(9)];
    % 
    % B = [params(10)  0          0         ;
    %      0          params(11)  0         ;
    %      0          0          params(12)];
    % 
A = [params(1)  params(2)   params(3)   params(4)   params(5)   params(6);
     params(7)  params(8)   params(9)   params(10)  params(11)  params(12);
     params(13) params(14)  params(15)  params(16)  params(17)  params(18);
     params(19) params(20)  params(21)  params(22)  params(23)  params(24);
     params(25) params(26)  params(27)  params(28)  params(29)  params(30);
     params(31) params(32)  params(33)  params(34)  params(35)  params(36)];

% 
% A = diag([params(1)  params(2)   params(3)   params(4)   params(5)   params(6)]);
B = [params(37)  0           0           0           0           0;
     0           params(38)  0           0           0           0;
     0           0           params(39)  0           0           0;
     0           0           0           params(40)  0           0;
     0           0           0           0           params(41)  0;
     0           0           0           0           0           params(42)];








   n_inputs = size(B,2);
   n_outputs= size(A,1); %#outputs = #states

    C = eye(n_outputs,n_inputs);
    D = zeros(n_outputs, n_inputs);
end

