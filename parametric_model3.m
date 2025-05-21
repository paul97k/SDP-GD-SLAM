
function [A, B, C, D] = parametric_model2(params, Ts, varargin)


    % A = [params(1)  params(2)   params(3);
    %      params(4)  params(5)   params(6);
    %      params(7)  params(8)   params(9)];
    % 
    % B = [params(10)  0          0         ;
    %      0          params(11)  0         ;
    %      0          0          params(12)];
    % 
    A = [1          0          0         Ts  0 0;
         0          1          0         0  Ts 0;
         0          0          1         0  0 Ts;
         % 0          0          0         params(1)  0 0;
         % 0          0          0         0  params(5)   0;
         % 0          0          0         0  0   params(9)];
         0          0          0         params(1)  params(2)   params(3);
         0          0          0         params(4)  params(5)   params(6);
         0          0          0         params(7)  params(8)   params(9)];

% 
B = [params(37)  params(38)  params(39);
     params(40)  params(41)  params(42);
     params(43)  params(44)  params(45);
     params(46)  params(47)  params(48);
     params(49)  params(50)  params(51);
     params(52)  params(53)  params(54)];








   n_inputs = size(B,2);
   n_outputs= size(A,1); %#outputs = #states

    C = eye(n_outputs,n_outputs);
    D = zeros(n_outputs, n_inputs);
end

