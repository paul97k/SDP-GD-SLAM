
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



    B = [0          0          0        0          0           0;
         0          0          0        0          0           0;
         0          0          0        0          0           0;
         % 0          0          0        params(10)  0 0;
         % 0          0          0        0 params(14)          0;
         % 0          0          0        0 0 params(18)];
         0          0          0        params(10) params(11) params(12);
         0          0          0        params(13) params(14) params(15);
         0          0          0        params(16) params(17) params(18)];

   n_inputs = size(B,2);
   n_outputs= size(A,1); %#outputs = #states

    C = eye(n_outputs,n_inputs);
    D = zeros(n_outputs, n_inputs);
end

