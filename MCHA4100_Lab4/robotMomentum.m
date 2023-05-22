function p3 = robotMomentum(x,param)

% Extract states
pr = x(1:2);   % Reduced momentum

% matrix calculations
% task 1g
Bc = param.Bc;
Bcp = param.Bcp;

% get precalculated M3 from parameter function
M3 = param.M3;


% Compute p from pr
A = [Bc' / M3; Bcp];
B = [0; pr];
p3 = A\B;

% old calculations
% get precalculated constraint matrices from parameter function
% Bc = param.Bc;
% Bcp = param.Bcp;
% 
% % get precalculated M3 from parameter function
% M3 = param.M3;


