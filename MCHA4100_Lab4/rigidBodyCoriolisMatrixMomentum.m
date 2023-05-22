function C = rigidBodyCoriolisMatrixMomentum(p)

% Coriolis matrix in terms of body-fixed momentum
% p_v = p(1:3);
% p_w = p(4:6);
% WHY WOULDN'T THIS WORK!?!?!?!
% C = [...
%     zeros(3,3), -skew(p_v);
%      -skew(p_v), -skew(p_w)];

% pull out momentum variables
p_v = p(1:3); 
p_w = p(4:6); 

% Coriolis matrix in terms of body-fixed momentum
% matrix calculations
X = zeros(3,3);
Y = -skew(p_v);
Z = -skew(p_w);
% construct coriolis matrix for momentum
C = [X,  Y;
     Y, Z];

end