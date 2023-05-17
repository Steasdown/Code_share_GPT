function [value,isterminal,direction] = eventCollision(t,x,param)

value = 1;
isterminal = 1;
direction = 0;

q = x(3:5);
rBNn = [q(1:2);0];
Rnb = [ ...
    cos(q(3)), -sin(q(3)), 0;
    sin(q(3)), cos(q(3)), 0;
    0, 0, 1];

persistent rOCb_all
if isempty(rOCb_all)
    % Outline for collision testing
    a = 0.15;
    r = 0.2;
    theta = linspace(3*pi/2,pi/2+1/pi,9-1);
    alpha = linspace(0,1-1/3,3-1);
    beta = linspace(0,1-1/5,5-1);
    rOCb_all = [ (1-alpha).*[0;r;0] + alpha.*[a;r;0], (1-beta).*[a;r;0] + beta.*[a;-r;0],(1-alpha).*[a;-r;0] + alpha.*[0;-r;0], r*[cos(theta);sin(theta);zeros(size(theta))] ];
end

rONn_all = Rnb*(rOCb_all + param.rCBb) + rBNn;

for i = 1:size(rONn_all,2)
    rONn = rONn_all(:,i);
    Nidx = find(param.Ngridlines <= rONn(1), 1, 'last');
    Eidx = find(param.Egridlines <= rONn(2), 1, 'last');
    if isempty(Nidx) || isempty(Eidx)
        continue
    end
    if param.map(Nidx,Eidx)
        value = 0;
        return
    end
end