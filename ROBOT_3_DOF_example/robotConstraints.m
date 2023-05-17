function [Bc,Bcp] = robotConstraints(param)

l = param.l;


Bc = [0; 1; l];     % constraint matrix

Bcp = [1, 0, 0;     % left annilhilator  
       0, -l, 1];
