function [p,R0n]=fwdkinrecursion(i,R,q,type,H,P,n)
%based off of Wen's Lecture code
%Euler Rodrigus
h_i=H(1:3,i);
R0=R;

% set up link vector and rotational matrix
if type(i)==0 %revolute
    p_i1_i=P(1:3,i);
    R=R*(rot(h_i,q(i)));
else %prismatic
    p_i1_i=P(1:3,i)+q(i)*h_i;
    R=R;
end

%end of chain: get last link vector
% not end of chain: keeps on propagating by calling itself

if i==n
    p=R*P(1:3,i+1);
    R0n=R;
else
    [p,R0n]=fwdkinrecursion(i+1,R,q,type,H,P,n);
end
%now pop the stack and add back all the links
p=p+R0*p_i1_i;
