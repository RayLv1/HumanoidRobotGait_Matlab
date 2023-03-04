function[Q,TEnd]=LegInverse(T0,leg,q,x0,y0,z0,Alpha,Beta,Gamma,name)
if strcmp(name, 'LegForward')
e=1e-2;
T0(1,4)=x0;
T0(2,4)=y0;
T0(3,4)=z0;
TEnd = T0 * trotx(Alpha)* troty(Beta)* trotz(Gamma);
Q=leg.ikine(TEnd,'q0',q,'tol',e);
end
if strcmp(name, 'LegBackward')
q=fliplr(q);
e=1e-2;
T0(1,4)=-z0;
T0(2,4)=-x0;
T0(3,4)=-y0;
TEnd = T0 * trotx(Alpha)* troty(Beta)* trotz(Gamma);
Q=leg.ikine(TEnd,'q0',q,'tol',e);
Q=fliplr(Q);
end
end