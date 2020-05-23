function y= fcn(x1,x2,r,h)


y=x1+h*x2;
d=rh;a0=(d^2+8*r*abs(y))^0.5;
if(abs(y)>d0)
    a=x2+(a0-d)/2;
else
    a=x2+y/h;
end

if(abs(a)<d)
    u=-r*a/d;
else
    u=-r*sgn(a);    
end

y=u;