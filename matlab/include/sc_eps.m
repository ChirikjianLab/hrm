function val = sc_eps(angle, eps, name)
%SuperEllipse.sc_eps: a sin/cos exponentiation function

if strcmp(name, 'sin')
    val = sign(sin(angle)).*abs(sin(angle)).^eps;
elseif strcmp(name, 'cos')
    val = sign(cos(angle)).*abs(cos(angle)).^eps;
else
    printf('The third input has to be either "cos" or "sin".\n')
end