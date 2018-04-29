function R = so2_Exp(r)
%SO2_EXP Exp: R -> SO(2)
c = cos(r);
s = sin(r);
R = [c, -s;
     s, c];
end

