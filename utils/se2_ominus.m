function v = se2_ominus(v1, v2)
%SE2_OMINUS v = v1 o- v2
v2_inv = se2_inverse(v2);
v = se2_oplus(v2_inv, v1);

end

