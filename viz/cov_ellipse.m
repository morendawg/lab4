function ellipse = cov_ellipse(mu, cov, scale)
%COV_ELLIPSE Compute ellipse to visualize covariance
% INPUTS:
%   mu: 2x1, [x; y]
%   cov: 2x2, covariance 
% OUTPUS:
%   ellipse: 2xN, points to draw
% Reference:
%   https://gist.github.com/Piyush3dB/bf2c83a8eb7344798644

[eig_vecs, eig_vals] = eig(cov, 'vector');
            
% find largest eigenvalue
k = find(eig_vals == max(eig_vals), 1);
% get the largest eigenvalue
max_eig_val = eig_vals(k);
max_eig_vec = eig_vecs(:, k);
% get the smallest eigenvalue
min_eig_val = eig_vals(3 - k);
% calculate angle between x-axis and largest eigenvector
phi = atan2(max_eig_vec(2), max_eig_vec(1));
phi = wrapToPi(phi); 
% get 95% confidence interval
chisquare_val = 2.4477;
a = chisquare_val * sqrt(max_eig_val);
b = chisquare_val * sqrt(min_eig_val);
R = so2_Exp(phi);
thetas = linspace(0, 2*pi);
ellipse = [a * cos(thetas); 
           b * sin(thetas)];
ellipse = R * ellipse * scale + mu(:);
end

