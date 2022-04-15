function T = dhtf(alpha, a, d, theta)
% Authors: Joseph Gioia & Prithve Shekar
% Find a transformation based on the 4 DH parameters
% 
% Args:
%  a: The scalar distance between z_i-1 and z_i
%  alpha: The scalar angle (in radians) between z_i-1 and z_i
%  d: The scalar distance between x_i-1 and x_i
%  theta: The scalar angle (in radains) between x_i-1 and x_i
%
% Returns: A 4x4 transformation matrix
ct = cos(theta);
st = sin(theta);
ca = cos(alpha);
sa = sin(alpha);

% Note this is just so that the output is readable and not a fraction that
% is basically 0
if (ca < 0.00001 && ca > -0.000001)
    ca = 0;
end
if (sa < 0.00001 && sa > -0.000001)
    sa = 0;
end

T = [
    ct,         -st,        0,      a;
    st * ca,    ct * ca,    -sa,    -sa * d;
    st * sa,    ct * sa,    ca,     ca * d;
    0,          0,          0,      1;
];

end