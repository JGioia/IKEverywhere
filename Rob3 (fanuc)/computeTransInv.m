function inverse_transformation = computeTransInv(transformation)
% Authors: Joseph Gioia and Prithve Shekar
% Computes a transformation matrix's inverse
% 
% Args:
%  transformation: A 4x4 transformation matrix
%
% Returns: A 4x4 transforamtion matrix

% Find the translation and rotation of the original transformation
rotation = transformation(1:3, 1:3);
translation = transformation (1:3, 4);

% Find the translation for the inverse transformation
translation_inv = -1 * rotation' * translation;

% Find the inverse transformation
inverse_transformation = [rotation' translation_inv; 0 0 0 1];

end