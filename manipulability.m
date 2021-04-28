function mu = manipulability(J,measure)
% Input: J       -> 6x6 Jacobian Matrix
%        measure -> a single string arguement defiens which manipulability is
%        used. ('sigmamin', 'detjac', 'invcond')
% Output: mu     -> Measure of manipulability  

if size(J) ~= [6,6]
    error('Error. \n Dimension of Jacobian Mtarix is not [6,6].');
end

if strcmp(measure, 'sigmamin')
    [u,s,v] = svd(J);
    mu = s(end,end);
    
elseif strcmp(measure , 'detjac')
    mu = det(J);
    
elseif strcmp(measure, 'invcond')
    mu = rcond(J);
    
else
    error('Error. \n Unexpected manipulability measure.');
end

end