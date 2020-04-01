function [ T_fast ] = findFastestUncontrollableSamplingTime( A_d )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
    eigVals = eig(A_d);
    
    j = 1;
    for i = 2:length(eigVals)
        if real(eigVals(i-1)-eigVals(i)) == 0
            validImEigValsDiff(j) = abs(imag(eigVals(i-1)-eigVals(i)));
            j = j+1;
        end
    end
    
    T_fast = (2*pi)/max(validImEigValsDiff);
end

