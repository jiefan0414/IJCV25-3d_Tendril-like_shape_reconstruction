function [k, b, fit_error] = HeightLineFit2(points)

    numPoints = size(points, 1);
	%compute sums for linear system
    
    fSumX  = sum(points(:,1));
	fSumY  = sum(points(:,2));
    fSumXX = sum(points(:,1).* points(:,1));
	fSumXY = sum(points(:,1).* points(:,2));
    
    
    temp = numPoints * fSumXX - fSumX * fSumX;
    
    % calculate slope m and intercept b
    
    % see wiki : https://zh.wikipedia.org/wiki/%E6%9C%80%E5%B0%8F%E4%BA%8C%E4%B9%98%E6%B3%95
    % least square
    % if enominator is 0, it means the LS fitting is a vertical line !!
    % but this case will not happen, since X represents arclength (never got same values)
    if temp == 0  
        k = 9999;
        b = 9999; 
    else 
        k =(numPoints * fSumXY - fSumX * fSumY) / temp;
        b =(fSumY - k * fSumX) / numPoints;
        
    end
    
    fit_error = sum( abs(k.*points(:,1)+b - points(:,2)) );

end