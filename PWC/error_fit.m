function [R_square,SSE,SST] = error_fit(curv0,curv_fit)

% Fitting Error: https://365datascience.com/tutorials/statistics-tutorials/sum-squares/
Mean_Final_X = sum(curv_fit(:,1))/length(curv_fit(:,1));
Mean_Final_Y = sum(curv_fit(:,2))/length(curv_fit(:,2));
Mean_Final_Z = sum(curv_fit(:,3))/length(curv_fit(:,3));

SSE = sum((curv_fit(:,1)-curv0(:,1)).^2 + (curv_fit(:,2)-curv0(:,2)).^2 +...
    (curv_fit(:,3)-curv0(:,3)).^2); %和方差

SST = sum((curv_fit(:,1)-Mean_Final_X).^2 + (curv_fit(:,2)-Mean_Final_Y).^2 + ...
    (curv_fit(:,3)-Mean_Final_Z).^2);

R_square = 1 - SSE/SST; % Goodness of fitting

end