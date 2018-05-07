function [ currently_not_using, error_without ] = computeUsefulPredictor( predicted, predictors,leaveNvars, epsfactor )
%COMPUTEUSEFULPREDICTOR Use local linear regression to compute the best
%predictor variables.
%
% Uses the parallel computing toolbox when available (parfor)
% For N datapoints:
% predicted  : NxD sized matrix of variables for which best prediction should be found
% predictors : NxP sized matrix of P predictor variable candidates with
% leaveNvars : amount of candidate variables left when finished
% epsfactor  : (inverse) scaling of the gaussian kernel width relative to 
% the median of the distance matrix.

if nargin < 4
    epsfactor = 1.8;
    if nargin < 3
        leaveNvars = 1;
    end
end
numPredictors = size(predictors,2);
numPredicted = size(predicted,2);

currently_not_using = zeros(1,numPredictors, 'uint16');
error_without = zeros(numPredictors - leaveNvars, numPredictors);
index_set = 1:numPredictors;
i = 1;
while i <= (numPredictors - leaveNvars)
    
    % loop over remaining items
    remaining_indices = index_set(~currently_not_using);
    these_errors = zeros(1,max(size(remaining_indices)));
    % coarse-grained parallel loop (needs consecutive indices)
    % v. computation bound, doesn't matter if we broadcast whole matrices
    
    % try-catch block for matlab parallel failures
    try
        parfor l = 1:max(size(remaining_indices))
            k = remaining_indices(l);
            without_this = ~currently_not_using;
            without_this(k) = false;
            if numPredictors - i > 10
                disp(['At i = ', num2str(i), ', examining k = ', num2str(k)])
            end
            % compute the regression error when not using this variable
            K = squareform(pdist(predictors(:,without_this)));
            eps = median(K(:))/epsfactor;
            K = (exp(-K.^2 / eps^2));
            
            for j = 1:numPredicted
                [~, regress_residual] = modified_loc_lin_regr(...
                    predicted(:,j), predictors(:,without_this), epsfactor, K);
                % sum squares
                these_errors(l) = these_errors(l) + regress_residual^2;
            end
        end
            % get correctly dimensioned errors with square root
        error_without(i, remaining_indices) = sqrt(these_errors);

        % take away that one which leaves the least error when removing
        [~, min_ind] = min(error_without(i,:) + ~(~currently_not_using) * 2 * max(error_without(:))); % add something big where we're not using them
        % take away the one and mark where
        currently_not_using(min_ind) = numPredictors - leaveNvars + 1 - i ;
        disp(['Variable k = ', num2str(min_ind), ' eliminated, ', num2str(currently_not_using(min_ind) - 1), '/', num2str(numPredictors), ' to go'])
        i = i + 1; % only increment if acually managing to reach this part
    catch E
        disp (E)
        % try to restart parallel pool
        try
            delete(gcp('nocreate'));
        catch F
            disp(F)
        end
        parpool;  
        % try to run again, don't care that one more iteration
    end
        
end



end

