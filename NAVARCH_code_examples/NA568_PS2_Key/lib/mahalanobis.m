function results = mahalanobis(filter,ground_truth)
[r,c] = size(filter.mu);
if c == 1
      ground_truth(3)=wrapToPi(ground_truth(3));
      diff = [(ground_truth(1:2)'-filter.mu(1:2));wrapToPi(ground_truth(3)-filter.mu(3))];
      results(1,1) = (diff)' * (filter.Sigma \ diff);
      results(2:4,1) = diff;
      results(5:7,1) = [3*sqrt(filter.Sigma(1,1));3*sqrt(filter.Sigma(2,2));3*sqrt(filter.Sigma(3,3))];
else
       ground_truth(3)=wrapToPi(ground_truth(3));
       diff = [(ground_truth(1:2)'-filter.mu_cart(1:2));wrapToPi(ground_truth(3)-filter.mu_cart(3))];
       results(1,1) = (diff)' * (filter.sigma_cart \ diff);
       results(2:4,1) = diff;
       results(5:7,1) = [3*sqrt(filter.sigma_cart(1,1));3*sqrt(filter.sigma_cart(2,2));3*sqrt(filter.sigma_cart(3,3))];
end

end

