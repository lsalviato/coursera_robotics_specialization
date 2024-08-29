load Samples.mat;
Samples = double(Samples)

N = length(Samples(:,1));
figure, 
s1 = scatter3(Samples(:,1),Samples(:,2),Samples(:,3),'x');
s1.SizeData = 3;
hold on

mu = mean(Samples)
save('mu','mu')
s = scatter3(mu(1),mu(2),mu(3),'r','filled')
s.SizeData = 100;

mu_mat = ones(size(Samples)) .* mu;
Sigma = 1/(N-1)*(Samples - mu_mat)'*(Samples - mu_mat);
save('Sigma','Sigma')