clc; close all;

%% This uses multivariate linear regression model building to create a model which takes present four state values
% creates a single output which is the sum of the next 4 states - 4 to 1
% mapping

input_train = zeros(timepts-1,4);
for i=1:timepts-1
    input_train(i,:) = final_state(:,i)';
end

output_train = zeros(timepts-1,1);

for i= 1:timepts-1
    output_train(i) = sum(final_state(:,i+1));
end

[beta,sigma,resid] = mvregress(input_train,output_train);

save('linear_reg.mat','beta');