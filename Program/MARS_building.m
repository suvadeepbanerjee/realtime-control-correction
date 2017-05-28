
%% MARS model building

input_train = zeros((num_expts/10)*(timepts-1),4);
for j=1:num_expts/10
    for i=1:timepts-1
        input_train((j-1)*(timepts-1)+i,:) = final_state{j}(:,i)';
    end
end

output_train = zeros((num_expts/10)*(timepts-1),1);

for j=1:num_expts/10
    for i= 1:timepts-1
    output_train((j-1)*(timepts-1)+i) = sum(final_state{j}(:,i+1));
    end
end

% trainParams = aresparams(2);
model = aresbuild(input_train,output_train);

save('MARS_model.mat','model');

