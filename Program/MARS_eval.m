function error_signal = MARS_eval(final_state,timepts)
%% This evaluates the built up MARS model

load MARS_model.mat;

output_val = zeros(timepts-1,1);
input_eval = zeros(timepts-1,4);
for i=1:timepts-1
    input_eval(i,:) = final_state(:,i)';
end


for i= 1:timepts-1
    output_val(i) = sum(final_state(:,i+1));
end

output_predic = arespredict(model,input_eval);

error_signal = output_val - output_predic;

% plot(output_predic);
% 
% hold on;
% 
% plot(output_val,'r');
% 
% figure(2)
% 
% plot(error_signal)


