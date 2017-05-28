clc; close all;

%% This evaluates the model built previously on new data to check for error

load linear_reg.mat;

output_val = zeros(timepts-1,1);
output_predic = zeros(timepts-1,1);

for i= 1:timepts-1
    output_val(i) = sum(final_state(:,i+1));
end

for i=1:timepts-1
    output_predic(i) = beta'*final_state(:,i);
end

error = output_predic-output_val;

% figure(2)
% 
% plot(output_val);
% hold on;
% plot(output_predic,'r');

% e = zeros(timepts-1,1);
% 
% for i=1:timepts-1
%     if abs(error(i)) > 0.011
%         e(i) = 10*(error(i) - sign(error(i))*0.011);
%     end
% end

figure(3)
plot(t(1:end-1),error,'r','linewidth',2)
title('Error Signal for Nominal System');
xlabel('Time');
ylabel('Error Signal');
