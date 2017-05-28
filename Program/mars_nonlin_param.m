input_train = error;

output_train = zeros(100,5);
output_train(:,1:4) = sensor_gain_faults;
output_train(:,5) = act_fault';

model1 = aresbuild(input_train,output_train(:,1));
model2 = aresbuild(input_train,output_train(:,2));
model3 = aresbuild(input_train,output_train(:,3));
model4 = aresbuild(input_train,output_train(:,4));
model5 = aresbuild(input_train,output_train(:,5));
