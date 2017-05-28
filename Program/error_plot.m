error_sig = cell(100,1);

for i = 1:100
    error_sig{i} = MARS_eval(final_state{i},timepts);
end

figure(1)
hold on;
for i=1:100
    plot(error_sig{i});
end

error = zeros(100,12000);

for i=1:100
    error(i,:) = error_sig{i};
end