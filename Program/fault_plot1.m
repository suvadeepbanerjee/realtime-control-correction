% clc; clearvars; close all;
% 
% nonlinear_pend;
% MARS_building;
% MARS_eval;
% 
% close all;
% 
% 
% volt = zeros(size(t));
% thres = zeros(size(t));
% nom = zeros(size(t));

% save('nominal_error.mat','error');
% 
volt(1:4963) = error(1:4963);
load ART2_50.mat;
volt(4964:8995) = error(4964:8995);
load nominal_error.mat;
volt(8996:12326) = error(8996:12326);
load MARS_10.mat;
volt(12327:end-1) = error(12327:end);

load ART2_10.mat;
thres(1:end-1) = error;

load ART2_50s.mat;
nom(1:end-1) = error;

figure(1)

plot(t(1:end-1),nom(1:end-1),'b:','linewidth',3);
hold on;
plot(t(1:end-1), thres(1:end-1),'k-.','linewidth',4);
plot(t(1:end-1), volt(1:end-1),'r','linewidth',4);

xlabel('Time (in s)','fontsize',40);
ylabel('Error Signals','fontsize',40);
axis([0 20 -0.3 0.35]);
h=legend('Nominal','Thresholding','Volterra','Location','NorthWest');
ch = findobj(get(h,'children'), 'type', 'line'); %// children of legend of type line
set(ch, 'Markersize', 20); 

plot(error,'b','linewidth',2)
hold on;

load MARS_10.mat;
plot(error,'r-.','linewidth',2)

load ART2_10.mat;
plot(error,'m+','linewidth',1)

load MARS_50.mat;
plot(error,'k:','linewidth',2)

load MARS_50s.mat;
plot(error,'go','linewidth',2)

load ART2_50.mat;
plot(error,'b*','linewidth',2)

load ART2_50s.mat;
plot(error,'yd','linewidth',2)
