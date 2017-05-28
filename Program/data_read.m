% Program to read .dat file resulting from the C simulation

fid = fopen('data.dat','r');
datacell = textscan(fid, '%f%f%f%f%f');
fclose(fid);