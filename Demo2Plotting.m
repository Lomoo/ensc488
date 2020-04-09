clear all
close all
clc
set(gcf,'color','w');

%for plotting ENSC 448 lab demo 2 data

%need to give filename and have directory
filenameP = 'plannedP.txt';
filenameCart = 'plannedCart.txt';
filenameV = 'plannedV.txt';
filenameA = 'plannedA.txt';

%Change to your file folder. Place above files in the folder
directory = 'C:\Users\sully\Desktop\488\';
full_nameP = strcat(directory, filenameP);
full_nameCart = strcat(directory, filenameCart);
full_nameV = strcat(directory, filenameV);
full_nameA = strcat(directory, filenameA);

fileIDP = fopen(full_nameP);
fileIDCart = fopen(full_nameCart);
fileIDV = fopen(full_nameV);
fileIDA = fopen(full_nameA);

f = fopen('../../file1.txt');
formatSpec = '%f';
ACart = fscanf(fileIDCart,formatSpec);
AP = fscanf(fileIDP,formatSpec);
AV = fscanf(fileIDV,formatSpec);
AA = fscanf(fileIDA,formatSpec);
len=length(AP);
len=length(ACart);
len=length(AV);
len=length(AA);
rows=len/5; %time, joint 1, joint 2, joint 3, joint 4

%put data into matrix
matrixCart = transpose(reshape(ACart,[5,rows]));
matrixP = transpose(reshape(AP,[5,rows]));
matrixV = transpose(reshape(AV,[5,rows]));
matrixA = transpose(reshape(AA,[5,rows]));

subplot(4,1,1);
hold on
plot(matrixP(:,1),matrixP(:,2),'b')
plot(matrixP(:,1),matrixP(:,3),'r')
plot(matrixP(:,1),matrixP(:,4),'k')
plot(matrixP(:,1),matrixP(:,5),'g')
title('Position')
xlabel('Time (s)')
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4')

subplot(3,1,2);
hold on
plot(matrixV(:,1),matrixV(:,2),'b')
plot(matrixV(:,1),matrixV(:,3),'r')
plot(matrixV(:,1),matrixV(:,4),'k')
plot(matrixV(:,1),matrixV(:,5),'g')
title('Position')
xlabel('Time (s)')
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4')

subplot(3,1,3);
hold on
plot(matrixA(:,1),matrixA(:,2),'b')
plot(matrixA(:,1),matrixA(:,3),'r')
plot(matrixA(:,1),matrixA(:,4),'k')
plot(matrixA(:,1),matrixA(:,5),'g')
title('Acceleration')
xlabel('Time (s)')
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4')
figure


hold on
plot(matrixCart(:,2),matrixCart(:,3),'b')
title('Cartesion Coordinates')
xlabel('Time (s)')
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4')


fclose(fileIDP);
fclose(fileIDV);
fclose(fileIDA);