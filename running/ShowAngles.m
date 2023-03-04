leftLegJoint1 = load('L1.txt');
leftLegJoint2 = load('L2.txt');
leftLegJoint3 = load( 'L3.txt');
leftLegJoint4 = load( 'L4.txt');
leftLegJoint5 = load( 'L5.txt');
leftLegJoint6 = load( 'L6.txt');
rightLegJoint1 = load('R1.txt');
rightLegJoint2 = load('R2.txt');
rightLegJoint3 = load( 'R3.txt');
rightLegJoint4 = load('R4.txt');
rightLegJoint5 = load('R5.txt');
rightLegJoint6 = load('R6.txt');

xcom = load('Xcom.txt');
ycom = load('Ycom.txt');
zcom = load('Zcom.txt');

leftX = load('LX.txt');
leftY = load('LY.txt');
leftZ = load('LZ.txt');

rightX = load('RX.txt');
rightY = load('RY.txt');
rightZ = load('RZ.txt');

% figure;
% plot(leftX); hold on;
% title('left_x');
% figure;
% plot(leftY); hold on;
% title('left_y');
% figure;
% plot(leftZ); hold on;
% title('left_z');
% 
% figure;
% plot(rightX); hold on;
% title('right_x');
% figure;
% plot(rightY); hold on;
% title('right_y');
% figure;
% plot(rightZ); hold on;
% title('right_z');


figure;
plot(xcom); hold on;
title('com_x');
figure;
plot(ycom); hold on;
title('com_y');
figure;
plot(zcom); hold on;
title('com_z');



