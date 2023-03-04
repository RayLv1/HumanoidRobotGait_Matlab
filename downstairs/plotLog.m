%% load robot gait into workspace
VarName1 = load( 'L1.txt');
VarName2 = load( 'L2.txt');
VarName3 = load( 'L3.txt');
VarName4 = load( 'L4.txt');
VarName5 = load( 'L5.txt');
VarName6 = load( 'L6.txt');
VarName7 = load( 'R1.txt');
VarName8 = load( 'R2.txt');
VarName9 = load( 'R3.txt');
VarName10 = load('R4.txt');
VarName11 = load('R5.txt');
VarName12 = load('R6.txt');

LeftPitch = load('LPitch.txt');
LeftFlag = load('LFlag.txt');

RightPitch = load('RPitch.txt');
RightFlag = load('RFlag.txt');

xComValue = load('Xcom.txt');
yComValue = load('Ycom.txt');
zComValue = load('Zcom.txt');

LeftX = load('LX.txt');
LeftY = load('LY.txt');
LeftZ = load('LZ.txt');

RightX = load('RX.txt');
RightY = load('RY.txt');
RightZ = load('RZ.txt');

RightHand1 = load('RH1.txt');
RightHand2 = load('RH2.txt');
RightHand3 = load('RH3.txt');

LeftHand1 = load('LH1.txt');
LeftHand2 = load('LH2.txt');
LeftHand3 = load('LH3.txt');

subplot(3,3,1);plot(LeftX);
subplot(3,3,2);plot(LeftY);
subplot(3,3,3);plot(LeftZ);
subplot(3,3,4);plot(RightX);
subplot(3,3,5);plot(RightY);
subplot(3,3,6);plot(RightZ);
subplot(3,3,7);plot(xComValue);
subplot(3,3,8);plot(yComValue);
subplot(3,3,9);plot(zComValue);

