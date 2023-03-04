function [] = Generator()

%% get the parameters from init function
[   timeAll,...                     %   total time of the gait
    stepTime,...                    %   step time gap
    cycleTime,...                   %   cycle time, two steps
    swingLength,...                 %   swing length left-right direction
    startSwingLength,...            %   start swing length
    courchTime,...                  %   courch time duiring start phase
    singleLegTime,...               %   the time of single leg phase
    doubleLegTime,...               %   the time of double leg phase
    stepNumbers,...                 %   step numbers of
    stepLength,...                  %   step length
    crouchLength,...                %   crouch length
    liftLegLength,...               %   lift leg length
    startJointValue,...             %   origin start value of joint space
    elbowAngle,...                  %   amplitude of elbow joint
    shoulderAngle,...               %   amplitude of sholder joint
    wistAngle,...                   %   amplitude of wist joint
    trunckAngle,...                 %   trunck angle duiring walking
    gradeAngle,...                  %   grade angle of the ground
    slopAngle,...                   %   slop angle of the ground
    singleSwingCoefficient,...      %   zmp coefficient of support zone
    AlpRight0,...                   %
    BetRight0,...                   %   right leg
    GamRight0,...                   %   ankle posture , Euler value
    AlpRight1,...                   %
    BetRight1,...                   %   right leg
    GamRight1,...                   %   hip posture , Euler value
    AlpLeft0,...                    %
    BetLeft0,...                    %   left leg
    GamLeft0,...                    %   ankle posture , Euler value
    AlpLeft1,...                    %
    BetLeft1,...                    %   left leg
    GamLeft1,...                    %   hip postur , Euler value
    thighLength,...                 %   thigh lenght
    legLength,...                   %   leg length
    totalLeglength,...              %   total length
    hipDistance...                  %   hip Distance
    ] = Init( );                        %   Init Function
    
%% generate leg gait
footleft();
footright();
handleft();
handright();

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

    for index = 1 : 1 : size( VarName1 )
        x(index) = index;
        RightLegJointValue = [VarName7(index,1) VarName8(index,1) VarName9(index,1) VarName10(index,1) VarName11(index,1) VarName12(index,1)];
        LeftLegJointValue = [VarName1(index,1) VarName2(index,1) VarName3(index,1) VarName4(index,1) VarName5(index,1) VarName6(index,1)];
        
        LeftPitchAngle = LeftPitch(index,1);
        LeftFlagValue = LeftFlag(index,1);
        RightPitchAngle = RightPitch(index,1);
        RightFlagValue = RightFlag(index,1);
        xComValueTemp = xComValue(index,1);
        yComValueTemp = yComValue(index,1);
        zComValueTemp = zComValue(index,1);
        RightHand1Value = RightHand1(index,1);
        RightHand2Value = RightHand2(index,1);
        RightHand3Value = RightHand3(index,1);
        LeftHand1Value = LeftHand1(index,1);
        LeftHand2Value = LeftHand2(index,1);
        LeftHand3Value = LeftHand3(index,1);



human = subplot(1,2,1);
       H5Flash(RightLegJointValue,...
                    LeftLegJointValue,...
                    thighLength,...
                    legLength,...
                    totalLeglength,...
                    index,timeAll,...
                    stepLength,...
                    0,...
                    stepTime,...
                    singleLegTime,...
                    cycleTime,...
                    courchTime,...
                    swingLength,...
                    startSwingLength,...
                    liftLegLength,...
                    crouchLength,...
                    hipDistance,...
                    LeftPitchAngle,...
                    LeftFlagValue,...
                    RightPitchAngle,...
                    RightFlagValue,...
                    xComValueTemp,...
                    yComValueTemp,...
                    zComValueTemp,...
                    RightHand1Value,...
                    RightHand2Value,...
                    RightHand3Value,...
                    LeftHand1Value,...
                    LeftHand2Value,...
                    LeftHand3Value)
axis([-500 500 -800 + yComValueTemp  800 + yComValueTemp -850 800]);
        view(50,25);
%         view(90,0);
        RemoveSubplotMarginArea(human,1,2,1);
        
        
        position = subplot(1,2,2);
%         title('Variable Dynamic watch window')
%         plot(x(1:index),LeftPitch(1:index),'Color',[1 0 0]);hold on;axis([index - 50 index+50 -2 2]);
%         xlabel('time : ms ')
%         ylabel('value : mm')
%         plot(x(1:index),RightPitch(1:index),'Color',[0 1 0]);hold on;axis([index - 50 index+50 -2 2]);
%         plot(x(1:index),LeftX(1:index),'Color',[1 0 0]);hold on;
%         plot(x(1:index),LeftY(1:index),'Color',[0 1 0]);hold on;
%         plot(x(1:index),LeftZ(1:index),'Color',[0 0 1]);hold on;
        plot(x(1:index),RightX(1:index),'Color',[1 0 0]);hold on;axis([index - 50 index+50 -800 300]);
        plot(x(1:index),RightY(1:index),'Color',[0 1 0]);hold on;
        plot(x(1:index),RightZ(1:index),'Color',[0 0 1]);hold on;
        
        RemoveSubplotMarginArea(position,1,2,2);

        pause(0.02);
        
        if index > size( VarName1 ) - 1
            hold on
        else
            clf; 
        end
    end
end