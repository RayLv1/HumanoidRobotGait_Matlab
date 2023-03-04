function []= handleft()
format long
fid1=fopen('LH1.txt','wt');
fid2=fopen('LH2.txt','wt');
fid3=fopen('LH3.txt','wt');

%% get the parameters from init function
[  timeAll,...                     %   total time of the gait
    stepTime,...                    %   step time gap
    cycleTime,...                   %   cycle time, two steps
    upTime,...                      %   one step time
    swingLength,...                 %   swing length left-right direction
    startStepLength,...             %   start length in head direction
    startSwingLength,...            %   start swing length
    startSwingLength_1,...          %   start phase swing length ssq
    crouchTime,...                  %   courch time duiring start phase
    flyTime,...                     %   the time of fly phase
    doubleLegTime,...               %   the time of double leg phase
    stepNumbers,...                 %   step numbers of
    stepLength,...                  %   step length
    stepLength_dLegPhase,...        %   step length of double leg phase
    stepLength_fPhase,...           %   step lenght of fly phase
    crouchLength,...                %   crouch length
    liftLegLength,...               %   lift leg length
    liftComLength,...               %   com lift length
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
    hipDistance,...                 %   hip Distance
    legComLength...                %   leg compensate length
    ] = Init( );                     %   Init Function

wpointCount = 1;
%%  ÊÖ±Û°Ú¶¯¹ì¼£
for t = 0 : stepTime : timeAll
    if(t <= crouchTime - upTime / 2)
        x1  =   shoulderAngle / 2 * sin( pi / 2 / (crouchTime-upTime / 2) * t);
        x2  =   elbowAngle / ( crouchTime - upTime / 2) * t;
        x3  =   wistAngle * sin(pi / 2 / (crouchTime-upTime / 2) * t);
    end
    
    if (crouchTime - upTime / 2 < t && t <= timeAll - crouchTime)
        x1  =   shoulderAngle / 2 + shoulderAngle * sin(pi / upTime * (t - crouchTime + upTime / 2));
        x2  =   elbowAngle;
        x3  =   wistAngle + wistAngle / 5 * sin( pi / upTime * (t - crouchTime + upTime / 2));
    end
    
    phase3 = timeAll - crouchTime;
    if(phase3 < t && t <= phase3 + crouchTime)
        x1  =   -shoulderAngle / 2  * cos(pi / 2 / crouchTime * (t-phase3));
        x2  =   elbowAngle - elbowAngle * sin( pi / 2 / crouchTime *( t - phase3));
        x3  =   wistAngle / 5 * 4 *(1 - sin( pi / 2 / wistAngle * (t-phase3)));
    end
    %% Ð´ÊÖ²¿½Ç¶È¹ì¼£
    fprintf(fid1,'%f \n',x1);
    fprintf(fid2,'%f \n',x2);
    fprintf(fid3,'%f \n',x3);
    wpointCount=wpointCount+1;
end
end