function []= footright()
format long
fid1 = fopen('R1.txt','wt');
fid2 = fopen('R2.txt','wt');
fid3 = fopen('R3.txt','wt');
fid4 = fopen('R4.txt','wt');
fid5 = fopen('R5.txt','wt');
fid6 = fopen('R6.txt','wt');
fid7 = fopen ('RPitch.txt','wt');
fid8 = fopen ('RFlag.txt','wt');
fid9 = fopen ('RX.txt','wt');
fid10 = fopen ('RY.txt','wt');
fid11 = fopen ('RZ.txt','wt');

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
    BetRight0,...                    %   left leg
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

%   ground angle, currently not used
%     ground_ptich    = gradeAngle / 180*pi;
%     ground_roll     = slopAngle / 180*pi;

pointCount      = 1;
%% 	robot model
L1 = Link( 'd' , 0 ,        'a' , 0 ,           'alpha' , 0 ,           'modified');
L2 = Link( 'd' , 0 ,        'a' , 0 ,           'alpha' , - pi / 2 ,    'modified');
L3 = Link( 'd' , 0 ,        'a' , 0 ,           'alpha' , - pi / 2 ,    'modified');
L4 = Link( 'd' , 0 ,        'a' , thighLength , 'alpha' , 0 ,           'modified');
L5 = Link( 'd' , 0 ,        'a' , legLength ,   'alpha' , 0 ,           'modified');
L6 = Link( 'd' , 0 ,        'a' , 0 ,           'alpha' , pi / 2 ,      'modified');

LeftLeg1 = SerialLink(  [ L1 , L2 , L3 , L4 , L5 , L6 ] , 'name' , 'LegForward');

% home point
T0 = LeftLeg1.fkine( startJointValue ).T;

%leg compensate
crouchLength = crouchLength + legComLength;

%% running gait
for t = 0 : stepTime : timeAll
    %phase1 start phase, crouch and move com to left leg
    if( t <= crouchTime)
        x0      =  startSwingLength * (t / crouchTime) * (t / crouchTime);
        y0      =  startStepLength * sin( pi / 2.0 * t / crouchTime);
        z0      = - totalLeglength + crouchLength  * ( sin( pi / 2.0 * t / crouchTime ));
        Alpha   = 0;
        Beta    = 0;
        Gamma   = 0;
        Alpha2  = 0;
        Beta2   = 0;
        Gamma2  = 0;
        Flag    = 0;
    end
    phase2 = crouchTime;
    % right leg start to lift
    if ( phase2 < t && t <= phase2 + upTime / 2.0 )                  
        phase23TimeLine = ( t - phase2 ) / (upTime / 2.0 + doubleLegTime / 2.0 );
        phase2TimeLine = ( t - phase2 ) / (upTime / 2.0);
        x0      = startSwingLength;
        y0      = startStepLength - stepLength_dLegPhase / 2 * phase23TimeLine^2;
        z0      = - totalLeglength + crouchLength + liftComLength / 2.0 * (1 - cos( pi * 2 * (t - phase2) / upTime)) + liftLegLength * sin(pi / 2.0 * phase2TimeLine);
        Alpha   = 0;
        Beta    = 0;               
        Gamma   = 0;
        Alpha2  = 0;
        Beta2   = 0;
        Gamma2  = 0;
        Flag=0;
    end
    phase3= phase2 + upTime / 2.0;
     %right leg as swing leg
    if( phase3 < t && t <= phase3 + doubleLegTime / 2.0 )                      
        phase23TimeLine = ( t - phase2 ) / (upTime / 2.0 + doubleLegTime / 2.0 );
        phase3TimeLine = (t - phase3) / (doubleLegTime / 2.0);
        x0      = startSwingLength * sin(pi * (t - phase3 + doubleLegTime / 2.0) / upTime);
        y0      = startStepLength - stepLength_dLegPhase / 2.0 * phase23TimeLine^2;                                          %匀速
        z0      = - totalLeglength + crouchLength + liftComLength / 2.0 * (1 - cos(pi * 2 * (t - phase2) / upTime)) + liftLegLength * (cos( pi / 2 * phase3TimeLine));
        Alpha   = 0;
        Beta    = BetRight0 * sin( pi * 2 / 3 * phase3TimeLine);
        Gamma   = 0;
        Alpha2  = 0;
        Beta2   = 0;
        Gamma2  = 0;
        Flag    = 0;
    end
    
    %% stable running phase
    for i = 0 : 1 : stepNumbers                                                         
        phase4 = phase3 + doubleLegTime / 2.0 + i * 2 * upTime;
         %right leg start to contact with ground
        if( phase4 <= t && t <= phase4 + doubleLegTime)
            phase4TimeLine = (t - phase4) / doubleLegTime;
            x0      = startSwingLength * sin(pi * (t - phase3 + doubleLegTime / 2.0) / upTime);
            y0      = startStepLength - stepLength_dLegPhase / 2.0 + stepLength_dLegPhase * phase4TimeLine;                                          %匀速
            z0      = - totalLeglength + crouchLength  + liftComLength / 2.0 * (1 - cos(pi * 2.0 * (t - phase2) / upTime));
            Alpha   = 0;
            Gamma   = 0;
            
            if(phase4  < t && t <= phase4 + doubleLegTime / 2.0)
                Beta    = BetRight0 * ( sin( pi * 2 / 3 ) * ( 1 - sin( pi / 2 * phase4TimeLine * 2)));
            end
            
            if (phase4 + doubleLegTime / 2.0 < t && t <= phase4 + doubleLegTime)
                z0      = - totalLeglength + crouchLength + 2 * crouchLength * sin( pi /doubleLegTime * (t - phase4 - doubleLegTime / 2.0))+ liftComLength / 2.0 *(1-cos( pi * 2 * (t - phase2) / upTime));
                Beta    = - BetRight0  * sin( pi / doubleLegTime * (t - phase4 - doubleLegTime / 2.0));
            end
            
            Alpha2  = AlpRight1 * sin(pi * phase4TimeLine);
            Beta2   = BetRight1 * sin(pi * phase4TimeLine);
            Gamma2  = GamRight1 * sin(pi * phase4TimeLine);
            Flag=1;
        end
        phase5 = phase4 + doubleLegTime;
         %right leg start to swing
        if (phase5 < t && t <= phase5 + upTime + flyTime)                                                
            phase5TimeLine = (t - phase5) / (upTime + flyTime);
            x0      = startSwingLength * sin(pi * (t - phase3 + doubleLegTime / 2.0) / upTime);
            y0      = startStepLength + stepLength_dLegPhase / 2.0 - stepLength_dLegPhase * phase5TimeLine;
            z0      = - totalLeglength + crouchLength + liftComLength / 2.0 * (1 - cos(pi * 2.0 * (t - phase2) / upTime)) + liftLegLength * sin(pi * phase5TimeLine)+ 2 * crouchLength * (1 - sin( pi / 2.0 * phase5TimeLine ));
            Alpha   = 0;
            Beta    = - BetRight0 +  BetRight0 * sin( pi / 2.0 * phase5TimeLine ) + BetRight0 * sin(pi * 2 / 3 * phase5TimeLine );
            Gamma   = 0;
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag=0;
        end
    end
    
    %% stop phase , left leg is back
    phase6 = phase5 + + upTime + flyTime;
    if( phase6 < t && t <= phase6 + doubleLegTime)
        phase6TimeLine = (t-phase6) / doubleLegTime;
        x0      = startSwingLength * sin(pi * (t - phase3 + doubleLegTime / 2.0) / upTime);
        y0      = startStepLength - stepLength_dLegPhase / 2.0 - (startStepLength - stepLength_dLegPhase / 2.0) * phase6TimeLine;                                          %匀速
        z0      = - totalLeglength + crouchLength + liftComLength / 2.0 * (1 - cos(pi * 2.0 * (t - phase2) / upTime));
        Alpha   = 0;
        Beta    = BetRight0 * sin( pi * 2 / 3 )*(1 - sin(pi / 2 * phase6TimeLine));
        Gamma   = 0;
        Alpha2  = AlpRight1 * sin(pi * phase6TimeLine);
        Beta2   = BetRight1 * sin(pi * phase6TimeLine);
        Gamma2  = GamRight1 * sin(pi * phase6TimeLine);
        Flag    =1;
    end
    phase7 =phase6 + doubleLegTime;
    if (phase7 < t && t <= phase7 + upTime + flyTime)                                                 %  右腿支撑腿阶段，脚板脚跟着地，然后收回
        phase7TimeLine = (t - phase7) / (upTime + flyTime);
        x0      = startSwingLength * sin(pi * (t - phase3 + doubleLegTime / 2.0) / upTime);
        y0      = 0;
        z0      = - totalLeglength + crouchLength + liftComLength / 2.0 * (1 - cos(pi * 2.0 * (t - phase2) / upTime)) + liftLegLength * sin(pi * phase7TimeLine);
        Alpha   = 0;
        Beta    = 0;
        Gamma   = 0;
        Alpha2  = 0;
        Beta2   = 0;
        Gamma2  = 0;
        Flag    = 0;
    end
    %% 原地踏步止步
    for i = 0 : 1 : 0
        phase8 = phase7 + upTime + flyTime + i * 2 * upTime;
        if (phase8 < t && t <= phase8 + doubleLegTime)
            phase8TimeLine = (t - phase8) / doubleLegTime;
            x0      = startSwingLength * sin(pi * (t - phase3 + doubleLegTime / 2.0) / upTime);
            y0      = 0;                                                                    %匀速
            z0      = - totalLeglength + crouchLength + liftComLength / 2.0 * (1 - cos(pi * 2.0 * (t - phase2) / upTime));
            Alpha   = 0;
            Beta    = 0;
            Gamma   = 0;
            Alpha2  = AlpRight1 * sin(pi * phase8TimeLine);
            Beta2   = BetRight1 * sin(pi * phase8TimeLine);
            Gamma2  = GamRight1 * sin(pi * phase8TimeLine);
            Flag=1;
        end
        phase9 = phase8 + doubleLegTime;
        if (phase9 < t && t <= phase9 + upTime + flyTime)                                                 %  右腿支撑腿阶段，脚板脚跟着地，然后收回
            phase9TimeLine = (t - phase9) / (upTime + flyTime);
            x0      = startSwingLength * sin(pi * (t - phase3 + doubleLegTime / 2.0) / upTime);
            y0      = 0;
            z0      = - totalLeglength + crouchLength + liftComLength / 2.0 * (1 - cos(pi * 2.0 * (t - phase2) / upTime)) + liftLegLength * sin(pi * phase9TimeLine);
            Alpha   = 0;
            Beta    = 0;
            Gamma   = 0;
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
        end
    end
    phase10 = phase9+ upTime + flyTime;
    if (phase10 < t && t < phase10 + crouchTime)
        phase10TimeLine = (t - phase10) / crouchTime;
        x0      = 0;
        y0      = 0;
        z0      = - totalLeglength + crouchLength - crouchLength * sin(pi / 2.0 * phase10TimeLine);
        Alpha   = 0;
        Beta    = 0;
        Gamma   = 0;
        Alpha2  = 0;
        Beta2   = 0;
        Gamma2  = 0;
        Flag    = 0;
    end
    %% 求逆解
    if Flag == 0 % 针对脚底
        T0(1,4)=x0;
        T0(2,4)=-y0;
        T0(3,4)=z0;
        TEnd = T0 * trotx(Alpha)* troty(Beta)* trotz(Gamma);
        [Q1,Q2]=ikineH5(TEnd);
    end
    if Flag == 1 % 针对重心
        T0(1,4)=x0;
        T0(2,4)=-y0;
        T0(3,4)=z0;
        TEnd= T0* trotx(Alpha)* troty(Beta)* trotz(Gamma);
        T1=TEnd^-1;
        T1=T1*trotx(Alpha2)*troty(Beta2)*trotz(Gamma2);
        TEnd=T1^-1;
        [Q1,Q2]=ikineH5(TEnd);
    end
    %% 写关节角度
    q=Q1;
    x(pointCount,1)=x0;
    y(pointCount,1)=y0;
    z(pointCount,1)=z0;
    fprintf(fid1,'%f \n',q(1,1));
    fprintf(fid2,'%f \n',q(1,2)- pi/2);
    fprintf(fid3,'%f \n',q(1,3));
    fprintf(fid4,'%f \n',q(1,4));
    fprintf(fid5,'%f \n',q(1,5));
    fprintf(fid6,'%f \n',q(1,6));
    fprintf(fid7,'%f \n',Beta);
    fprintf(fid8,'%f \n',Flag);
    fprintf(fid9,'%f \n',x0);
    fprintf(fid10,'%f \n',y0);
    fprintf(fid11,'%f \n',z0);
    
    pointCount = pointCount + 1;
end
end



