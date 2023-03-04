function []= footleft()
startup_rvc;
format long
fid1 = fopen ('L1.txt','wt');
fid2 = fopen ('L2.txt','wt');
fid3 = fopen ('L3.txt','wt');
fid4 = fopen ('L4.txt','wt');
fid5 = fopen ('L5.txt','wt');
fid6 = fopen ('L6.txt','wt');
fid7 = fopen ('LPitch.txt','wt');
fid8 = fopen ('LFlag.txt','wt');
fid9 = fopen ('LX.txt','wt');
fid10 = fopen ('LY.txt','wt');
fid11 = fopen ('LZ.txt','wt');
fid12 = fopen ('Xcom.txt','wt');
fid13 = fopen ('Ycom.txt','wt');
fid14 = fopen ('Zcom.txt','wt');

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

%       ground angle, currently not used
%     ground_ptich    = gradeAngle / 180*pi;
%     ground_roll     = slopAngle / 180*pi;

pointCount      = 1;

%% robot model
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
    % phase1 start phase,   crouch and move zmp to left leg
    if( t <= crouchTime)
        x0      =  swingLength * (t / crouchTime) * (t / crouchTime);
        y0      =  startStepLength * sin( pi / 2.0 * t / crouchTime);      % backward
        z0      = - totalLeglength + crouchLength * sin( pi / 2.0 * t / crouchTime);
        Alpha   = 0;
        Beta    = 0;
        Gamma   = 0;
        Alpha2  = 0;
        Beta2   = 0;
        Gamma2  = 0;
        Flag    = 0;
        
        xcom    = swingLength * (t / crouchTime) * (t / crouchTime);
        ycom    = startStepLength * sin( pi / 2.0 * t / crouchTime);
        zcom    = - crouchLength * sin( pi / 2.0 * t / crouchTime);
    end
    phase2 = crouchTime;
    % single leg stand phase, half of step, left leg as stand leg
    % lift right leg. com goes down.
    if ( phase2 < t && t <= phase2 + upTime / 2.0 )
        phase23TimeLine = ( t - phase2 ) / (upTime / 2.0 + doubleLegTime / 2.0);
        x0      = swingLength;
        y0      = startStepLength + stepLength_dLegPhase / 2.0 * phase23TimeLine^2;
        z0      = - totalLeglength + crouchLength + liftComLength / 2.0 * (1 - cos( pi * 2 * (t - phase2) / upTime));
        Alpha   = 0;
        Beta    = 0;
        Gamma   = 0;
        Alpha2  = 0;
        Beta2   = 0;
        Gamma2  = 0;
        Flag=0;
        
        xcom    = swingLength;
        ycom    = startStepLength + stepLength_dLegPhase / 2.0 * phase23TimeLine^2;
        zcom    = - crouchLength - liftComLength / 2.0 * (1 - cos( pi * 2 * (t - phase2) / upTime));
    end
    phase3 = phase2 + upTime / 2.0;
    % left Leg as lift leg, right leg as swing leg
    % lift leg ankle pitch 
    if(phase3 < t && t <= phase3 + doubleLegTime / 2.0)                                                      
        phase23TimeLine = ( t - phase2 ) / (upTime / 2.0 + doubleLegTime / 2.0);
        phase3TimeLine = (t - phase3) / doubleLegTime * 2;
        x0      = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
        y0      = startStepLength + stepLength_dLegPhase / 2.0 * phase23TimeLine^2;
        z0      = - totalLeglength + crouchLength + liftComLength / 2.0 *(1-cos( pi * 2 * (t - phase2) / upTime));
        Alpha   = 0;
        Beta    = 0;
        Gamma   = 0;
        
        if (phase3 + doubleLegTime / 4.0 < t && t <= phase3 + doubleLegTime / 2.0)
            z0 = - totalLeglength + crouchLength + liftComLength / 2.0 *(1-cos( pi * 2 * (t - phase2) / upTime)) + 2 * crouchLength * sin( pi / doubleLegTime * 2 * (t - phase3 - doubleLegTime / 4.0));
            Beta = - BetLeft0 * sin( pi * 2.0 / doubleLegTime * (t - phase3 - doubleLegTime / 4.0));
        end
        
        Alpha2  = AlpLeft1 * sin( pi * phase3TimeLine);
        Beta2   = BetLeft1 * sin( pi * phase3TimeLine);
        Gamma2  = GamLeft1 * sin( pi * phase3TimeLine);
        Flag    = 1;
        
        xcom    = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
        ycom    = startStepLength + stepLength_dLegPhase / 2.0 * phase23TimeLine^2;
        zcom    = - crouchLength - liftComLength / 2.0 * (1 - cos( pi * 2 * (t - phase2) / upTime));
    end
    %%  稳定双足跑动
    for i = 0 : 1 : stepNumbers
        phase4 = phase3 + flyTime + doubleLegTime / 2 + i * 2 * upTime;
        %left leg start to swing and ended with contact with ground
        if ( phase4 <= t && t <= phase4 + upTime + flyTime)
            phase4TimeLine = (t - phase4) / (upTime + flyTime);
            x0      = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
            y0      = startStepLength + stepLength_dLegPhase / 2.0 - stepLength_dLegPhase * phase4TimeLine;
            z0      = - totalLeglength + crouchLength + liftComLength / 2.0 *(1-cos( pi * 2 * (t - phase2) / upTime)) + liftLegLength * sin( pi * phase4TimeLine)+ 2 * crouchLength * (1 - sin( pi / 2.0 * phase4TimeLine ));
            Alpha   = 0;
            Beta    = - BetLeft0 +  BetLeft0 * sin( pi / 2.0 * phase4TimeLine ) + BetLeft0 * sin(pi * 2 / 3 * phase4TimeLine );
            Gamma   = 0;
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
            
            xcom    = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
            ycom    = startStepLength + stepLength_dLegPhase / 2.0  + stepLength_dLegPhase * (t - phase3) / doubleLegTime;
            zcom    = - crouchLength - liftComLength / 2.0 * (1 - cos( pi * 2 * (t - phase2) / upTime));
        end
        phase5 = phase4 + upTime + flyTime;
        %left leg contact with ground , and ended with start to swing
        if ( phase5 < t && t <= phase5 + doubleLegTime)                                              
            phase5TimeLine = ( t - phase5 ) / doubleLegTime;
            x0      = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
            y0      = startStepLength - stepLength_dLegPhase / 2 + stepLength_dLegPhase * phase5TimeLine;
            z0      = -totalLeglength + crouchLength + liftComLength / 2.0 *(1-cos( pi * 2 * (t - phase2) / upTime));
            Alpha   = 0;
            if(phase5  < t && t <= phase5 + doubleLegTime / 2.0)
                Beta    = BetLeft0 * ( sin( pi * 2 / 3 ) * ( 1 - sin( pi / 2 * (( t - phase5 ) / doubleLegTime * 2.0))));
            end
            
            if (phase5 + doubleLegTime / 2.0 < t && t <= phase5 + doubleLegTime)
                z0      = - totalLeglength + crouchLength + 2 * crouchLength * sin( pi /doubleLegTime * (t - phase5 - doubleLegTime / 2.0))+ liftComLength / 2.0 *(1-cos( pi * 2 * (t - phase2) / upTime));
                Beta    = - BetLeft0  * sin( pi / doubleLegTime * (t - phase5 - doubleLegTime / 2.0));
            end
            Gamma   = 0;
            Alpha2  = AlpLeft1 * sin( pi * phase5TimeLine);
            Beta2   = BetLeft1 * sin( pi * phase5TimeLine);
            Gamma2  = GamLeft1 * sin( pi * phase5TimeLine);
            Flag=1;
            
            xcom    = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
            ycom    = startStepLength + stepLength_dLegPhase / 2.0  + stepLength_dLegPhase * (t - phase3) / doubleLegTime;
            zcom    = - crouchLength - liftComLength / 2.0 * (1 - cos( pi * 2 * (t - phase2) / upTime));
        end
    end
    
    %% stop phase  （left leg on the back）
    phase6 = phase5 + doubleLegTime;
    %left leg half swing phase
    if ( phase6 < t && t <= phase6 + upTime + flyTime)                                                      
        phase6TimeLine = (t - phase6) / (upTime + flyTime);
        x0      = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
        y0      = startStepLength + stepLength_dLegPhase / 2.0 - (startStepLength + stepLength_dLegPhase / 2.0) * phase6TimeLine;
        z0      = -totalLeglength + crouchLength + liftComLength / 2.0 *(1-cos( pi * 2 * (t - phase2) / upTime)) + liftLegLength * sin(pi * phase6TimeLine)+ 2 * crouchLength * (1 - sin( pi / 2.0 * phase6TimeLine ));
        Alpha   = 0;
        Beta    = - BetLeft0  + BetLeft0 * sin(pi / 2 * phase6TimeLine);
        Gamma   = 0;
        Alpha2  = 0;
        Beta2   = 0;
        Gamma2  = 0;
        Flag    = 0;
        
        xcom    = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
        ycom    = startStepLength + stepLength_dLegPhase / 2.0  + stepLength_dLegPhase * (phase6 - phase3) / doubleLegTime;
        zcom    = - crouchLength - liftComLength / 2.0 * (1 - cos( pi * 2 * (t - phase2) / upTime));
    end
    phase7 = phase6 + upTime + flyTime;
    if (phase7 < t && t <= phase7 + doubleLegTime)                                               %左腿飞行时间段，脚板
        phase7TimeLine = (t - phase7) / doubleLegTime;
        x0      = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
        y0      = 0;
        z0      = -totalLeglength + crouchLength + liftComLength / 2.0 *(1-cos( pi * 2 * (t - phase2) / upTime));
        Alpha   = 0;
        Beta    = 0;
        Gamma   = 0;
        Alpha2  = AlpLeft1 * sin(pi * phase7TimeLine);
        Beta2   = BetLeft1 * sin(pi * phase7TimeLine);
        Gamma2  = GamLeft1 * sin(pi * phase7TimeLine);
        Flag=1;
        
        xcom    = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
        ycom    = startStepLength + stepLength_dLegPhase / 2.0  + stepLength_dLegPhase * (phase6 - phase3) / doubleLegTime;
        zcom    = - crouchLength - liftComLength / 2.0 * (1 - cos( pi * 2 * (t - phase2) / upTime));
    end
    %% marktime to stop. two steps
    for i = 0 : 1 : 0
        phase8 = phase7 + doubleLegTime + i * (upTime + doubleLegTime);
        if ( phase8 < t && t <= phase8 + upTime + flyTime)                                                      %左腿支撑时间，脚板收回
            phase8TimeLine = (t - phase8) / (upTime + flyTime);
            x0      = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
            y0      = 0;
            z0      = -totalLeglength + crouchLength + liftComLength / 2.0 *(1-cos( pi * 2 * (t - phase2) / upTime)) + liftLegLength * sin(pi * phase8TimeLine);
            Alpha   = 0;
            Beta    = 0;
            Gamma   = 0;
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
            
            xcom    = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
            ycom    = startStepLength + stepLength_dLegPhase / 2.0  + stepLength_dLegPhase * (phase6 - phase3) / doubleLegTime;
            zcom    = - crouchLength - liftComLength / 2.0 * (1 - cos( pi * 2 * (t - phase2) / upTime));
        end
        phase9 = phase8 + upTime + flyTime;
        if ( phase9 < t && t <= phase9 + doubleLegTime)                                            %左腿飞行时间段，脚板
            phase9TimeLine = (t - phase9) / doubleLegTime;
            x0      = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
            y0      = 0;
            z0      = -totalLeglength + crouchLength + liftComLength / 2.0 *(1-cos( pi * 2 * (t - phase2) / upTime));
            Alpha   = 0;
            Beta    = 0;
            Gamma   = 0;
            Alpha2  = AlpLeft1 * sin(pi * phase9TimeLine);
            Beta2   = BetLeft1 * sin(pi * phase9TimeLine);
            Gamma2  = GamLeft1 * sin(pi * phase9TimeLine);
            Flag    = 1;
            
            xcom    = swingLength * sin(pi * ((t - phase3) + doubleLegTime / 2.0 ) / upTime);
            ycom    = startStepLength + stepLength_dLegPhase / 2.0  + stepLength_dLegPhase * (phase6 - phase3) / doubleLegTime;
            zcom    = - crouchLength - liftComLength / 2.0 * (1 - cos( pi * 2 * (t - phase2) / upTime));
        end
    end
    phase10 = phase9 + doubleLegTime;
    if (phase10 < t && t <= phase10 + crouchTime)
        phase10TimeLine = (t - phase10) / crouchTime;
        x0      = 0;
        y0      = 0;
        z0      = -totalLeglength + crouchLength - crouchLength * sin(pi / 2 * phase10TimeLine);
        Alpha   = 0;
        Beta    = 0;
        Gamma   = 0;
        Alpha2  = 0;
        Beta2   = 0;
        Gamma2  = 0;
        Flag    = 0;
        
        xcom    = 0;
        ycom    = startStepLength + stepLength_dLegPhase / 2.0  + stepLength_dLegPhase * (phase6 - phase3) / doubleLegTime;
        zcom    = - crouchLength + crouchLength * sin(pi / 2 * phase10TimeLine);
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
    fprintf(fid12,'%f \n',-xcom);
    fprintf(fid13,'%f \n',ycom);
    fprintf(fid14,'%f \n',zcom);
    
    pointCount = pointCount + 1;
end
end


