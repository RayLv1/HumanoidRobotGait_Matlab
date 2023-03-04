function []= footright()
    format long;
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

    ground_pitch    = gradeAngle / 180*pi;
    ground_roll     = slopAngle / 180*pi;
    pointCount      = 1;
    
    singlePhaseK    = singleLegTime / cycleTime;
    doublePhaseK    = doubleLegTime / cycleTime;
    
    %% robot model
    L1 = Link( 'd' , 0 ,        'a' , 0 ,           'alpha' , 0 ,           'modified');
    L2 = Link( 'd' , 0 ,        'a' , 0 ,           'alpha' , - pi / 2 ,    'modified');
    L3 = Link( 'd' , 0 ,        'a' , 0 ,           'alpha' , - pi / 2 ,    'modified');
    L4 = Link( 'd' , 0 ,        'a' , thighLength , 'alpha' , 0 ,           'modified');
    L5 = Link( 'd' , 0 ,        'a' , legLength ,   'alpha' , 0 ,           'modified');
    L6 = Link( 'd' , 0 ,        'a' , 0 ,           'alpha' , pi / 2 ,      'modified');

    rightLeg = SerialLink(  [ L1 , L2 , L3 , L4 , L5 , L6 ] , 'name' , 'LegForward');

    % home point
    T0 = rightLeg.fkine( startJointValue ).T;
    
    %% gait
    for t = 0 : stepTime : timeAll
        %  start phase : crouch and move to left 
        if ( t <= courchTime )                                
            x0      = startSwingLength * (t / courchTime) * (t / courchTime);
            y0      = 0;
            z0      = - totalLeglength + ( crouchLength + hipDistance * sin( ground_roll )) * sin( pi * t / 2 / courchTime);
            Alpha   = 0;
            Beta    = 0;
            Gamma   = 0;
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
        end
        
        phase2 = courchTime;
        % single leg phase, right leg as swing leg
        if ( phase2 < t && t <= phase2 + singleLegTime )        
            phase2TimeLine = ( t - phase2 ) / singleLegTime;
            x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
            y0      = - stepLength * (1 - ( singlePhaseK  * singleSwingCoefficient )) * phase2TimeLine;
            z0      = - totalLeglength + crouchLength + liftLegLength * sin( pi * phase2TimeLine) + stepLength * tan( ground_pitch ) * phase2TimeLine + hipDistance * sin( ground_roll );
            %liftLegLength / 2.0 * (1 + sin( pi * phase2TimeLine - pi/2))
            Alpha   = AlpRight0 * sin( pi * phase2TimeLine);
            Beta    = BetRight0 * sin( pi * 2 / 3 * phase2TimeLine);
            Gamma   = GamRight0 * sin( pi * phase2TimeLine);
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
        end
        
        phase3 = phase2 + singleLegTime;
        % double leg phase, mpc from left to right
        if ( phase3 < t && t <= phase3 + doubleLegTime )    
            phase3TimeLine = ( t - phase3 ) / doubleLegTime;
            x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
            y0      = - stepLength * (1 - ( singlePhaseK  * singleSwingCoefficient )) + ( doublePhaseK * stepLength ) * phase3TimeLine;
            z0      = - totalLeglength + crouchLength + stepLength * tan( ground_pitch ) * ( 1 - phase3TimeLine ) + hipDistance * sin( ground_roll );
            Alpha   = 0;
            Beta    = BetRight0 * sin( pi * 2 / 3 ) * ( 1 - sin( pi / 2 * phase3TimeLine ) );
            Gamma   = 0;
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
        end
        
       %% steady walking stage
        for i = 0 : 1 : stepNumbers - 1
            phase4 = phase3 + doubleLegTime + i * 2 *( singleLegTime + doubleLegTime );
            % single leg phase, right leg as stand leg
            if ( phase4 < t && t <= phase4 + singleLegTime)    
                phase4TimeLine = ( t - phase4 ) / singleLegTime;
                x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                y0      = ( -1 + singleSwingCoefficient ) * singlePhaseK * stepLength + singlePhaseK * stepLength * phase4TimeLine;
                z0      = - totalLeglength + crouchLength + hipDistance * sin( ground_roll );
                Alpha   = 0;
                Beta    = 0;
                Gamma   = 0;
                Alpha2  = AlpRight1 * sin( pi * phase4TimeLine );
                Beta2   = BetRight1 * sin( pi * phase4TimeLine );
                Gamma2  = GamRight1 * sin( pi * phase4TimeLine );
                Flag    = 1;
            end
            
            phase5 = phase4 + singleLegTime;
            % double leg phase, MPC from right to left
            if ( phase5 < t && t <= phase5 + doubleLegTime)                             
                phase5TimeLine = ( t - phase5 ) / doubleLegTime;
                x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                y0      = singleSwingCoefficient * singlePhaseK * stepLength + ( 1 - singlePhaseK) * stepLength * phase5TimeLine;
                z0      = - totalLeglength + crouchLength - stepLength * tan( ground_pitch ) * phase5TimeLine + hipDistance * sin(ground_roll);
                Alpha   = 0;
                Beta    = 0;
                if (phase5 + doubleLegTime / 2.0 < t && t <= phase5 + doubleLegTime)
                    z0      = - totalLeglength + crouchLength + crouchLength * 2 * sin( pi / doubleLegTime * ( t - phase5 - doubleLegTime / 2.0 ));
                    Beta    = - BetRight0 * sin(pi / doubleLegTime * ( t - phase5 - doubleLegTime / 2.0 ));
                end
                Gamma   = 0;
                Alpha2  = 0;
                Beta2   = 0;
                Gamma2  = 0;
                Flag    = 0;
            end
            
            phase6 = phase5 + doubleLegTime;
            % single leg phase, right leg as swing leg
            if ( phase6 < t && t <= phase6 + singleLegTime ) 
                phase6TimeLine = ( t - phase6 ) / singleLegTime;
                x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                y0      = stepLength - ( 1 - singleSwingCoefficient ) * singlePhaseK * stepLength - ( 2 - singlePhaseK ) * stepLength * phase6TimeLine;
                z0      = - totalLeglength + crouchLength + liftLegLength * sin( pi * phase6TimeLine) + 2 * crouchLength * (1 - sin( pi / 2.0 * phase6TimeLine )) -  stepLength * tan(ground_pitch) + 2 * stepLength * tan( ground_pitch ) * phase6TimeLine + hipDistance * sin(ground_roll);
                %liftLegLength / 2.0 * (1 + sin( pi * phase6TimeLine - pi/2))
                Alpha   = 0;
                Beta    = - BetRight0 +  BetRight0 * sin( pi / 2.0 * phase6TimeLine) +  BetRight0 * sin(pi * 2 / 3 * phase6TimeLine);
                Gamma   = 0;
                Alpha2  = 0;
                Beta2   = 0;
                Gamma2  = 0;
                Flag    = 0;
            end
            
            phase7 = phase6 + singleLegTime;
            %double leg phase, MPC from left to right
            if ( phase7 < t && t <= phase7 + doubleLegTime )    
                phase7TimeLine = ( t - phase7 ) / doubleLegTime;
                x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                y0      = - stepLength + singleSwingCoefficient * singlePhaseK * stepLength + ( 1 - singlePhaseK ) * stepLength * phase7TimeLine;
                z0      = - totalLeglength + crouchLength + stepLength * tan(ground_pitch) - stepLength * tan(ground_pitch) * phase7TimeLine + hipDistance * sin( ground_roll );
                Alpha   = 0;
                Beta    = BetRight0 * sin( pi * 2 / 3 ) * ( 1 - sin( pi / 2 * phase7TimeLine));
                Gamma   = 0;
                Alpha2  = 0;
                Beta2   = 0;
                Gamma2  = 0;
                Flag    = 0;
            end
        end
        %% stop phase
        
        if i >= stepNumbers - 1
             phase8 = phase7 + doubleLegTime;

            %left leg as swing leg, right leg as stand leg
            if ( phase8 < t && t <= phase8 + singleLegTime )  
                phase8TimeLine = ( t - phase8 ) / singleLegTime;
                x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                y0      = ( singleSwingCoefficient - 1 ) * singlePhaseK * stepLength + ( 1 - singleSwingCoefficient ) * singlePhaseK * stepLength * phase8TimeLine ;
                z0      = - totalLeglength + crouchLength + hipDistance * sin( ground_roll );
                Alpha   = 0;
                Beta    = 0;
                Gamma   = 0;
                Alpha2  = AlpRight1 * sin(pi * phase8TimeLine);
                Beta2   = BetRight1 * sin(pi * phase8TimeLine);
                Gamma2  = GamRight1 * sin(pi * phase8TimeLine);
                Flag    =1;
            end

            phase9 = phase8 + singleLegTime;
            % double leg phase, stand up and move to mid point
            if (phase9 < t && t <= phase9 + courchTime)     
                phase9TimeLine = ( t - phase9 ) / courchTime;
                x0      = -startSwingLength * ( 1 - phase9TimeLine * phase9TimeLine);
                y0      = 0;
                z0      = - totalLeglength + crouchLength* (1 - phase9TimeLine * phase9TimeLine) + hipDistance * sin( ground_roll );
                Alpha   = 0;
                Beta    = 0;
                Gamma   = 0;
                Alpha2  = 0;
                Beta2   = 0;
                Gamma2  = 0;
                Flag    = 0;
            end
        end
        
        %% 求逆解
        if Flag == 0     % 针对脚底
            T0(1,4)=x0;
            T0(2,4)=-y0;
            T0(3,4)=z0;
            TEnd = T0 * trotx(Alpha)* troty(Beta)* trotz(Gamma);
            [Q1,Q2]=ikineH5(TEnd);
        end
        if Flag == 1     % 针对髋关节
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
        x(pointCount,1)=x0;
        y(pointCount,1)=y0;
        z(pointCount,1)=z0;
        fprintf(fid1,'%f \n',Q1(1,1));
        fprintf(fid2,'%f \n',Q1(1,2) - pi/2);
        fprintf(fid3,'%f \n',Q1(1,3));
        fprintf(fid4,'%f \n',Q1(1,4));
        fprintf(fid5,'%f \n',Q1(1,5));
        fprintf(fid6,'%f \n',Q1(1,6));
        fprintf(fid7,'%f \n',Beta);
        fprintf(fid8,'%f \n',Flag);
        fprintf(fid9,'%f \n',x0);
        fprintf(fid10,'%f \n',y0);
        fprintf(fid11,'%f \n',z0);
        
        pointCount = pointCount+1;
    end
end



