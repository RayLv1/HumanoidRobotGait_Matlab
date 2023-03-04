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
    [   timeAll,...                     %   total time of the gait
        stepTime,...                    %   step time gap
        cycleTime,...                   %   cycle time, two steps
        swingLength,...                 %   swing length left-right direction
        startSwingLength,...            %   start swing length
        crouchTime,...                  %   courch time duiring start phase
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

    ground_ptich    = gradeAngle / 180*pi;
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

    LeftLeg1 = SerialLink(  [ L1 , L2 , L3 , L4 , L5 , L6 ] , 'name' , 'LegForward');

    % home point
    T0 = LeftLeg1.fkine( startJointValue ).T;

    %% gait
    for t = 0 : stepTime : timeAll
        % phase1 start phase,   crouch and move zmp to left leg
        if( t <= crouchTime)
            x0      =  startSwingLength * (t / crouchTime) * (t / crouchTime);
            y0      = 0;
            z0      = - totalLeglength + ( crouchLength - hipDistance * sin( ground_roll )) * ( sin( pi / 2.0 * t / crouchTime ));
            Alpha   = 0;
            Beta    = 0;
            Gamma   = 0;
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
            
            xcom    = startSwingLength * (t / crouchTime) * (t / crouchTime);
            ycom    = 0;
            zcom    =  -( crouchLength - hipDistance * sin( ground_roll )) * ( sin( pi / 2.0 * t / crouchTime ));
        end

        phase2 = crouchTime;
        % single leg stand phase, half of step, left leg as stand leg
        if( phase2 < t && t <= phase2 + singleLegTime)
            phase2TimeLine = ( t - phase2 ) / singleLegTime;
            x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
            y0      = ( singlePhaseK * stepLength ) * singleSwingCoefficient * phase2TimeLine;      
            z0      = - totalLeglength + crouchLength - hipDistance * sin( ground_roll );
            Alpha   = 0;   % do not need to set this angle
            Beta    = 0;
            Gamma   = 0;
            Alpha2  = AlpLeft1 * sin( pi * phase2TimeLine);
            Beta2   = BetLeft1 * sin( pi * phase2TimeLine);
            Gamma2  = GamLeft1 * sin( pi * phase2TimeLine);
            Flag    = 1;
            
            xcom    = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
            ycom    = ( singlePhaseK * stepLength ) * singleSwingCoefficient * phase2TimeLine; 
            zcom    =  - crouchLength + hipDistance * sin( ground_roll );
            
        end

        phase3 = phase2 + singleLegTime;
        % double leg stand phase, mpc from left leg to right leg
        if ( phase3 < t && t <= phase3 + doubleLegTime )
            phase3TimeLine = ( t - phase3 ) / doubleLegTime;
            x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
            y0      = ( singlePhaseK * stepLength ) * singleSwingCoefficient + stepLength * doublePhaseK * phase3TimeLine;
            z0      = - totalLeglength + crouchLength - stepLength * tan( ground_ptich ) * phase3TimeLine - hipDistance * sin( ground_roll );
            Alpha   = 0;
            Beta    = 0;
            if (phase3 + doubleLegTime / 2.0 < t && t <= phase3 + doubleLegTime)
                z0 = - totalLeglength + crouchLength + 2 * crouchLength * sin( pi / doubleLegTime * (t - phase3 - doubleLegTime / 2.0))- stepLength * tan( ground_ptich ) * phase3TimeLine - hipDistance * sin( ground_roll );
                Beta = - BetLeft0 * sin( pi / doubleLegTime * (t - phase3 - doubleLegTime / 2.0));
            end
            Gamma   = 0;
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
            
            xcom    = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
            ycom    = ( singlePhaseK * stepLength ) * singleSwingCoefficient + stepLength * doublePhaseK * phase3TimeLine;
            zcom    =   - crouchLength + hipDistance * sin( ground_roll );
            
        end

        %% steady walking stage
        for i = 0 : 1 : stepNumbers - 1
            phase4 = phase3 + doubleLegTime + i * 2 *( singleLegTime + doubleLegTime );
            % single leg phase, right leg as stand leg
            if ( phase4 < t && t <= phase4 + singleLegTime )
                phase4TimeLine = ( t - phase4 ) / singleLegTime;
                x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                y0      = singlePhaseK * stepLength * singleSwingCoefficient + stepLength * doublePhaseK - (2 - singlePhaseK) * stepLength * phase4TimeLine;
                z0      = - totalLeglength + crouchLength + liftLegLength * sin( pi * phase4TimeLine ) - stepLength * tan( ground_ptich ) + 2 * crouchLength * (1 - sin( pi / 2.0 * phase4TimeLine ))  + 2 * stepLength * tan( ground_ptich ) * phase4TimeLine - hipDistance * sin( ground_roll );
                % liftLegLength / 2.0 * (1 + sin( pi * phase4TimeLine - pi/2))
                Alpha   = AlpLeft0 * sin( pi * phase4TimeLine);
                Beta    = - BetLeft0 +  BetLeft0 * sin( pi / 2.0 * phase4TimeLine ) + BetLeft0 * sin(pi * 2 / 3 * phase4TimeLine );
                Gamma   = GamLeft0 * sin( pi * phase4TimeLine);
                Alpha2  = 0;
                Beta2   = 0;
                Gamma2  = 0;
                Flag    = 0;
                
                xcom    = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                ycom    = ( singlePhaseK * stepLength ) * singleSwingCoefficient + stepLength * doublePhaseK + i * 2 * stepLength +  singlePhaseK * stepLength * phase4TimeLine;
                zcom    =   - crouchLength + hipDistance * sin( ground_roll );
            end

            phase5 = phase4 + singleLegTime;
            % double leg phase, mpc from right leg to left leg
            if ( phase5 < t && t <= phase5 + doubleLegTime )
                phase5TimeLine = ( t - phase5 ) / doubleLegTime;
                x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                y0      = - stepLength + singleSwingCoefficient * singlePhaseK * stepLength + doublePhaseK * stepLength * phase5TimeLine;
                z0      = - totalLeglength + crouchLength + stepLength * tan( ground_ptich ) - stepLength * tan( ground_ptich ) * phase5TimeLine + hipDistance * sin( ground_roll );
                Alpha   = 0;
                Beta    = BetLeft0 * ( sin( pi * 2 / 3 ) * ( 1 - sin( pi / 2 * phase5TimeLine)));
                Gamma   = 0;
                Alpha2  = 0;
                Beta2   = 0;
                Gamma2  = 0;
                Flag    = 0;
                xcom    = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                ycom    = ( singlePhaseK * stepLength ) * singleSwingCoefficient + stepLength * doublePhaseK + i * 2 * stepLength +  singlePhaseK * stepLength + doublePhaseK * stepLength * phase5TimeLine;
                zcom    =   - crouchLength + hipDistance * sin( ground_roll );
            end

            phase6 = phase5 + doubleLegTime;
            % single leg phase, left leg as stand leg
            if ( phase6 < t && t <= phase6 + singleLegTime )
                phase6TimeLine = ( t - phase6 ) / singleLegTime;
                x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                y0      = singlePhaseK * stepLength * ( singleSwingCoefficient - 1 + phase6TimeLine );
                z0      = - totalLeglength + crouchLength - hipDistance * sin( ground_roll );
                Alpha   = 0;
                Beta    = 0;
                Gamma   = 0;
                Alpha2  = AlpLeft1 * sin( pi * phase6TimeLine );
                Beta2   = BetLeft1 * sin( pi * phase6TimeLine);
                Gamma2  = GamLeft1 * sin( pi * phase6TimeLine);
                Flag    = 1;
                xcom    = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                ycom    = ( singlePhaseK * stepLength ) * singleSwingCoefficient + stepLength * doublePhaseK + i * 2 * stepLength +  singlePhaseK * stepLength + doublePhaseK * stepLength + singlePhaseK * stepLength * phase6TimeLine;
                zcom    =   - crouchLength + hipDistance * sin( ground_roll );
            end

            phase7 = phase6 + singleLegTime;
            % double leg phase, mpc from right to left
            if ( phase7 < t && t <= phase7 + doubleLegTime)  
                phase7TimeLine = ( t - phase7 ) / doubleLegTime;
                x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                y0      = singleSwingCoefficient *singlePhaseK * stepLength + doublePhaseK * stepLength * phase7TimeLine;
                z0      = - totalLeglength + crouchLength - stepLength * tan( ground_ptich ) * phase7TimeLine - hipDistance * sin( ground_roll );
                Alpha   = 0;
                Beta    = 0;
                if (phase7 + doubleLegTime / 2.0 < t && t <= phase7 + doubleLegTime)
                    z0      = - totalLeglength + crouchLength + 2 * crouchLength * sin( pi /doubleLegTime * (t - phase7 - doubleLegTime / 2.0))- stepLength * tan( ground_ptich ) * phase7TimeLine - hipDistance * sin( ground_roll );
                    Beta    = - BetLeft0  * sin( pi / doubleLegTime * (t - phase7 - doubleLegTime / 2.0));
                end
                Gamma   = 0;
                Alpha2  = 0;
                Beta2   = 0;
                Gamma2  = 0;
                Flag    = 0;
                
                xcom    = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                ycom    = ( singlePhaseK * stepLength ) * singleSwingCoefficient + stepLength * doublePhaseK + stepNumbers * 2 * stepLength;
                zcom    =   - crouchLength + hipDistance * sin( ground_roll );
            end
        end

       %%  stop phase 
       if i >= stepNumbers -1 
            phase8 = phase7 + doubleLegTime;

            % single leg phase, left leg as swing leg
            if ( phase8 < t && t <= phase8 + singleLegTime ) 
                phase8TimeLine = ( t - phase8 ) / singleLegTime;
                x0      = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                y0      = ((singleSwingCoefficient * singlePhaseK + doublePhaseK ) * stepLength ) * ( 1 - phase8TimeLine);
                z0      = - totalLeglength + crouchLength + liftLegLength * sin( pi * phase8TimeLine ) + 2 * crouchLength * (1 - sin( pi / 2.0 * phase8TimeLine )) - stepLength * tan( ground_ptich ) * (1 - phase8TimeLine ) - hipDistance * sin( ground_roll );
                Alpha   = AlpLeft0 * sin( pi * phase8TimeLine);
                Beta    = BetLeft0 * sin( pi * phase8TimeLine);
                Gamma   = GamLeft0 * sin( pi * phase8TimeLine);
                Alpha2  = 0;
                Beta2   = 0;
                Gamma2  = 0;
                Flag    = 0;
                xcom    = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                ycom    = ( singlePhaseK * stepLength ) * singleSwingCoefficient + stepLength * doublePhaseK + stepNumbers * 2 * stepLength + ( 1 - singleSwingCoefficient ) * singlePhaseK * stepLength * phase8TimeLine;
                zcom    =   - crouchLength + hipDistance * sin( ground_roll );
            end

            phase9 = phase8 + singleLegTime;
            % double leg phase, stand up and move to mid point
            if ( phase9 < t && t <= phase9 + crouchTime )      
                phase9TimeLine = ( t - phase9 ) / crouchTime;
                x0      = - startSwingLength * ( 1 - phase9TimeLine * phase9TimeLine);
                y0      = 0;
                z0      = - totalLeglength + crouchLength * (1 -phase9TimeLine * phase9TimeLine) - hipDistance * sin( ground_roll );
                Alpha   = 0;
                Beta    = 0;
                Gamma   = 0;
                Alpha2  = 0;
                Beta2   = 0;
                Gamma2  = 0;
                Flag=0;
                xcom    = swingLength * sin( pi * ( t - phase2 + doubleLegTime / 2 ) / cycleTime );
                ycom    = stepNumbers * 2 * stepLength + stepLength;
                zcom    =   - crouchLength + hipDistance * sin( ground_roll );
            end
       end

        %% 求逆解
        if Flag == 0 % 针对脚底
            T0(1,4) = x0;
            T0(2,4) = -y0;
            T0(3,4) = z0;
            TEnd    = T0 * trotx(Alpha) * troty(Beta) * trotz(Gamma);
            [Q1,Q2] = ikineH5(TEnd);
        end

        if Flag == 1 % 针对重心
            T0(1,4) = x0;
            T0(2,4) = -y0;
            T0(3,4) = z0;
            TEnd    = T0 * trotx(Alpha) * troty(Beta) * trotz(Gamma);
            T1      = TEnd^-1;
            T1      = T1 * trotx(Alpha2) * troty(Beta2) * trotz(Gamma2);
            [theta,v] = tr2angvec( trotx(Alpha2) * troty(Beta2) * trotz(Gamma2) );
            TEnd    = T1^-1;
            [Q1,Q2] = ikineH5(TEnd);
        end

        %% write value into files
        x(pointCount,1)=T0(1,4);
        y(pointCount,1)=T0(2,4);
        z(pointCount,1)=T0(3,4);
        
        fprintf(fid1,'%f \n',Q1(1,1));
        fprintf(fid2,'%f \n',Q1(1,2)-pi/2);
        fprintf(fid3,'%f \n',Q1(1,3));
        fprintf(fid4,'%f \n',Q1(1,4));
        fprintf(fid5,'%f \n',Q1(1,5));
        fprintf(fid6,'%f \n',Q1(1,6));
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


