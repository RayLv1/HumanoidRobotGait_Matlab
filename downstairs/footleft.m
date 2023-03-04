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
        swingLength,...                 %   swing length left-right direction
        startSwingLength,...            %   start swing length          
        startDistance,...               %   start distance in ahead direction
        crouchTime,...                  %   courch time duiring start phase
        singleLegTime,...               %   the time of single leg phase 
        doubleLegTime,...               %   the time of double leg phase
        stepNumbers,...                 %   step numbers of 
        stepLength,...                  %   step length 
        stepLength_dLegPhase,...        %   step length of double leg phase
        crouchLength,...                %   crouch length 
        liftLegLength,...               %   lift leg length 
        liftComLength,...
        stairHeight,...                 %   the height of single stair
        FinalLeftHeight,...
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
        ] = Init( );                     %   Init Function
    
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

    M_PI    = pi;
    M_PI_2  = pi / 2;
    
    %% gait
    for t = 0 : stepTime : timeAll
        %phase1 start phase, crouch
        if(t <= crouchTime / 2.0)
            x0      = 0;
            y0      = 0;
            z0      = - totalLeglength + crouchLength * sin(M_PI * t / crouchTime);
            Alpha   = 0;
            Beta    = 0;
            Gamma   = 0;
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
            xcom    = x0;
            ycom    = y0;
            zcom    = totalLeglength - crouchLength * sin(M_PI * t / crouchTime);
        end
        
        if( crouchTime / 2.0 < t && t < crouchTime)
            phase1TimeLine = (t - crouchTime / 2.0) / (crouchTime / 2.0);
            x0      = startSwingLength * phase1TimeLine * phase1TimeLine;
            y0      = startDistance * sin(M_PI_2 * phase1TimeLine);
            z0      = - totalLeglength + crouchLength;
            Alpha   = 0;
            Beta    = 0;
            Gamma   = 0;
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
            xcom    = x0;
            ycom    = y0;
            zcom    = totalLeglength - crouchLength;
        end
        
        %% single leg phase , left leg as stand leg
        phase2 = crouchTime;
        if (phase2 < t && t <= phase2 + singleLegTime / 2.0)
            phase2TimeLine = (t - phase2) / (singleLegTime / 2.0);
            x0      = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
            y0      = startDistance + stepLength * singleSwingCoefficient * phase2TimeLine;
            z0      = - totalLeglength + crouchLength ;
            Alpha   = 0;
            Beta    = 0;
            Gamma   = 0;
            Alpha2  = AlpLeft1 * sin(M_PI * 2 / singleLegTime * (t - phase2));
            Beta2   = BetLeft1 * sin(M_PI / singleLegTime * (t - phase2));
            Gamma2  = GamLeft1 * sin(M_PI / singleLegTime * (t - phase2));
            Flag    = 1;
            xcom    = x0;
            ycom    = y0;
            zcom    = totalLeglength - crouchLength;
        end
        
        phase3 = phase2 + singleLegTime / 2.0;
        if(phase3 < t && t <= phase3 + singleLegTime / 2.0)
            phase3TimeLine = (t - phase3) / singleLegTime * 2.0;
            x0      = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
            y0      = startDistance + stepLength * singleSwingCoefficient;
            z0      = - totalLeglength + crouchLength + stairHeight * sin(M_PI_2 * phase3TimeLine);
            Alpha   = 0;
            Beta    = 0;
            Gamma   = 0;
            Alpha2  = AlpLeft1 * sin(M_PI * 2 / singleLegTime * (t - phase2));
            Beta2   = BetLeft1 * sin(M_PI / singleLegTime * (t - phase2));
            Gamma2  = GamLeft1 * sin(M_PI / singleLegTime * (t - phase2));
            Flag    = 1;
            xcom    = x0;
            ycom    = y0;
            zcom    = totalLeglength - crouchLength - stairHeight * sin(M_PI_2 * phase3TimeLine);
        end
        
        for i = 0 : 1 : stepNumbers
            %% double leg phase, from left to right
            phase4 = phase3 + singleLegTime / 2.0 + i * 2 * cycleTime;
            if(phase4 < t && t <= phase4 + doubleLegTime)
                phase4TimeLine = (t - phase4) / doubleLegTime;
                x0      = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
                y0      = startDistance + stepLength * singleSwingCoefficient + stepLength * (1-singleSwingCoefficient) * phase4TimeLine;
                z0      = - totalLeglength + crouchLength + stairHeight - liftComLength * sin(M_PI * phase4TimeLine);
                Alpha   = 0;
                Beta    = BetLeft0 * sin(M_PI / cycleTime * (t - phase4));
                Gamma   = 0;
                Alpha2  = 0;
                Beta2   = 0;
                Gamma2  = 0;
                Flag    = 1;
                xcom    = x0;
                ycom    = startDistance + i * 2 * stepLength + stepLength * singleSwingCoefficient + stepLength * (1-singleSwingCoefficient) * phase4TimeLine;
                zcom    = totalLeglength - crouchLength - stairHeight - i * stairHeight * 2  + liftComLength * sin(M_PI * phase4TimeLine);
            end
            
            %% single leg phase, right leg as stand leg
            phase5 = phase4 + doubleLegTime;
            if(phase5 < t && t <= phase5 + singleLegTime / 2.0)
                phase5TimeLine = (t - phase5) / (singleLegTime / 2.0); 
                x0      = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
                y0      = startDistance + stepLength - (stepLength * (2-singleSwingCoefficient)) * phase5TimeLine;
                z0      = - totalLeglength + crouchLength + stairHeight + liftLegLength * sin(M_PI * phase5TimeLine);
                Alpha   = 0;
                Beta    = BetLeft0 * sin(M_PI / cycleTime * (t - phase4));
                Gamma   = GamLeft0 * sin(M_PI / (singleLegTime) * (t - phase5));
                Alpha2  = 0;
                Beta2   = 0;
                Gamma2  = 0;
                Flag    = 0;
                xcom    = x0;
                ycom    = startDistance + i * 2 * stepLength + stepLength + (stepLength * singleSwingCoefficient) * phase5TimeLine;
                zcom    = totalLeglength - crouchLength - stairHeight - i * stairHeight * 2  + liftComLength * sin(M_PI * (t - phase5) / singleLegTime);
            end
            
            phase6 = phase5 + singleLegTime / 2.0;
            if (phase6 < t && t <= phase6 + singleLegTime / 2.0 )
                phase6TimeLine = (t - phase6) / (singleLegTime / 2.0);
                x0      = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
                y0      = startDistance - stepLength + stepLength * singleSwingCoefficient;
                z0      = - totalLeglength + crouchLength + stairHeight - stairHeight * sin(M_PI_2 * phase6TimeLine);
                Alpha   = 0;
                Beta    = BetLeft0 * sin(M_PI / cycleTime * (t - phase4));
                Gamma   = GamLeft0 * sin(M_PI / (singleLegTime) * (t - phase5));
                Alpha2  = 0;
                Beta2   = 0;
                Gamma2  = 0;
                Flag    = 0;
                xcom    = x0;
                ycom    = startDistance + i * 2 * stepLength + stepLength + stepLength * singleSwingCoefficient;
                zcom    = totalLeglength - crouchLength - stairHeight - i * stairHeight * 2 + liftComLength * sin(M_PI * (t-phase5) /singleLegTime)- stairHeight * sin(M_PI / 2 * phase6TimeLine);
            end
            
            %% double leg phase, from right to left
            phase7 = phase6 + singleLegTime / 2.0;
            if(phase7 < t && t <= phase7 + doubleLegTime)
                phase7TimeLine = (t - phase7) / doubleLegTime;
                x0      = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
                y0      = startDistance + (-stepLength + stepLength * singleSwingCoefficient) * (1 - phase7TimeLine);
                z0      = - totalLeglength + crouchLength - liftComLength * sin(M_PI * phase7TimeLine);
                Alpha   = 0;
                Beta    = 0;
                Gamma   = 0;
                Alpha2  = 0;
                Beta2   = 0;
                Gamma2  = 0;
                Flag    = 0;
                xcom    = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
                ycom    =  startDistance + i * 2 * stepLength + stepLength + stepLength * singleSwingCoefficient +  stepLength *(1-singleSwingCoefficient) * phase7TimeLine ;
                zcom    = totalLeglength - crouchLength - stairHeight - i * stairHeight * 2  -stairHeight + liftComLength * sin(M_PI * (t - phase7) / doubleLegTime);
            end
            
            %% single leg phase, left leg as stand leg
            phase8 = phase7 + doubleLegTime;
            if(phase8 < t && t <= phase8 + singleLegTime / 2)
                phase8TimeLine = (t - phase8) / (singleLegTime / 2);
                x0      = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
                y0      = startDistance + stepLength * singleSwingCoefficient * phase8TimeLine;
                z0      = - totalLeglength + crouchLength - liftComLength * sin(M_PI * (t-phase8) /singleLegTime);
                Alpha   = 0;
                Beta    = 0;
                Gamma   = 0;
                Alpha2  = AlpLeft1 * sin(M_PI * 2 / singleLegTime * (t - phase8));
                Beta2   = BetLeft1 * sin(M_PI / singleLegTime * (t - phase8));
                Gamma2  = GamLeft1 * sin(M_PI / singleLegTime * (t - phase8));
                Flag    = 1;
                xcom    = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
                ycom    = startDistance + i * 2 * stepLength + stepLength * 2 + stepLength * singleSwingCoefficient * phase8TimeLine;
                zcom    = totalLeglength - crouchLength - stairHeight - i * stairHeight * 2 - stairHeight + liftComLength * sin(M_PI * (t-phase8) /singleLegTime);
            end
            
            phase9 = phase8 + singleLegTime / 2;
            if(phase9 < t && t <= phase9 + singleLegTime / 2)
                phase9TimeLine = (t - phase9) / (singleLegTime / 2);
                x0      = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
                y0      = startDistance + stepLength * singleSwingCoefficient;
                z0      = - totalLeglength + crouchLength + stairHeight * sin(M_PI_2 * phase9TimeLine) - liftComLength * sin(M_PI * (t-phase8) /singleLegTime);
                Alpha   = 0;
                Beta    = 0;
                Gamma   = 0;
                Alpha2  = AlpLeft1 * sin(M_PI * 2 / singleLegTime * (t - phase8));
                Beta2   = BetLeft1 * sin(M_PI / singleLegTime * (t - phase8));
                Gamma2  = GamLeft1 * sin(M_PI / singleLegTime * (t - phase8));
                Flag    = 1;
                xcom    = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
                ycom    = startDistance + i * 2 * stepLength  + stepLength * 2 + stepLength * singleSwingCoefficient;
                zcom    = totalLeglength - crouchLength - stairHeight - i * stairHeight * 2 - stairHeight + liftComLength * sin(M_PI * (t-phase8) /singleLegTime)- stairHeight * sin(M_PI / 2 * phase9TimeLine);
            end
        end
        
        %% double leg phase, from left to right
        phase10 = phase3 + singleLegTime / 2.0 + stepNumbers * 2 * cycleTime;
        if(phase10 < t && t <= phase10 + doubleLegTime)
            phase10TimeLine = (t - phase10) / doubleLegTime;
            x0      = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
            y0      = startDistance + stepLength *singleSwingCoefficient + stepLength * (1-singleSwingCoefficient) * phase10TimeLine;
            z0      = - totalLeglength + crouchLength + stairHeight - liftComLength * sin(M_PI * phase10TimeLine);
            Alpha   = 0;
            Beta    = BetLeft0 * sin(M_PI * 2 / cycleTime * (t - phase10));
            Gamma   = 0;
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
            xcom    = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
            ycom    = startDistance + stepNumbers * 2 * stepLength + stepLength * singleSwingCoefficient + stepLength * (1-singleSwingCoefficient) * phase10TimeLine;
            zcom    = totalLeglength -stepNumbers * 2 * stairHeight - crouchLength - stairHeight + liftComLength * sin(M_PI * phase10TimeLine);  
        end
        %% single leg phase, right as stand leg
        phase11 = phase10 + doubleLegTime;
        if(phase11 < t && t <= phase11 + singleLegTime / 2)
            phase11TimeLine = (t - phase11) / (singleLegTime / 2);
            x0      = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
            y0      = startDistance + stepLength - stepLength * phase11TimeLine;
            z0      = - totalLeglength + crouchLength + stairHeight + FinalLeftHeight * sin(M_PI * phase11TimeLine);
            Alpha   = 0;
            Beta    = BetLeft0 * sin(M_PI*2 / cycleTime * (t - phase10));
            Gamma   = GamLeft0 * sin(M_PI / (singleLegTime) * (t - phase11));
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
            xcom    = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
            ycom    = startDistance + stepNumbers * 2 * stepLength + stepLength ; %%+ (singleSwingCoefficient + 0.1) * stepLength * phase11TimeLine;
            zcom    = totalLeglength - stepNumbers * 2 * stairHeight - crouchLength - stairHeight + liftComLength * sin(M_PI * ( t- phase11) /singleLegTime);
        end
        phase12 = phase11 + singleLegTime / 2;
        if(phase12 < t  && t <= phase12 + singleLegTime / 2.0)
            phase12TimeLine = (t - phase12) / (singleLegTime / 2.0);
            x0      = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
            y0      = startDistance;
            z0      = - totalLeglength + crouchLength + stairHeight - stairHeight * sin(M_PI / 2 * phase12TimeLine);
            Alpha   = 0;
            Beta    = BetLeft0 * sin(M_PI*2 /  cycleTime * (t - phase10));
            Gamma   = GamLeft0 * sin(M_PI / (singleLegTime) * (t - phase11));
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
            xcom    = swingLength * sin(M_PI * (t - phase2 + doubleLegTime / 2) / cycleTime);
            ycom    = startDistance + stepNumbers * 2 * stepLength + stepLength ; %%+ (singleSwingCoefficient + 0.1) * stepLength;
            zcom    = totalLeglength - stepNumbers * 2 * stairHeight - crouchLength - stairHeight + liftComLength * sin(M_PI * ( t- phase11) /singleLegTime);
        end
        
        phase13 = phase11 + singleLegTime;
        if(phase13 < t && t <= phase13 + crouchTime)
            phase13TimeLine = (t - phase13) / crouchTime;
            x0      = -startSwingLength * cos(M_PI / 2 * phase13TimeLine);
            y0      = startDistance - startDistance * sin(M_PI / 2 * phase13TimeLine);
            z0      = - totalLeglength + crouchLength - crouchLength * sin(M_PI_2 * phase13TimeLine);
            Alpha   = 0;
            Beta    = 0;
            Gamma   = 0;
            Alpha2  = 0;
            Beta2   = 0;
            Gamma2  = 0;
            Flag    = 0;
            xcom    = -startSwingLength * cos(M_PI / 2 * phase13TimeLine);
            ycom    = startDistance + stepNumbers * 2 * stepLength + stepLength - startDistance * sin(M_PI_2 * phase13TimeLine);%%+ (singleSwingCoefficient + 0.1) * stepLength - (startDistance+ (singleSwingCoefficient+0.1)*stepLength) * sin(M_PI_2 * phase13TimeLine);
            zcom    = totalLeglength -stepNumbers * 2 * stairHeight- crouchLength - stairHeight + crouchLength * sin(M_PI_2 * phase13TimeLine);
        end

    %% ?????
    if Flag == 0 %
        T0(1,4)=x0;
        T0(2,4)=-y0;
        T0(3,4)=z0;
        TEnd = T0 * trotx(Alpha)* troty(Beta)* trotz(Gamma);
        [Q1,Q2]=ikineH5(TEnd);
    end
    if Flag == 1 % ???????
        T0(1,4)=x0;
        T0(2,4)=-y0;
        T0(3,4)=z0;
        TEnd= T0* trotx(Alpha)* troty(Beta)* trotz(Gamma);
        T1=TEnd^-1;
        T1=T1*trotx(Alpha2)*troty(Beta2)*trotz(Gamma2);
        TEnd=T1^-1;
        [Q1,Q2]=ikineH5(TEnd);
    end
    %%
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



