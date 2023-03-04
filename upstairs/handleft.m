function []= handleft()
    format long
    fid1=fopen('LH1.txt','wt');
    fid2=fopen('LH2.txt','wt');
    fid3=fopen('LH3.txt','wt');

    %% 参数修改
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
        
    %%  手臂摆动轨迹
    for t = 0 : stepTime : timeAll
        
        if( t <= crouchTime)
            x1 = 0;
            x2 = elbowAngle * t / crouchTime;
            x3 = wistAngle * t / crouchTime;
        end
        
        if ( crouchTime < t && t <= timeAll - crouchTime)
            x1 = shoulderAngle * sin(pi * (t - crouchTime) / cycleTime);
            x2 = elbowAngle;
            x3 = wistAngle + wistAngle * sin(pi * (t - crouchTime) / cycleTime);
        end
        
        if ( timeAll - crouchTime < t && t <= timeAll)
            phase3TimeLine = (t - timeAll + crouchTime) / crouchTime;
            x1 = shoulderAngle * sin(pi * (timeAll - crouchTime - crouchTime) / cycleTime)*(1-sin(pi / 2.0 * phase3TimeLine));
            x2 = elbowAngle* (1 - phase3TimeLine);
            x3 = (wistAngle + wistAngle * sin(pi * (timeAll - crouchTime - crouchTime) / cycleTime)) * (1 - phase3TimeLine);
        end
        
        
        %% 写手部角度轨迹
        fprintf(fid1,'%f \n',x1);
        fprintf(fid2,'%f \n',x2);
        fprintf(fid3,'%f \n',x3);
        
        pointCount = pointCount + 1;
        
    end
end