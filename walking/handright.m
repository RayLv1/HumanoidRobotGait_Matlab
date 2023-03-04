function []= handright()
format long
fid1=fopen('RH1.txt','wt');
fid2=fopen('RH2.txt','wt');
fid3=fopen('RH3.txt','wt');

 %% 参数修改
[  timeAll,...                     %   total time of the gait 
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
            ] = Init( );                     %   Init Function
        
wpointCount = 1;
%%  手臂轨迹

for t = 0 : stepTime : timeAll
    if(t <= crouchTime)
        x1  = 0;
        x2  = elbowAngle / crouchTime * t;
        x3  = wistAngle * sin(pi / 2 / crouchTime * t);
    end
    
    if(crouchTime < t && t <= timeAll - crouchTime)
        x1  = shoulderAngle * sin(pi / cycleTime * (t - crouchTime));
        x2  = elbowAngle;
        x3  = wistAngle + wistAngle * sin(pi / cycleTime * (t-crouchTime));
    end
    
    if(timeAll - crouchTime < t && t <= timeAll) 
        x1  = shoulderAngle * sin(pi/cycleTime * (timeAll-crouchTime))-shoulderAngle*sin(pi/cycleTime * (timeAll-crouchTime))*sin(pi/2*(t-timeAll+crouchTime)/crouchTime);
        x2  = elbowAngle - elbowAngle / (cycleTime)*(t-timeAll+crouchTime);
        x3  = wistAngle / 2 *(cos(pi/cycleTime*(t-timeAll+crouchTime))+1);
    end

    fprintf(fid1,'%f \n',x1);
    fprintf(fid2,'%f \n',x2);
    fprintf(fid3,'%f \n',x3);
    wpointCount = wpointCount + 1;
end
    