%% init function of gait parameters.
% Author        :   Ray
% Date          :   2022/07/16

function [  timeAll,...                     %   total time of the gait 
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
            ] = Init( )                     %   Init Function

%% modle parameters
    thighLength         =   373;
    legLength           =   364;
    totalLeglength      =   thighLength + legLength;
    hipDistance         =   180;
    
%% arm parameters
    shoulderAngle       =   -20 / 180 * pi;                        
    elbowAngle          =   10 / 180 * pi;          
    wistAngle           =   20 / 180 * pi;
    
%% right leg posture
    gradeAngle          =   0;                            
    slopAngle           =   0;
    AlpRight0           =   0 / 180 * pi;               
    BetRight0           =   - gradeAngle / 180 * pi;                  
    GamRight0           =   - slopAngle / 180 * pi;         
    AlpRight1           =   - 0 / 180 * pi;               
    BetRight1           =   0 / 180 * pi;                
    GamRight1           =   0 / 180 * pi;        
    
%% left leg posture
    AlpLeft0            =   0 / 180 * pi;                       
    BetLeft0            =   - gradeAngle / 180 * pi;            
    GamLeft0            =   - slopAngle / 180 * pi;           
    AlpLeft1            =   - 0 / 180 * pi;                
    BetLeft1            =   - 0 / 180 * pi ;              
    GamLeft1            =   - 0 / 180 * pi;    
    
%% gait parameters init
    stepNumbers         =   2;                                                      
    cycleTime           =   0.2;                                          
    stepTime            =   0.01;
    swingLength         =   120;                                         
    startSwingLength    =   swingLength * 0.5;                            
    crouchTime          =   0.5;                                          
    singleLegTime       =   (1 - 2 * asin( startSwingLength / swingLength) / pi) * cycleTime;           
    doubleLegTime       =   2 * asin( startSwingLength / swingLength)/ pi * cycleTime;                  

    singleSwingCoefficient  =   0.335;
    startDistance       =   0;
    stepLength          =   300;           
    stepLength_dLegPhase = doubleLegTime / cycleTime * stepLength;
    crouchLength        =   50;                                                   
    liftComLength       =   0;                                                   
    stairHeight         =   50;                                                     
    liftLegLength       =   80;
    FinalLeftHeight     =   10;
    trunckAngle         =   0;                         
    timeAll             =   crouchTime + singleLegTime + 2 * stepNumbers * cycleTime + doubleLegTime + singleLegTime + crouchTime;
    startJointValue     =   [0 pi/2 0 0 0 0];

end
