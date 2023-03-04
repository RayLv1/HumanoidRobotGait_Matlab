%% init function of gait parameters.
%   Author  :   Ray
%   date    :   2022/07/16

function [  timeAll,...                     %   total time of the gait 
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
            ] = Init( )                     %   Init Function

%% modle parameters
    thighLength         =   373;
    legLength           =   364;
    totalLeglength      =   thighLength + legLength;
    hipDistance         =   180;
    
%% arm parameters
    shoulderAngle       =   50 / 180 * pi;                        
    elbowAngle          =   5 / 180 * pi;          
    wistAngle           =   40 / 180 * pi;
    
%% right leg posture
    gradeAngle          =   0;                            
    slopAngle           =   0;
    AlpRight0           =   0 / 180 * pi;               
    BetRight0           =   10 / 180 * pi;                  
    GamRight0           =   - slopAngle / 180 * pi;         
    AlpRight1           =   - 0 / 180 * pi;               
    BetRight1           =   0 / 180 * pi;                
    GamRight1           =   0 / 180 * pi;        
    
%% left leg posture
    AlpLeft0            =   0 / 180 * pi;                       
    BetLeft0            =   10 / 180 * pi;            
    GamLeft0            =   - slopAngle / 180 * pi;           
    AlpLeft1            =   - 0 / 180 * pi;                
    BetLeft1            =   - 0 / 180 * pi ;              
    GamLeft1            =   - 0 / 180 * pi;    
    
%% for position desigin
    startStepLength         =   0;
    singleSwingCoefficient  =   0.9;
    legComLength            =   0;                                                                 
    swingLength             =   40;                               
    startSwingLength        =   swingLength * 1;
    startSwingLength_1      =   swingLength * 0.8;    

%% gait parameters init
    stepTime                =   0.01;
    crouchTime              =   0.5;
    flyTime                 =   0.00;                               
    doubleLegTime           =   0.2;                               
    upTime                  =   flyTime+doubleLegTime; 
    cycleTime               =   upTime; 

    stepNumbers             =   100;                                 
    stepLength              =   440;                              
    stepLength_dLegPhase    =   doubleLegTime/(doubleLegTime+flyTime) * stepLength;
    stepLength_fPhase       =   flyTime/(doubleLegTime+flyTime) * stepLength;
    crouchLength            =   35;                                                    
    liftLegLength           =   50;                              
    liftComLength           =   60;                 
    trunckAngle             =   0;                                  
    timeAll                 =   2 * crouchTime + upTime/2 + flyTime + doubleLegTime/2 + (8 + 2 * stepNumbers) * upTime; 
    startJointValue         =   [0 pi/2 0 0 0 0];
    
end

