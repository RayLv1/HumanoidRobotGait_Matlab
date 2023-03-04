function H5Flash(RightLegJointValue,...
                    LeftLegJointValue,...
                    thighLength,...
                    legLength,...
                    totalLeglength,...
                    stairHeight,...
                    hipDistance,...
                    LeftPitchAngle,...
                    RightPitchAngle,...
                    xComValue,...
                    yComValue,...
                    zComValue,...
                    RightHand1Value,...
                    RightHand2Value,...
                    RightHand3Value,...
                    LeftHand1Value,...
                    LeftHand2Value,...
                    LeftHand3Value,...
                    stepNumbers,...
                    stepLength) 
                
    %% robot model
    L1 = Link( 'd' , 0 ,        'a' , 0 ,           'alpha' , 0 ,           'modified');
    L2 = Link( 'd' , 0 ,        'a' , 0 ,           'alpha' , - pi / 2 ,    'modified');
    L3 = Link( 'd' , 0 ,        'a' , 0 ,           'alpha' , - pi / 2 ,    'modified');
    L4 = Link( 'd' , 0 ,        'a' , thighLength , 'alpha' , 0 ,           'modified');
    L5 = Link( 'd' , 0 ,        'a' , legLength ,   'alpha' , 0 ,           'modified');
    L6 = Link( 'd' , 0 ,        'a' , 0 ,           'alpha' , pi / 2 ,      'modified');

    %% stairs construct
    stair_wide_min = -300;
    stair_wide_max = 300;
    stair_length_start = -80;
    stair_length_total = stepLength;

%     for i = 0 :1: 2 * (stepNumbers + 1)
%         x = stair_wide_min:stair_wide_max;
%         y = (stair_length_start + i * stair_length_total):(stair_length_total + stair_length_start + i * stair_length_total);
%         z = - i * stairHeight ;
%         [x,y,z] = meshgrid(x,y,z);
%         mesh(x,y,z);
%         hold on;
%     end
%     set(gcf,'unit','normalized','position',[0,0,1,1]);
% 
    for i = 0: 1: 2 * (stepNumbers + 1)
        posse(i * 6 + 1, 1) = stair_wide_max;           posse(i * 6 + 1, 2) = (i + 1) * stair_length_total + stair_length_start;            posse(i * 6 + 1, 3) = - i * stairHeight;
        posse(i * 6 + 2, 1) = stair_wide_max;           posse(i * 6 + 2, 2) = i * stair_length_total + stair_length_start;                  posse(i * 6 + 2, 3) = - i * stairHeight;
        posse(i * 6 + 3, 1) = stair_wide_min;           posse(i * 6 + 3, 2) = i * stair_length_total + stair_length_start;                  posse(i * 6 + 3, 3) = - i * stairHeight;
        posse(i * 6 + 4, 1) = stair_wide_min;           posse(i * 6 + 4, 2) = (i + 1) * stair_length_total + stair_length_start;            posse(i * 6 + 4, 3) = - i * stairHeight;
        posse(i * 6 + 5, 1) = stair_wide_max;           posse(i * 6 + 5, 2) = (i + 1) * stair_length_total + stair_length_start;            posse(i * 6 + 5, 3) = - i * stairHeight;
        posse(i * 6 + 6, 1) = stair_wide_max;           posse(i * 6 + 6, 2) = (i + 1) * stair_length_total + stair_length_start;            posse(i * 6 + 6, 3) = - i * stairHeight;
        posse(i * 6 + 6, 1) = stair_wide_max;           posse(i * 6 + 6, 2) = (i + 1) * stair_length_total + stair_length_start;            posse(i * 6 + 6, 3) = - i * stairHeight;
        posse(i * 6 + 6, 1) = stair_wide_max;           posse(i * 6 + 6, 2) = (i + 1) * stair_length_total + stair_length_start;            posse(i * 6 + 6, 3) = - i * stairHeight;
        posse(i * 6 + 6, 1) = stair_wide_max;           posse(i * 6 + 6, 2) = (i + 1) * stair_length_total + stair_length_start;            posse(i * 6 + 6, 3) = - i * stairHeight;
    end
     plot3(posse(:,1),posse(:,2),posse(:,3),'-o','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',1)
     hold on

    %% plot Right Leg
    T0 = trotx(0); 
    T0 = T0 * transl( xComValue,yComValue,zComValue);

    T0_1        = T0;
    T0_1(1,4)   = T0_1(1,4) + hipDistance / 2;
    T1          = T0_1 * L1.A(RightLegJointValue(1)).T;
    T2          = T1   * L2.A(RightLegJointValue(2)+pi/2).T;
    T3          = T2   * L3.A(RightLegJointValue(3)).T;
    T4          = T3   * L4.A(RightLegJointValue(4)).T;
    T5          = T4   * L5.A(RightLegJointValue(5)).T;
    T6          = T5   * L6.A(RightLegJointValue(6)).T;
    poss = [ T0(1,4) T0(2,4) T0(3,4);
             T1(1,4) T1(2,4) T1(3,4);
             T2(1,4) T2(2,4) T2(3,4);
             T3(1,4) T3(2,4) T3(3,4);
             T4(1,4) T4(2,4) T4(3,4);
             T5(1,4) T5(2,4) T5(3,4);
             T6(1,4) T6(2,4) T6(3,4);];
    plot3(poss(:,1),poss(:,2),poss(:,3),'-o','LineWidth',15,'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',8)
    set(gca,'xtick',-400:100:400)
    set(gca,'ztick',-750:100:100)
    hold on

    xlabel('X');
    ylabel('y');
    zlabel('z');
    hold on
    
    % plot the foot
    RightFoot = [ T6(1,4) + 40  T6(2,4) - 60 * cos( RightPitchAngle )  T6(3,4) - 60 * tan( RightPitchAngle );
                  T6(1,4) + 40  T6(2,4) + 80 * cos( RightPitchAngle )  T6(3,4) + 80 * tan( RightPitchAngle ); 
                  T6(1,4) + 20  T6(2,4) + 140 * cos( RightPitchAngle )  T6(3,4) + 140 * tan( RightPitchAngle );
                  T6(1,4) - 20  T6(2,4) + 140 * cos( RightPitchAngle )  T6(3,4) + 140 * tan( RightPitchAngle );
                  T6(1,4) - 40  T6(2,4) + 80 * cos( RightPitchAngle )  T6(3,4) + 80 * tan( RightPitchAngle );
                  T6(1,4) - 40  T6(2,4) - 60 * cos( RightPitchAngle )  T6(3,4) - 60 * tan( RightPitchAngle );
                  T6(1,4) + 40  T6(2,4) - 60 * cos( RightPitchAngle )  T6(3,4) - 60 * tan( RightPitchAngle );];
              
    plot3(RightFoot(:,1),RightFoot(:,2),RightFoot(:,3),'-o','LineWidth',5,'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',6);
    hold on

    %% plot Left Leg
    T0_1        = T0;
    T0_1(1,4)   = T0_1(1,4) - hipDistance / 2;
    T1          = T0_1 * L1.A(LeftLegJointValue(1)).T;
    T2          = T1 * L2.A(LeftLegJointValue(2)+pi/2).T;
    T3          = T2 * L3.A(LeftLegJointValue(3)).T;
    T4          = T3 * L4.A(LeftLegJointValue(4)).T;
    T5          = T4 * L5.A(LeftLegJointValue(5)).T;
    T6          = T5 * L6.A(LeftLegJointValue(6)).T;
    poss = [ T0(1,4) T0(2,4) T0(3,4);
             T1(1,4) T1(2,4) T1(3,4);
             T2(1,4) T2(2,4) T2(3,4);
             T3(1,4) T3(2,4) T3(3,4);
             T4(1,4) T4(2,4) T4(3,4);
             T5(1,4) T5(2,4) T5(3,4);
             T6(1,4) T6(2,4) T6(3,4);];
    plot3(poss(:,1),poss(:,2),poss(:,3),'-o','LineWidth',15,'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',8);
    hold on ;
    
     % plot the foot
    LeftFoot = [ T6(1,4) + 40  T6(2,4) - 60 * cos( LeftPitchAngle )  T6(3,4) - 60 * tan( LeftPitchAngle );
                  T6(1,4) + 40  T6(2,4) + 80 * cos( LeftPitchAngle )  T6(3,4) + 80 * tan( LeftPitchAngle ); 
                  T6(1,4) + 20  T6(2,4) + 140 * cos( LeftPitchAngle )  T6(3,4) + 140 * tan( LeftPitchAngle );
                  T6(1,4) - 20  T6(2,4) + 140 * cos( LeftPitchAngle )  T6(3,4) + 140 * tan( LeftPitchAngle );
                  T6(1,4) - 40  T6(2,4) + 80 * cos( LeftPitchAngle )  T6(3,4) + 80 * tan( LeftPitchAngle );
                  T6(1,4) - 40  T6(2,4) - 60 * cos( LeftPitchAngle )  T6(3,4) - 60 * tan( LeftPitchAngle );
                  T6(1,4) + 40  T6(2,4) - 60 * cos( LeftPitchAngle )  T6(3,4) - 60 * tan( LeftPitchAngle );];
              
    plot3(LeftFoot(:,1),LeftFoot(:,2),LeftFoot(:,3),'-o','LineWidth',5,'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',6);
    hold on

    %% plot trunck Leg
    poss = [ xComValue - hipDistance / 4,yComValue,zComValue ;
             xComValue - hipDistance / 4,yComValue,zComValue + 100;
             xComValue + hipDistance / 4,yComValue,zComValue + 100;
             xComValue + hipDistance / 4,yComValue,zComValue;];
    plot3(poss(:,1),poss(:,2),poss(:,3),'-o','LineWidth',15,'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',8);
    hold on ;
    
    poss = [ xComValue - hipDistance / 4 - 20 ,yComValue,zComValue + 100;
             xComValue + hipDistance / 4 + 20 ,yComValue,zComValue + 100;
             xComValue + hipDistance / 2 + 85 ,yComValue,zComValue + 500;
             xComValue - hipDistance / 2 - 85 ,yComValue,zComValue + 500;
             xComValue - hipDistance / 4 - 20 ,yComValue,zComValue + 100;];
    plot3(poss(:,1),poss(:,2),poss(:,3),'-o','LineWidth',15,'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',8);
    hold on ;
   
    %% left hand
    T0_Hand_Left = T0 * transl(-hipDistance / 2 - 85, 0, 500);
    L1=Link('d',0,'a',0,'alpha',-pi/2,'modified');
    L2=Link('d',0,'a',0,'alpha',pi/2,'modified');
    L3=Link('d',0,'a',220,'alpha',pi/2,'modified' );
    L4=Link('d',0,'a',173,'alpha',0,'modified' );
    T0_Hand_Left = T0_Hand_Left* trotz(pi/2);
    T1 = T0_Hand_Left*L1.A(pi/2+LeftHand1Value).T;
    T2 = T1*L2.A(LeftHand2Value).T;
    T3 = T2*L3.A(LeftHand3Value).T;
    T4 = T3*L4.A(0).T;
    poss =[ T0_Hand_Left(1,4) T0_Hand_Left(2,4) T0_Hand_Left(3,4);
           T1(1,4) T1(2,4) T1(3,4);
           T2(1,4) T2(2,4) T2(3,4);
           T3(1,4) T3(2,4) T3(3,4);
           T4(1,4) T4(2,4) T4(3,4);];

    plot3(poss(:,1),poss(:,2),poss(:,3),'-o','LineWidth',15,'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',8);
    hold on
    
    %% Right hand
    T0_Hand_Right = T0 * transl(hipDistance / 2 + 85, 0, 500);
    L1=Link('d',0,'a',0,'alpha',-pi/2,'modified');
    L2=Link('d',0,'a',0,'alpha',pi/2,'modified');
    L3=Link('d',0,'a',220,'alpha',pi/2,'modified' );
    L4=Link('d',0,'a',173,'alpha',0,'modified' );
    T0_Hand_Right = T0_Hand_Right* trotz(pi/2);
    T1 = T0_Hand_Right*L1.A(pi/2+RightHand1Value).T;
    T2 = T1*L2.A(-RightHand2Value).T;
    T3 = T2*L3.A(RightHand3Value).T;
    T4 = T3*L4.A(0).T;
    poss =[ T0_Hand_Right(1,4) T0_Hand_Right(2,4) T0_Hand_Right(3,4);
           T1(1,4) T1(2,4) T1(3,4);
           T2(1,4) T2(2,4) T2(3,4);
           T3(1,4) T3(2,4) T3(3,4);
           T4(1,4) T4(2,4) T4(3,4);];

    plot3(poss(:,1),poss(:,2),poss(:,3),'-o','LineWidth',15,'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',8);
    hold on

    %% head
    T0_Head1 = T0 * transl(0, 0, 500);
    T0_Head2 = T0 * transl(0, 0, 700);
    poss =[ T0_Head1(1,4) T0_Head1(2,4) T0_Head1(3,4);
           T0_Head2(1,4) T0_Head2(2,4) T0_Head2(3,4);];

    plot3(poss(:,1),poss(:,2),poss(:,3),'-o','LineWidth',15,'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',8);
    hold on
    poss =[ T0_Head2(1,4) T0_Head2(2,4) T0_Head2(3,4);
           T0_Head2(1,4) T0_Head2(2,4) T0_Head2(3,4);];
    plot3(poss(:,1),poss(:,2),poss(:,3),'-o','LineWidth',15,'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',20);
    hold on

    axis equal
end
