function [] = zhuan1(DateString)
[u,ss,ssl,tq,tqs,td,ts,N,sw0,sh,diffh,h,qnew,ds,db,dt,S,K,AlpRight0,BetRight0,GamRight0,AlpRight1,BetRight1,GamRight1,AlpLeft0,BetLeft0,GamLeft0,AlpLeft1,BetLeft1,GamLeft1,hz,hl,tqq,tq1] = Init( );
handleft();
handright();
footleft();
footright();
VarName1 = load( 'L1.txt');
VarName2 = load( 'L2.txt');
VarName3 = load( 'L3.txt');
VarName4 = load( 'L4.txt');
VarName5 = load( 'L5.txt');
VarName6 = load( 'L6.txt');
VarName7 = load( 'R1.txt');
VarName8 = load( 'R2.txt');
VarName9 = load( 'R3.txt');
VarName10 = load('R4.txt');
VarName11 = load('R5.txt');
VarName12 = load('R6.txt');
VarName13 = load('LH1.txt');
VarName14 = load('LH2.txt');
VarName15 = load('LH3.txt');
VarName16 = load('RH1.txt');
VarName17 = load('RH2.txt');
VarName18 = load('RH3.txt');

VarName19 = load('L7.txt');
VarName20 = load('L8.txt');
VarName21 = load('L9.txt');

VarName22 = load('L10.txt');
VarName23 = load('L11.txt');
VarName24 = load('L12.txt');

fid12 = fopen([DateString,'/','12.txt'],'wt');
fid4  = fopen([DateString,'/','4.txt'],'wt');
fid2  = fopen([DateString,'/','2.txt'],'wt');
fid6  = fopen([DateString,'/','6.txt'],'wt');
fid8  = fopen([DateString,'/','8.txt'],'wt');
fid10 = fopen([DateString,'/','10.txt'],'wt');
fid11 = fopen([DateString,'/','11.txt'],'wt');
fid3  = fopen([DateString,'/','3.txt'],'wt');
fid1  = fopen([DateString,'/','1.txt'],'wt');
fid5  = fopen([DateString,'/','5.txt'],'wt');
fid7  = fopen([DateString,'/','7.txt'],'wt');
fid9  = fopen([DateString,'/','9.txt'],'wt');
fid14 = fopen([DateString,'/','14.txt'],'wt');
fid16 = fopen([DateString,'/','16.txt'],'wt');
fid18 = fopen([DateString,'/','18.txt'],'wt');
fid13 = fopen([DateString,'/','13.txt'],'wt');
fid15 = fopen([DateString,'/','15.txt'],'wt');
fid17 = fopen([DateString,'/','17.txt'],'wt');


for j=1:1:50
    y2=-S*sin(pi/2*j/50);
    y1=S*sin(pi/2*j/50);%+8;
    y14=-60;
    y13=60;
    y16=-85;
    y15=85;
    
    fprintf(fid12,'%f \n',0);
    fprintf(fid4,'%f \n',0);
    fprintf(fid2,'%f \n',y2);
    fprintf(fid6,'%f \n',0);%+2
    fprintf(fid8,'%f \n',0);          %前翘为正-1
    fprintf(fid10,'%f \n',-2);%-2
    
    fprintf(fid11,'%f \n',0);
    fprintf(fid3,'%f \n',0);
    fprintf(fid1,'%f \n',y1);
    fprintf(fid5,'%f \n',0);%-10;
    fprintf(fid7,'%f \n',0);%+9;
    fprintf(fid9,'%f \n',+1);%-1.5
    
    fprintf(fid14,'%f \n',y14);
    fprintf(fid16,'%f \n',y16);
    fprintf(fid18,'%f \n',0);
    fprintf(fid13,'%f \n',y13);
    fprintf(fid15,'%f \n',y15);
    fprintf(fid17,'%f \n',0);
end
for i=1:1:size(VarName1)
    x1  = VarName1(i,1);
    x2  = VarName2(i,1);
    x3  = VarName3(i,1);
    x4  = VarName4(i,1);
    x5  = VarName5(i,1);
    x6  = VarName6(i,1);
    x1y = VarName7(i,1);
    x2y = VarName8(i,1);
    x3y = VarName9(i,1);
    x4y = VarName10(i,1);
    x5y = VarName11(i,1);
    x6y = VarName12(i,1);
    
    x14 = VarName13(i,1);
    x16 = VarName14(i,1);
    x18 = VarName15(i,1);
    
    x13 = VarName16(i,1);
    x15 = VarName17(i,1);
    x17 = VarName18(i,1);
    
    x19 = VarName19(i,1);
    x20 = VarName20(i,1);
    x21 = VarName21(i,1);
    
    x22 = VarName22(i,1);
    x23 = VarName23(i,1);
    x24 = VarName24(i,1);
    
    
     q_LegR = [x1y x2y x3y x4y x5y x6y];
    q_LegL = [x1 x2 x3 x4 x5 x6];
    q_HandR = [x13 x15 x17];
    q_HandL = [x14 x16 x18];
    cog = [x19 x20 x21];
    cog2=[x22 x23 x24];
    subplot(1,2,1)
    H5Flash(q_LegR,q_LegL,q_HandR,q_HandL,i,cog);
    title('下台阶侧视图');
    view(90,0);
    subplot(1,2,2)
    H5Flash(q_LegR,q_LegL,q_HandR,q_HandL,i,cog);
    title('轴侧视图');
    view(50,25);
%     subplot(2,2,3)
%     H5Flash(q_LegR,q_LegL,q_HandR,q_HandL,i,cog2);
%     title('轴侧视图');
%     view(90,0);
%     subplot(2,2,4)
%     H5Flash(q_LegR,q_LegL,q_HandR,q_HandL,i,cog2);
%     title('轴侧视图');
%     view(50,25);
%     subplot(1,4,3)
%     H5Flash(q_LegR,q_LegL,q_HandR,q_HandL,i,cog);
%   
%     view(0,90);
%      subplot(1,4,4)
%     H5Flash(q_LegR,q_LegL,q_HandR,q_HandL,i,cog);
% 
%     view(0,0);
    pause(0.05);
    clf;
    
    y12 = x1*180/pi;                          %12  =  1
    y4  = -x2*180/pi+90;                      %4  =  -2
    y2  = -x3*180/pi-S;                         %2  =  -3
    y6  = -x4*180/pi;%+1;                         %6  =  -4
    y8  = -x5*180/pi;%-1;                         %8  = -5
    y10 = x6*180/pi-2;%-2;                          %10  =  6
    
    y11 = x1y*180/pi;                         %11  =  1y
    y3  = -x2y*180/pi+90;                     %3  =   -2y
    y1  = x3y*180/pi+S;%+8;                         %1  =  3y
    y5  = x4y*180/pi;%-10;                         %5  =  4y
    y7  = x5y*180/pi;%+9;                         %7  =  5y
    y9  = x6y*180/pi+1;%-1.5;                         %9 =  6y
    
    y14 = x14*180/pi - 60;                    %14  =  1al-60
    y16 = x16*180/pi - 85;                    %16  =  2al-85
    y18 = -x18*180/pi;                        %18  =  -3al
    y13 = -x13*180/pi + 60;                   %13  =  -1ar+60
    y15 = -x15*180/pi+ 85;                    %15  =  -2ar+85
    y17 = x17*180/pi ;                        %17  =  3ar
    
    fprintf(fid12,'%f \n',y12);
    fprintf(fid4,'%f \n',y4);
    fprintf(fid2,'%f \n',y2);
    fprintf(fid6,'%f \n',y6);
    fprintf(fid8,'%f \n',y8);
    fprintf(fid10,'%f \n',y10);
    
    fprintf(fid11,'%f \n',y11);
    fprintf(fid3,'%f \n',y3);
    fprintf(fid1,'%f \n',y1);
    fprintf(fid5,'%f \n',y5);
    fprintf(fid7,'%f \n',y7);
    fprintf(fid9,'%f \n',y9);
    
    fprintf(fid14,'%f \n',y14);
    fprintf(fid16,'%f \n',y16);
    fprintf(fid18,'%f \n',y18);
    fprintf(fid13,'%f \n',y13);
    fprintf(fid15,'%f \n',y15);
    fprintf(fid17,'%f \n',y17);
    
end
fclose('all');
end