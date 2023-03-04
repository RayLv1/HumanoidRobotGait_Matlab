function[]=ZMP_check_1(DateString)
fid=fopen([DateString,'/','zmp_check_1.txt'],'wt');
[u,ss,ssl,fydelay,fydgree,kuanadd,kuandelay,tq,tqs,td,ts,N,sw,sh,diffh,h,qnew,ds,db,dt,S] = Init( );

g=9806.65;
pz=0;%地面高度，平地时为0
l=135;w=90.6;h=10;Ss=ss;      %Ss半步长,稳定行走时  步长小于l  ，单位为mm
Tq=tq;

Cz=186.05;

Sw=sw;
Tt=ts;
% a2=-42.55;a3=-65;a4=-65;a5=-33.5;
% 
% A=0;
% B=0;
% M=0;
n=4;
T=ts+td;
%%
            figure(1)
            line([l/2-20+17,l/2-20+17,-(l/2-20)+17,-(l/2-20)+17,l/2-20+17],...
                 [w/2-20,-(3*w/2+h-20),-(3*w/2+h-20),w/2-20,w/2-20],'Marker','.','LineStyle','-','Color','b')
            hold on
for i=0:n
%%
for t=0+i*T:0.1:2+i*T
    %figure(1)
    x=(Sw*t/(tqs+td));
    y=Ss*sin(pi*(t+td/2)/(td+ts))-50;
    plot(x,y,'*');
    hold on
%% Tq为起步双腿支撑时间,重心下降、左移（可不测）
%     if(t>0&&(t<=Tq))                        
%         if(X_zmp>=-(l/2-2)&&X_zmp<=(l/2-2)&&Y_zmp>=-(3*w/2+h-2)&&Y_zmp<=(w/2-2))
%             
%             %figure(1)
%             line([l/2-20,l/2-20,-(l/2-20),-(l/2-20),l/2-20],...
%                 [w/2-20,-(3*w/2+h-20),-(3*w/2+h-20),w/2-20,w/2-20],'Marker','.','LineStyle','-','Color','b')
%             hold on
%             %%%%%%ZMP计算
%             
%             plot(X_zmp,Y_zmp,'Marker','o','Color','r');
%             hold on
%             fprintf(fid,'ZMP IN Polygon,%d,%d\n',X_zmp,Y_zmp);
%         else
%             fprintf(fid,'ZMP OUT Polygon\n');
%         end
%     end
%% Tt为起步单腿支撑，即左脚支撑右脚抬起到右脚落地前时间
        if(Tq+i*T<t<=Tt+Tq+i*T)                     
            %画出多边形区域
            %figure(1)
            line([l/2-20+17+2*i*Sw,l/2-20+17+2*i*Sw,-(l/2-20)+17+2*i*Sw,-(l/2-20)+17+2*i*Sw,l/2-20+17+2*i*Sw],...
                    [w/2-20,-(w/2-20),-(w/2-20),w/2-20,w/2-20],'Marker','.','LineStyle','-','Color',[0 0.8 0])
            line([l/2+17+2*i*Sw,l/2+17+2*i*Sw,-(l/2)+17+2*i*Sw,-(l/2)+17+2*i*Sw,l/2+17+2*i*Sw],...
                 [w/2,-(w/2),-(w/2),w/2,w/2],'Marker','.','LineStyle','-.','Color',[0.8 0 0])
            %%%%%%ZMP计算
            Y_zmp = (Ss*sin(pi*(t+td/2)/(td+ts))-Cz*(-(pi/(td+ts)^2*Ss*sin(pi*(t+td/2)/(td+ts))))/g)-50;
            X_zmp=(Sw*t/(tqs+td));
            plot(X_zmp,Y_zmp,'Marker','o','Markersize',1.2,'Color','k');
            hold on
            %ZMP判断
            if(X_zmp>=-(l/2-20)+17+2*i*Sw && X_zmp<=l/2-20+17+2*i*Sw && Y_zmp>=-(w/2-20) && Y_zmp<=(w/2-20))
                fprintf(fid,'ZMP IN Polygon,%d,%d\n',X_zmp,Y_zmp);
            else
                fprintf(fid,'ZMP OUT Polygon\n');
            end
        end
%% Td为右脚落地后双腿支撑时间，即重心由左腿向右腿转移过程
        if(Tt+Tq+i*T<t<=Tt+Tq+td+i*T)           
        %左脚支撑结束，双脚支撑
        %画出多边形区域
        %figure(1)
        line([l/2-20+17+2*i*Sw,l/2-20+Sw+17+2*i*Sw,l/2-20+Sw+17+2*i*Sw,-(l/2-20-Sw)+17+2*i*Sw,-(l/2-20)+17+2*i*Sw,-(l/2-20)+17+2*i*Sw,l/2-20+17+2*i*Sw],...
             [w/2-20,-(w/2+h+20),-(3*w/2+h-20),-(3*w/2+h-20),-(w/2-20),w/2-20,w/2-20],'Marker','.','LineStyle','-','LineWidth',2,'Color','c')
        hold on
        %%%%%%计算ZMP
        Y_zmp=(Ss*sin(pi*(t+td/2)/(td+ts))-Cz*(-(pi/(td+ts)^2*Ss*sin(pi*(t+td/2)/(td+ts))))/g)-50;
        X_zmp=(Sw*t/(tqs+td));
        plot(X_zmp,Y_zmp,'Marker','o','Color','r');
        hold on
                
        l1=(w+h)/(-Sw)*(X_zmp-(l/2-20+Sw+2*i*Sw))-(w/2+h+20);
        l2=(w+h)/(-Sw)*(X_zmp+(l/2-20-Sw+2*i*Sw))-(3*w/2+h-20);
            %ZMP判断
            if(X_zmp>=-(l/2-20)+17+2*i*Sw && X_zmp<=-(l/2-20-Sw)+17+2*i*Sw && Y_zmp>=l2 && Y_zmp<=(w/2-20))
                    fprintf(fid,'ZMP IN Polygon,%d,%d\n',X_zmp,Y_zmp);
            else
                if(X_zmp>=-(l/2-20-Sw)+17+2*i*Sw && X_zmp<=l/2-20+17+2*i*Sw && Y_zmp>=-(3*w/2+h-20) && Y_zmp<=(w/2-20))
                   fprintf(fid,'ZMP IN Polygon,%d,%d\n',X_zmp,Y_zmp);
                else
                    if(X_zmp>=l/2-20+17+2*i*Sw && X_zmp<=l/2-20+17+Sw+2*i*Sw && Y_zmp>=-(3*w/2+h-20) && Y_zmp<=l1)
                       fprintf(fid,'ZMP IN Polygon,%d,%d\n',X_zmp,Y_zmp);
                    else
                       fprintf(fid,'ZMP OUT Polygon\n');
                    end
                end
            end
        end
%% Ts为单腿支撑时间，即右脚支撑，左脚抬起到落地过程
       if(Tt+Tq+td+i*T<t<=Tt+Tq+td+ts+i*T)      
       %右脚支撑
       %画出多边形区域
       %figure(1)
       line([l/2-20+Sw+17+2*i*Sw,l/2-20+Sw+17+2*i*Sw,-(l/2-20)+Sw+17+2*i*Sw,-(l/2-20-Sw)+17+2*i*Sw,l/2-20+Sw+17+2*i*Sw],...
            [-(w/2+h+20),-(3*w/2+h-20),-(3*w/2+h-20),-(w/2+h+20),-(w/2+h+20)],'Marker','.','LineStyle','-','Color',[0 0.8 0])
       line([l/2+Sw+17+2*i*Sw,l/2+Sw+17+2*i*Sw,-(l/2)+Sw+17+2*i*Sw,-(l/2-Sw)+17+2*i*Sw,l/2+Sw+17+2*i*Sw],...
            [-(w/2+h),-(3*w/2+h),-(3*w/2+h),-(w/2+h),-(w/2+h)],'Marker','.','LineStyle','-.','Color',[0.8 0 0])
       hold on
       %%%%%%计算ZMP
       Y_zmp=(Ss*sin(pi*(t+td/2)/(td+ts))-Cz*(-(pi/(td+ts)^2*Ss*sin(pi*(t+td/2)/(td+ts))))/g)-50;
       X_zmp=(Sw*t/(tqs+td));
       plot(X_zmp,Y_zmp,'Marker','o','Markersize',1.2,'Color','k');
       hold on      
           %ZMP判断
           if(X_zmp>=-(l/2-20)+17+Sw+2*i*Sw && X_zmp<=l/2-20+Sw+17+2*i*Sw && Y_zmp>=-(3*w/2+h-20) && Y_zmp<=-(w/2+h+20))
              fprintf(fid,'ZMP IN Polygon,%d,%d\n',X_zmp,Y_zmp);
           else
              fprintf(fid,'ZMP OUT Polygon');
           end
       end
%% 左脚落地后双腿支撑时间，即重心由右腿向左腿转移过程
       if(Tt+Tq+td+ts+i*T<t<=Tt+Tq+2*td+ts+i*T)
       %画出多边形区域
       %figure(1)
       line([l/2-20+Sw+17+2*i*Sw,l/2-20+2*Sw+17+2*i*Sw,l/2-20+2*Sw+17+2*i*Sw,-(l/2-20-2*Sw)+17+2*i*Sw,-(l/2-20-Sw)+17+2*i*Sw,-(l/2-20-Sw)+17+2*i*Sw,l/2-20+Sw+17+2*i*Sw],...
            [-(3*w/2+h-20),-(w/2-20),(w/2-20),(w/2-20),-(w/2+h+20),-(3*w/2+h-20),-(3*w/2+h-20)],'Marker','.','LineStyle','-','LineWidth',2,'Color',[0.3 0.4 0.5])
       %%%%%%ZMP计算
       Y_zmp=(Ss*sin(pi*(t+td/2)/(td+ts))-Cz*(-(pi/(td+ts)^2*Ss*sin(pi*(t+td/2)/(td+ts))))/g)-50;
       X_zmp=(Sw*t/(tqs+td));
       plot(X_zmp,Y_zmp,'Marker','o','Color','r');
       hold on
       l3=(w+h)/Sw*(X_zmp-(l/2-20+2*i*Sw))-w/2+20;
       l4=(w+h)/Sw*(X_zmp+(l/2-20+2*i*Sw))+w/2-20;
            %ZMP判断
            if(X_zmp>=-(l/2-20-Sw)+17+2*i*Sw && X_zmp<=-(l/2-20-2*Sw)+17+2*i*Sw && Y_zmp>=-(3*w/2+h-20) && Y_zmp<=l4)
               fprintf(fid,'ZMP IN Polygon,%d,%d\n',X_zmp,Y_zmp);
            else
               if(X_zmp>=-(l/2-20-2*Sw)+17+2*i*Sw && X_zmp<=l/2-20+Sw+17+2*i*Sw && Y_zmp>=-(3*w/2+h-20) && Y_zmp<=(w/2-20))
                  fprintf(fid,'ZMP IN Polygon,%d,%d\n',X_zmp,Y_zmp);
               else
                  if(X_zmp>=l/2-20+Sw+17+2*i*Sw && X_zmp<=l/2-20+2*Sw+17+2*i*Sw && Y_zmp>=l3 && Y_zmp<=(w/2-20))
                     fprintf(fid,'ZMP IN Polygon,%d,%d\n',X_zmp,Y_zmp);
                  else
                     fprintf(fid,'ZMP OUT Polygon'); 
                  end
               end
            end
       end
    %end
end
end
end