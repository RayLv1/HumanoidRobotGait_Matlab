function[]=ZMP_check_1(DateString)
fid=fopen([DateString,'/','zmp_check_1.txt'],'wt');
[u,ss,ssl,fydelay,fydgree,kuanadd,kuandelay,tq,tqs,td,ts,N,sw,sh,diffh,h,qnew,ds,db,dt,S] = Init( );

g=9806.65;
pz=0;%����߶ȣ�ƽ��ʱΪ0
l=135;w=90.6;h=10;Ss=ss;      %Ss�벽��,�ȶ�����ʱ  ����С��l  ����λΪmm
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
%% TqΪ��˫��֧��ʱ��,�����½������ƣ��ɲ��⣩
%     if(t>0&&(t<=Tq))                        
%         if(X_zmp>=-(l/2-2)&&X_zmp<=(l/2-2)&&Y_zmp>=-(3*w/2+h-2)&&Y_zmp<=(w/2-2))
%             
%             %figure(1)
%             line([l/2-20,l/2-20,-(l/2-20),-(l/2-20),l/2-20],...
%                 [w/2-20,-(3*w/2+h-20),-(3*w/2+h-20),w/2-20,w/2-20],'Marker','.','LineStyle','-','Color','b')
%             hold on
%             %%%%%%ZMP����
%             
%             plot(X_zmp,Y_zmp,'Marker','o','Color','r');
%             hold on
%             fprintf(fid,'ZMP IN Polygon,%d,%d\n',X_zmp,Y_zmp);
%         else
%             fprintf(fid,'ZMP OUT Polygon\n');
%         end
%     end
%% TtΪ�𲽵���֧�ţ������֧���ҽ�̧���ҽ����ǰʱ��
        if(Tq+i*T<t<=Tt+Tq+i*T)                     
            %�������������
            %figure(1)
            line([l/2-20+17+2*i*Sw,l/2-20+17+2*i*Sw,-(l/2-20)+17+2*i*Sw,-(l/2-20)+17+2*i*Sw,l/2-20+17+2*i*Sw],...
                    [w/2-20,-(w/2-20),-(w/2-20),w/2-20,w/2-20],'Marker','.','LineStyle','-','Color',[0 0.8 0])
            line([l/2+17+2*i*Sw,l/2+17+2*i*Sw,-(l/2)+17+2*i*Sw,-(l/2)+17+2*i*Sw,l/2+17+2*i*Sw],...
                 [w/2,-(w/2),-(w/2),w/2,w/2],'Marker','.','LineStyle','-.','Color',[0.8 0 0])
            %%%%%%ZMP����
            Y_zmp = (Ss*sin(pi*(t+td/2)/(td+ts))-Cz*(-(pi/(td+ts)^2*Ss*sin(pi*(t+td/2)/(td+ts))))/g)-50;
            X_zmp=(Sw*t/(tqs+td));
            plot(X_zmp,Y_zmp,'Marker','o','Markersize',1.2,'Color','k');
            hold on
            %ZMP�ж�
            if(X_zmp>=-(l/2-20)+17+2*i*Sw && X_zmp<=l/2-20+17+2*i*Sw && Y_zmp>=-(w/2-20) && Y_zmp<=(w/2-20))
                fprintf(fid,'ZMP IN Polygon,%d,%d\n',X_zmp,Y_zmp);
            else
                fprintf(fid,'ZMP OUT Polygon\n');
            end
        end
%% TdΪ�ҽ���غ�˫��֧��ʱ�䣬������������������ת�ƹ���
        if(Tt+Tq+i*T<t<=Tt+Tq+td+i*T)           
        %���֧�Ž�����˫��֧��
        %�������������
        %figure(1)
        line([l/2-20+17+2*i*Sw,l/2-20+Sw+17+2*i*Sw,l/2-20+Sw+17+2*i*Sw,-(l/2-20-Sw)+17+2*i*Sw,-(l/2-20)+17+2*i*Sw,-(l/2-20)+17+2*i*Sw,l/2-20+17+2*i*Sw],...
             [w/2-20,-(w/2+h+20),-(3*w/2+h-20),-(3*w/2+h-20),-(w/2-20),w/2-20,w/2-20],'Marker','.','LineStyle','-','LineWidth',2,'Color','c')
        hold on
        %%%%%%����ZMP
        Y_zmp=(Ss*sin(pi*(t+td/2)/(td+ts))-Cz*(-(pi/(td+ts)^2*Ss*sin(pi*(t+td/2)/(td+ts))))/g)-50;
        X_zmp=(Sw*t/(tqs+td));
        plot(X_zmp,Y_zmp,'Marker','o','Color','r');
        hold on
                
        l1=(w+h)/(-Sw)*(X_zmp-(l/2-20+Sw+2*i*Sw))-(w/2+h+20);
        l2=(w+h)/(-Sw)*(X_zmp+(l/2-20-Sw+2*i*Sw))-(3*w/2+h-20);
            %ZMP�ж�
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
%% TsΪ����֧��ʱ�䣬���ҽ�֧�ţ����̧����ع���
       if(Tt+Tq+td+i*T<t<=Tt+Tq+td+ts+i*T)      
       %�ҽ�֧��
       %�������������
       %figure(1)
       line([l/2-20+Sw+17+2*i*Sw,l/2-20+Sw+17+2*i*Sw,-(l/2-20)+Sw+17+2*i*Sw,-(l/2-20-Sw)+17+2*i*Sw,l/2-20+Sw+17+2*i*Sw],...
            [-(w/2+h+20),-(3*w/2+h-20),-(3*w/2+h-20),-(w/2+h+20),-(w/2+h+20)],'Marker','.','LineStyle','-','Color',[0 0.8 0])
       line([l/2+Sw+17+2*i*Sw,l/2+Sw+17+2*i*Sw,-(l/2)+Sw+17+2*i*Sw,-(l/2-Sw)+17+2*i*Sw,l/2+Sw+17+2*i*Sw],...
            [-(w/2+h),-(3*w/2+h),-(3*w/2+h),-(w/2+h),-(w/2+h)],'Marker','.','LineStyle','-.','Color',[0.8 0 0])
       hold on
       %%%%%%����ZMP
       Y_zmp=(Ss*sin(pi*(t+td/2)/(td+ts))-Cz*(-(pi/(td+ts)^2*Ss*sin(pi*(t+td/2)/(td+ts))))/g)-50;
       X_zmp=(Sw*t/(tqs+td));
       plot(X_zmp,Y_zmp,'Marker','o','Markersize',1.2,'Color','k');
       hold on      
           %ZMP�ж�
           if(X_zmp>=-(l/2-20)+17+Sw+2*i*Sw && X_zmp<=l/2-20+Sw+17+2*i*Sw && Y_zmp>=-(3*w/2+h-20) && Y_zmp<=-(w/2+h+20))
              fprintf(fid,'ZMP IN Polygon,%d,%d\n',X_zmp,Y_zmp);
           else
              fprintf(fid,'ZMP OUT Polygon');
           end
       end
%% �����غ�˫��֧��ʱ�䣬������������������ת�ƹ���
       if(Tt+Tq+td+ts+i*T<t<=Tt+Tq+2*td+ts+i*T)
       %�������������
       %figure(1)
       line([l/2-20+Sw+17+2*i*Sw,l/2-20+2*Sw+17+2*i*Sw,l/2-20+2*Sw+17+2*i*Sw,-(l/2-20-2*Sw)+17+2*i*Sw,-(l/2-20-Sw)+17+2*i*Sw,-(l/2-20-Sw)+17+2*i*Sw,l/2-20+Sw+17+2*i*Sw],...
            [-(3*w/2+h-20),-(w/2-20),(w/2-20),(w/2-20),-(w/2+h+20),-(3*w/2+h-20),-(3*w/2+h-20)],'Marker','.','LineStyle','-','LineWidth',2,'Color',[0.3 0.4 0.5])
       %%%%%%ZMP����
       Y_zmp=(Ss*sin(pi*(t+td/2)/(td+ts))-Cz*(-(pi/(td+ts)^2*Ss*sin(pi*(t+td/2)/(td+ts))))/g)-50;
       X_zmp=(Sw*t/(tqs+td));
       plot(X_zmp,Y_zmp,'Marker','o','Color','r');
       hold on
       l3=(w+h)/Sw*(X_zmp-(l/2-20+2*i*Sw))-w/2+20;
       l4=(w+h)/Sw*(X_zmp+(l/2-20+2*i*Sw))+w/2-20;
            %ZMP�ж�
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