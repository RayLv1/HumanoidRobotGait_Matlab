function [theta_vec1,theta_vec2] =ikineH5(T)
a3 = 0;
a4 = 373;
a5 = 364;
as = 737;
a6 = 0;

% T=robot.fkine(q).T;
nx=T(1,1);ny=T(2,1);nz=T(3,1);
ox=T(1,2);oy=T(2,2);oz=T(3,2);
ax=T(1,3);ay=T(2,3);az=T(3,3);
px=T(1,4);py=T(2,4);pz=T(3,4);

% theta 1
tmp1 = az*px - pz*ax;
tmp2 = pz*ay - az*py;
if abs(tmp1)<1e-12 && abs(tmp2) <1e-12
    theta_1 = 0;
else
    theta_1 = atan(tmp1/tmp2);
end
s1 = sin(theta_1);
c1 = cos(theta_1);

% theta 2
tmp3 = -px*c1 - py*s1;
% if abs(tmp3) < 1e-12 && abs(pz) < 1e-12
%     theta_2 = 0;
% else
    theta_2 = atan(pz/tmp3);
    if theta_2 < 0
        theta_2 = theta_2 + pi;
%     else
%         if theta_2 < -pi
%             theta_2 = theta_2 + pi;
%         end
    end
% end
s2 = sin(theta_2);
c2 = cos(theta_2);

% theta 34
s34 = ax*c1*c2 - az*s2 + ay*c2*s1;
c34 = ay*c1 - ax*s1;
theta_34 = atan2(s34, c34);

% theta 6
theta_6 = atan2(-nz*c2 - nx*c1*s2 - ny*s1*s2, -oz*c2 - ox*c1*s2 - oy*s1*s2);
s6 = sin(theta_6);
c6 = cos(theta_6);

% theta 3
s3_tmp = px*s1 - a6*s34 - py*c1;
s3 = s3_tmp/as;
c3_tmp = px*c1*c2 - pz*s2 + py*c2*s1- a3 - a6*c34;
c3 = c3_tmp/as;
theta_3 = atan2(s3,c3);
as = (s3_tmp^2+c3_tmp^2)^0.5;

% theta 4????
theta_4 = theta_34 - theta_3;
if abs(as) > (abs(a4 + a5) + 1e-2)
    as
    error('as is too large!!!\n');
end

%%
% theta 4 new
tmp1 = (a4^2 + a5^2 - as^2)/(2 * a4 * a5);

if tmp1 >1
    tmp1 = 1;
else
    if tmp1 < -1
        tmp1 = -1;
    end
end

% ????1
theta_4_new1 = acos(-tmp1);
% ????2
theta_4_new2 = -theta_4_new1;
% ????1
t1 = atan2(a5 * sin(theta_4_new1), (a5 * cos(theta_4_new1) + a4));
theta_3_new1 = atan2(as * s3, as * c3) - t1;
% ????2
t2 = atan2(a5 * sin(theta_4_new2), (a5 * cos(theta_4_new2) + a4));
theta_3_new2 = atan2(as * s3, as * c3) - t2;
% ????1
theta_5_new1 = theta_3 + theta_4 - theta_4_new1 - theta_3_new1;
% ????2
theta_5_new2 = theta_3 + theta_4 - theta_4_new2 - theta_3_new2;

theta_vec1 = Normal_ZFPI([theta_1 theta_2 theta_3_new1 theta_4_new1 theta_5_new1 theta_6]);
theta_vec2 = Normal_ZFPI([theta_1 theta_2 theta_3_new2 theta_4_new2 theta_5_new2 theta_6]);
% theta_vec1 = [theta_1 theta_2 theta_3_new1 theta_4_new1 theta_5_new1 theta_6];
% theta_vec2 = [theta_1 theta_2 theta_3_new2 theta_4_new2 theta_5_new2 theta_6];

end

function theta2 = Normal_ZFPI(theta)
theta2 = theta;
m=size(theta);
for i = 1:m(2)
    if theta(i) > pi
        theta2(i) = theta2(i)-2*pi;
    end
    if theta(i) <-pi
        theta2(i) = theta2(i)+2*pi;
    end
end
end
