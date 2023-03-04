a2=0;
a3=373;
a4=364;
a5=0;
asum=a2+a3+a4+a5;
L1=Link('d',0,'a',0,'alpha',0,'modified');
L2=Link('d',0,'a',0,'alpha',-pi/2,'modified');
L3=Link('d',0,'a',0,'alpha',-pi/2,'modified' );
L4=Link('d',0,'a',a3,'alpha',0,'modified');
L5=Link('d',0,'a',a4,'alpha',0,'modified');
L6=Link('d',0,'a',0,'alpha',pi/2,'modified');
LeftLeg1=SerialLink([L1,L2,L3,L4,L5,L6],'name','LegForward');


q=[0,-pi/2,pi/6,-pi/3,pi/6,0];
LeftLeg1.plot(q);
teach(LeftLeg1);
