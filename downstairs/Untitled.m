L1=Link('d',0,'a',0,'alpha',0,'modified');
L2=Link('d',0,'a',35.5,'alpha',pi/2,'modified');
L3=Link('d',0,'a',65,'alpha',0,'modified' );
L4=Link('d',0,'a',65,'alpha',0,'modified');
L5=Link('d',0,'a',42.5,'alpha',-pi/2,'modified');
L6=Link('d',0,'a',0,'alpha',-pi/2,'modified');
LeftLeg2=SerialLink([L1,L2,L3,L4,L5,L6],'name','LegBackward');   %seriallink????
% teach(LeftLeg2);
q = [0 pi/2 0 0 0 0 0];
LeftLeg2.teach;