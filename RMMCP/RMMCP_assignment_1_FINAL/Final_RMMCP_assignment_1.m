clear 
close all
clc
% Modified DH parameteres for SCARA robot  with four links

disp('Please remove ";"  or comments to see desired output in some cases accordingly')

disp('Prasun Biswas-267948')
% syms alpha theta d a
%disp('generate transformation matrix')

%  Ti2iminus=trotx(alpha)*transl([a 0 0])*trotz(theta)*transl([0 0 d])
% Ti2iminus=simplify (Ti2iminus)
% 
%  t21 = subs(Ti2iminus,[alpha theta d a],[0 0 0.4 0])
% t32 = subs(Ti2iminus,[alpha theta d a],[0 0 0 0.5])
% t43 = subs(Ti2iminus,[alpha theta d a],[pi 0 0 0.4])
% 
% t14 =t21*t32*t43

%-----------------------------------
L(1)=Link([0 0.4 0 0 0],'modified')
L(2)=Link([0 0 0.5 0 0],'modified')
L(3)=Link([0 0 0.4 pi 1],'modified')
L(4)=Link([0 0 0 0 0],'modified')
%fixing limit for prismatic joint>
L(3).qlim=[0 0.8]


disp('Link frames')

%Joining all the links%
SCARA=SerialLink(L,'name','scara robot');
% % SCARA.base=transl(0.1,0.1,0.2);

SCARA.base=transl(0.1,0.1,0.2);%value for z axis 0.2 instead of 0 to represent the heitht of base

% 
disp('robot in zero angle position')
%%Zero Angle Position of Robot 
sfk=SCARA.fkine([0 0 0 0]);

SCARA.tool=transl(0, 0, -0.135)% modified for checking considering only for z direction

SCARA.teach([0 0 0 0])
% disp('with a offset value')
% SCARA.plot([0.5 0 0.4 0])
SCARA.plot([0 0 0 0],'workspace',[-2 2 -2 2 -2 2],'reach',1)
title('robot plot at zero angle')


% declaring point frame red
Tr = transl([0.3, 0.8, 0.1]);
hold on
trplot(Tr,'frame','P1','color','r')
% 
%declaring point frame green
Tg = transl([0.6, 0.6, 0])*trotz(-pi/4);
hold on
trplot(Tg,'frame','P2','color','g')
w=waitforbuttonpress
disp('forward kinematics for green box')
 %forward kinematics for green box
TgFK=SCARA.fkine([-4.8 -82.9 0.728 -31.915])
SCARA.plot([-4.9 -82.9 0.728 -31.915]) 
title('At green box after fKine')


% 
w=waitforbuttonpress
disp('forward kinematics for red box')
 %forward kinematics for red box
TrFK=SCARA.fkine([-4.45 -82.88 0.64 -32.35]);
SCARA.plot([-4.45 -82.88 0.64 -32.35]) 
title('At red box after fKine')

% 
 w=waitforbuttonpress
disp('inverse kinematics for red box')

 %inverse kinematics for red box
q1=SCARA.ikine(transl([0.3, 0.8, 0.1]),[2*pi, pi, pi],'mask',[1 1 1 0 0 0.01]);
hold on
SCARA.plot(q1,'workspace',[-2 2 -2 2 -2 2],'reach',1)
title('At red box after iKine')

 w=waitforbuttonpress
disp('inverse kinematics for green box')

 %inverse kinematics for green box
q2=SCARA.ikine(transl([0.6, 0.6, 0])*trotz(-pi/4),[2*pi, pi, pi],'mask',[1 1 1 0 0 0.033]);
hold on
SCARA.plot(q2,'workspace',[-3 3 -3 3 -3 3],'reach',1)
title('At green box after iKine')

 w=waitforbuttonpress
disp('run animation')
title('animation of position change')

% to show the transformation as a end effector as animation
for i=1:10
t= [0:1:20];
q=jtraj(q1,q2,t)
SCARA.plot(q)
t= [0:1:20];
q=jtraj(q2,q1,t)
SCARA.plot(q)
end
 %shows error if the plot is stopped before the animation is completed