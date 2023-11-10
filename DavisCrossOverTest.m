% function [C1,C2] = DavisCrossOverTest(P1,P2)
P1 = [0 1 2 3 4 ];
P2 = [1 4 3 0 2 ];

Size = length(P1);
C1 = zeros(Size,1);
C2 = zeros(Size,1);

P1Taken = zeros(Size,1);
P2Taken = zeros(Size,1);
r1=0;
r2=0;
% 
% while (r2 == r1)
% r1 = rand*Size+1;
% r1 = fix(r1);
% r2 = rand*Size+1;
% r2 = fix(r2);
% end
% if r1>r2
% rb =r1;
% rs=r2;
% 
% else
% 
% rb =r2;
% rs=r1;
% end

rs=3;
rb=4;


C1(rs:rb)= P1(rs:rb);
C2(rs:rb)= P2(rs:rb);
P1Taken(rs:rb)=1;
P2Taken(rs:rb)=1;
i=rb+1;
z=rb+1;


