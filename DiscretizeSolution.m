function finalsol=DiscretizeSolution(x,NumberOfRobots,NumberOfWorkstations)

% x=[0.2 , 7, 0.8,2, 0.3,0.05];
% NumberOfRobots=3;
% NumberOfWorkstations=4;
total = NumberOfWorkstations+NumberOfRobots-1;
sol = zeros(1,total);
finalsol=zeros(1,total);
for i=1:NumberOfRobots-1
sol(i) = -1;
end

for i=NumberOfRobots:total
sol(i) = i-NumberOfRobots+1;
end

[~,rank]=sort(x,'descend');

for i=1:length(sol)
finalsol(i) =sol(rank(i));
end