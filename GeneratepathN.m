function [totalpaths, numberofCollisions, distance, time ] =GeneratepathN(InitPos,Targets,WsX,WsY)

NumberOfRobots=size(InitPos,1);
Targets = NewtoOldTargets(Targets,NumberOfRobots,WsX,WsY);
Robots = InitPos;
Robotpaths=zeros (NumberOfRobots,2,1);
distance=0;
numberofCollisions=0;
CurrentTarget=zeros(NumberOfRobots,2,1);

sTargets = Targets;
Workstations =sTargets;
penalty=0;
index =[];
B=any(Targets);
B=any(B);
B=any(B);
B=any(B);
i=1;
while( B )
    [CurrentTarget ,index]= GetCurrenttarget(Targets);
    Robotspath(:,:,i)= Robots;
    [Robots ,  penaltytemp, distancetemp] =Obstacle_Avoidance(Robots,CurrentTarget);
    penalty = penalty + penaltytemp;
    distance = distance + distancetemp;

    for j=1:NumberOfRobots
        if (Robots(j,1)==CurrentTarget(j,1)  && Robots(j,2)== CurrentTarget(j,2) )
            Targets(index(j),:,j) = [0 0];
        end
    end


    %checking if all targets are visited
    B=any(Targets);
    B=any(B);
    B=any(B);
    B=any(B);

     i=i+1;
end
i=i+1;



time = i;
Robotspath(:,:,i-1)= Robots;
totalpaths = Robotspath;
numberofCollisions=penalty;


