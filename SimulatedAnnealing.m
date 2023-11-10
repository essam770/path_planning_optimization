%cleaning up the working environment before starting
 clear , close all
tic
%Initialize Map parameters:
Mapsize = 100;

%Initialize Start and Goal points:
% InitalPosition = [1 1;10 10; 20 20; 35 35; 50 50; 70 70; 80 80;85 85; 90 90; 100 100];  %(X-Y)initial coordinates of Robots
% Workstations=[1 4; 6 3; 5 9;9 1; 1 10; 1 7;70 70; 20 20 ; 20 70; 27 77]; %(X-Y)coordinates of Workstations

InitalPosition = [1 1; 20 20; 35 35; 50 50; 70 70; 80 80];  %(X-Y)initial coordinates of Robots
Workstations=[1 4; 6 3; 5 9;9 1; 1 10; 1 7;70 70; 20 20 ; 20 70; 27 77;25 77;69 69;43 6; 55 85; 76 20];


NumberOfRobots = size(InitalPosition,1);
MaximumWorkstationperRobot = 10;


%Initialize the optimization problem parameters:

Beta = 1;  % linear cooling parameter (beta)
T_init = 250;  %initial temperature used in SA

T_final = 0;  %define the final temperature to be reached
Max_num_iter =(T_init-T_final)/Beta;  %maximum number of iterations
T_current = T_init;  %save initial temperature as current temperature

alpha = 0.99;  %for geometric cooling schedule
Linear_Cooling = 1; %Flag to select linear cooling, if (0) then geometric cooling
Chk_Probability = 0;  %Flag to decide whether to check probability or to bypass it
CostArray=[];
BestCostArray=[];
timeArray=[];

%Visualizing the map:
Figures.Main_fig = figure;  %create new figure
% subplot(1,3,1); plot(Map.Start(1),Map.Start(2),'bs','MarkerSize',15,'MarkerFaceColor','b'); %plot the start point




% plot(Map.Goal(1),Map.Goal(2),'ro','MarkerSize',10,'LineWidth',3);  %plot the goal point
xlim([0, Mapsize]); %define limits in X-direction
ylim([0, Mapsize]); %define limits in Y-direction
grid on; %initialize the grid to be on
axis square; %make the axes look like square
xlabel('X-direction (m)'); ylabel('Y-direction (m)');
title('The Map used');  %give title to figure
% legend('Start point','Goal point');

%Initialize a random solution:
Targets = randomM(NumberOfRobots,Workstations);


%set old and best solutions for first iteration
CurrertSol = Targets;  %save initial solution as the current solution
[CurrentPath ,CurrentCollisions ,CurrentDistance ,CurrentTime]=Generatepath(InitalPosition,Targets);
CurrentCost = CurrentDistance+CurrentCollisions+CurrentTime;
Init_cost=CurrentCost;
OldSol =  Targets;
OldPath =CurrentPath ;
OldCollisions=CurrentCollisions;
OldDistance =CurrentDistance;
OldTime=CurrentTime;
OldCost=CurrentCost;



BestSol =  Targets;
BestPath =CurrentPath ;
BestCollisions=CurrentCollisions;
BestDistance =CurrentDistance;
BestTime=CurrentTime;
BestCost=CurrentCost;

clear k;  %clear used temporary variables
for  i=0:Max_num_iter

    %%setOldSolution
    OldSol =  Targets;
    OldPath =CurrentPath ;
    OldCollisions=CurrentCollisions;
    OldDistance =CurrentDistance;
    OldTime=CurrentTime;
    OldCost=CurrentCost;

    %%generate new Solution and path that comes with solution
    Targets=swap(Targets);
    TempSol  = Targets;
    [TempPath ,TempCollisions ,TempDistance ,TempTime]=Generatepath(InitalPosition,Targets);
    TempCost = TempDistance+TempCollisions+TempTime;

    %%check that new solution is feesable 
   [maxValue, maxInd] = max(TempPath, [], "all", "linear");

    if (size(Targets,1)>MaximumWorkstationperRobot || maxValue>Mapsize )
 %% if not feesable generate new solution
    Targets= randomM(NumberOfRobots,Workstations);
        TempSol  = Targets;
    [TempPath ,TempCollisions ,TempDistance ,TempTime]=Generatepath(InitalPosition,Targets);
    TempCost = TempDistance+TempCollisions+TempTime;
    end

%% check if better solution if not check if acceptable solution

    if (TempCost<OldCost)
        BetterSolution=1;
    else
        BetterSolution=0;
    end
    cost_diff = CurrentCost-OldCost;
    P = exp((-1*cost_diff)/T_current);
    rand_num = rand;

    if (BetterSolution || P>rand_num)

        CurrentSol = TempSol;
        CurrentPath=TempPath;
        CurrentCollisions=TempCollisions;
        CurrentDistance=TempDistance;
        CurrentTime=TempTime;
        CurrentCost=TempCost;


    end








    %%if current is better than global best Update Best

    if (CurrentCost<BestCost)

        BestSol =  Targets;
        BestPath =CurrentPath ;
        BestCollisions=CurrentCollisions;
        BestDistance =CurrentDistance;
        BestTime=CurrentTime;
        BestCost=CurrentCost;
    end


    %%Update time
    T_current=T_current-Beta;


    timeArray(i+1)=T_current;
    CostArray(i+1)=CurrentCost;
    BestCostArray(i+1)=BestCost;

    %%Visualisation
%     subplot(2,3,1);
% plot(0)
% hold on
%     PlotPaths(CurrentPath,Workstations);
%     xlim([0, Mapsize]); %define limits in X-direction
%     ylim([0, Mapsize]); %define limits in Y-direction
%     grid on; %initialize the grid to be on
%     axis square; %make the axes look like square
%     xlabel('X-direction (m)'); ylabel('Y-direction (m)');
%     title('The CurrentMap used');  %give title to figure
% hold off
%     subplot(2,3,4);
%     plot(0)
% hold on
%     PlotPaths(BestPath,Workstations);
%     xlim([0, Mapsize]); %define limits in X-direction
%     ylim([0, Mapsize]); %define limits in Y-direction
%     grid on; %initialize the grid to be on
%     axis square; %make the axes look like square
%     xlabel('X-direction (m)'); ylabel('Y-direction (m)');
%     title('The BestMap used');  %give title to figure
%     hold off
% 
% 
% 
%     subplot(2,3,2);
%     plot(CostArray)
%     xlim([0, T_init]); %define limits in X-direction
%     ylim([0, 1000]); %define limits in Y-direction
%     grid on; %initialize the grid to be on
%     axis square; %make the axes look like square
%     xlabel('Temp'); ylabel('Cost');
%     title('Current Cost');  %give title to figure
% 
% 
%         subplot(2,3,5);
%     plot(BestCostArray)
%     xlim([0, T_init]); %define limits in X-direction
%     ylim([0, Init_cost]); %define limits in Y-direction
%     grid on; %initialize the grid to be on
%     axis square; %make the axes look like square
%     xlabel('Temp'); ylabel('Cost');
%     title('Best Cost');  %give title to figure
% 
% 
% 
%     subplot(2,3,3);
% 
% 
% 
%     plot(timeArray)
% 
% 
%     xlim([0, T_init]); %define limits in X-direction
%     ylim([0, T_init]); %define limits in Y-direction
%     grid on; %initialize the grid to be on
%     axis square; %make the axes look like square
%     xlabel('time'); ylabel('temp');
%     title('Temprature');  %give title to figure
%     drawnow;

end


disp("best cost   "+BestCost)
toc














