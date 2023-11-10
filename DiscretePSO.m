clc;
clear;

% Initialize Map parameters:
Mapsize = 100;

% Initialize Start and Goal points:
InitalPosition = [1 1; 20 20; 35 35; 50 50; 70 70; 80 80];  %(X-Y)initial coordinates of Robots
Workstations=[1 4; 6 3; 5 9;9 1; 1 10; 1 7;70 70; 20 20 ; 20 70; 27 77;25 77;69 69;43 6; 55 85; 76 20];
% InitalPosition = [1 1;10 10; 20 20; 35 35; 50 50; 70 70; 80 80;85 85; 90 90; 100 100];  %(X-Y)initial coordinates of Robots
% Workstations=[1 4; 6 3; 5 9;9 1; 1 10; 1 7;70 70; 20 20 ; 20 70; 27 77]; %(X-Y)coordinates of Workstations



WorkstationsX=[];
WorkstationsY=[];
for i=1:size(Workstations,1)
    WorkstationsX(i)=Workstations(i,1);
    WorkstationsY(i)=Workstations(i,2);
end



% Initialize the  problem parameters:
NumberOfRobots = size(InitalPosition,1);
MaximumWorkstationperRobot = 15;
NumberOfWorkstations = length(WorkstationsX);


%Initialize the optimization  parameters:
Num_iterations = 100;  %maximum number of iterations to be used.
Swarm_size = 80;  %number of particles in the swarm
w1 = 0.9;  %inertia weight factor
c1 = 0.9;  %acceleration coefficent due to personal best
c2 = 0.1;  %acceleration coefficent due to global best

BestCostArray=[];
timeArray=[];




%%setup population
SwarmSolution= zeros(Swarm_size,NumberOfRobots+NumberOfWorkstations-1);
SwarmBestSolutions=zeros(Swarm_size,NumberOfRobots+NumberOfWorkstations-1);
SwarmPath=zeros(Swarm_size ,NumberOfRobots,2,NumberOfWorkstations*Mapsize);
TotalBestPath =zeros(NumberOfRobots,2,NumberOfWorkstations*Mapsize);
SwarmCost = zeros(Swarm_size,1);
SwarmBestCost = zeros(Swarm_size,1);
SwarmVelocity=zeros(Swarm_size,NumberOfRobots+NumberOfWorkstations-1);
Figures.Main_fig = figure;  %create new figure

% initialize first solution
for iter=1:Swarm_size
    Targets=assigntarget(NumberOfRobots,NumberOfWorkstations);
    CurrertSol = Targets;  %save initial solution as the current solution
    

    [CurrentPath ,CurrentCollisions ,CurrentDistance ,CurrentTime]=GeneratepathN(InitalPosition,CurrertSol,WorkstationsX,WorkstationsY);
    [maxValue, ~] = max(CurrentPath, [], "all", "linear");
    TotalBestPath=CurrentPath;
    %% if solution exceeds map length generate another solution
    while(maxValue>Mapsize)
        [CurrentPath ,CurrentCollisions ,CurrentDistance ,CurrentTime]=GeneratepathN(InitalPosition,CurrertSol,WorkstationsX,WorkstationsY);
        [maxValue, ~] = max(tempPath, [], "all", "linear");
    end

    CurrentCost = CurrentDistance+CurrentCollisions+CurrentTime;
    SwarmSolution(iter,:)=CurrertSol;
    SwarmBestSolutions(iter,:)=CurrertSol;
    SwarmCost(iter)=CurrentCost;
    SwarmBestCost(iter)=CurrentCost;
    SwarmPath(iter,:,:,1:size(CurrentPath,3)) =CurrentPath;
end

[InitialMinimumCostValue,GlobalbestLocation]=min(SwarmCost);

for itteration=1:Num_iterations


    [MinimumCostValue,GlobalbestLocation]=min(SwarmCost);

    for i=1:Swarm_size

        %% find velocity
        currentbird=SwarmSolution(i,:);
        previousvelocity=SwarmVelocity(i,:);
        personalbestbird=SwarmBestSolutions(i,:);
        globalbestbird=SwarmSolution(GlobalbestLocation,:);
        r1=rand;
        r2=rand;
        velocity = w1*previousvelocity+c1*r1*(personalbestbird-currentbird)+c2*r2*(globalbestbird-currentbird);
        SwarmVelocity(i,:)=velocity;
        %update current bird position
        CurrertSol = currentbird+velocity;
        CurrertSol=DiscretizeSolution(CurrertSol,NumberOfRobots,NumberOfWorkstations);
       while( MaximumNumberOfWorskstationsCheck(CurrertSol,MaximumWorkstationperRobot))
        CurrertSol=assigntarget(NumberOfRobots,NumberOfWorkstations);
       end



        [CurrentPath ,CurrentCollisions ,CurrentDistance ,CurrentTime]=GeneratepathN(InitalPosition,CurrertSol,WorkstationsX,WorkstationsY);
        [maxValue, ~] = max(CurrentPath, [], "all", "linear");




        
        %% if solution exceeds map length generate another solution
        while(maxValue>Mapsize)
            [CurrentPath ,CurrentCollisions ,CurrentDistance ,CurrentTime]=GeneratepathN(InitalPosition,CurrertSol,WorkstationsX,WorkstationsY);
            [maxValue, ~] = max(tempPath, [], "all", "linear");
        end


        
        
        CurrentCost = CurrentDistance+CurrentCollisions+CurrentTime;
%         TotalBestPath = CurrentPath;
       



        %% update Solution
        SwarmSolution(i,:)=CurrertSol;

        SwarmCost(i)=CurrentCost;

        SwarmPath(i,:,:,1:size(CurrentPath,3)) =CurrentPath;




        %%check if new is personal best
        if (CurrentCost<SwarmBestCost(i))
            SwarmBestSolutions(i,:)=CurrertSol;
            SwarmBestCost(i)=CurrentCost;
        end

%         [MinimumCostValue,GlobalbestLocation]=min(SwarmBestCost);
        if (CurrentCost<MinimumCostValue)

            TotalBestPath = CurrentPath;
        end


    end

    %%Visualization
    [MinimumCostValue,GlobalbestLocation]=min(SwarmCost);
    BestCostArray(itteration) = min(SwarmBestCost);
    %%plot Best Chromosome
    pathtoplot = squeeze(TotalBestPath);
    subplot(3,3,1);
    plot(0);
    hold on;
    PlotPaths(pathtoplot,Workstations)
    hold off;


    subplot(3,3,4);
    plot(0);
    hold on;
    for iter=1:Swarm_size
        pathtoplot = squeeze(SwarmPath(iter,:,:,:));
        PlotPaths(pathtoplot,Workstations)
    end
    hold off


    subplot(3,3,2);
    plot(BestCostArray)
    grid on; %initialize the grid to be on
    axis square; %make the axes look like square
    xlim([0, Num_iterations]); %define limits in X-direction
    ylim([0, InitialMinimumCostValue]); %define limits in Y-direction
    xlabel('Generation'); ylabel('Cost');
    title('Current Cost');  %give title to figure


  timeArray(itteration)=Num_iterations-itteration;
    subplot(3,3,3);
    plot(timeArray)


    xlim([0, Num_iterations]); %define limits in X-direction
    ylim([0, Num_iterations]); %define limits in Y-direction
    grid on; %initialize the grid to be on
    axis square; %make the axes look like square
    xlabel('time'); ylabel('Generation');
    title('Time');  %give title to figure

     drawnow;




end







