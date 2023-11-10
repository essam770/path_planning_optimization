% clc;
clear;
% Initialize Map parameters:
Mapsize = 100;

% Initialize Start and Goal points:
% InitalPosition = [1 1; 20 20; 35 35; 50 50; 70 70; 80 80];  %(X-Y)initial coordinates of Robots
% Workstations=[1 4; 6 3; 5 9;9 1; 1 10; 1 7;70 70; 20 20 ; 20 70; 27 77;25 77;69 69;43 6; 55 85; 76 20];
InitalPosition = [1 1;10 10; 20 20; 35 35; 50 50; 70 70; 80 80;85 85; 90 90; 100 100];  %(X-Y)initial coordinates of Robots
Workstations=[1 4; 6 3; 5 9;9 1; 1 10; 1 7;70 70; 20 20 ; 20 70; 27 77]; %(X-Y)coordinates of Workstations

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
lp1=0.7; %learning parameters for exploration and exploitation
lp2=0.5; %learning parameters for exploration and exploitation
Num_iterations = 100;  %maximum number of iterations to be used.
Num_buffalos=20;
BestCostArray=[];
timeArray=[];
n=7;

%%setup population
BuffaloSolution= zeros(Num_buffalos,NumberOfRobots+NumberOfWorkstations-1);
mkSolution= zeros(Num_buffalos,NumberOfRobots+NumberOfWorkstations-1);
BuffaloBestSolutions=zeros(Num_buffalos,NumberOfRobots+NumberOfWorkstations-1);
BuffaloPath=zeros(Num_buffalos,NumberOfRobots,2,NumberOfWorkstations*Mapsize);
TotalBestPath =zeros(NumberOfRobots,2,NumberOfWorkstations*Mapsize);
BuffaloCost = zeros(Num_buffalos,1);
BuffaloBestCost = zeros(Num_buffalos,1);
Figures.Main_fig = figure;  %create new figure

% initialize first solution
for iter=1:Num_buffalos
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
    BuffaloSolution(iter,:)=CurrertSol;
    BuffaloBestSolutions(iter,:)=CurrertSol;
    BuffaloCost(iter)=CurrentCost;
    BuffaloBestCost(iter)=CurrentCost;
    BuffaloPath(iter,:,:,1:size(CurrentPath,3)) =CurrentPath;
end
[InitialMinimumCostValue,GlobalbestLocation]=min(BuffaloCost);

for itteration=1:Num_iterations
    [MinimumCostValue,GlobalbestLocation]=min(BuffaloCost);

    for i=1:Num_buffalos

        %% find mk
        currentBuffalo=BuffaloSolution(i,:);
        previousmk=mkSolution(i,:);
        personalbestbuffalo=BuffaloBestSolutions(i,:);
        globalbestbuffalo=BuffaloSolution(GlobalbestLocation,:);
        r1=rand;
        r2=rand;
        mk = previousmk+lp2*r1*(personalbestbuffalo-currentBuffalo)+lp1*r2*(globalbestbuffalo-currentBuffalo);
        mkSolution(i,:)=mk;
        %update current buffalo position
        CurrertSol = (currentBuffalo+mk);
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
        BuffaloSolution(i,:)=CurrertSol;
        BuffaloCost(i)=CurrentCost;
        BuffaloPath(i,:,:,1:size(CurrentPath,3)) =CurrentPath;

        %%check if new is personal best
        if (CurrentCost<BuffaloBestCost(i))
            BuffaloBestSolutions(i,:)=CurrertSol;
            BuffaloBestCost(i)=CurrentCost;
        end

        %         [MinimumCostValue,GlobalbestLocation]=min(SwarmBestCost);
        if (CurrentCost<MinimumCostValue)

            TotalBestPath = CurrentPath;
        end
    end
    %%Visualization
    [MinimumCostValue,GlobalbestLocation]=min(BuffaloCost);
    BestCostArray(itteration) = min(BuffaloBestCost);

    %ABO if the best buffalo is not improving generate a new solution
    if(length(BestCostArray)>n)
        flag=1;
        for i=length(BestCostArray)-n:length(BestCostArray)-1
            if(BestCostArray(i)==BestCostArray(i+1))
            else
                flag=0;
            end
        end
    end
    if(flag==1)
        for iter=1:Num_buffalos
            if(iter==1)
                [~,totalGlobalbestLocation]=min(BuffaloBestCost);
                CurrertSol=BuffaloBestSolutions(totalGlobalbestLocation,:);
            else
                Targets=assigntarget(NumberOfRobots,NumberOfWorkstations);
                CurrertSol = Targets;  %save initial solution as the current solution
            end
            [CurrentPath ,CurrentCollisions ,CurrentDistance ,CurrentTime]=GeneratepathN(InitalPosition,CurrertSol,WorkstationsX,WorkstationsY);
            [maxValue, ~] = max(CurrentPath, [], "all", "linear");
            TotalBestPath=CurrentPath;
            %% if solution exceeds map length generate another solution
            while(maxValue>Mapsize)
                [CurrentPath ,CurrentCollisions ,CurrentDistance ,CurrentTime]=GeneratepathN(InitalPosition,CurrertSol,WorkstationsX,WorkstationsY);
                [maxValue, ~] = max(tempPath, [], "all", "linear");
            end
            CurrentCost = CurrentDistance+CurrentCollisions+CurrentTime;
            BuffaloSolution(iter,:)=CurrertSol;
            BuffaloBestSolutions(iter,:)=CurrertSol;
            BuffaloCost(iter)=CurrentCost;
            BuffaloBestCost(iter)=CurrentCost;
            BuffaloPath(iter,:,:,1:size(CurrentPath,3)) =CurrentPath;
        end
    end
%         %%plot Best
%         pathtoplot = squeeze(TotalBestPath);
%         subplot(3,3,1);
%         plot(0);
%         hold on;
%         PlotPaths(pathtoplot,Workstations)
%         hold off;
%     
%     
%         subplot(3,3,4);
%         plot(0);
%         hold on;
%         for iter=1:Num_buffalos
%             pathtoplot = squeeze(BuffaloPath(iter,:,:,:));
%             PlotPaths(pathtoplot,Workstations)
%         end
%         hold off
%     
%     
        subplot(3,3,2);
        plot(BestCostArray)
        grid on; %initialize the grid to be on
        axis square; %make the axes look like square
        xlim([0, Num_iterations]); %define limits in X-direction
        ylim([0, InitialMinimumCostValue]); %define limits in Y-direction
        xlabel('Generation'); ylabel('Cost');
        title('Current Cost');  %give title to figure
    
%     
%       timeArray(itteration)=Num_iterations-itteration;
%         subplot(3,3,3);
%         plot(timeArray)
%     
%     
%         xlim([0, Num_iterations]); %define limits in X-direction
%         ylim([0, Num_iterations]); %define limits in Y-direction
%         grid on; %initialize the grid to be on
%         axis square; %make the axes look like square
%         xlabel('time'); ylabel('Generation');
%         title('Time');  %give title to figure
%     
         drawnow;


end

        %%plot Best
        pathtoplot = squeeze(TotalBestPath);
        subplot(3,3,1);
        plot(0);
        hold on;
        PlotPaths(pathtoplot,Workstations)
        hold off;
    
    
        subplot(3,3,4);
        plot(0);
        hold on;
        for iter=1:Num_buffalos
            pathtoplot = squeeze(BuffaloPath(iter,:,:,:));
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

BestCostArray(length(BestCostArray));


