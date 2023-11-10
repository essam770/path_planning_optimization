tic
% Initialize Map parameters:
clear;
Mapsize = 100;

% Initialize Start and Goal points:
InitalPosition = [1 1; 20 20; 35 35; 50 50; 70 70; 80 80];  %(X-Y)initial coordinates of Robots
Workstations=[1 4; 6 3; 5 9;9 1; 1 10; 1 7;70 70; 20 20 ; 20 70; 27 77;25 77;69 69;43 6; 55 85; 76 20];
WorkstationsX=[];
WorkstationsY=[];

for i=1:size(Workstations,1)
WorkstationsX(i)=Workstations(i,1);
WorkstationsY(i)=Workstations(i,2);
end



% Initialize the optimization problem parameters:
NumberOfRobots = size(InitalPosition,1);
MaximumWorkstationperRobot = 15;
NumberOfWorkstations = length(WorkstationsX);

Num_Generations = 200;  %maximum number of generations to be tested
Pop_size = 230;  %the population size
Elite_ratio = 0.2;  %percentage of elitism
CrossOver_ratio = 0.6; %percentage of cross over processes
Mutation_ratio = 1 - Elite_ratio - CrossOver_ratio; %the rest of mutation ratio
BestCostArray=[];
timeArray=[];

%%setup population
PopulationSolution= zeros(Pop_size,NumberOfRobots+NumberOfWorkstations-1);
PopulationPath=zeros(Pop_size ,NumberOfRobots,2,NumberOfWorkstations*Mapsize);
PopulationCost = zeros(Pop_size,1);
Num_elite = round(Elite_ratio*Pop_size); 
Num_CO_children = round(CrossOver_ratio*Pop_size); 
Num_Mutation = round(Mutation_ratio*Pop_size); 

Figures.Main_fig = figure;  %create new figure

% initialize first solution
for iter=1:Pop_size
Targets=assigntarget(NumberOfRobots,NumberOfWorkstations);
CurrertSol = Targets;  %save initial solution as the current solution


[CurrentPath ,CurrentCollisions ,CurrentDistance ,CurrentTime]=GeneratepathN(InitalPosition,CurrertSol,WorkstationsX,WorkstationsY);
[maxValue, ~] = max(CurrentPath, [], "all", "linear");
%% if solution exceeds map length generate another solution
 while(maxValue>Mapsize)
[CurrentPath ,CurrentCollisions ,CurrentDistance ,CurrentTime]=GeneratepathN(InitalPosition,CurrertSol,WorkstationsX,WorkstationsY);
 [maxValue, ~] = max(tempPath, [], "all", "linear");
 end



CurrentCost = CurrentDistance+CurrentCollisions+CurrentTime;
PopulationSolution(iter,:)=CurrertSol;

PopulationCost(iter)=CurrentCost;
PopulationPath(iter,:,:,1:size(CurrentPath,3)) =CurrentPath;
end

[InitialMinimumCostValue,BestChromosomeLocation]=min(PopulationCost);


% pathtoplot = squeeze(PopulationPath(iter,:,:,:));
%     subplot(3,3,1);







for itteration=1:Num_Generations

%% Generate New Solution
PopulationSolution = GenerateNewPopulation(PopulationSolution,PopulationCost,Num_elite,Num_CO_children,MaximumWorkstationperRobot);
 PopulationPath=zeros(Pop_size ,NumberOfRobots,2,NumberOfWorkstations*Mapsize);
%% Update Path Based on New Solution
for iter=1:Pop_size
tempSol = PopulationSolution(iter,:);  %save initial solution as the current solution
[tempPath ,tempCollisions ,tempDistance ,tempTime]=GeneratepathN(InitalPosition,tempSol,WorkstationsX,WorkstationsY);
 [maxValue, ~] = max(tempPath, [], "all", "linear");

%% if solution exceeds map length generate another solution
 while(maxValue>Mapsize)
[tempPath ,tempCollisions ,tempDistance ,tempTime]=GeneratepathN(InitalPosition,tempSol,WorkstationsX,WorkstationsY);
 [maxValue, ~] = max(tempPath, [], "all", "linear");
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Calculate Cost of New Population 
tempCost = tempDistance+tempCollisions+tempTime;
PopulationCost(iter)=tempCost;
PopulationPath(iter,:,:,1:size(tempPath,3)) =tempPath;
end



%%% Update cost and time arrays to plot  and find best chromosome
[MinimumCostValue,BestChromosomeLocation]=min(PopulationCost);
 timeArray(itteration)=Num_Generations-itteration;
 CostArray(itteration)=MinimumCostValue;






%%Visualization

%plot Best Chromosome
pathtoplot = squeeze(PopulationPath(BestChromosomeLocation,:,:,:));
subplot(3,3,1);
plot(0);
hold on;
 PlotPaths(pathtoplot,Workstations)
 hold off;
 
% 
%  subplot(3,3,4);
%  plot(0);
% hold on;
% for iter=1:Pop_size
% pathtoplot = squeeze(PopulationPath(iter,:,:,:));
%  PlotPaths(pathtoplot,Workstations)
% end
% hold off
% 

 subplot(3,3,2);
 plot(CostArray)
    grid on; %initialize the grid to be on
    axis square; %make the axes look like square
   xlim([0, Num_Generations]); %define limits in X-direction
    ylim([0, InitialMinimumCostValue]); %define limits in Y-direction
    xlabel('Generation'); ylabel('Cost');
    title('Current Cost');  %give title to figure



%  subplot(3,3,3);
%   plot(timeArray)
% 
% 
%     xlim([0, Num_Generations]); %define limits in X-direction
%     ylim([0, Num_Generations]); %define limits in Y-direction
%     grid on; %initialize the grid to be on
%     axis square; %make the axes look like square
%     xlabel('Generation'); ylabel('Generation');
%     title('Time');  %give title to figure
% 
 drawnow;




end
toc
min(CostArray)
