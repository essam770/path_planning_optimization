function  [NewPopulation] = GenerateNewPopulation(Population, PopulationCost,Num_elite,Num_CO_children,MaximumWorkstationperRobot)

%This function creates the new generation for the next iteration

%initialize the variables containing the arranged current population and
%the next generation
ArrPopulation = zeros(size(Population));
NewPopulation= zeros(size(Population));
%Sort previous population according to the cost function of each
[~,Rank] = sort(PopulationCost,'ascend');
for i= 1:length(Rank)
    ArrPopulation(i,:)=Population(Rank(i),:);
end
%Assigning the elite chromosomes to the new population
for i= 1: Num_elite
    NewPopulation(i,:)=ArrPopulation(i,:);
end
%Cross Selection of parents by the rank where each parent is directly after
%the other parent in rank
for i = Num_elite+1 :2: (Num_CO_children + Num_elite)
    generate_random = randsample(size(ArrPopulation,2), 2);
	c1 = min(generate_random);
	c2 = max(generate_random);
   
    %Depending on the number of elements to be crossed over the parents are
    %determined by the arranged list of population chromosomes and the
    %followed chromosome and in case of being at the end of the list the
    %chromosome is crossed with the previous chromosome

    if i< (Num_CO_children + Num_elite)
        [DavisResult1 ,DavisResult2]= Crossover_Ordered_Operator(ArrPopulation(i,:),ArrPopulation(i+1,:),c1,c2);
        package1 = find(DavisResult1==-1);
        package2 = find(DavisResult2==-1);

        %A fesability check is needed to be done to see if the constraint
        %of max number of elemnts in a given robot is valid or not and act
        %accordingly by preforming the crossover again with diffrent davis
        %technique boundries
        for j=1:(max(length(package1),length(package2)) -1)
            if length(package1)<j && (package1(j+1)-package1(j)) > MaximumWorkstationperRobot
                generate_random = randsample(size(ArrPopulation,2), 2);
	            c1 = min(generate_random);
	            c2 = max(generate_random);
                [DavisResult1 ,DavisResult2]= Crossover_Ordered_Operator(ArrPopulation(i,:),ArrPopulation(i+1,:),c1,c2);
                package1 = find(DavisResult1==-1);
                package2 = find(DavisResult2==-1);
                j=1;
            elseif length(package2)<j && (package2(j+1)-package2(j)) > MaximumWorkstationperRobot
                generate_random = randsample(size(ArrPopulation,2), 2);
	            c1 = min(generate_random);
	            c2 = max(generate_random);
                [DavisResult1 ,DavisResult2]= Crossover_Ordered_Operator(ArrPopulation(i,:),ArrPopulation(i+1,:),c1,c2);
                package1 = find(DavisResult1==-1);
                package2 = find(DavisResult2==-1);
                j=1;
            end
        end
        NewPopulation(i,:)=DavisResult1;
        NewPopulation(i+1,:)=DavisResult2;
    else
        [DavisResult ,~]= Crossover_Ordered_Operator(ArrPopulation(i,:),ArrPopulation(i-1,:),c1,c2);
        NewPopulation(i,:)=DavisResult;
    end
end
%Preform mutation on the rest of the chromosomes
for i = (Num_elite + Num_CO_children +1) : size(Population,1)
    Mutated = swapN(ArrPopulation(i,:));
    package = find(Mutated == -1);
    for j= 1:(length(package) -1)
        if (package(j+1)-package(j))> MaximumWorkstationperRobot
            Mutated = swapN(ArrPopulation(i,:));
            package = find(Mutated == -1);
            j=1;
        end
    end
    NewPopulation(i,:)=Mutated;
end

end 