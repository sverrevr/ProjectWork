clear;
%%
simulationSpeed = 1;
skipToEnd = 1;
plotOnlyUncertanty = 0;
comparasingPlot = 0;

droneRaw = csvread('droneOptimal.txt');
icebergRaw = csvread('icebergOptimal.txt');
figure(1);
pbaspect([1 1 1])

communicationRadius = droneRaw(1,1);
droneRaw = droneRaw(2:end,:);

deliminatingLinesTimestep_drone = find(droneRaw(:,1) == -2);
deliminatingLinesTimestep_icebergs = find(icebergRaw(:,1) == -2);
lineStartDrone = 1;
lineStartIceberg = 1;

maxUncertanty_optimal = NaN(length(deliminatingLinesTimestep_drone),1);
meanUncertanty_optimal = NaN(length(deliminatingLinesTimestep_drone),1);


CM = lines();

if(skipToEnd == 0)
    for t = 1:simulationSpeed:length(deliminatingLinesTimestep_drone)
        if(t > 1)
            pause(0.001);
            clf();
            figure(1);
                pbaspect([1 1 1])
        end
        subplot(2,3,[1,2,4,5]);
        axis([0 1000 0 1000]);
        hold on;

        %todo read the actuall ship pos
        scatter(500,500,'*');
        viscircles([500, 500],communicationRadius, 'LineWidth',0.03, 'Color', CM(1,:));

         %% icebergsRaw
        icebergs = icebergRaw(lineStartIceberg:deliminatingLinesTimestep_icebergs(t)-1,:);
        lineStartIceberg = deliminatingLinesTimestep_icebergs(t)+1;
        icebergPossitions = icebergs(:,2:3);
        icebergSpeeds = icebergs(:,4:5);

        scatter(icebergPossitions(:,1),icebergPossitions(:,2), 25, CM(1,:));
        quiver(icebergPossitions(:,1),icebergPossitions(:,2),icebergSpeeds(:,1),icebergSpeeds(:,2),0.2,'color',CM(1,:));
        text(icebergPossitions(:,1),icebergPossitions(:,2),num2str(round(icebergs(:,6))));
        maxUncertanty_optimal(t) = max(icebergs(:,6));
        meanUncertanty_optimal(t) = mean(icebergs(:,6));

        %% Drones
        drones = droneRaw(lineStartDrone:deliminatingLinesTimestep_drone(t)-1,:);
        lineStartDrone = deliminatingLinesTimestep_drone(t)+1;
        deliminatingLines = find(drones(:,1) == -1);


        %color index 1 is used for icebergsRaw
        colorIndex = 2;
        j= 1;
        for i = deliminatingLines'
           drone =  drones(j:i-1,:);

           dronePos = drone(1,:);
           plannedPath = drone(2:end,:);

           scatter(dronePos(1),dronePos(2),25 ,CM(colorIndex,:),'filled');
           viscircles(dronePos(1:2),communicationRadius, 'LineWidth',0.03,'Color', CM(colorIndex,:), 'EnhanceVisibility', false);
           plot(plannedPath(:,1),plannedPath(:,2), 'color',CM(colorIndex,:));
           scatter(plannedPath(:,1),plannedPath(:,2), 25 ,CM(colorIndex,:));

           j=i+1;
           colorIndex = colorIndex+1;
        end

        subplot(2,3,3);
        plot(maxUncertanty_optimal);
        title('Highest uncertanty');
        subplot(2,3,6);
        plot(meanUncertanty_optimal);
        title('Mean uncertanty');
        if(t==1)
            pause(1);
        end
    end
else
    for t = 1:simulationSpeed:length(deliminatingLinesTimestep_drone)
        %subplot(2,3,[1,2,4,5]);
        %axis([0 1000 0 1000]);
        %hold on;

        %todo read the actuall ship pos
        %scatter(500,500,'*');
        %viscircles([500, 500],communicationRadius, 'LineWidth',0.03, 'Color', CM(1,:));

         %% icebergsRaw
        icebergs = icebergRaw(lineStartIceberg:deliminatingLinesTimestep_icebergs(t)-1,:);
        lineStartIceberg = deliminatingLinesTimestep_icebergs(t)+1;
        icebergPossitions = icebergs(:,2:3);
        icebergSpeeds = icebergs(:,4:5);

        %scatter(icebergPossitions(:,1),icebergPossitions(:,2), 25, CM(1,:));
        %quiver(icebergPossitions(:,1),icebergPossitions(:,2),icebergSpeeds(:,1),icebergSpeeds(:,2),0.2,'color',CM(1,:));
        %text(icebergPossitions(:,1),icebergPossitions(:,2),num2str(round(icebergs(:,6))));
        maxUncertanty_optimal(t) = max(icebergs(:,6));
        meanUncertanty_optimal(t) = mean(icebergs(:,6));

        %% Drones
        drones = droneRaw(lineStartDrone:deliminatingLinesTimestep_drone(t)-1,:);
        lineStartDrone = deliminatingLinesTimestep_drone(t)+1;
        deliminatingLines = find(drones(:,1) == -1);


        %color index 1 is used for icebergsRaw
        %colorIndex = 2;
        %j= 1;
        %for i = deliminatingLines'
        %   drone =  drones(j:i-1,:);

%           dronePos = drone(1,:);
 %          plannedPath = drone(2:end,:);

           %scatter(dronePos(1),dronePos(2),25 ,CM(colorIndex,:),'filled');
           %viscircles(dronePos(1:2),communicationRadius, 'LineWidth',0.03,'Color', CM(colorIndex,:), 'EnhanceVisibility', false);
           %plot(plannedPath(:,1),plannedPath(:,2), 'color',CM(colorIndex,:));
           %scatter(plannedPath(:,1),plannedPath(:,2), 25 ,CM(colorIndex,:));

  %         j=i+1;
 %          colorIndex = colorIndex+1;
   %     end

        %subplot(2,3,3);
        %plot(maxUncertanty_optimal);
        %title('Highest uncertanty');
        %subplot(2,3,6);
        %plot(meanUncertanty_optimal);
        %title('Mean uncertanty');
        %if(t==1)
        %    pause(1);
        %end
    end
    
    if(plotOnlyUncertanty == 0)
        subplot(2,3,[1,2,4,5]);
        axis([0 1000 0 1000]);
        hold on;

        scatter(500,500,'*');
        viscircles([500, 500],communicationRadius, 'LineWidth',0.03, 'Color', CM(1,:));
        scatter(icebergPossitions(:,1),icebergPossitions(:,2), 25, CM(1,:));
        quiver(icebergPossitions(:,1),icebergPossitions(:,2),icebergSpeeds(:,1),icebergSpeeds(:,2),0.2,'color',CM(1,:));
        text(icebergPossitions(:,1),icebergPossitions(:,2),num2str(round(icebergs(:,6))));


        %color index 1 is used for icebergsRaw
        colorIndex = 2;
        j= 1;
        for i = deliminatingLines'
            drone =  drones(j:i-1,:);

            dronePos = drone(1,:);
            plannedPath = drone(2:end,:);

            scatter(dronePos(1),dronePos(2),25 ,CM(colorIndex,:),'filled');
            viscircles(dronePos(1:2),communicationRadius, 'LineWidth',0.03,'Color', CM(colorIndex,:), 'EnhanceVisibility', false);
            plot(plannedPath(:,1),plannedPath(:,2), 'color',CM(colorIndex,:));
            scatter(plannedPath(:,1),plannedPath(:,2), 25 ,CM(colorIndex,:));

            j=i+1;
            colorIndex = colorIndex+1;
        end    
    end
    
    if(comparasingPlot == 1)
       load('uncertantyOptimal.mat');
    end
    
    if(plotOnlyUncertanty == 0)
        subplot(2,3,3);
    else
        subplot(1,2,1);
    end
    plot(maxUncertanty_optimal);
    title('Highest uncertanty');
    if(comparasingPlot == 1)
       hold on;
       plot(maxUncertanty_optimal);
       legend('Decentralized','Optimal')
    end
    if(plotOnlyUncertanty == 0)
        subplot(2,3,6);
    else
        subplot(1,2,2);
    end
    plot(meanUncertanty_optimal);
    title('Mean uncertanty');
    if(comparasingPlot == 1)
       hold on;
       plot(meanUncertanty_optimal);
       legend('Decentralized','Optimal')
    end
    
    
end

save('uncertantyOptimal.mat', 'maxUncertanty_optimal', 'meanUncertanty_optimal');
