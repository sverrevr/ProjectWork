%close all;

figure;
problem = dlmread('p04');

num_depo = problem(1,3);
problem = problem(num_depo+2:end, 1:5);

solution = dlmread('s04.res');
fitness = solution(1,1);
solution = solution(2:end,:);
[r w] = size(solution); 


ax = gca;
hold on;

prev_depo_id = 0;
colorOffset = 0;

for i=1:r
    
    if not (prev_depo_id == solution(i,1))
        prev_depo_id = solution(i,1);
        %colorOffset = i-1;
    end
    
    depo_x = problem(end - num_depo + solution(i,1) ,2);
    depo_y = problem(end - num_depo + solution(i,1) ,3);
    scatter(depo_x,depo_y,'b');
    prev_x = depo_x;
    prev_y = depo_y;
    
    j = 6;
    while not(solution(i,j)==0)
        curr_x = problem(solution(i,j) ,2);
        curr_y = problem(solution(i,j) ,3);
        ax.ColorOrderIndex = mod(i - colorOffset,7)+1;
        plot([prev_x,curr_x],[prev_y, curr_y]);
        ax.ColorOrderIndex = mod(i - colorOffset,7)+1;
        scatter(curr_x,curr_y,'+');
        prev_x = curr_x;
        prev_y = curr_y;
        
        j = j+1;
    end
    ax.ColorOrderIndex = mod(i - colorOffset,7)+1;
    plot([prev_x,depo_x],[prev_y, depo_y])
    
end

str = sprintf('Fitness: %f', fitness);
title(str);

hold off;