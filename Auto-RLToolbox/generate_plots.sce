

numStepsPerEpisode=1000;
numCols= 6;
numEpisodes= 20000;
evaluationStride= 200;
numRunsPerExperiment= 3;
numExperiments=6;
filename="";


for i=0:numExperiments-1
    fileName= "exp-" + string(i*numRunsPerExperiment+1) + "\test-" + string(numEpisodes) + ".txt";

    testData=zeros(numStepsPerEpisode,1);
    testData= read(fileName,numStepsPerEpisode,numCols);
    
    clf();
    plot (testData(:,1),testData(:,2),testData(:,1),testData(:,3));
    hLegend=legend(["$w(t)$","$x(t)$"]);
    figure_entity = gcf();
    axes_entity = figure_entity.children;
    axes_entity.font_style=xlfont('Verdana'); 
    hLegend.font_style=xlfont('Verdana');
    xlabel('t','fontsize',3);
    ylabel('x','fontsize',3);
    xs2eps(figure_entity, 'evaluation-exp' + string(i) + '.eps','landscape');
end










    avg= zeros(numEpisodes/evaluationStride+1,numExperiments);
for experiment=0:1:numExperiments-1

    episodeindex= zeros(numEpisodes/evaluationStride+1,1);
    for run=1:1:numRunsPerExperiment

     for episode=0:evaluationStride:numEpisodes
         fileName= "exp-" + string(experiment*numRunsPerExperiment+run) + "\test-" + string(episode)+ ".txt";
         filedata= read(fileName,numStepsPerEpisode,numCols);
         error=abs(filedata(:,3)-filedata(:,2))
         absAvgError= sum(error)/numStepsPerEpisode;
         arrayindex= 1+episode/evaluationStride;
         episodeindex(arrayindex)=episode;
         avg(arrayindex,1+experiment)=avg(arrayindex,1+experiment)+ (absAvgError/numRunsPerExperiment);
     end
   end
end

clf();
plot (episodeindex,avg(:,1),episodeindex,avg(:,2),episodeindex,avg(:,3),episodeindex,avg(:,4),episodeindex,avg(:,5),episodeindex,avg(:,6));
hLegend=legend(["$R_1(s,0.005)$","$R_1(s,0.05)$","$R_1(s,0.5)$","$R_2(s,0.005)$","$R_2(s,0.05)$","$R_2(s,0.5)$",]);
figure_entity = gcf();
axes_entity = figure_entity.children;
axes_entity.font_style=xlfont('Verdana'); 
hLegend.font_style=xlfont('Verdana');
xlabel('Episodes','fontsize',3);
ylabel('Average offset error','fontsize',3);
xs2eps(figure_entity, 'controller-greedy-evaluation.eps','landscape');
//clf();
//plot (episodeindex,avg(:,1),episodeindex,avg(:,2),episodeindex,avg(:,3),episodeindex,avg(:,4));
////hLegend=legend(["R_1(s,0.005)","R_1(s,0.01)","R_1(s,0.015)","R_1(s,0.02)","R_2(s,0.005)","R_2(s,0.01)","R_2(s,0.015)","R_2(s,0.02)","R_3(s)","R_4(s))"]);
//hLegend=legend(["R_1(s,0.005)","R_1(s,0.01)","R_2(s,0.005)","R_2(s,0.01)","R_3(s)","R_4(s))"]);
//figure_entity = gcf();
//axes_entity = figure_entity.children;
//axes_entity.font_style=xlfont('Verdana'); 
//hLegend.font_style=xlfont('Verdana');
//xlabel('Episodes','fontsize',3);
//ylabel('Average offset error','fontsize',3);
//xs2eps(figure_entity, 'controller-greedy-evaluation-reward-2.eps','landscape');
//
