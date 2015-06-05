ExpID='E2';
%'EST','PDST','KPIECE','FRRT',
Algorithms={'BKPIECE','LBKPIECE','RRTConnect','BFRRT','FRRTConnect'};
n=size(Algorithms,2);

DataNames=cell(1,n);
clear Data
clear Data_S
Data=cell(1,n);
Data_S=cell(1,n);
for i=1:n
    DataNames{i}=strcat(ExpID,'_',char(Algorithms{i}));
    Data{i}=eval(char(DataNames(i)));
    for j=1:50
        if Data{i}(j,1)==1
            Data_S{i}(size(Data_S{i},1)+1,:)=Data{i}(j,:);
        end
    end
    clear min
    clear min_index
    clear max
    clear max_index
    [min,min_index]=min(Data_S{i}(:,2));
    [max,max_index]=max(Data_S{i}(:,2));
    Data_S{i}([min_index,max_index],:) = [];
end

pd=[];
pg=[];
pp=[];
for i=1:n
pd=[pd;Data_S{i}(:,2)];
pg=[pg;(i-1)*ones(size(Data_S{i},1),1)];
pp=[pp i];
end
boxplot(pd,pg,'positions',pp);
xtix = Algorithms;
xtixloc = pp;
set(gca,'XTickMode','auto','XTickLabel',xtix,'XTick',xtixloc,'FontSize',15);
set(findobj(gca,'type','line'),'linew',1.5)
ylabel('Planning Time (sec)');
title(strcat('Valkyrie Upperbody ',ExpID,' (17 DOF)'));