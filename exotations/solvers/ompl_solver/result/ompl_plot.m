% boxplot([freerrtresult(:,2),freefrrtresult(:,2),simplerrtresult(:,2),simplefrrtresult(:,2)], 'positions',[1 2 4 5])
% 
% xtix = {'FreeSpace RRT','FreeSpace FRRT','SimpleObs RRT','SimpleObs FRRT'}; 
% xtixloc = [1 2 4 5];
% set(gca,'XTickMode','auto','XTickLabel',xtix,'XTick',xtixloc);
% 
% figure
boxplot([freerrtresult(:,3),freefrrtresult(:,3),simplerrtresult(:,3),simplefrrtresult(:,3)], 'positions',[1 2 4 5])

xtix = {'FreeSpace RRT','FreeSpace FRRT','SimpleObs RRT','SimpleObs FRRT'}; 
xtixloc = [1 2 4 5];
set(gca,'XTickMode','auto','XTickLabel',xtix,'XTick',xtixloc);

% boxplot([freerrtresult(:,2)./freerrtresult(:,3),freefrrtresult(:,2)./freefrrtresult(:,3),simplerrtresult(:,2)./simplerrtresult(:,3),simplefrrtresult(:,2)./simplefrrtresult(:,3)], 'positions',[1 2 4 5])
% 
% xtix = {'FreeSpace RRT','FreeSpace FRRT','SimpleObs RRT','SimpleObs FRRT'}; 
% xtixloc = [1 2 4 5];
% set(gca,'XTickMode','auto','XTickLabel',xtix,'XTick',xtixloc);