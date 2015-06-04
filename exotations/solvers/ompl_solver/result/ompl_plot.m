% boxplot([freerrtresult(:,2),freefrrtresult(:,2),simplerrtresult(:,2),simplefrrtresult(:,2)], 'positions',[1 2 4 5])
% 
% xtix = {'FreeSpace RRT','FreeSpace FRRT','SimpleObs RRT','SimpleObs FRRT'}; 
% xtixloc = [1 2 4 5];
% set(gca,'XTickMode','auto','XTickLabel',xtix,'XTick',xtixloc);
% 
% figure
E2_EST_S=[];
E2_KPIECE_S=[];
E2_PDST_S=[];
E2_FRRT_S=[];
for i=1:50
   if E2_EST(i,1)==1
      E2_EST_S(size(E2_EST_S,1)+1,:)=E2_EST(i,:); 
   end
   if E2_KPIECE(i,1)==1
      E2_KPIECE_S(size(E2_KPIECE_S,1)+1,:)=E2_KPIECE(i,:); 
   end
   if E2_PDST(i,1)==1
      E2_PDST_S(size(E2_PDST_S,1)+1,:)=E2_PDST(i,:); 
   end
   if E2_FRRT(i,1)==1
      E2_FRRT_S(size(E2_FRRT_S,1)+1,:)=E2_FRRT(i,:); 
   end
end
clear min
clear min_index
clear max
clear max_index
[min,min_index]=min(E2_EST_S(:,2));
[max,max_index]=max(E2_EST_S(:,2));
E2_EST_S([min_index,max_index],:) = [];
clear min
clear min_index
clear max
clear max_index
[min,min_index]=min(E2_KPIECE_S(:,2));
[max,max_index]=max(E2_KPIECE_S(:,2));
E2_KPIECE_S([min_index,max_index],:) = [];
clear min
clear min_index
clear max
clear max_index
[min,min_index]=min(E2_PDST_S(:,2));
[max,max_index]=max(E2_PDST_S(:,2));
E2_PDST_S([min_index,max_index],:) = [];
clear min
clear min_index
clear max
clear max_index
[min,min_index]=min(E2_FRRT_S(:,2));
[max,max_index]=max(E2_FRRT_S(:,2));
E2_FRRT_S([min_index,max_index],:) = [];
clear min
clear min_index
clear max
clear max_index
boxplot([E2_EST_S(:,2);E2_KPIECE_S(:,2);E2_PDST_S(:,2);E2_FRRT_S(:,2)], [zeros(size(E2_EST_S,1),1);ones(size(E2_KPIECE_S,1),1);2*ones(size(E2_PDST_S,1),1);3*ones(size(E2_FRRT_S,1),1)],'positions',[1 2 3 4])
xtix = {'EST', 'KPIECE','PDST','FRRT'}; 
xtixloc = [1 2 3 4];
set(gca,'XTickMode','auto','XTickLabel',xtix,'XTick',xtixloc);

ylabel('Planning Time (sec)');
title('Valkyrie Upperbody E1 (17 DOF)');