%% UAV_Swarm
%% By Hanze L
% ---------TODO----------
% field
% Jamming
% position in altitude
% remaining energy
% buffer
% speed
% -----------------------
%% initialization
%-------------param set------------------
clc;
% clear all;
close all;
area_l = 2000;
area_w = 2000;
r = 200;                    %R = range
% node_num = 500;
%----------------------------------------

%-----------node position----------------
load('node.mat');
node_num = length(node);
global node;

% for i=1:node_num
%     node(i).id = i;
%     node(i).x = area_l*rand(1,1);
%     node(i).y = area_w*rand(1,1);
%     node(i).z = randi([50 50],1,1);
%     node(i).speed = 50;
%     node(i).pwr = [];
%     node(i).buffer = [];
%     node(i).nebSet = [];
%     node(i).rngSet = [];
%     node(i).pv = 0;
% end

for i = 1:node_num
node(i).nebSet = neb_set(node(i),r,node);
end
[~,total_connection] = size([node.nebSet]);
mean_connection_num = total_connection/node_num;

sour_id = randi([1 node_num],1,1);
% sour_id = source_test(rnd_cnt);
% cnt = 10;
% 
% while(isempty(node(sour_id).nebSet) )
%     sour_id = randi([1 node_num],1,1);
%     cnt = cnt - 1;
%     if(cnt == 0)
%         error('unreasonable network, please reset! source isolated!');
%         break;
%     end
% end
% 
% clear cnt;
des_id = randi([1 node_num],1,1);
% des_id = sink_test(rnd_cnt);
% while(sour_id == des_id)
%   des_id = randi([1 node_num],1,1);  
% end
% cnt = 10;
% while(isempty(node(des_id).nebSet) )
%     des_id = randi([1 node_num],1,1);
%     cnt = cnt - 1;
%     if(cnt == 0)
%         error('unreasonable network, please reset!destination isolated!');
%         break;
%     end
% end
% clear cnt;

for i = 1:node_num
    node(i).rngSet = node(i).nebSet;
end

%----------figure---------
figure('Name','GPSR routing path');
for cnt = 1:node_num
    plot(node(cnt).x,node(cnt).y,'k*');
    id_text = num2str(node(cnt).id);
    text(node(cnt).x,node(cnt).y,id_text);
    hold on;
end
    plot(node(sour_id).x,node(sour_id).y,'ro');
    plot(node(des_id).x,node(des_id).y,'yo');
    hold on;
    clear cnt;
    clear id_text;
%------------------------

%% RNG Graph
for cnt = 1:node_num
    nebSet = node(cnt).nebSet;
    nebNum = length(nebSet);
    rngSet = nebSet;
    for cntx = 1: nebNum
        for cnty = 1: nebNum
            if (nebSet(cntx) == nebSet(cnty))
                continue;
            else
                if (cal_dis2(cnt,nebSet(cntx)) > max(cal_dis2(cnt,nebSet(cnty)),cal_dis2(nebSet(cntx),nebSet(cnty))))
                    rngSet(find(rngSet == nebSet(cntx))) = [];
                    break;
                end
            end
        end
    end
    node(cnt).rngSet = rngSet;
end
clear nebSet;
clear rngSet;
clear nebNum;
% figure,
% for i = 1:node_num
%     nebSet = node(i).nebSet;
%     size_nebSet = length(node(i).nebSet);
%         for cnt = 1:size_nebSet
%             line([node(i).x, node(nebSet(cnt)).x],[node(i).y, node(nebSet(cnt)).y]);
%             hold on;
%         end
% end

% figure,
% for cnt = 1:node_num
%     plot(node(cnt).x,node(cnt).y,'k*');
%     id_text = num2str(node(cnt).id);
%     text(node(cnt).x,node(cnt).y,id_text);
%     hold on;
% end
%     plot(node(sour_id).x,node(sour_id).y,'ro');
%     plot(node(des_id).x,node(des_id).y,'yo');
%     hold on;
%  clear id_text;
 for i = 1:node_num
    rngSet = node(i).rngSet;
    size_rngSet = length(node(i).rngSet);
        for cnt = 1:size_rngSet
            line([node(i).x, node(rngSet(cnt)).x],[node(i).y, node(rngSet(cnt)).y]);
            hold on;
        end
end
%% begin routing
% global flag_no_neb = 0;
%flag_no_closer = 0;
global flag_reach_des;
flag_reach_des = 0;
route = [];
void_node = [];    
ini_void_node = [];
min_dist2des = cal_dis2(sour_id,des_id);
hop_cnt = 0;
flag_cnt = 0;
action_flag_cnt = 0;
detour_size = 50;
while(~flag_reach_des)
    if (action_flag_cnt >= detour_size)
        disp('transmission failed')
        route = [];
%         error('transmission failed')
        break;
    end
    [next_hop,action_flag,min_dist2des] = GPSR(sour_id,des_id,route,min_dist2des,ini_void_node);
    if (action_flag)
       action_flag_cnt = action_flag_cnt + 1;
       if (isempty(route))
           void_node = [void_node sour_id];
       else
       void_node  = [void_node route(end)];
       end
       if (~flag_cnt)
           if ( isempty(route) )
               ini_void_node = sour_id;
           else
        ini_void_node = route(end);
           end
        flag_cnt = 1;
       end
    else
        ini_void_node = [];
        flag_cnt = 0;
    end
%     if (action_flag == 1 && ismember(next_hop,void_node))
%         error('transmission failed!');
%     end
    
    route = [route next_hop];
    hop_cnt = hop_cnt + 1;
    route_GPSR(hop_cnt).route = route(end);
    route_GPSR(hop_cnt).routing_flag = action_flag;
    
   %------figure-------------
   if (length(route)>1)
    line([node(route(end-1)).x,node(route(end)).x],[node(route(end-1)).y,node(route(end)).y],'Color','red','LineStyle','--','LineWidth',1);
    hold on;
   else
    line([node(sour_id).x,node(route(end)).x],[node(sour_id).y,node(route(end)).y],'Color','red','LineStyle','--','LineWidth',1);
    hold on;
   end
    %------------------------
    if(next_hop == des_id)
        flag_reach_des = 1;
    end

end
%%

figure('Name','Connection Graph'),
for cnt = 1:node_num
    plot(node(cnt).x,node(cnt).y,'k*');
    id_text = num2str(node(cnt).id);
    text(node(cnt).x,node(cnt).y,id_text);
    hold on;
end
    plot(node(sour_id).x,node(sour_id).y,'ro');
    plot(node(des_id).x,node(des_id).y,'yo');
    hold on;
    for i = 1:node_num
    nebSet = node(i).nebSet;
    size_nebSet = length(node(i).nebSet);
        for cnt = 1:size_nebSet
            line([node(i).x, node(nebSet(cnt)).x],[node(i).y, node(nebSet(cnt)).y]);
            hold on;
        end
    end

  %%
%   cnt = 0;
%   for  i = 1:1:node_num
%       if find(length(node(i).rngSet) > 2)
%           cnt = cnt + 1;
%       end
%   end
%   bone_ratio = cnt/node_num;
  %% bone structure
 
%    cnt = 0;
%   for  i = 1:1:node_num
%       if find(length(node(i).rngSet) > 2)
%           cnt = cnt + 1;
%           bone_node(cnt)= node(i);
%           bone_node(cnt).nebSet = [];
%           bone_node(cnt).rngSet = [];
%       end
%   end
%   
% for i = 1:length(bone_node)
%     bone_node(i).nebSet = neb_set(bone_node(i),r,bone_node);
% end
%   
%   figure('Name','Bone Structure')
% for cnt = 1:length(bone_node)
%     plot(bone_node(cnt).x,bone_node(cnt).y,'k*');
%     id_text = num2str(bone_node(cnt).id);
%     text(bone_node(cnt).x,bone_node(cnt).y,id_text);
%     hold on;
% end
%     clear cnt;
%     clear id_text;
%     
% %     for i = 1:length(bone_node)
% %     lock = [];
% %     nebSet = bone_node(i).nebSet;
% %     size_nebSet = length(bone_node(i).nebSet);
% %     for n = 1:size_nebSet
% %         temp = find([bone_node.id] == nebSet(n));
% %         lock = [lock temp];
% %     end
% %         for cnt = 1:size_nebSet
% %             line([bone_node(i).x, bone_node(lock(cnt)).x],[bone_node(i).y, bone_node(lock(cnt)).y]);
% %             hold on;
% %         end
% %     end
% %     cal_field(bone_node,node)
%% 
%------
env_center = randi([0,2000],1,2);
env_radius = 500;
r = env_radius;
theta = 0:pi/100:2*pi;
x = env_center(1) + r*cos(theta); 
y = env_center(2) + r*sin(theta);
% plot(nodes(node_id).x,nodes(node_id).y);
% plot(x,y);
in = inpolygon([node.x],[node.y],x,y);
% plot(nodes_x(in),nodes_y(in),'r+') % points inside
adj = find(in);
for i = 1:length(adj)
    node(adj(i)).pv = 1;
end
%%
% clearvars -except route;