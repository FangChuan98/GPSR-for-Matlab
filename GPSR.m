function [next_hop routing_flag min_dist2des] = GPSR(sour_id,des_id,route,min_dist2des,ini_void_node)
 global node;
 if (isempty(route))
     cur_hop = sour_id;
     pre_hop = 0;
 else
     if (size(route) == 1)
         pre_hop = sour_id;
         cur_hop = route(end);
     else
         pre_hop = route(end-1);
         cur_hop = route(end);
     end
 end
 
 neb_set = node(cur_hop).nebSet;
 rng_set = node(cur_hop).rngSet;
%--------------destination is one hop away--
     if(ismember(des_id,neb_set) || ismember(des_id,rng_set) )
         
         next_hop = des_id;
         routing_flag = 0;
%------------------------------------------
     else
            for cnt = 1:length(neb_set)
                next_hop_list(cnt).id = neb_set(cnt);
                next_hop_list(cnt).dis = cal_dis2(neb_set(cnt),des_id);
            end
            [min_dis, min_index] = min([next_hop_list.dis]);
%------------------greedy routing----------
            if(min_dis < min_dist2des)
                next_hop = next_hop_list(min_index).id;
                min_dist2des = cal_dis2(next_hop,des_id);
                routing_flag = 0;

%------------------right hand rule---------
            else
                clear next_hop_list;
                routing_flag = 1;
%                 if (length(rng_set) <= 1 )
%                     next_hop  = pre_hop;
%                 else
%                     rng_set(find(rng_set == pre_hop)) = [];
                %-----first
                if (isempty(ini_void_node))
                    ini_void_node = cur_hop;
                    vec1 = node(des_id).x - node(cur_hop).x + (node(des_id).y - node(cur_hop).y)*i;
                    ang1 = angle(vec1)*180/pi;
                else   
                %----------
                        line([node(ini_void_node).x,node(des_id).x],[node(ini_void_node).y,node(des_id).y],'Color','yellow','LineStyle','--','LineWidth',1);
                        vec1 = node(pre_hop).x - node(cur_hop).x + (node(pre_hop).y - node(cur_hop).y)*i;
%                         vec1 = node(des_id).x - node(cur_hop).x + (node(des_id).y - node(cur_hop).y)*i;
                        ang1 = angle(vec1)*180/pi;
                end
                for cnt = 1:length(rng_set)
                    vec2 = node(rng_set(cnt)).x - node(cur_hop).x + (node(rng_set(cnt)).y - node(cur_hop).y)*i;
%                     vec2 = node(rng_set(cnt)).x - node(ini_void_node).x + (node(rng_set(cnt)).y - node(ini_void_node).y)*i;
                    ang2 = angle(vec2)*180/pi;

                        next_hop_list(cnt).id = rng_set(cnt);
                        next_hop_list(cnt).ang = ang2-ang1;
                end
                
%                       if ( ismember(pre_hop,[next_hop_list.id]))
%                             next_hop_list(find([next_hop_list.id] == pre_hop)) = [];
%                       end
                list_id = [next_hop_list.id];
                discard = [];
                for cnt = 1:length(list_id)
                        if_cross = lineSegmentIntersect([node(ini_void_node).x, node(ini_void_node).y,node(des_id).x, node(des_id).y],...
                                [node(cur_hop).x, node(cur_hop).y,node(list_id(cnt)).x,node(list_id(cnt)).y]);
                            if(if_cross.intAdjacencyMatrix)
                        discard =[discard cnt]; 
                    end
                end
                next_hop_list(discard) = [];
                
                if ( isempty(next_hop_list) )
                    disp('no valide node');
                    return;
                else
                list = [next_hop_list.ang];
                hop_list = find(list > 0 );
                if (~isempty(hop_list))
                    [~,index] = min(list(hop_list));
                    next_hop = next_hop_list(hop_list(index)).id;
                else
                    [~,index] = min(list);
                    next_hop = next_hop_list(index).id;
                end
%                 else
% 
%                 if ( length(rng_set) <= 1 )
%                     next_hop = pre_hop;
%                 else
%                     rng_set(find(rng_set == pre_hop)) = [];
%                     for cnt = 1:length(rng_set)
%                         next_hop_list(cnt).id = rng_set(cnt);
%                         next_hop_list(cnt).dis = point2line( node(ini_void_node).x, node(ini_void_node).y,node(des_id).x,node(des_id).y,...
%                             node(rng_set(cnt)).x,node(rng_set(cnt)).y );
%                     end
%                 list_id = [next_hop_list.id];
%                 discard = [];
%                 for cnt = 1:length(list_id)
%                     if (    iscro([node(ini_void_node).x, node(ini_void_node).y],[node(des_id).x,node(des_id).y],...
%                                     [node(cur_hop).x, node(cur_hop).y], [node(list_id(cnt)).x,node(list_id(cnt)).y])  )
%                         discard =[discard cnt]; 
%                     end
%                 end
%                 next_hop_list(discard) = [];
%                 if ( isempty(next_hop_list) )
%                     next_hop = pre_hop;
%                 else
%                     list = [next_hop_list.dis ];
%                     [~,index] = min(list);
%                     next_hop = next_hop_list(index).id;
%                 end
%                     
%                 end
                end
%             end
            end
     end
     end
