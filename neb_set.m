function neb = neb_set(source,range,node)
node_x = [node.x]';
node_y = [node.y]';
node_z = [node.z]';
insph = ((node_x-source.x).^2 + (node_y-source.y).^2 + (node_z-source.z).^2) <= range^2;
neb = find(insph == 1);
neb = [node(neb).id];
neb(find(neb == source.id)) = [];
end
