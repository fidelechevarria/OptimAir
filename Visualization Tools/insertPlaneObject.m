
function insertPlaneObject(position,attitude)

    % Import PATCH-compatible face-vertex structure
    load('ZivkoEdgeMesh.mat')

    [N_Objects,~] = size(position);
    [N_vertices,~] = size(referenceGeometry.vertices);
    
    geometry = cell(N_Objects,1);
    
    for i = 1:N_Objects
       DCM = angle2dcm(deg2rad(attitude(i,1)),deg2rad(attitude(i,2)),deg2rad(attitude(i,3)),'ZYX');
       for j = 1:N_vertices  
           geometry{i}.vertices(j,:) = DCM * referenceGeometry.vertices(j,:)';
       end
       geometry{i}.vertices(:,1) = geometry{i}.vertices(:,1) + position(i,1);
       geometry{i}.vertices(:,2) = geometry{i}.vertices(:,2) + position(i,2);
       geometry{i}.vertices(:,3) = geometry{i}.vertices(:,3) + position(i,3);
       geometry{i}.faces = referenceGeometry.faces;
    end

    for i = 1:N_Objects

        patch(geometry{i},'FaceColor', [0.53 0.62 0.58],...
                          'EdgeColor', 'none',...
                          'FaceLighting', 'gouraud',...
                          'AmbientStrength', 0.15);
                  
    end

    % Add a camera light, and tone down the specular highlighting
    camlight('headlight');
    material('dull');

end


