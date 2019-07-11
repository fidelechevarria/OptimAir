
function insertPlaneObject(position,attitude,scale)

    % Import PATCH-compatible face-vertex structure
    load('ZivkoEdgeMesh.mat')

    [N_Objects,~] = size(position);
    [N_vertices,~] = size(referenceGeometry.vertices);
    
    geometry = cell(N_Objects,1);
    
    % First scale, then rotate, then apply offset
    for i = 1:N_Objects
       geometry{i}.vertices(:,1) = referenceGeometry.vertices(:,1).*scale;
       geometry{i}.vertices(:,2) = referenceGeometry.vertices(:,2).*scale;
       geometry{i}.vertices(:,3) = referenceGeometry.vertices(:,3).*scale;
       DCM = angle2dcm(attitude(i,1),-attitude(i,2),-attitude(i,3),'ZYX');
       for j = 1:N_vertices  
           geometry{i}.vertices(j,:) = inv(DCM) * geometry{i}.vertices(j,:)';
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

    camlight('headlight');
    material('shiny');

end


