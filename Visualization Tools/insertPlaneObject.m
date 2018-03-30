
function insertPlaneObject(figure,position)

    % Import PATCH-compatible face-vertex structure
    load('ZivkoEdgeMesh.mat')

    [N_Objects,~] = size(position);

    geometry = cell(N_Objects,1);
    for i = 1:N_Objects
       geometry{i}.vertices(:,1) = referenceGeometry.vertices(:,1) + position(i,1);
       geometry{i}.vertices(:,2) = referenceGeometry.vertices(:,2) + position(i,2);
       geometry{i}.vertices(:,3) = referenceGeometry.vertices(:,3) + position(i,3);
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


