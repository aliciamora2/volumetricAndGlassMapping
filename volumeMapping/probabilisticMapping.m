%% Mapa con los scans 3D del Ouster PROBABILISTICO
clear
clc
load('Special_Issue_Localization.mat') % Carga datos
clearvars -except dataset sensor_3Dposes_absolute

%%
step=5;

%  ---------- CREACION DE MAPA LIBRE ----------
escaneos = cell(0);
roi = [-inf inf -inf inf 0.3 0.5];  % Region mas alta que el robot
for i=1:step:380
    i
    indices = findPointsInROI(dataset.clouds{i},roi); % Elimina los puntos que se salen del tamaño del recorte
    nubecita = select(dataset.clouds{i},indices);
    nubePuntos = nubecita.Location;
    nubePuntos = nubePuntos(any(nubePuntos,2),:); %Quito filas de ceros
    [theta,rho,zh]=cart2pol(nubePuntos(:,1),nubePuntos(:,2),nubePuntos(:,3)); % Cambia nube de puntos a coord. polares
    nubePuntos(rho<0.5,:)=[]; % Quita hombros del robot
    escaneos{end+1} = lidarScan(double(nubePuntos(:,1:2)));
end
mapaLibre = buildMap(escaneos,[sensor_3Dposes_absolute(1:step:380,1:2) sensor_3Dposes_absolute(1:step:380,6)],50,50);
inflate(mapaLibre,0.05);
mat = occupancyMatrix(mapaLibre);
mat = imbinarize(mat,0.9);
mat = imclose(mat,strel('disk',3));
mat = bwareaopen(mat,100); %Matriz con los elementos estructurales del entorno (paredes)
clearvars escaneos

%%
% Variables
num_lines = 1500; % Define líneas para generar láser 2D
laser_angles = deg2rad(linspace(-180, 180, num_lines));
laser_lengths = ones(1, num_lines) * 20 * mapaLibre.Resolution; % Multiplico por la resolución para que esté en px
roi = [-inf inf -inf inf -1.2 0.3]; % Region de altura del robot

mapaGlobal = zeros(mapaLibre.DataSize);

for i=1:step:380
    i
    
    % ---------- PROCESAMIENTO Y PROYECCIÓN DE LA NUBE 3D ----------
    indices = findPointsInROI(dataset.clouds{i},roi); % Elimina los puntos que se salen del tamaño del recorte
    nubecita = select(dataset.clouds{i},indices);
    nubePuntos = nubecita.Location;
    nubePuntos = nubePuntos(any(nubePuntos,2),:); %Quito filas de ceros
    [theta,rho,zh]=cart2pol(nubePuntos(:,1),nubePuntos(:,2),nubePuntos(:,3)); % Cambia nube de puntos a coord. polares
    nubePuntos(rho<0.5,:)=[]; % Quita hombros del robot

    % ---------- GENERACIÓN DEL INVERSE SENSOR MODEL PARA CADA BEAM ----------
    nubePuntos = round(nubePuntos,2);
    nubePuntos = unique(nubePuntos,'rows'); % Simplifico nº de puntos para aligerar proceso
    nubePuntos(nubePuntos(:,1)>20,:)=[]; nubePuntos(nubePuntos(:,1)<-20,:)=[]; nubePuntos(nubePuntos(:,2)>20,:)=[]; nubePuntos(nubePuntos(:,2)<-20,:)=[]; % Elimino puntos más alejados del rango establecido del sensor
    puntosRot = spatialRotation(nubePuntos,sensor_3Dposes_absolute(i,:)); % Transformo los puntos en funcion de la pose del robot
    puntosPx = [round((puntosRot(:,1)-mapaLibre.XWorldLimits(1))*mapaLibre.Resolution) round((mapaLibre.YWorldLimits(2)-puntosRot(:,2))*mapaLibre.Resolution)];
    puntosPx = unique(puntosPx,'rows'); % De metros a px
    localSensor = zeros(mapaLibre.DataSize); %Guardo los pixels en una matriz global
    for j=1:length(puntosPx) % Marca celdas ocupadas
        localSensor(puntosPx(j,2),puntosPx(j,1)) = 1;
    end
    localSensor=imdilate(localSensor,strel('disk',3));

    probSensor = zeros(mapaLibre.DataSize)+0.5;
    for j = 1:num_lines
        origen = [round((sensor_3Dposes_absolute(i,1)-mapaLibre.XWorldLimits(1))*mapaLibre.Resolution) round((mapaLibre.YWorldLimits(2)-sensor_3Dposes_absolute(i,2))*mapaLibre.Resolution)];
        x1 = origen(1); y1 = origen(2); % Punto del que parte el beam
        laser_x_endpoints = round(laser_lengths .* cos(laser_angles)) + x1;
        laser_y_endpoints = round(laser_lengths .* sin(laser_angles)) + y1;
        x2 = laser_x_endpoints(j); y2 = laser_y_endpoints(j); % Punto max. del beam
        [profX, profY, profVal] = improfile(localSensor,[x1 x2],[y1 y2]);
        if any(profVal) %Si hay algún obstáculo
            lastOccupied = find(profVal==1, 1,'last'); % Elimino los puntos sobre los que no tengo info. (más allá del último punto ocupado)
            [profX2, profY2, profVal2] = improfile(mat,[x1 x2],[y1 y2]); % Elimino los puntos que sobrepasan paredes (reflejos)
            if any(profVal2) %MODIFICACION
                firstOccupied = find(profVal2==1, 1);
                lastOccupied=min(firstOccupied,lastOccupied);
            end
            profX = profX(1:lastOccupied); profY = profY(1:lastOccupied); profVal = profVal(1:lastOccupied);
            occCellIdx = find(profVal); % Índices del profile que están ocupados
            empCellIdx = find(~profVal); % Índices del profile que están libres
            for oc=1:length(occCellIdx)
                probSensor(round(profY(occCellIdx(oc))),round(profX(occCellIdx(oc))))=0.9;
            end
            for emp=1:length(empCellIdx)
                probSensor(round(profY(empCellIdx(emp))),round(profX(empCellIdx(emp))))=0.1;
            end
        end
    end

    % ---------- MAP UPDATING MODEL ----------
    mapaGlobal = log(probSensor./(1-probSensor)) + mapaGlobal; % Acumular info usando Bayes

end

minVal=min(min(mapaGlobal));
maxVal=max(max(mapaGlobal));

mapaGlobalNorm=zeros(size(mapaGlobal));
mapaGlobalNorm(mapaGlobal<=0)=(mapaGlobal(mapaGlobal<=0)-minVal)/(-minVal*2);
mapaGlobalNorm(mapaGlobal>0)=(mapaGlobal(mapaGlobal>0))/(maxVal*2)+0.5;

mapaGlobalNorm(mapaGlobalNorm>0.52)=1;
mapaGlobalNorm(mapaGlobalNorm<0.48)=0;
mapaGlobalNorm(imbinarize((mapaGlobalNorm>=0.48) .* (mapaGlobalNorm<=0.52)))=0.5;

mapaFinal = occupancyMap(mapaGlobalNorm);
setOccupancy(mapaFinal,[mapaLibre.XWorldLimits(1) mapaLibre.YWorldLimits(1)],mapaGlobalNorm)
inflate(mapaFinal,0.05);
show(mapaFinal)

function transformed = spatialRotation(point,p)

cAlpha = cos(p(4)); sAlpha = sin(p(4));
cBeta = cos(p(5)); sBeta = sin(p(5));
cGamma = cos(p(6)); sGamma = sin(p(6));

transformed = ([[cBeta*cGamma, -cBeta*sGamma,sBeta];...
               [cAlpha*sGamma + cGamma*sAlpha*sBeta, cAlpha*cGamma - sAlpha*sBeta*sGamma, -cBeta*sAlpha];...
               [sAlpha*sGamma - cAlpha*cGamma*sBeta, cGamma*sAlpha + cAlpha*sBeta*sGamma,  cAlpha*cBeta]]...
                *point'+[p(1);p(2);p(3)])';
end