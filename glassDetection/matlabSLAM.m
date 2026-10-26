clear
clc
% addpath('/home/alicia/Documentos/Lab/Codigo/DeteccionCristal')

%% 1: CARGAR DATOS DEL BAG Y RELACIONARLOS ENTRE SÍ
bag = rosbag('2022-03-25-14-22-16.bag'); %Cambiar por el nombre del bag correspondiente

%Leemos odometría estimada:
selPose = select(bag,'Topic','/robot/robotnik_base_control/odom');
msgPose = readMessages(selPose,'DataFormat','struct');
%Leemos láser:
selLaser = select(bag,'Topic','/robot/front_laser/scan');
msgLaser = readMessages(selLaser,'DataFormat','struct');
%Leemos tf:
selTf = select(bag,'Topic','/tf');
msgTf = readMessages(selTf,'DataFormat','struct');

% Se guardan los tf relacionados con robot_odom
msgTfOdom={};
for i=1:length(msgTf)
    if length(msgTf{i}.Transforms)==1 && strcmp(msgTf{i}.Transforms.ChildFrameId,'robot_odom')
        msgTfOdom=[msgTfOdom;msgTf{i}.Transforms];
    end
end

% Nos quedamos con el mismo número de msg para láser, odom y TfOdom
[~,idx]=min([length(msgLaser) length(msgPose) length(msgTfOdom)]);

if idx==1 %Láser tiene menos mensajes
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgLaser,msgPose);
    msgPose=msgPose(indexes);
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgLaser,msgTfOdom);
    msgTfOdom=msgTfOdom(indexes);
elseif idx==2 %Odometría tiene menos mensajes
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgPose,msgLaser);
    msgLaser=msgLaser(indexes);
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgPose,msgTfOdom);
    msgTfOdom=msgTfOdom(indexes);
elseif idx==3 %Tf tiene menos mensajes
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgTfOdom,msgPose);
    msgPose=msgPose(indexes);
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgTfOdom,msgLaser);
    msgLaser=msgLaser(indexes);
end

load('mapaNuevo.mat')

%% 2: ESTIMAR POSES CON SLAM

%load('offlineSlamData.mat');
clc
maxLidarRange = 30;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 90; 
slamAlg.LoopClosureSearchRadius = 15;
refAngles = linspace(2.1511,-2.1511,987);
downAngles = downsample(refAngles,5);
for i=150:10:length(msgLaser)
    i
    refRanges=msgLaser{i}.Ranges';
    downRanges=double(downsample(refRanges,5)); 
    scan = lidarScan(downRanges,downAngles);

    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan);
end

figure
show(slamAlg);
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});

[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map2 = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

figure; 
show(map2);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map Built Using Lidar SLAM');

%% 3: GUARDAR ZONAS DE CRISTAL
refAngles = linspace(2.1511,-2.1511,987);
glass=[];
loopIdx=0;
for i=150:10:length(msgLaser)
    loopIdx=loopIdx+1;

    refRanges = msgLaser{i}.Ranges';
    refInt= msgLaser{i}.Intensities';

    glassLoc=refInt>3000;
    if nnz(glassLoc)~=0 %CRISTAL DETECTADO
        [~,locs]=findpeaks(refInt,'MinPeakHeight',3000); %Location of intensity peaks
        refRangesReduced = reducepoly([refRanges;1:length(refRanges)]',0.0007); %Col1: valor int., col2: index
        for j=1:length(locs)
            leftIdx=find(refRangesReduced(:,2)<locs(j),1,'last'); %rightIdx es left+1
            removeLeft = find((refRangesReduced(:,2)<refRangesReduced(leftIdx,2)).*(abs(refRangesReduced(:,1)-refRangesReduced(leftIdx,1))<0.75),1,'last');
            removeRight = find((refRangesReduced(:,2)>refRangesReduced(leftIdx+1,2)).*(abs(refRangesReduced(:,1)-refRangesReduced(leftIdx+1,1))<0.75),1);
            if ~isempty(removeLeft) && ~isempty(removeRight) && (leftIdx-removeLeft)>1 && (leftIdx-removeLeft)<6 && (removeRight-leftIdx-1)>1 && (removeRight-leftIdx-1)<6
                refRanges(refRangesReduced(removeLeft,2):refRangesReduced(leftIdx,2))=Inf;
                refRanges(refRangesReduced(leftIdx+1,2):refRangesReduced(removeRight,2))=Inf;
            end
        end

        %---------LÁSER---------%
        %Desplazamiento a pose estimada
        relPose=lidarScan(refRanges,refAngles);
        relPose = transformScan(relPose,optimizedPoses(loopIdx,:));
        glass=[glass;relPose.Cartesian(glassLoc,:)];
    end

end

% Ploteamos los resultados
show(map2)
hold on
plot(glass(:,1),glass(:,2),'r.')

%Se agrupan las zonas de cristal por clusters
glass2=glass;
T=clusterdata(glass2,'Cutoff',0.4,'Criterion','distance');
gscatter(glass2(:,1),glass2(:,2),T) %Plotea grupos de puntos
maxT=max(T); %# de clusters

%Cada grupo de puntos se simplifica con su cerco
glassSegments=[]; %Coordenadas de los boundaries que limitan las zonas de cristal
for i=1:maxT
    if sum(T==i)>10 %El grupo tiene más de diez elementos
        indices=find(T==i); %Indices de los puntos del grupo
        k = boundary(double(glass2(indices,1)),double(glass2(indices,2))); %Cerco del grupo de puntos
        P_reduced2 = reducepoly([glass2(indices(k),1) glass2(indices(k),2)],0);
        line(P_reduced2(:,1),P_reduced2(:,2),'Color','g');
         for j=1:length(P_reduced2)-1
            glassSegments=[glassSegments;P_reduced2(j,1) P_reduced2(j,2) P_reduced2(j+1,1) P_reduced2(j+1,2)];
         end
    end
end
hold off
%% 4: COMPROBAR QUÉ SCANS PASAN A TRAVÉS DEL CRISTAL
loopIdx=0;
modScans={};
for i=150:10:length(msgLaser)
    loopIdx=loopIdx+1;
    refRanges=scans{loopIdx}.Ranges';
    relPose = transformScan(scans{loopIdx},optimizedPoses(loopIdx,:));
    robotPose=optimizedPoses(loopIdx,1:2);
    
    %Comprobamos si hay intersección
    newCart=relPose.Cartesian;
    P_reduced = reducepoly(newCart);
    P_reduced2=P_reduced(2:length(P_reduced),:);
    P_reduced(length(P_reduced),:)=[];
    P_reduced(:,3:4)=P_reduced2;
    intersecciones=lineSegmentIntersect(P_reduced,glassSegments);
    
    if ~isempty(find(intersecciones.intAdjacencyMatrix)) %Si no está vacío, hay intersección
        localRanges=refRanges;
        for j=1:length(scans{1}.Ranges) %Comprobar qué scans cortan
            segmentoLaser=[relPose.Cartesian(j,1) relPose.Cartesian(j,2) robotPose(1) robotPose(2)]; %X1 Y1 X2 Y2
            localIntersect=lineSegmentIntersect(segmentoLaser,glassSegments);
            localIdx=find(localIntersect.intAdjacencyMatrix);
            if ~isempty(localIdx) %Si corta, modificar coordenada
                distToPose=pdist2(robotPose,[localIntersect.intMatrixX(localIdx)' localIntersect.intMatrixY(localIdx)']);
                [~,minIdx]=min(distToPose);
                localRanges(j)=distToPose(minIdx);
            end
            
        end
        localRanges(localRanges>50)=NaN;
    else
        localRanges=refRanges;
        localRanges(localRanges>50)=NaN;
    end
    newScan=lidarScan(localRanges,downAngles);
    modScans{loopIdx}=newScan;
end

map3 = buildMap(modScans, optimizedPoses, mapResolution, maxLidarRange);

figure; 
show(map3);
title('Occupancy Grid Map Without Glass Surfaces');
