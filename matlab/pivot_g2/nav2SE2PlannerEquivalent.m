function result = nav2SE2PlannerEquivalent(planningOccupancy,startPose,goalPose, ...
        map,config,comparison,plannerName,captureTrace)
%NAV2SE2PLANNEREQUIVALENT Hybrid-A*/State Lattice SE(2) dang MATLAB.
% Day la tai hien nghien cuu, khong phai plugin C++ Nav2 thuc thi.
plannerName=upper(char(plannerName));
isLattice=strcmp(plannerName,'SMAC_STATE_LATTICE');
if ~isLattice&&~strcmp(plannerName,'SMAC_HYBRID')
    error('plannerName phai la SMAC_HYBRID hoac SMAC_STATE_LATTICE.');
end
se2=comparison.se2;binCount=se2.angleBins;binWidth=2*pi/binCount;
[rowCount,columnCount]=size(planningOccupancy);stateCount=rowCount*columnCount*binCount;
if isLattice
    plugin='nav2_smac_planner::SmacPlannerLattice';
    primitiveSet=makeLatticePrimitives(se2,binWidth);
else
    plugin='nav2_smac_planner::SmacPlannerHybrid';
    primitiveSet=makeHybridPrimitives(se2);
end
trace=struct('currentCells',zeros(0,2),'newCells',{cell(0,1)},'planner',plannerName);
result=struct('name',plannerName,'plugin',plugin,'implementation','MATLAB_EQUIVALENT', ...
    'success',false,'path',zeros(0,2),'poses',zeros(0,3), ...
    'modes',{cell(0,1)},'radii',zeros(0,1),'trace',trace, ...
    'planningTime',nan,'expandedNodes',0,'pathCost',inf,'message','');
[startRow,startColumn,startValid]=worldToGrid(startPose(1:2),map);
[goalRow,goalColumn,goalValid]=worldToGrid(goalPose(1:2),map);
if ~startValid||~goalValid||planningOccupancy(startRow,startColumn)|| ...
        planningOccupancy(goalRow,goalColumn)
    result.message='Start/goal SE2 khong hop le.';return;
end
startBin=angleToBin(startPose(3));goalBin=angleToBin(goalPose(3));
startState=encode(startRow,startColumn,startBin);
gScore=inf(stateCount,1);gScore(startState)=0;
closed=false(stateCount,1);parent=zeros(stateCount,1,'uint32');
parentPrimitive=zeros(stateCount,1,'uint8');
poseX=nan(stateCount,1);poseY=nan(stateCount,1);poseTheta=nan(stateCount,1);
poseX(startState)=startPose(1);poseY(startState)=startPose(2);poseTheta(startState)=startPose(3);
heapStates=zeros(max(64,stateCount),1,'uint32');heapValues=inf(max(64,stateCount),1);heapSize=0;
pushState(uint32(startState),heuristic(startPose));
timer=tic;goalState=0;iterations=0;
while heapSize>0&&iterations<se2.maxIterations
    currentState=double(popState());
    if closed(currentState),continue;end
    closed(currentState)=true;iterations=iterations+1;result.expandedNodes=result.expandedNodes+1;
    [currentRow,currentColumn,currentBin]=decode(currentState); %#ok<ASGLU>
    currentPose=[poseX(currentState) poseY(currentState) poseTheta(currentState)];
    newly=zeros(0,2);
    if captureTrace,trace.currentCells(end+1,:)=[currentRow currentColumn];end %#ok<AGROW>
    binError=abs(currentBin-goalBin);binError=min(binError,binCount-binError);
    if norm(currentPose(1:2)-goalPose(1:2))<=se2.goalPositionTolerance && ...
            binError<=se2.goalHeadingToleranceBins
        goalState=currentState;
        if captureTrace,trace.newCells{end+1,1}=newly;end
        break;
    end
    for primitiveIndex=1:numel(primitiveSet)
        primitive=primitiveSet(primitiveIndex);
        samples=propagatePrimitive(currentPose,primitive,se2.primitiveSampleSpacing);
        if ~primitiveIsFree(samples),continue;end
        endpoint=samples(end,:);
        [nextRow,nextColumn,nextValid]=worldToGrid(endpoint(1:2),map);
        if ~nextValid,continue;end
        nextBin=angleToBin(endpoint(3));
        if nextRow==currentRow&&nextColumn==currentColumn&&nextBin==currentBin,continue;end
        nextState=encode(nextRow,nextColumn,nextBin);
        traversalCost=primitive.cost;
        if isfinite(comparison.smac2DCostWeight)
            traversalCost=traversalCost*(1+0.35*comparison.smac2DCostWeight* ...
                double(planningOccupancy(nextRow,nextColumn)));
        end
        tentative=gScore(currentState)+traversalCost;
        if tentative+eps<gScore(nextState)
            gScore(nextState)=tentative;parent(nextState)=uint32(currentState);
            parentPrimitive(nextState)=uint8(primitiveIndex);
            poseX(nextState)=endpoint(1);poseY(nextState)=endpoint(2);poseTheta(nextState)=endpoint(3);
            h=heuristic(endpoint);pushState(uint32(nextState),tentative+h+1e-6*h);
            if captureTrace,newly(end+1,:)=[nextRow nextColumn];end %#ok<AGROW>
        end
    end
    if captureTrace,trace.newCells{end+1,1}=unique(newly,'rows');end
end
result.planningTime=toc(timer);result.trace=trace;
if goalState==0
    result.message=sprintf('Khong tim duoc duong SE2 sau %d lan mo rong.',iterations);return;
end

stateChain=zeros(stateCount,1,'uint32');chainCount=1;stateChain(chainCount)=uint32(goalState);
state=goalState;
while state~=startState
    state=double(parent(state));
    if state==0,result.message='Loi truy vet SE2.';return;end
    chainCount=chainCount+1;stateChain(chainCount)=uint32(state);
end
stateChain=double(flipud(stateChain(1:chainCount)));
poses=startPose;modes={'STRAIGHT'};radii=inf;
for chainIndex=2:numel(stateChain)
    child=stateChain(chainIndex);parentState=stateChain(chainIndex-1);
    parentPose=[poseX(parentState) poseY(parentState) poseTheta(parentState)];
    primitive=primitiveSet(double(parentPrimitive(child)));
    dense=propagatePrimitive(parentPose,primitive,min(config.arcSampleSpacing,0.02));
    poses=[poses;dense(2:end,:)]; %#ok<AGROW>
    modes=[modes;repmat({primitive.mode},size(dense,1)-1,1)]; %#ok<AGROW>
    radii=[radii;repmat(primitive.radius,size(dense,1)-1,1)]; %#ok<AGROW>
end
% Noi toi dung goal cho robot vi sai: can huong, di thang, can huong cuoi.
% Day tuong duong analytic approach ket hop rotation shim cua he Nav2.
if norm(poses(end,1:2)-goalPose(1:2))>1e-9|| ...
        abs(wrapAngle(poses(end,3)-goalPose(3)))>1e-9
    startConnector=poses(end,:);
    connectorDistance=norm(goalPose(1:2)-startConnector(1:2));
    if connectorDistance>1e-9
        bearing=atan2(goalPose(2)-startConnector(2),goalPose(1)-startConnector(1));
        firstTurn=wrapAngle(bearing-startConnector(3));
        if abs(firstTurn)>1e-6
            turnCount=max(2,ceil(abs(firstTurn)/config.pivotAngleStep)+1);
            turnTheta=startConnector(3)+linspace(0,firstTurn,turnCount).';
            poses=[poses;[repmat(startConnector(1:2),turnCount-1,1),turnTheta(2:end)]];
            turnMode=ternary(firstTurn>0,'PIVOT_LEFT','PIVOT_RIGHT');
            modes=[modes;repmat({turnMode},turnCount-1,1)];
            radii=[radii;zeros(turnCount-1,1)];
        end
        lineCount=max(2,ceil(connectorDistance/0.02)+1);alpha=linspace(0,1,lineCount).';
        line=[startConnector(1)+alpha*(goalPose(1)-startConnector(1)), ...
            startConnector(2)+alpha*(goalPose(2)-startConnector(2)), ...
            repmat(bearing,lineCount,1)];
        poses=[poses;line(2:end,:)];modes=[modes;repmat({'STRAIGHT'},lineCount-1,1)];
        radii=[radii;inf(lineCount-1,1)];
    else
        bearing=startConnector(3);
    end
    finalTurn=wrapAngle(goalPose(3)-bearing);
    if abs(finalTurn)>1e-6
        turnCount=max(2,ceil(abs(finalTurn)/config.pivotAngleStep)+1);
        turnTheta=bearing+linspace(0,finalTurn,turnCount).';
        poses=[poses;[repmat(goalPose(1:2),turnCount-1,1),turnTheta(2:end)]];
        turnMode=ternary(finalTurn>0,'PIVOT_LEFT','PIVOT_RIGHT');
        modes=[modes;repmat({turnMode},turnCount-1,1)];
        radii=[radii;zeros(turnCount-1,1)];
    end
end
result.poses=poses;result.path=poses(:,1:2);result.modes=modes;result.radii=radii;
result.pathCost=gScore(goalState);result.success=true;result.message='OK';

    function stateId=encode(row,column,bin)
        stateId=sub2ind([rowCount columnCount binCount],row,column,bin);
    end
    function [row,column,bin]=decode(stateId)
        [row,column,bin]=ind2sub([rowCount columnCount binCount],stateId);
    end
    function bin=angleToBin(angle)
        bin=mod(round((wrapAngle(angle)+pi)/binWidth),binCount)+1;
    end
    function h=heuristic(pose)
        h=norm(pose(1:2)-goalPose(1:2))+0.05*abs(wrapAngle(pose(3)-goalPose(3)));
    end
    function tf=primitiveIsFree(samples)
        [sampleRows,sampleColumns,valid]=worldToGrid(samples(:,1:2),map);
        if ~all(valid),tf=false;return;end
        indices=sub2ind([rowCount columnCount],sampleRows,sampleColumns);
        tf=~any(planningOccupancy(indices));
    end
    function pushState(id,value)
        heapSize=heapSize+1;
        if heapSize>numel(heapStates),heapStates(end+stateCount,1)=0;heapValues(end+stateCount,1)=inf;end
        heapIndex=heapSize;
        while heapIndex>1
            heapParent=floor(heapIndex/2);if heapValues(heapParent)<=value,break;end
            heapStates(heapIndex)=heapStates(heapParent);heapValues(heapIndex)=heapValues(heapParent);heapIndex=heapParent;
        end
        heapStates(heapIndex)=id;heapValues(heapIndex)=value;
    end
    function id=popState()
        id=heapStates(1);lastState=heapStates(heapSize);lastValue=heapValues(heapSize);heapSize=heapSize-1;
        if heapSize==0,return;end
        heapIndex=1;
        while true
            leftChild=2*heapIndex;if leftChild>heapSize,break;end
            rightChild=leftChild+1;bestChild=leftChild;
            if rightChild<=heapSize&&heapValues(rightChild)<heapValues(leftChild),bestChild=rightChild;end
            if heapValues(bestChild)>=lastValue,break;end
            heapStates(heapIndex)=heapStates(bestChild);heapValues(heapIndex)=heapValues(bestChild);heapIndex=bestChild;
        end
        heapStates(heapIndex)=lastState;heapValues(heapIndex)=lastValue;
    end
end

function primitives=makeHybridPrimitives(se2)
R=se2.minimumTurningRadius;L=se2.primitiveLength;
primitives=[makeMove(0,L,'STRAIGHT',inf,1.0), ...
    makeMove(1/R,L,'ARC_LEFT',R,1.15),makeMove(-1/R,L,'ARC_RIGHT',R,1.15)];
end

function primitives=makeLatticePrimitives(se2,binWidth)
L=se2.primitiveLength;primitives=makeMove(0,L,'STRAIGHT',inf,1.0);
for R=se2.latticeRadii
    primitives(end+1)=makeMove(1/R,L,'ARC_LEFT',R,1.12); %#ok<AGROW>
    primitives(end+1)=makeMove(-1/R,L,'ARC_RIGHT',R,1.12); %#ok<AGROW>
end
pivotCost=0.30*binWidth;
primitives(end+1)=struct('kind','PIVOT','curvature',0,'length',0, ...
    'deltaTheta',binWidth,'mode','PIVOT_LEFT','radius',0,'cost',pivotCost);
primitives(end+1)=struct('kind','PIVOT','curvature',0,'length',0, ...
    'deltaTheta',-binWidth,'mode','PIVOT_RIGHT','radius',0,'cost',pivotCost);
end

function primitive=makeMove(curvature,lengthValue,mode,radius,costMultiplier)
primitive=struct('kind','MOVE','curvature',curvature,'length',lengthValue, ...
    'deltaTheta',0,'mode',mode,'radius',radius,'cost',lengthValue*costMultiplier);
end

function samples=propagatePrimitive(pose,primitive,spacing)
if strcmp(primitive.kind,'PIVOT')
    count=max(2,ceil(abs(primitive.deltaTheta)/(pi/90))+1);
    theta=pose(3)+linspace(0,primitive.deltaTheta,count).';
    samples=[repmat(pose(1:2),count,1),theta];return;
end
count=max(2,ceil(primitive.length/spacing)+1);s=linspace(0,primitive.length,count).';
k=primitive.curvature;theta=pose(3)+k*s;
if abs(k)<1e-12
    x=pose(1)+s*cos(pose(3));y=pose(2)+s*sin(pose(3));
else
    x=pose(1)+(sin(theta)-sin(pose(3)))/k;
    y=pose(2)-(cos(theta)-cos(pose(3)))/k;
end
samples=[x y theta];
end

function value=ternary(condition,a,b)
if condition,value=a;else,value=b;end
end
