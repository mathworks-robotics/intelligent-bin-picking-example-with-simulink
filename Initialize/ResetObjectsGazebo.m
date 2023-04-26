%%Reset objects
% create mesh
px1 = -0.20:0.12:0.24;
py1 = -0.12:0.11:0.12;

py1 = py1(randperm(length(py1)))+0.48;
px1 = px1(randperm(length(px1)));

temp = cell(length(px1),length(py1));

for i = 1:length(px1)
    for j = 1:length(py1)
        temp(i,j) = {[px1(i), py1(j)]};
    end
end

newPose = reshape(temp,[1,length(px1)*length(py1)]);

% get the list of the world models
modelList = gzmodel("list");

% list of the target objects (naming starts with part1, part2 etc.)
partList = modelList(contains(modelList,"part"));


ii=randperm(length(newPose));
b=newPose(ii);
partGT = zeros(length(partList), 4);
% spawn the world
for i=1:length(partList)
    partGT(i, 1:2) = cell2mat(b(i));
    partGT(i, 3) = 0.8;
    partGT(i, 4) = rand*pi*0.8;
    gzlink('set',partList(i),'link','Position',[partGT(i, 2) partGT(i, 1) 0.8],'Orientation',eul2quat([partGT(i, 4),0,0]));
end

%Reset collision environment
binLength = 0.395; % Along X axis
binWidth = 0.585; % Along Y axis
binHeight = 0.11;
binCenterPosition = [0.48 0 -0.038];
binRotation = 0;

numOfParts = size(partGT,1); % Number of parts
