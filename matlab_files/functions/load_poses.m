function armMat = load_poses(poseFolder)
poseFolder = string(poseFolder)
poselist = dir(poseFolder+'/*.txt');
armMat = zeros(4,4,length(poselist))
for i=1:length(poselist)
  loadname = poseFolder+"/"+string(poselist(i).name)
  armMat(:,:,i) =load(loadname)
end
end