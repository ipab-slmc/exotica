tic;
% Start the exotica core
Core=exotica.Core;

% Prepare an initializer
SceneInit=exotica.Initializer('Scene',struct('Name','MyScene','VisualDebug','1','Solver',{{'Kinematica',struct('Joints','lwr_arm_0_joint,lwr_arm_1_joint,lwr_arm_2_joint,lwr_arm_3_joint,lwr_arm_4_joint,lwr_arm_5_joint,lwr_arm_6_joint','Root',{{'Limb',struct('Segment','base')}})}}));

% Create a scene
Scene=exotica.Scene('MyScene');

% Instantate the scene from the initializer
Scene.InstantiateInternal(SceneInit);
Scene.activateTaskMaps();

% Get collision scene
CollisionScene=Scene.getCollisionScene();

display(['Scene setup time ',num2str(toc),'s'])

% Test speed of self collision check
n=100;
tt=0;
for i=1:n
    tic;
    Scene.update((rand(7,1)-0.5)*pi);
    CollisionScene.isStateValid;
    tt=tt+toc;
end
tt=tt/n;
display(['Average collision query time ',num2str(tt),'s'])
