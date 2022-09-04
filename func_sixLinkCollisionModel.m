%空間６自由度マニピュレータの基本モデル作成(rigidbodyjointを使う)
%DH法で座標系を設定し、オイラで座標系間の変換を行う。
%Matlab 2021a, Robotics System Toolboxが必要
%軸座標系を修正2021/11/10
%pillar30.stlもjoint8として挿入
%% 
% まず、ワークスペースをクリア
%clear 
%% 剛体ツリーを作成
function result = func_sixLinkCollisionModel(L1,L2,L3,L4,L5,L6)

robot = rigidBodyTree("DataFormat","column");
%% Joint Parameter

% L1 = 0.135;            %link 1の長さ
% L2 = 0.2;            %link 2の長さ
% L3 = 0.2;            %link 3の長さ
% L4 = 0;              %link 4の長さ
% L5 = 0;              %link 5の長さ
% L6 = 0.1;              %link 6の長さ（メカニカルポイントから手先先端原点までの距離）

th1_i = 0*pi/180;           %joint 1の初期角度(rad)
th2_i = 0*pi/180;           %Joint 2の初期角度(rad)
th3_i = 0*pi/180;           %Joint 3の初期角度(rad)
th4_i = 0*pi/180;           %joint 4の初期角度(rad)
th5_i = 0*pi/180;           %Joint 5の初期角度(rad)
th6_i = 0*pi/180;           %Joint 6の初期角度(rad)



%% 1番目のボディobjectを作成

body = rigidBody('link1');                      %bodyを作成
body.Mass = 1;
body.CenterOfMass = [L1/2 0 0];
joint = rigidBodyJoint('joint1','revolute');    %bodyjointを回転ジョイントで作成
joint.JointAxis = [0 0 1];                      %回転軸の設定 (X,Y,Z)
joint.HomePosition = th1_i;                     %jointの初期回転角度
tform = trvec2tform([0, 0, 0]);                  %基準座標系からの初期距離を同時変換に変換  
setFixedTransform(joint,tform);                 %jointから親への変換
body.Joint = joint;                             %jointをbodyに追加
addBody(robot,body,'base');                     % bodyをbase bodyに追加。
%% 2番目のボディobjectを作成

body = rigidBody('link2');                      %bodyを作成
body.Mass = 1;
body.CenterOfMass = [L2/2 0 0];
joint = rigidBodyJoint('joint2','revolute');    %bodyjointを回転ジョイントで作成
joint.JointAxis = [0 0 1];                      %回転軸の設定 (X,Y,Z)
joint.HomePosition = th2_i;                     %joint初期回転角度
tform = trvec2tform([0, 0, L1]);                %joint1座標系からZ方向へL1移動(DH法適用できない）
tform = tform*axang2tform([1 0 0 pi/2]);        %その後、x軸回りpi/2(※これで、j2からj1への変換行列が生成される。
setFixedTransform(joint,tform);                 %jointから親への変換
body.Joint = joint;                             %jointをbodyに追加
addBody(robot,body,'link1');                    % bodyをlink1 bodyに追加。
%% 3番目のボディobjectを作成

body = rigidBody('link3');                      %bodyを作成
body.Mass = 1;
body.CenterOfMass = [L3/2 0 0];
joint = rigidBodyJoint('joint3','revolute');    %bodyjointを回転ジョイントで作成
joint.JointAxis = [0 0 1];                      %回転軸の設定 (X,Y,Z)
joint.HomePosition = th3_i;                     %jointの初期回転角度
tform = trvec2tform([L2, 0, 0]);                %joint2座標系から、X方向へL2移動する変換行列  
tform = tform*axang2tform([0 0 1 -pi/2]);        %その後、z軸回りpi/2(※これで、j3からj2への変換行列が生成される。
setFixedTransform(joint,tform);                 %jointから親への変換
body.Joint = joint;                             %jointをbodyに追加
addBody(robot,body,'link2');                    % link3bodyをlink2 bodyに追加。
%% 4番目のボディobjectを作成

body = rigidBody('link4');                      %bodyを作成
body.Mass = 0.5;
body.CenterOfMass = [0 0 0];
joint = rigidBodyJoint('joint4','revolute');    %bodyjointを回転ジョイントで作成
joint.JointAxis = [0 0 1];                      %回転軸の設定 (X,Y,Z)
joint.HomePosition = th4_i;                     %jointの初期回転角度
tform = trvec2tform([L3, 0, 0]);               %joint3座標系からX方向へ移動する同時変換行列 
tform = tform*axang2tform([0 0 1 pi/2]);        %移動後、X軸回りpi/2回転。
tform = tform*axang2tform([1 0 0 pi/2]);        %移動後、X軸回りpi/2回転。
setFixedTransform(joint,tform);                 %joint3からjoint4の変換行列適用
body.Joint = joint;                             %jointをbodyに追加
addBody(robot,body,'link3');                    %link4 bodyをlink3 bodyに追加。
%% 5番目のボディobjectを作成

body = rigidBody('link5');                      %bodyを作成
body.Mass = 0.5;
body.CenterOfMass = [0 0 0];
joint = rigidBodyJoint('joint5','revolute');    %bodyjointを回転ジョイントで作成
joint.JointAxis = [0 0 1];                      %回転軸の設定 (X,Y,Z)
joint.HomePosition = th5_i;                     %jointの初期回転角度
tform = trvec2tform([0, 0, L4]);                %join4座標系からZ方向へL4移動 
tform = tform*axang2tform([1 0 0 -pi/2]);       %その後、X軸回り-pi/2回転 
setFixedTransform(joint,tform);                 %joint4からjoint5への変換行列適用
body.Joint = joint;                             %jointをbodyに追加
addBody(robot,body,'link4');                    %link5 bodyをlink4 bodyに追加。
%% 6番目のボディobjectを作成

body = rigidBody('link6');                      %bodyを作成
body.Mass = 0.5;
body.CenterOfMass = [0 0 0];
joint = rigidBodyJoint('joint6','revolute');    %bodyjointを回転ジョイントで作成
joint.JointAxis = [0 0 1];                      %回転軸の設定 (X,Y,Z)
joint.HomePosition = th6_i;                     %jointの初期回転角度
tform = trvec2tform([0, -L5, 0]);               %joint5から-Y方向へL5移動 
tform = tform*axang2tform([1 0 0 pi/2]);        %その後、X軸回りpi/2回転
setFixedTransform(joint,tform);                 %joint5からjoint6への変換
body.Joint = joint;                             %jointをbodyに追加
addBody(robot,body,'link5');                    %link6 bodyをlink5 bodyに追加。
%% エンドエフェクタボディobjectを作成

body = rigidBody('tool');                       %tool bodyを作成
body.Mass = 0.5;
body.CenterOfMass = [L6/2 0 0];
joint = rigidBodyJoint('fix1','fixed');         %fix jointを回転ジョイントで作成
tform = trvec2tform([0, 0, L6]);                %joint6座標系からZ方向へL6移動  
setFixedTransform(joint,tform);                 %joint6からtoolへの変換
body.Joint = joint;                             %jointをbodyに追加
addBody(robot,body,'link6');                    %tool bodyをlink6 bodyに追加。

%% pillar
body = rigidBody('pillar');
body.Mass = 0;
joint = rigidBodyJoint('pillarfix','fixed');         %fix jointを回転ジョイントで作成
tform = trvec2tform([0, 0,0]);                %joint6座標系からZ方向へL6移動  
setFixedTransform(joint,tform);                 %joint6からtoolへの変換
body.Joint = joint;                             %jointをbodyに追加
addBody(robot,body,'base');                    %tool bodyをlink6 bodyに追加。
%% Collision bodyの生成
%collisionobjの座標系は、各bodyの座標系に従う。
%link1
collisionObj = collisionCylinder(0.01,L1/2);    %(radius, length)のシリンダーobject(見やすくするために長さ半分にしている)
mat = trvec2tform([0, 0, L1/2]);              %シリンダーの底面を基準座標系に合わせるために
collisionObj.Pose = mat;                      %シリンダーのPoseとして入力
addCollision(robot.Bodies{1},collisionObj)    %bodyへ追加


%link2
collisionObj = collisionCylinder(0.01,L2/2);  %長さはjoint2座標系のZ軸方向 
mat = trvec2tform([L2/2, 0, 0]);              %joint2座標系基準で、X方向へシリンダ原点を移動
mat = mat*axang2tform([0 1 0 pi/2]);          %Z方向長さをX方向長さとなるようにY軸回り回転 
collisionObj.Pose = mat;
addCollision(robot.Bodies{2},collisionObj)

%link3
collisionObj = collisionCylinder(0.01,L3/2);
mat = trvec2tform([0, -L3/2, 0]);
mat = mat*axang2tform([1 0 0 pi/2]);
collisionObj.Pose = mat;
addCollision(robot.Bodies{3},collisionObj)

%link4
collisionObj = collisionCylinder(0.01,0.05);
addCollision(robot.Bodies{4},collisionObj)

%link5
collisionObj = collisionCylinder(0.01,0.05);
addCollision(robot.Bodies{5},collisionObj)

%link6
collisionObj = collisionCylinder(0.01,0.05);
addCollision(robot.Bodies{6},collisionObj)

%tool
collisionObj = collisionSphere(0.02);
addCollision(robot.Bodies{7},collisionObj)

%%
tform = trvec2tform([0,0,L1]); 
%addVisual(robot.Bodies{1},"Mesh",'./armSTL/Link1.stl',tform);
tform = trvec2tform([0,0,0]); 
%addVisual(robot.Bodies{2},"Mesh",'./armSTL/Link2.stl',tform);
tform = trvec2tform([0,0,0]); 
%addVisual(robot.Bodies{3},"Mesh",'./armSTL/Link3.stl',tform);
tform = trvec2tform([0,0,0]); 
%addVisual(robot.Bodies{4},"Mesh",'./armSTL/Link4.stl',tform);
tform = trvec2tform([0,0,0]); 
%addVisual(robot.Bodies{5},"Mesh",'./armSTL/Link5.stl',tform);
tform = trvec2tform([0,0,L6-0.05]); 
%addVisual(robot.Bodies{6},"Mesh",'./armSTL/Link6.stl',tform);
tform = trvec2tform([0,0,-0.135]); 
addVisual(robot.Bodies{8},"Mesh",'./Pillar30.stl',tform);

%% コンフィギュレーションを変更してみる
% showdetails(robot);
% config = homeConfiguration(robot);
% config(1).JointPosition = 30*pi/180; 
% config(2).JointPosition = 105*pi/180; 
% config(3).JointPosition = 0*pi/180; 
% config(4).JointPosition = 0*pi/180; 
% config(5).JointPosition = 0*pi/180; 
% config(6).JointPosition = 0*pi/180; 
% 
% tform = getTransform(robot, config, 'tool','base'); %同時変換行列 
% TipPos = tform(1:3,4); %エンドエフェクタの位置
% hold on
% grid on
% show(robot,config,'Collisions','on','Visuals','off'); %configに従ったrobotを表示 
% axis([-1 1 -1 1 -1 1]);         %表示軸の範囲 x- x+ y- y+
% xlabel X;
% ylabel Y;
% zlabel Z;
% view(40,25);   
% camproj('orthographic');           %正投影
% 
result = robot;
end
