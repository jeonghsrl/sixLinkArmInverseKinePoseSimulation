%% 鉄柱鋼材1の幾何学モデル
% Lx1: L鋼縦　Ly1: 横 Lz1:長さ Lt1: 厚さ
function [S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,objCenterPoints,objPoints]= func_pillarModelPoints(Lx1,Ly1,Lz1,Lt1,tformModel)

% 鋼材１の寸法パラメータ

%Lx1=0.044; Ly1=0.044; Lz1=0.430; Lt1=0.006;   %Obj1の寸法
% 点の位置
% 
P1=  tformModel*trvec2tform([0  Ly1/2  -Lz1/2]);
P2=  tformModel*trvec2tform([0  Ly1/2  Lz1/2]);
P3=  tformModel*trvec2tform([0 -Ly1/2  -Lz1/2]);
P4=  tformModel*trvec2tform([0 -Ly1/2  Lz1/2]);
P5=  tformModel*trvec2tform([Lx1 -Ly1/2 -Lz1/2]);
P6=  tformModel*trvec2tform([Lx1 -Ly1/2 Lz1/2]);
P7=  tformModel*trvec2tform([Lx1 -Ly1/2-Lt1 -Lz1/2]);
P8=  tformModel*trvec2tform([Lx1 -Ly1/2-Lt1 Lz1/2]);
P9=  tformModel*trvec2tform([-Lt1 -Ly1/2-Lt1 -Lz1/2]);
P10= tformModel*trvec2tform([-Lt1 -Ly1/2-Lt1 Lz1/2]) ;
P11= tformModel*trvec2tform([-Lt1 Ly1/2 -Lz1/2]);
P12= tformModel*trvec2tform([-Lt1 Ly1/2 Lz1/2]);

S1_rot=axang2tform([1 1 1 0]);
S2_rot=axang2tform([0 0 1 pi/2]);
S3_rot=axang2tform([1 1 1 0]);
S4_rot=axang2tform([0 0 1 -pi/2]);
S5_rot=axang2tform([0 0 1 -pi]);
S6_rot=axang2tform([0 0 1 pi/2]);
S7_rot=axang2tform([0 1 0 pi/2])*axang2tform([1 0 0 -pi/2]); 
S8_rot=axang2tform([0 1 0 pi/2]);
S9_rot=axang2tform([0 1 0 -pi/2])*axang2tform([1 0 0 -pi/2]);
S10_rot=axang2tform([0 1 0 -pi/2]);


%点群の行列
objPoints=[P1(1:3,4)';P2(1:3,4)';P3(1:3,4)';P4(1:3,4)';
    P5(1:3,4)';P6(1:3,4)';P7(1:3,4)';P8(1:3,4)';
    P9(1:3,4)';P10(1:3,4)';P11(1:3,4)';P12(1:3,4)'];

%平面に属した４つの点の同時変換行列(4x4)を要素とする多次元行列Sを作成 
S1(:,:,1) = P1*S1_rot; S1(:,:,2) = P2*S1_rot;   S1(:,:,3) = P3*S1_rot; S1(:,:,4) = P4*S1_rot;

S2(:,:,1) = P3*S2_rot; S2(:,:,2) = P4*S2_rot;   S2(:,:,3) = P5*S2_rot; S2(:,:,4) = P6*S2_rot;
S3(:,:,1) = P5*S3_rot; S3(:,:,2) = P6*S3_rot;   S3(:,:,3) = P7*S3_rot; S3(:,:,4) = P8*S3_rot;
S4(:,:,1) = P7*S4_rot; S4(:,:,2) = P8*S4_rot;   S4(:,:,3) = P9*S4_rot; S4(:,:,4) = P10*S4_rot;
S5(:,:,1) = P9*S5_rot; S5(:,:,2) = P10*S5_rot;  S5(:,:,3) = P11*S5_rot; S5(:,:,4) = P12*S5_rot;
S6(:,:,1) = P11*S6_rot;  S6(:,:,2) = P12*S6_rot;  S6(:,:,3) = P1*S6_rot; S6(:,:,4) = P2*S6_rot;
S7(:,:,1) = P1*S7_rot;   S7(:,:,2) = P3*S7_rot;   S7(:,:,3) = P11*S7_rot; S7(:,:,4) = P9*S7_rot;
S8(:,:,1) = P3*S8_rot;   S8(:,:,2) = P5*S8_rot;   S8(:,:,3) = P9*S8_rot; S8(:,:,4) = P7*S8_rot;
S9(:,:,1) = P2*S9_rot;   S9(:,:,2) = P4*S9_rot;   S9(:,:,3) = P12*S9_rot; S9(:,:,4) = P10*S9_rot;
S10(:,:,1) = P4*S10_rot; S10(:,:,2) = P6*S10_rot; S10(:,:,3) = P10*S10_rot; S10(:,:,4) = P8*S10_rot;
% 

%各平面の中心点
S1_cen = (S1(1:3,4,1)+ S1(1:3,4,2)+ S1(1:3,4,3)+ S1(1:3,4,4))/4;
S2_cen = (S2(1:3,4,1)+ S2(1:3,4,2)+ S2(1:3,4,3)+ S2(1:3,4,4))/4;
S3_cen = (S3(1:3,4,1)+ S3(1:3,4,2)+ S3(1:3,4,3)+ S3(1:3,4,4))/4;
S4_cen = (S4(1:3,4,1)+ S4(1:3,4,2)+ S4(1:3,4,3)+ S4(1:3,4,4))/4;
S5_cen = (S5(1:3,4,1)+ S5(1:3,4,2)+ S5(1:3,4,3)+ S5(1:3,4,4))/4;
S6_cen = (S6(1:3,4,1)+ S6(1:3,4,2)+ S6(1:3,4,3)+ S6(1:3,4,4))/4;
S7_cen = (S7(1:3,4,1)+ S7(1:3,4,2)+ S7(1:3,4,3)+ S7(1:3,4,4))/4;
S8_cen = (S8(1:3,4,1)+ S8(1:3,4,2)+ S8(1:3,4,3)+ S8(1:3,4,4))/4;
S9_cen = (S9(1:3,4,1)+ S9(1:3,4,2)+ S9(1:3,4,3)+ S9(1:3,4,4))/4;
S10_cen = (S10(1:3,4,1)+ S10(1:3,4,2)+ S10(1:3,4,3)+ S10(1:3,4,4))/4;

%各面の中心点
objCenterPoints=[S1_cen';S2_cen';S3_cen';S4_cen';
    S5_cen';S6_cen';S7_cen';S8_cen';S9_cen';S10_cen'];

end
