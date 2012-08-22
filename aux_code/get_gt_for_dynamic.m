function [T,q,R]=get_gt_for_dynamic(idx1,idx2)
global myCONFIG
load([myCONFIG.PATH.DATA_FOLDER,'\gt\d1_gt.dat'])
x= d1_gt(idx2,1)-d1_gt(idx1,1);
T=[x/1000;0;0];
R=eye(3);
q=[1;0;0;0];