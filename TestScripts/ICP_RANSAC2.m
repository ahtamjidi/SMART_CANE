function [R,T,error,BestFit]=ICP_RANSAC2(Ya,Yb,options)
%%% Implementation of the RANSAC algorithm of the paper
% @INPROCEEDINGS {gpandey-2011b,
%     AUTHOR = { Gaurav Pandey and James R. McBride and Silvio Savarese and Ryan M. Eustice },
%     TITLE = { Visually Bootstrapped Generalized ICP },
%     BOOKTITLE = { Proceedings of the IEEE International Conference on Robotics and Automation },
%     YEAR = { 2011 },
%     ADDRESS = { Shanghai, China },
%     NOTE = { Accepted, To Appear },
% }

% 1: input: YA, YB, SA, SB,
% 2: output: The estimated transformation [R0; t0]
% 3: Establish camera constrained SIFT correspondences between
% SA and SB.
% 4: Store the matches in a list L.
% 5: while iter < MAXITER do
% 6: Randomly pick 3 pairs of points from the list L.
% 7: Retrieve these 3 pair of points from YA and YB.
% 8: Calculate the 6-DOF rigid body transformation [R; t]
% that best aligns these 3 points.
% 9: Store this transformation in an array M, M[iter] =
% [R; t]
% 10: Apply the transformation to YB to map Scan B�s
% points into the reference frame of Scan A: Y0
% B = RYB + t
% 11: Calculate the set cardinality of pose-consistent SIFT
% correspondences that agree with the current transformation
% (i.e., those that satisfy a Euclidean threshold
% on spatial proximity): n = j(Y0
% B (L) ? YA(L)) < j
% 12: Store the number of pose-consistent correspondences
% in an array N, N[iter] = n
% 13: iter = iter + 1
% 14: end while
% 15: Find the index i that has maximum number of correspondences
% in N.
% 16: Retrieve the transformation corresponding to index i
% 17: Recalculate the Transformation and Rotation
% from M. [R0; T0] = M[i]. This is the required transformation.




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% matches matrix has the following structure
% Yb   Currnet Frame    matches First Row
% Ya   Previous Frame   matches Second Row
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

iter=1;
Y0b=zeros(size(Yb));
M=struct('R',[],'T',[],'ErrorSum',[]);
Cardinality=zeros(1,options.MaxIteration);
ErrorSum =0;
while iter<options.MaxIteration
    % 6: Randomly pick 3 pairs of points from the list L.
    RandIndex = get_rand(5, size(Ya,2) );  %% Remember that indices should not be the same
    % 7: Retrieve these 3 pair of points from YA and YB.
    SetPoints_a=Ya(:,RandIndex); %% Point sets should be in the form P={P1,P2,..}
    SetPoints_b=Yb(:,RandIndex); %%
    % 8: Calculate the 6-DOF rigid body transformation [R; t]
    % that best aligns these 3 points.
    [Rot, Trans, State] = find_transform_matrix(SetPoints_a, SetPoints_b);
%    [RR1, TT1] = icp(SetPoints_a',SetPoints_b');
%     [RR2, TT2] = icp(Ya,Yb);
    
    % 9: Store this transformation in an array M, M[iter] =
    % [R; t]
    M(iter).R=Rot;
    M(iter).T=Trans;
    M(iter).ErrorSum=0;
    % 10: Apply the transformation to YB to map Scan B�s
    % points into the reference frame of Scan A: Y0B = RYB + t
    %%% For later usage we need to keep the support set of each Hypothesis.
    %%% This is especially useful in recalculation step
    M(iter).SupportSet.Ya=[];
    M(iter).SupportSet.Yb=[];
    for (i=1:size(Ya,2))
        try
        Y0b(:,i)=Rot*Yb(:,i)+Trans;
        catch
            disp('Salam Guguli')
        end
        % 11: Calculate the set cardinality of pose-consistent SIFT
        % correspondences that agree with the current transformation
        % (i.e., those that satisfy an Euclidean threshold
        % on spatial proximity): n = norm(Y0b(L) - YA(L)) < epsilon
        % 12: Store the number of pose-consistent correspondences
        % in an array N, N[iter] = n
        TempError=norm(Y0b(:,i)-Ya(:,i));
        M(iter).ErrorSum = M(iter).ErrorSum+ TempError;
        if TempError<=options.DistanceThreshold
            M(iter).SupportSet.Ya=[M(iter).SupportSet.Ya,Ya(:,i)];
            M(iter).SupportSet.Yb=[M(iter).SupportSet.Yb,Yb(:,i)];
            Cardinality(iter)=Cardinality(iter)+1;
        end
    end
    iter=iter+1;
end
% 15: Find the index "iter" that has maximum number of correspondences
% in N.
for iii=1:length(M)
eee1(iii)=M(iii).ErrorSum;
eee2(iii)=Cardinality(iii);
end
MaxCardinality=max(eee2);
eee2(eee2~=max(eee2))=0;
eee1(eee2==0)=10000;
[C,I] = min(eee1);
BestError=C
BestFitIdx=I
BestFit=MaxCardinality
% [BestFit BestFitIdx]=max(Cardinality);
% 16: Retrieve the transformation corresponding to index i
% from M. [R0; T0] = M[i]. This is the required transformation.
R= M(BestFitIdx).R;
T= M(BestFitIdx).T;
% 17:  Recalculate the Transformation and Rotationfor the best hypothesis
[R, T, State] = find_transform_matrix(M(BestFitIdx).SupportSet.Ya, M(BestFitIdx).SupportSet.Yb);
error=M(BestFitIdx).ErrorSum;
