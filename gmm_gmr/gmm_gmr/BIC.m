function [bicValue] = BIC(Data, nbStates, Priors, Mu, Sigma )
%
% BIC Criterion

% Inputs -----------------------------------------------------------------
%   o Data:    D x N array representing N datapoints of D dimensions.
%   o nbStates: Number K of GMM components.
%   o Priors: 1 x K array representing the initial prior probabilities 
%              of the K GMM components.
%   o Mu:     D x K array representing the initial centers of the K GMM 
%              components.
%   o Sigma:  D x D x K array representing the initial covariance matrices 
%              of the K GMM components.
% Outputs ----------------------------------------------------------------
%   o bicValue:  1 x 1 

%% Initialization of the parameters
[nbVar, nbData] = size(Data);
nbStates = size(Sigma,3);

%% Computation of p(x)
  for i=1:nbStates
    %Compute probability p(xi)
    Pxi(:,i) = Priors(i).*gaussPDF(Data, Mu(:,i), Sigma(:,:,i));
  end
  %Compute probability p(xi)
  Px=sum(Pxi,2);
  %Compute L
  L=sum(log(Px));
  %Compute np
  np=nbStates-1+nbStates*nbVar;
  %Compute bicValue
  bicValue=-L+0.5*np*log(nbData);
  


