function out = myNormalize(in, varargin)
% OUT = MYNORMALIZE(IN) -> [0, 1]
% OUT = MYNORMILIZA(IN, [A,b]) -> [A, B]

if nargin < 2
    A = 0;
    B = 1;
else
    A = varargin{1}(1);
    B = varargin{1}(2);
end

out = A + (B-A)*(in - min(in(:)))./(max(in(:)) - min(in(:)));

    