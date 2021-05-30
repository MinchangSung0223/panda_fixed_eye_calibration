




function ret =invsqrt(mat)
[U,S,V]=svd(mat);
V = V';
s = diag(S);
ret = U*diag(1./sqrt(s))*V;
end
