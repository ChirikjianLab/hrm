function R = par2rotm(q)
if length(q) == 3
    R = expm(skew(q));
elseif length(q) == 4
    if size(q,2) ~= 4
        q = q';
    end
    R = axang2rotm(q);
end
