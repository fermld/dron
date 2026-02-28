function val = randUniformSym(max_abs)
% Random scalar in [-max_abs, +max_abs]
    val = (2*rand - 1) * max_abs;
end

