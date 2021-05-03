function f = gauss_distribution(x, y, x0, y0, sx, sy, A)
px = -.5 * ((x - x0)/sx) .^ 2;
py = -.5 * ((y - y0)/sy) .^ 2;
% p2 = (s * sqrt(2*pi));
% f = exp(p1) ./ p2; 
f = A * exp(px + py);
end
