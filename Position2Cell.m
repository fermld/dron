function [i, j] = Position2Cell(x_km, y_km, cell_km, N)
%This convert continuous position (km) to grid indices (i,j).
%   x_km, y_km: position in km (continuous)
%   cell_km: cell size in km
%   N: grid size (NxN)
%   i = row index (Y), j = column index (X)

    j = floor(x_km / cell_km) + 1;  % X -> column
    i = floor(y_km / cell_km) + 1;  % Y -> row

    % Saturate to grid bounds
    j = min(max(j, 1), N);
    i = min(max(i, 1), N);
end