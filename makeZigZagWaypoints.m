function wp = makeZigZagWaypoints(park_km, lane_spacing_km, margin_km)
%MAKEZIGZAGWAYPOINTS Creates zigzag waypoints across a square park.
%   lane_spacing_km: separation between zigzag lanes (precision control)
%   margin_km: keep trajectory away from borders (recommended)

    y_lanes = margin_km : lane_spacing_km : (park_km - margin_km);
    wp = [];

    for k = 1:numel(y_lanes)
        y = y_lanes(k);
        if mod(k,2) == 1
            wp = [wp; margin_km, y; park_km - margin_km, y]; %#ok<AGROW>
        else
            wp = [wp; park_km - margin_km, y; margin_km, y]; %#ok<AGROW>
        end
    end
end