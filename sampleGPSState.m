function state = sampleGPSState(p_ok, p_bad, ~)
% Randomly samples GPS state based on probabilities
% Returns: 1=GPS_OK, 2=GPS_BAD, 3=GPS_LOST

    r = rand;

    if r < p_ok
        state = 1;           % GPS_OK
    elseif r < (p_ok + p_bad)
        state = 2;           % GPS_BAD
    else
        state = 3;           % GPS_LOST
    end
end