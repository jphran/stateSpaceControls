function settlingTime = findSettlingTime(time, state)
    yss = mean(state((length(state)-20):end));   
    Ts_check = state > 0.96 * yss;
    t_check = time(Ts_check);
    settlingTime = t_check(1); %settling time
end

