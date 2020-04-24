function RMSE = findRMSE(time, state, ss_des)
    Ts = findSettlingTime(time, state);
    
    idx = find(time == Ts);
    state_check = state(idx:end);

    RMSE = sqrt(mean((state_check - ss_des).^2));
end

