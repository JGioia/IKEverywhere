function [dif] = GetDiff(pos_desired, pos_real)
    dif = 0;
    for i = 1:length(pos_desired)
        dif = dif + abs(pos_desired(i) - pos_real(i));
    end
end