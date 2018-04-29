function time = rostime2double(stamp)
%ROSTIME2DOUBLE Convert ros time to doulbe
time = double(stamp.Sec) + double(stamp.Nsec) * 10^-9;
end

