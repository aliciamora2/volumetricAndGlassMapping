function [indexes, fromTimes, toTimes, diffs] = matchTimestamps(matchFrom, matchTo, maxDelta, matchFromOffset)
if(~exist('maxDelta','var'))
    maxDelta = Inf;
end
if(~exist('matchFromOffset','var'))
    matchFromOffset = 0;
end
fromTimes = zeros(size(matchFrom,1),1);
toTimes = zeros(size(matchTo,1),1);
%Read both times into arrays
for n = 1:size(matchFrom,1)
   fromTimes(n) = double(matchFrom{n}.Header.Stamp.Sec) + (10^-9)*double(matchFrom{n}.Header.Stamp.Nsec);
end
for n = 1:size(matchTo,1)
   toTimes(n) = double(matchTo{n}.Header.Stamp.Sec) + (10^-9)*double(matchTo{n}.Header.Stamp.Nsec);
end
%For each element in the 'from' array, find the closest in the 'to' array
diffs = zeros(size(fromTimes,1),1);
indexes = zeros(size(fromTimes,1),1);
for n = 1:size(fromTimes,1)
   diff = toTimes - (fromTimes(n) + matchFromOffset);
   [diffs(n), idx] = min(abs(diff));
   if(diffs(n) < maxDelta)
       indexes(n) = idx;
   else
       indexes(n) = -1;
   end
   
end
end