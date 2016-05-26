prompt = 'Enter the number of infrared LEDs: ';
numberOfILEDs = input(prompt);
prompt = 'Enter time for all measurements (in microseconds): ';
requiredTime = input(prompt);
numberOfMeasurements = 0;
ILEDsArray = [];
for i = 1:1:numberOfILEDs
    ILEDsArray = [ILEDsArray; i];
end
resultMatrix = [];
while true
    prompt = 'Enter vector with the measurement (only with "0" and "1")\n(to stop enter -1, to clear the matrix enter -2): ';
    measurements = input(prompt);
    if measurements == -1 
       break;
    end
    if measurements == -2
       resultMatrix = [];
       continue;
    end
    if length(measurements) ~= numberOfILEDs 
         error('Input must be a vector with the exact number of infrared LEDs')
    end
    numberOfMeasurements = numberOfMeasurements + 1;
    resultMatrix = [resultMatrix; measurements];
    
    sumArray = [];
    for j = 1:1:numberOfILEDs
        sumColomn = sum(resultMatrix(:,j));
        sumArray = [sumArray, sumColomn];
    end 
end

timeForEachMeasurement = requiredTime/numberOfMeasurements;
maxILEDSum = numberOfMeasurements;
probabilityToBeOne = [];
for k = 1:1:numberOfILEDs
    probabilityToBeOne = [probabilityToBeOne, sumArray(1,k)/maxILEDSum*100];
end
strRequiredTime = num2str(requiredTime);
strTimeForEachMeasurement = num2str(timeForEachMeasurement);
bar(ILEDsArray,probabilityToBeOne,0.5,'FaceColor',[0.2 0.2 0.5])
xlabel('Infrared LEDs','FontSize',12)
ylabel('% probability to be "1"','FontSize',12)
title({'Required time (microseconds)', strRequiredTime, 'Time for each measurement (microseconds)', strTimeForEachMeasurement},'FontSize',12)
