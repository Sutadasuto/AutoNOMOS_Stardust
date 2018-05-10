path = "Experiment3";

for test = 1:10
   try
       filename = strcat(path, "/pathFollowing", num2str(test), ".csv");
       filename2 = strcat(path, "/statistics", num2str(test), ".csv");
       M = csvread(filename);
       M2 = [mean(M), std(M)];
       csvwrite(filename2,M2);
   catch
       null = 0;
   end
end