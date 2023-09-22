%% Takes in the requirements excel and turns it into a latex table

clc; close all; clear;

reqsTable = readtable("Data/requirements.csv");
lvl0Tab = reqsTable(reqsTable.Level == 0, 2:6);
lvl1Tab = reqsTable(reqsTable.Level == 1, 2:6);
lvl2Tab = reqsTable(reqsTable.Level == 2, 2:6);

table2latex(lvl0Tab, 'Data/lvl0');
table2latex(lvl1Tab, 'Data/lvl1');
table2latex(lvl2Tab, 'Data/lvl2');

fid1 = fopen('Data/lvl2.tex');
fid2 = fopen('Data/output.txt', 'w');
while 1
    line = fgetl(fid1);
    if line ~= ""
        fprintf(fid2, "%s\n", line);
    else
        break;
    end
    fprintf(fid2, "\\hline\n");
end

fclose(fid1);
fclose(fid2);


