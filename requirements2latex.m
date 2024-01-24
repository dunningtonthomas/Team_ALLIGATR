%% Takes in the requirements excel and turns it into a latex table

clc; close all; clear;

reqsTable = readtable("Data/requirements2.csv");
lvl0Tab = reqsTable(reqsTable.Level == 0, 2:6);
lvl1Tab = reqsTable(reqsTable.Level == 1, 2:6);
lvl2Tab = reqsTable(reqsTable.Level == 2, 2:6);

table2latex(lvl0Tab, 'Data/lvl0');
table2latex(lvl1Tab, 'Data/lvl1');
table2latex(lvl2Tab, 'Data/lvl2');

for i=0:2
    fid1 = fopen("Data/lvl" + i + ".tex");
    fid2 = fopen("Data/output_" + i + ".txt", 'w');
    while 1
        line = string(fgetl(fid1));
        if line ~= "-1"
            fprintf(fid2, "%s\n", line);
        else
            break;
        end
        fprintf(fid2, "\\hline\n");
    end
    
    fclose(fid1);
    fclose(fid2);
end


