%% 
clc
close all
clear 
%%
hdlsetuptoolpath('ToolName','Xilinx Vivado','ToolPath','E:\xilinx\xilinx\Vivado\2022.1\bin\vivado.bat')

% filProgramFPGA('Xilinx Vivado','Alice_test51.bit',1); % correct

% filProgramFPGA('Xilinx Vivado','Bob_test51.bit',1);   % correct

filProgramFPGA('Xilinx Vivado','Alice_Sifting.bit',1); % correct

% filProgramFPGA('Xilinx Vivado','Alice_ER_test5.bit',1); % correct

% filProgramFPGA('Xilinx Vivado','Alice_PA_test3.bit',1);% correct
%% DEPTH = 8192
mem = aximanager('Xilinx');
B_rd0 = readmemory(mem,'c0000000',8192,'BurstType','Increment');
B_rd1 = readmemory(mem,'c2000000',8192,'BurstType','Increment');

for i = 1:2048
   if ((B_rd0(i)~=0) && (B_rd1(i) ~= 0))
       disp("address: "+ num2str(i) + ", stored: " + dec2hex(B_rd0(i), 8) + dec2hex(B_rd1(i), 8));
   end
end
%% Save the key
key = {};
for i = 1:2048
    if ((B_rd0(i)~=0) && (B_rd1(i) ~= 0))
        key{i} = [dec2bin(B_rd0(i), 32) dec2bin(B_rd1(i), 32)];
    end
end
fileID = fopen('A_Secret_Key.txt','w+');
for i = 1:length(key)
    fprintf(fileID,'%s',key{i});
    fprintf(fileID,'\n');
end
fclose(fileID);

%% DEPTH = 65536
mem = aximanager('Xilinx');
B_rd0 = readmemory(mem,'c0000000',16384,'BurstType','Increment');
B_rd1 = readmemory(mem,'c2000000',16384,'BurstType','Increment');

% for i = 1:16384
%     if ((B_rd0(16385-i)~=0) && (B_rd1(16385-i) ~= 0))
%         disp("address: "+ num2str(16385-i) + ", stored: " + dec2hex(B_rd0(16385-i), 8) + dec2hex(B_rd1(16385-i), 8));
%     end
% end

for i = 1:16384
    if ((B_rd0(i)~=0) && (B_rd1(i) ~= 0))
        disp("address: "+ num2str(i) + ", stored: " + dec2hex(B_rd0(i), 8) + dec2hex(B_rd1(i), 8));
    end
end
%% Save the key
key = {};
for i = 1:16384
    if ((B_rd0(i)~=0) && (B_rd1(i) ~= 0))
        key{i} = [dec2bin(B_rd0(i), 32) dec2bin(B_rd1(i), 32)];
    end
end

for i = 1:16384
    if ((B_rd0(i)~=0) && (B_rd1(i) ~= 0))
        key{i} = [dec2hex(B_rd0(i), 8) dec2hex(B_rd1(i), 8)];
    end
end

fileID = fopen('A_ER_Key_HEX.txt','w+');
for i = 1:length(key)
    fprintf(fileID,'%s',key{i});
    fprintf(fileID,'\n');
end
fclose(fileID);
release(mem);

