clc
if ~isempty(instrfind)
     fclose(instrfind);
     delete(instrfind);
end
clf

%% ���ô���
s=serial('com7'); %ѡ�񴮿ں�  
set(s,'BaudRate',115200,'StopBits',1,'Parity','none');%���ò�����  ֹͣλ  У��λ  
fopen(s);

% ADC����Ƶ��100Hz��ÿ��10���ֽڣ�8λ����10s������Ҫ1e4���ֽ�
%Ϊ��ֹ�յ���һ���ֽڲ���AB����ȡһ������
%% �����ڲ�������
Sample_rate=100;%100Hz
Sample_time=1/Sample_rate;
Period=3;%��10s
Frame_size=22;%byte 2+4*5

Sample_point=Sample_rate*Period;
data=zeros(1,Sample_point*Frame_size);

t=0:Sample_time:(Period-Sample_time);

%% ����֡��������
run_time_in_dec= Period;%�ź��ܷ���ʱ��
param1_in_dec=3300;    %���б���ź�ʱ������ʼ��ѹ����������ź�ʱ���Ʒ�ֵ��PIDʱ���Ʋο�����(mm)
param2_in_dec=3300;    %���б���ź�ʱ������ֵ��ѹ����������ź�ʱ����Ƶ�ʣ�PIDʱ���ƿ�������������(ms)
deadzone_compensation_dac1_in_dec=0;
deadzone_compensation_dac2_in_dec=0;
PID_Kp=25;
PID_Ki=0;
PID_Kd=0;
Occupied=0;

%% ��������֡
Frame_head          = '02';                                 %֡ͷ
DAC_cmd             = '01';                                 %�����ĸ�DAC,01���أ�10���
mode_cmd            = '22';                                 %�ֶ�/�Զ�����ʽ
run_time            =  num2str(dec2hex(run_time_in_dec,2));                %�������������15s�����಻����60s
param1              =  num2str(num2hex(single(param1_in_dec)));
param2              =  num2str(num2hex(single(param2_in_dec)));
deadzone_compensation_dac1=num2str(num2hex(single(deadzone_compensation_dac1_in_dec)));
deadzone_compensation_dac2=num2str(num2hex(single(deadzone_compensation_dac2_in_dec)));
controller_param1   =  num2str(num2hex(single(PID_Kp)));
controller_param2   =  num2str(num2hex(single(PID_Ki)));
controller_param3   =  num2str(num2hex(single(PID_Kd)));
controller_param4   =  num2str(num2hex(single(Occupied)));
Reserved            = '';
Frame_tail          = '0d0a'; 

controller_param   =  strcat(controller_param1, controller_param2, controller_param3, controller_param4);
cmd = strcat(Frame_head, DAC_cmd, mode_cmd, run_time, param1, param2, deadzone_compensation_dac1, deadzone_compensation_dac2, controller_param, Reserved, Frame_tail);

cmd_in_hex = sscanf(cmd, '%2x'); %���ַ���ת����ʮ����������
%% ��������֡�����Ƶ�Ƭ��������
ack=1;  %��ʼĬ�϶Է����մ��������·���
ack_length=4;%4 byte

while(ack)
    ack=0;
    fwrite(s, cmd_in_hex, 'uint8') %�Զ���Ĵ���s���͸�����
    
    if strcmp(Frame_head,'04')
        %% ��ȡ����
        for i = 1:1 %ѭ����ȡ 
          out=fread(s,Frame_size,'uint8');%��ȡ ���ݸ��� �� ����
          if(i==1&&out(1)==3)   %�ж�����֡����
              ack=1;
              out=zeros(Frame_size,1);
              break;        
          end
          out1=reshape(out,1,Frame_size);   %ת��������
          for j=1:Frame_size
              data((i-1)*Frame_size+j)=out1(j);
          end
        end
    else
        %% ��ȡ����
        for i = 1:Sample_point %ѭ����ȡ 
          out=fread(s,Frame_size,'uint8');%��ȡ ���ݸ��� �� ����
          if(i==1&&out(1)==3)   %�ж�����֡����
              ack=1;
              out=zeros(Frame_size,1);
              break;        
          end
          out1=reshape(out,1,Frame_size);   %ת��������
          for j=1:Frame_size
              data((i-1)*Frame_size+j)=out1(j);
          end
        end
    end

    if(ack==0)
       %������ȷʱ��������ͼ
        fclose(s);
        delete(s);

        %% ��������
        for i = 1:Frame_size
              if data(i)==171 %%AB
                  offset=i;
                  break;
              end
        end

        Disp=zeros(1,Sample_point);
        dac_ch1=zeros(1,Sample_point);
        dac_ch2=zeros(1,Sample_point);
        time_stamp=zeros(1,Sample_point);
        
        for i=1:(Sample_point-1)
            origin=(i-1)*Frame_size+1+offset;
            u_bound=typecast(uint8([data(origin) data(origin+1) data(origin+2) data(origin+3)]),'single');
            Disp_temp=typecast(uint8([data(origin+4) data(origin+5) data(origin+6) data(origin+7)]),'single');
            dac_ch1_temp=typecast(uint8([data(origin+8) data(origin+9) data(origin+10) data(origin+11)]),'single');
            dac_ch2_temp=typecast(uint8([data(origin+12) data(origin+13) data(origin+14) data(origin+15)]),'single');
            l_bound=typecast(uint8([data(origin+16) data(origin+17) data(origin+18) data(origin+19)]),'single');
            
            Disp(i)=Disp_temp/125;
            dac_ch1(i)=dac_ch1_temp/3.3;
            dac_ch2(i)=dac_ch2_temp/3.3;
        end
        
        lower_bound=l_bound;
        upper_bound=u_bound;

       %% ����
        path = 'E:\WeederProject\ADRC\��������\';
        
        if strcmp(DAC_cmd, '01')
            Filehead = '����';
        elseif strcmp(DAC_cmd, '10')
            Filehead = '���';
        elseif strcmp(DAC_cmd, 'FF')
            if strcmp(mode_cmd, '12')
                Filehead = '����';
            elseif strcmp(DAC_cmd, '11')
                Filehead = 'PID';
            end
        end
        filename1 = strcat(path, Filehead, num2str(param1_in_dec), '-', num2str(param2_in_dec), '.fig');
        filename2 = strcat(path, Filehead, num2str(param1_in_dec), '-', num2str(param2_in_dec), '.jpg');
        
        %% ��ͼ
        %����ѹ�ź���DAC2�������ֵ������ѹ�ź���DAC1���
        
        figure(1)
        xlabel('Time /s');
        plot(t,Disp);
        hold on
        plot(t,dac_ch1);
        hold on
        plot(t,(-1).*dac_ch2);
        axis([0 Period -1.5 1.5]); % ������������ָ��������
        legend('Normalized Displacement','Normalized DAC channel 1 Voltage','Normalized DAC channel 2 Voltage');
        title(strcat(Filehead, num2str(param1_in_dec), '-', num2str(param2_in_dec)));
        
       %% ����ͼ��
        saveas(gcf, filename1);
        saveas(gcf, filename2);
    end
end

