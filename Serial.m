clc
if ~isempty(instrfind)
     fclose(instrfind);
     delete(instrfind);
end
clf

%% 设置串口
s=serial('com7'); %选择串口号  
set(s,'BaudRate',115200,'StopBits',1,'Parity','none');%设置波特率  停止位  校验位  
fopen(s);

% ADC采样频率100Hz，每次10个字节（8位），10s数据需要1e4个字节
%为防止收到第一个字节不是AB，多取一组数据
%% 读串口参数设置
Sample_rate=100;%100Hz
Sample_time=1/Sample_rate;
Period=3;%读10s
Frame_size=22;%byte 2+4*5

Sample_point=Sample_rate*Period;
data=zeros(1,Sample_point*Frame_size);

t=0:Sample_time:(Period-Sample_time);

%% 数据帧参数定义
run_time_in_dec= Period;%信号总发生时间
param1_in_dec=3300;    %输出斜坡信号时控制起始电压，输出正弦信号时控制幅值，PID时控制参考输入(mm)
param2_in_dec=3300;    %输出斜坡信号时控制终值电压，输出正弦信号时控制频率，PID时控制控制器采样周期(ms)
deadzone_compensation_dac1_in_dec=0;
deadzone_compensation_dac2_in_dec=0;
PID_Kp=25;
PID_Ki=0;
PID_Kd=0;
Occupied=0;

%% 定义数据帧
Frame_head          = '02';                                 %帧头
DAC_cmd             = '01';                                 %启用哪个DAC,01缩回，10伸出
mode_cmd            = '22';                                 %手动/自动，方式
run_time            =  num2str(dec2hex(run_time_in_dec,2));                %正弦输出不超过15s，其余不超过60s
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

cmd_in_hex = sscanf(cmd, '%2x'); %将字符串转换成十六进制数据
%% 发送数据帧（控制单片机工作）
ack=1;  %初始默认对方接收错误，需重新发送
ack_length=4;%4 byte

while(ack)
    ack=0;
    fwrite(s, cmd_in_hex, 'uint8') %对定义的串口s发送该数据
    
    if strcmp(Frame_head,'04')
        %% 读取数据
        for i = 1:1 %循环读取 
          out=fread(s,Frame_size,'uint8');%读取 数据个数 与 类型
          if(i==1&&out(1)==3)   %判断数据帧类型
              ack=1;
              out=zeros(Frame_size,1);
              break;        
          end
          out1=reshape(out,1,Frame_size);   %转换矩阵方向
          for j=1:Frame_size
              data((i-1)*Frame_size+j)=out1(j);
          end
        end
    else
        %% 读取数据
        for i = 1:Sample_point %循环读取 
          out=fread(s,Frame_size,'uint8');%读取 数据个数 与 类型
          if(i==1&&out(1)==3)   %判断数据帧类型
              ack=1;
              out=zeros(Frame_size,1);
              break;        
          end
          out1=reshape(out,1,Frame_size);   %转换矩阵方向
          for j=1:Frame_size
              data((i-1)*Frame_size+j)=out1(j);
          end
        end
    end

    if(ack==0)
       %接受正确时解析并绘图
        fclose(s);
        delete(s);

        %% 解析数据
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

       %% 命名
        path = 'E:\WeederProject\ADRC\测试数据\';
        
        if strcmp(DAC_cmd, '01')
            Filehead = '缩回';
        elseif strcmp(DAC_cmd, '10')
            Filehead = '伸出';
        elseif strcmp(DAC_cmd, 'FF')
            if strcmp(mode_cmd, '12')
                Filehead = '正弦';
            elseif strcmp(DAC_cmd, '11')
                Filehead = 'PID';
            end
        end
        filename1 = strcat(path, Filehead, num2str(param1_in_dec), '-', num2str(param2_in_dec), '.fig');
        filename2 = strcat(path, Filehead, num2str(param1_in_dec), '-', num2str(param2_in_dec), '.jpg');
        
        %% 绘图
        %负电压信号由DAC2输出绝对值，正电压信号由DAC1输出
        
        figure(1)
        xlabel('Time /s');
        plot(t,Disp);
        hold on
        plot(t,dac_ch1);
        hold on
        plot(t,(-1).*dac_ch2);
        axis([0 Period -1.5 1.5]); % 设置坐标轴在指定的区间
        legend('Normalized Displacement','Normalized DAC channel 1 Voltage','Normalized DAC channel 2 Voltage');
        title(strcat(Filehead, num2str(param1_in_dec), '-', num2str(param2_in_dec)));
        
       %% 保存图像
        saveas(gcf, filename1);
        saveas(gcf, filename2);
    end
end

