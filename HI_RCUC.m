clc
clear all
warning off

tic
yalmip;
Cplex;
[ data_b, data_g, data_l, data_r, data_s, data_gcost ] = chongqing()
for i = [22;23;31]
data_g(i,10)=0;
end
for i = [24;25]
data_g(i,10)=900;
end  
for i = [40;41;43;44]
data_g(i,10)=0;
end
 baseMVA = 100;
 Cap_c = [5;10;44;64.5;70;100;125;161.25;300;350;360;467;600;660;1000;1800;2000];
 c = length(Cap_c);
 Ng_c = [1;1;2;1;1;1;2;3;7;4;4;1;4;4;6;2;2];
 Rud = [0.25;0.25;0.25;0.25;0.25;0.25;0.19;0.25;0.25;0.25;0.25;0.25;0.27;0.27;0.27;0.27;0.27;0.27;0.27;0.3;0.3;0.3;0.3;0.25;0.25;0.25;0.25;
     0.26;0.25;0.25;0.25;0.25;0.295;0.295;0.295;0.295;0.25;0.25;0.25;0.25;0.25;0.25;0.25;0.25;0.25;0.25];
 Cap = [5;10;44;44;64.5;70;100;125;125;161.25;161.25;161.25;300;300;300;300;300;300;300;350;350;350;350;360;360;360;360;467;600;600;600;600;
     660;660;660;660;1000;1000;1000;1000;1000;1000;1800;1800;2000;2000];
 Ton_c = repmat(6,1,c)';
 Toff_c = repmat(6,1,c)';
 nanmeda_c = [0.5;0.5;0.5;0.5;0.5;0.62;0.5;0.5;0.46;0.4;0.5;0.48;0.5;0.41;0.5;0.5;0.5];
 quanyi = ones(17,1);
 chazhi = quanyi-nanmeda_c;
 Ru_c = chazhi/2;
 Rd_c = chazhi/2;
 SC_c = 3000*Cap;
 PTDF = makePTDF(baseMVA, data_b, data_l, 1)
T = 24;
eff_c  = data_s(:,2);
eff_dc = data_s(:,3);
P_max = data_g(:,9);
P_min = data_g(:,10);
E_lower = data_s(:,5);
Pmin=data_g(:,10);
Pmax=data_g(:,9);
N_g = length(data_g(:,1))-2;
N_l = length(data_l(:,1));
N_b = length(data_b(:,1));
N_r = 1;
N_s = length(data_s(:,1));
 beta = 0;
% beta = sdpvar(1);
H = 2; %持续充放电时间

%% 定义变量 注意顺序！
% 连续变量
P_g = sdpvar(N_g,T,'full'); % 发电功率
P_c = sdpvar(N_s,T,'full'); % 储能充电
P_dc = sdpvar(N_s,T,'full'); % 储能放电
E_init = sdpvar(N_s,1,'full'); %储能初始状态
dem = sdpvar(N_b,T,'full'); % 负荷
P_s = sdpvar(N_s,1,'full'); % 
E_s = sdpvar(N_s,T,'full');% 
P_r = sdpvar(N_r,T,'full');% 新能源变量
costH = sdpvar(N_g,T,'full');%24时刻优化启动成本
costJ = sdpvar(N_g,T,'full');%24时刻优化关停成本
cur = sdpvar(N_r,T,'full'); % 新能源削减量

P_clu = sdpvar(c,T,'full');
O_c = sdpvar(c,T,'full');
SU_c = sdpvar(c,T,'full');
SD_c = sdpvar(c,T,'full');
cost_c = sdpvar(c,T,'full');
SU_all = sdpvar(c,T,'full');
SD_all = sdpvar(c,T,'full');


%% 负荷曲线-标准
% load load_data;
% dem = bus_load;

%% 新能源曲线
load P_r_max;
%       bus  P_r
P_r_max =   P_r34+ P_r52+ P_r53+ P_r78+ P_r119+ P_r225 ;  
% P_r_all = P_r;
P_r_max = P_r_max(25345:25440,1);
P_r_max = P_r_max';
 P_r_max = 10*P_r_max;

%% 负荷曲线-重庆。 
load load_all;
dem = load_all(25345:25440,1);
n1 = 1:4:length(dem);
dem = dem(n1);
dem = dem';

%% 可再生能源发电约束-标准
% C_r = [];
% w_1 = w_ref(data_r); 
% w_0 = w_1(1:T);
% w_0 = w_0/150;
% for t = 1:T
%     C_r = [C_r; 0<=P_r(:,t)<=(data_r(:,2))*w_0(t)]; %新能源出力小于预测的波动出力
%     P_r_max(:,t) = (data_r(:,2))*w_0(t);
%     cur(:,t) = (data_r(:,2))*w_0(t)-P_r(:,t);
%     %    C_r = [C_r; 0<=P_r(:,t)<=0];%测试的时候才用。
% end
%    C_r = [C_r; sum(P_r(:,t))>=beta*sum(sum(dem(:,t)))]; %新能源出力小于预测的波动出力
% %      C_r = [C_r; sum(P_r(:,t))>=beta*sum(sum(dem(:,t)))]; %新能源出力小于预测的波动出力
% %      C_r = [C_r; 0<=beta<=0.5]; %新能源出力小于预测的波动出力
%     beta_true = sum(P_r(:,t))/sum(sum(dem(:,t)));

%% 可再生能源发电约束-重庆
 C_r = [];

 for t = 1:T
     C_r = [C_r; P_r(:,t)>=0];
     C_r = [C_r; P_r(:,t)<=P_r_max(:,t)];
     cur(:,t) = P_r_max(:,t)-P_r(:,t);
     %    C_r = [C_r; 0<=P_r(:,t)<=0];%测试的时候才用。
 end
    C_r = [C_r; sum(P_r(:,t))==beta*sum(dem(:,t))]; %新能源出力小于预测的波动出力
% %      C_r = [C_r; sum(P_r(:,t))>=beta*sum(sum(dem(:,t)))]; %新能源出力小于预测的波动出力
%     C_r = [C_r; 0<=beta<=0.5]; %新能源出力小于预测的波动出力
     beta_true = sum(P_r(:,t))/sum(dem(:,t));
     
% %% PTDF-重庆因为没有各个节点的负荷所以先忽略
% C_PTDF = [];
% for t = 1:T
%     C_PTDF = [C_PTDF; -data_l(:,6) <=( (PTDF(:,data_g(:,1)) * P_g(:,t))...
%         - (PTDF * dem(:,t))...
%         + (PTDF(:,data_r(:,1)) * P_r(:,t))...
%         + (PTDF(:,data_s(:,1)) * (P_dc(:,t)-P_c(:,t)))  )...
%         <= data_l(:,6)];
% end


%% 集群机组出力上下限、爬坡、启停约束
%% 发电机
C_gen = [];
% 单台机组出力和爬坡约束
for     t = 1:T
    for   i = 1:N_g
         C_gen = [C_gen,0<=P_g(i,t)];
         C_gen = [C_gen,P_g(i,t)<=Pmax(i)];
    end
end

for t=2:T
    for i=1:N_g
        C_gen=[C_gen,-Rud(i)*Cap(i)<=P_g(i,t)-P_g(i,t-1)];%上坡
        C_gen=[C_gen,P_g(i,t)-P_g(i,t-1)<=Rud(i)*Cap(i)];%下坡
    end
end
%机组集群耦合约束
for t = 1:T
    C_gen=C_gen+[P_g(21,t)==P_clu(1,t)];
    C_gen=C_gen+[P_g(42,t)==P_clu(2,t)];
    C_gen=C_gen+[P_g(26,t)+P_g(27,t)==P_clu(3,t)];
    C_gen=C_gen+[P_g(30,t)==P_clu(4,t)];
    C_gen=C_gen+[P_g(20,t)==P_clu(5,t)];
    C_gen=C_gen+[P_g(5,t)==P_clu(6,t)];
    C_gen=C_gen+[P_g(12,t)+P_g(13,t)==P_clu(7,t)];
    C_gen=C_gen+[P_g(14,t)+P_g(15,t)+P_g(16,t)==P_clu(8,t)];
    C_gen=C_gen+[P_g(6,t)+P_g(17,t)+P_g(18,t)+P_g(19,t)+P_g(35,t)+P_g(45,t)+P_g(46,t)==P_clu(9,t)];
    C_gen=C_gen+[P_g(9,t)+P_g(10,t)+P_g(11,t)+P_g(34,t)==P_clu(10,t)];
    C_gen=C_gen+[P_g(1,t)+P_g(2,t)+P_g(3,t)+P_g(4,t)==P_clu(11,t)];
    C_gen=C_gen+[P_g(33,t)==P_clu(12,t)];
    C_gen=C_gen+[P_g(7,t)+P_g(8,t)+P_g(37,t)+P_g(38,t)==P_clu(13,t)];
    C_gen=C_gen+[P_g(28,t)+P_g(29,t)+P_g(36,t)+P_g(39,t)==P_clu(14,t)];
    C_gen=C_gen+[P_g(22,t)+P_g(23,t)+P_g(31,t)+P_g(32,t)+P_g(40,t)+P_g(41,t)==P_clu(15,t)];
    C_gen=C_gen+[P_g(24,t)+P_g(25,t)==P_clu(16,t)];
    C_gen=C_gen+[P_g(43,t)+P_g(44,t)==P_clu(17,t)];
end

%机组出力上下限约束
for     t = 1:T
    for   i = 1:c
         C_gen = C_gen+[nanmeda_c(i)*O_c(i,t)<=P_clu(i,t)];
         C_gen = C_gen+[P_clu(i,t)<=O_c(i,t)];
    end
end
%开停机容量非负约束
for     t = 1:T
    for   i = 1:c
         C_gen = C_gen+[0<=SU_c(i,t)];
         C_gen = C_gen+[0<=SD_c(i,t)];
    end
end

%爬坡约束
for t=2:T
    for i=1:c
        C_gen=C_gen+[-Rd_c(i)*O_c(i,t)<=P_clu(i,t)-P_clu(i,t-1)];%上坡
        C_gen=C_gen+[P_clu(i,t)-P_clu(i,t-1)<=Ru_c(i)*O_c(i,t)];%下坡
                    O_c(i,t) = O_c(i,t-1)+SU_c(i,t)-SD_c(i,t);
    end
end

%启动约束
for t=1:T
    for i=1:c
        for tau = 1:6
            SU_all(i,t) = 0;
            SU_all(i,t) = SU_all(i,t)+SU_c(i,t-tau);
        end
        C_gen=C_gen+[SU_all(i,t)<=O_c(i,t)];
    end
end
%停机约束
for t=1:T
    for i=1:c
        for tau = 1:6
            SD_all(i,t) = 0;
            SD_all(i,t) = SD_all(i,t)+SD_c(i,t-tau);
        end
        C_gen=C_gen+[O_c(i,t)<=Ng_c(i)*Cap_c(i)-SD_all(i,t)];
    end
end

% %启停成本约束
for t=1:T   %启停成本零限约束
    for i=1:c
    C_gen=C_gen+[0<=cost_c(i,t)];
    cost_c(i,t) = SC_c(i)*SU_c(i,t);
    end
end


%% 储能
C_sto = [];
%    C_sto = [C_sto; 0 <= P_s <= 0; 0<= E_s <= 0]; %建模时必须去掉这个约束；测试的时候才用。
bigM = 10^5;
for i = 1:N_s
    C_sto = [C_sto; P_s(i)*H(i)*E_lower(i)<=E_init(i)<=P_s(i)*H(i);];
    E_s(i,1) = E_init(i);
    for t = 2:T
        E_s(i,t) = E_s(i,t-1)+ sum(P_c(i,1:t))*eff_c(i)-sum(P_dc(i,1:t))/eff_dc(i);
        C_sto = [C_sto; P_s(i)*H(i)*E_lower(i)<=E_s(i,t)<=P_s(i)*H(i)];
    end
    for t =1:T
        C_sto = [C_sto; 0<=P_c(i,t)<=P_s(i)];
        C_sto = [C_sto; 0<=P_dc(i,t)<=P_s(i)];
    end
end
        C_sto = [C_sto; 0<=P_s(i)<=1000000];



%% 系统功率平衡
C_bal = [];
for t = 1:T
    C_bal = [C_bal; dem(:,t)>=sum(P_g(:,t))+P_dc(:,t)-P_c(:,t)+P_r(:,t) ];
    C_bal = [C_bal; dem(:,t)<=sum(P_g(:,t))+P_dc(:,t)-P_c(:,t)+P_r(:,t) ];
end

%% 约束集
C = [C_gen;C_sto;C_bal;C_r];
%% 目标函数： obj_1: 运行成本+startup成本+shutdown成本, obj_:储能运行成本
obj_1=0;%24点联合优化的总成本
for  i = 1:N_g
    for  t = 1:T
        obj_1=obj_1+35*(P_g(i,t));%煤耗成本
    end
end
obj_5 = sum(sum(cost_c));%加上机组启停产生的开停机成本
obj_2 = 200*sum(cur);
obj_3 = 120000*P_s;
obj_4 = 3*sum(P_c);
obj = obj_1+obj_2+obj_3+obj_4+obj_5;




%% 求解
ops = sdpsettings('verbose',0,'solver','cplex');
result = optimize(C,obj,ops)
value(P_s)
value(obj)
subplot(1,2,1)
% bar(value(P_r)','stack')%阶梯图
bar(value(dem)','stack')%阶梯图
P_C = P_dc-P_c;
subplot(1,2,2)
% bar(value(E_s)','stack')%阶梯图
bar(value(P_g)','stack')%阶梯图
P_g_double = double(P_g);
O_c_double = double(O_c);
SU_c_double = double(SU_c);
SD_c_double = double(SD_c);
Pmin_double = double(Pmin);

P_fix = zeros(N_g,T);

infea = 0;
infea_deta = 0;
for i = 1:N_g
    for j = 1:T
         if (0<P_g_double(i,j) && P_g_double(i,j)<Pmin_double(i))
             infea = infea + 1;
         end
        if (0<P_g_double(i,j) && P_g_double(i,j)<0.1*Pmin_double(i))
            P_fix(i,j) = P_g_double(i,j);
            infea_deta = infea_deta + 1;
        end 
    end
end

 %% 开始求解可行域
% plp = Opt (C, obj, beta, P_clu); % 利用MPT3工具包建模
% solution = plp.solve; % 利用MPT3工具包求解多参数规划，结果存在solution里面
% % regions = solution.xopt.Set;% 提取每个区域信息
% % regions.plot;% 画区域
% figure;
% solution.xopt.fplot('obj','linewidth',3);

toc
disp(['运行时间: ',num2str(toc)]);

save('HI_RCUC_result','P_fix')

