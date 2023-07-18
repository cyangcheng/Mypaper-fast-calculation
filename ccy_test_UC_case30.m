

clear
clc
tic
yalmip;
Cplex;
tic

%%系统参数
%所有参数均用有名值表示
paragen=xlsread('excel2017','机组参数');
loadcurve=xlsread('excel2017','负荷曲线');
netpara=xlsread('excel2017','网络参数');
%% 规模变量
%机组数
gennum=size(paragen);
gennum=gennum(1,1);
%节点数
numnodes=size(loadcurve);
numnodes=numnodes(1,1)-1;
%时间范围
T=size(loadcurve);
T=T(1,2)-1;
%% 储能参数
% bus   eff_c  eff_dc   E_init E_lower
data_s = [
	6	0.95  0.95  0.5   0.2
];
%% 新能源参数
% bus   capacity
data_r = [
 	6	1.5;
];
branch_num=size(netpara);%网络中的支路
branch_num=branch_num(1,1);
PL_max=netpara(:,6);%线路最大负荷
PL_min=netpara(:,7);%线路最小负荷
limit=paragen(:,3:4);%机组出力上下限//limit(:,1)表示上限，limit(:,2)表示下限
Pmin=limit(:,2);             
Pmin = 2*Pmin;
Pmax=limit(:,1);
Ru = [0.3;0.3;0.2;0.2;0.15;0.15];
Rd = [0.3;0.3;0.2;0.2;0.15;0.15];
% Su=(limit(:,1)+limit(:,2))/2;
% Sd=(limit(:,1)+limit(:,2))/2;
c = 3;
eff_c  = data_s(:,2);
eff_dc = data_s(:,3);
E_lower = data_s(:,5);
N_r = length(data_r(:,1));
N_s = length(data_s(:,1));
 beta = 0;% 新能源渗透率
% beta = sdpvar(1);
H = 2; %持续充放电时间
P_c = sdpvar(N_s,T,'full'); % 储能充电
P_dc = sdpvar(N_s,T,'full'); % 储能放电
E_init = sdpvar(N_s,1,'full'); %储能初始状态
P_s = sdpvar(N_s,1,'full'); % 
E_s = sdpvar(N_s,T,'full');% 
P_r = sdpvar(N_r,T,'full');% 新能源变量

para=paragen(:,5:7);%成本系数//para(:,1)表示系数a,para(:,2)表示系数b,para(:,3)表示系数c。
price=100;
para=price*para;%价格换算
lasttime=paragen(:,9);%持续时间
lasttime=3*lasttime;
Rud=paragen(:,8);%上下爬坡速率//因题中简化上坡下坡速度相同
H_start=paragen(:,10);%启动成本
J_stop =paragen(:,11);%关停成本
power_gen=paragen(:,2);%发电机对应节点
BaseMVA=100;%参考电压
slack_bus=26;%参考节点



%% 直流潮流下的导纳矩阵节点参数初始化
netpara(:,4)=1./netpara(:,4);  %电抗求倒数成电纳，这个地方只能计算一次，之前这个值多次计算出了问题
Y=zeros(numnodes,numnodes);
YY=zeros(numnodes,numnodes);
% 直流潮流的导纳矩阵计算
for k=1:branch_num
    i=netpara(k,2);%首节点
    j=netpara(k,3);%尾节点
    Y(i,j)=-netpara(k,4);%导纳矩阵中非对角元素
    Y(j,i)= Y(i,j);
end
for k=1:numnodes
    Y(k,k)=-sum(Y(k,:)); %导纳矩阵中的对角元素
end
%再删除掉平衡节点所在的行与列
YY=Y;
Y(slack_bus,:)=[];
Y(:,slack_bus)=[];

%% 各时刻节点负荷
PL=loadcurve(numnodes+1,2:T+1);
pd=loadcurve(1:numnodes,2:T+1);
%%
%优化变量
p          = sdpvar(gennum,T,'full');%24时刻优化的机组实时功率p(i,t)
u           = binvar(gennum,T,'full');%24时刻优化状态变量
costH       = sdpvar(gennum,T,'full');%24时刻优化启动成本
costJ       = sdpvar(gennum,T,'full');%24时刻优化关停成本


%约束条件
st1  = [];

%机组出力上下限约束
for     t = 1:T
    for   i = 1:gennum
         st1 = [st1,u(i,t)*Pmin(i)<=p(i,t)];
         st1 = [st1,p(i,t)<=u(i,t)*Pmax(i)];
    end
end
%爬坡约束
% for t=2:T
%     for i=1:gennum
%         st1=st1+[(p(i,t)-p(i,t-1))<=Rud(i,1)*u(i,t-1)+(u(i,t)-u(i,t-1))*Su(i)+(1-u(i,t))*Pmax(i)];%上坡
%         st1=st1+[(p(i,t-1)-p(i,t))<=Rud(i,1)*u(i,t)+(u(i,t-1)-u(i,t))*Sd(i)+(1-u(i,t-1))*Pmax(i)];%下坡
%     end
% end
for t=2:T
    for i=1:gennum
        st1=st1+[-Ru(i)*Pmax(i)<=p(i,t)-p(i,t-1)];%上坡
        st1=st1+[p(i,t)-p(i,t-1)<=Rd(i)*Pmax(i)];%下坡
    end
end
%启动约束
for t=2:T
    for i=1:gennum
        indicator=u(i,t)-u(i,t-1);%启停时间约束的简化表达式（自己推导的）,indicator为1表示启动，为0表示停止
        range=t:min(T,t+lasttime(i)-1);
        st1=st1+[u(i,range)>=indicator];
    end
end
%停机约束
for t=2:T
    for i=1:gennum
        indicator=u(i,t-1)-u(i,t);%启停时间约束
        range=t:min(T,t+lasttime(i)-1);%特别限制时间上限
        st1=st1+[u(i,range)<=1-indicator];
    end
end
%启停成本约束
for t=1:T   %启停成本零限约束
    for i=1:gennum
        st1=st1+[costH(i,t)>=0];
        st1=st1+[costJ(i,t)>=0];
    end
end
for i=1:gennum  %启停成本条件约束
    for t=2:T
        st1=st1+[costH(i,t)>=H_start(i,1)*(u(i,t)-u(i,t-1))];
        st1=st1+[costJ(i,t)>=J_stop(i,1)*(u(i,t-1)-u(i,t))];
    end
end
% 储能约束
%    C_sto = [C_sto; 0 <= P_s <= 0; 0<= E_s <= 0]; %建模时必须去掉这个约束；测试的时候才用。
bigM = 10^5;
for i = 1:N_s
    st1 = [st1; P_s(i)*H(i)*E_lower(i)<=E_init(i)<=P_s(i)*H(i);];
    E_s(i,1) = E_init(i);
    for t = 1:T
                st1 = [st1; 0<=P_c(i,t)<=P_s(i); 0<=P_dc(i,t)<=P_s(i)];
    end
    for t = 2:T
        E_s(i,t) = E_s(i,t-1)+ P_c(i,t-1)*eff_c(i)-P_dc(i,t-1)/eff_dc(i);
        st1 = [st1; P_s(i)*H(i)*E_lower(i)<=E_s(i,t)<=P_s(i)*H(i)];
    end
end
        st1 = [st1; 0<=P_s(i)<=10000];
% 可再生能源发电约束
w_1 = w_ref(data_r);
w_0 = w_1(1:T);
% w_0 = 0.5*w_0;
for t = 1:T
    P_r_max(:,t) = (data_r(:,2))*w_0(t);
    st1 = [st1; 0<=P_r(:,t)<=P_r_max(:,t)]; %新能源出力小于预测的波动出力
    cur(:,t) =  (data_r(:,2))*w_0(t)-P_r(:,t);
    %    C_r = [C_r; 0<=P_r(:,t)<=0];%测试的时候才用。
end
     st1 = [st1; sum(P_r(:,t))==beta*sum(sum(pd(:,t)))]; %新能源渗透约束
     % st1 = [st1; 0<=beta<=0.5]; %新能源出力小于预测的波动出力
    beta_true = sum(P_r(:,t))/sum(sum(pd(:,t)));
    
% 支路安全约束
theta1=sdpvar(numnodes,T,'full');
jili=sdpvar(branch_num,T,'full');
for t=1:T
    theta1(slack_bus,t)=0;
end
for t=1:T
    for k=1:branch_num 
        m=netpara(k,2);%首端节点
        n=netpara(k,3);%末端节点
        xk=netpara(k,4);%支路k的阻抗值，在前面一步已经变成了导纳值
        jili(k,t)=( theta1(m,t)-theta1(n,t) )*xk;
        st1=[st1,PL_min(k,1)<=( theta1(m,t)-theta1(n,t) )*xk];%支路功率上下限
        st1=[st1,( theta1(m,t)-theta1(n,t) )*xk<=PL_max(k,1)];
    end
end


%负荷平衡约束
        st1=[st1,sum(p)+P_r+P_dc-P_c==sum(pd)];

%目标函数
obj_1 = 0;
obj_2 = 0;
    for  t = 1:T
        for  i = 1:gennum
            obj_2=obj_2+costH(i,t)+costJ(i,t);%加上机组启停产生的开停机成本
            obj_1=obj_1+para(i,2)*(BaseMVA*p(i,t));%煤耗成本
        end
    end
obj_3 = 10000*sum(cur);
obj_4 = 12000000*P_s;
obj_5 = 300*sum(P_dc+P_c);% 因为用标幺值计算 这里都乘了100
     obj=obj_1+obj_2+obj_3+obj_4+obj_5;     


%% 求解
ops1 = sdpsettings('solver', 'cplex','savesolveroutput',1);
% ops1.cplex= cplexoptimset('cplex');
% ops1.cplex.mip.tolerances.absmipgap = 0.01;
result1 = solvesdp(st1,obj,ops1);
solve1=double(obj) ;
p1_double=double(p);
value(P_s)
value(obj)
P_C = P_dc-P_c;

%% 参数规划
% plp = Opt (st1, totalcost1, beta, P_s); % 利用MPT3工具包建模
% solution = plp.solve; % 利用MPT3工具包求解多参数规划，结果存在solution里面
% 
% for i = 1:N_s
%     figure;
%     solution.xopt.fplot('primal', 'position', i);
%     xlabel('beta');
%     ylabel(sprintf('x_%d(beta)', i));
% end


%% 绘制出力曲线
p_P_r = sum(p) + P_r;
subplot(1,2,1)
bar(value(pd)','stack')%阶梯图
% legend('Unit 1','Unit 2','Unit 3','Unit 4','Unit 5','Unit 6');	%在坐标轴上添加图例
subplot(1,2,2)
bar(value(p)','stack')%阶梯图
% legend('Unit 1','Unit 2','Unit 3','Unit 4','Unit 5','Unit 6');	%在坐标轴上添加图例
% stairs(value(p)')
% legend('Unit 1','Unit 2','Unit 3','Unit 4','Unit 5','Unit 6');	%在坐标轴上添加图例
cur_double = double(cur);


set(0,'ShowHiddenHandles','On')
set(gcf,'menubar','figure')
toc
disp(['运行时间: ',num2str(toc)]);
u_double = double(u);
save('MILP_result','u_double')