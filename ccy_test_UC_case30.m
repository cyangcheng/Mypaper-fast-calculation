

clear
clc
tic
yalmip;
Cplex;
tic

%%ϵͳ����
%���в�����������ֵ��ʾ
paragen=xlsread('excel2017','�������');
loadcurve=xlsread('excel2017','��������');
netpara=xlsread('excel2017','�������');
%% ��ģ����
%������
gennum=size(paragen);
gennum=gennum(1,1);
%�ڵ���
numnodes=size(loadcurve);
numnodes=numnodes(1,1)-1;
%ʱ�䷶Χ
T=size(loadcurve);
T=T(1,2)-1;
%% ���ܲ���
% bus   eff_c  eff_dc   E_init E_lower
data_s = [
	6	0.95  0.95  0.5   0.2
];
%% ����Դ����
% bus   capacity
data_r = [
 	6	1.5;
];
branch_num=size(netpara);%�����е�֧·
branch_num=branch_num(1,1);
PL_max=netpara(:,6);%��·��󸺺�
PL_min=netpara(:,7);%��·��С����
limit=paragen(:,3:4);%�������������//limit(:,1)��ʾ���ޣ�limit(:,2)��ʾ����
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
 beta = 0;% ����Դ��͸��
% beta = sdpvar(1);
H = 2; %������ŵ�ʱ��
P_c = sdpvar(N_s,T,'full'); % ���ܳ��
P_dc = sdpvar(N_s,T,'full'); % ���ܷŵ�
E_init = sdpvar(N_s,1,'full'); %���ܳ�ʼ״̬
P_s = sdpvar(N_s,1,'full'); % 
E_s = sdpvar(N_s,T,'full');% 
P_r = sdpvar(N_r,T,'full');% ����Դ����

para=paragen(:,5:7);%�ɱ�ϵ��//para(:,1)��ʾϵ��a,para(:,2)��ʾϵ��b,para(:,3)��ʾϵ��c��
price=100;
para=price*para;%�۸���
lasttime=paragen(:,9);%����ʱ��
lasttime=3*lasttime;
Rud=paragen(:,8);%������������//�����м����������ٶ���ͬ
H_start=paragen(:,10);%�����ɱ�
J_stop =paragen(:,11);%��ͣ�ɱ�
power_gen=paragen(:,2);%�������Ӧ�ڵ�
BaseMVA=100;%�ο���ѹ
slack_bus=26;%�ο��ڵ�



%% ֱ�������µĵ��ɾ���ڵ������ʼ��
netpara(:,4)=1./netpara(:,4);  %�翹�����ɵ��ɣ�����ط�ֻ�ܼ���һ�Σ�֮ǰ���ֵ��μ����������
Y=zeros(numnodes,numnodes);
YY=zeros(numnodes,numnodes);
% ֱ�������ĵ��ɾ������
for k=1:branch_num
    i=netpara(k,2);%�׽ڵ�
    j=netpara(k,3);%β�ڵ�
    Y(i,j)=-netpara(k,4);%���ɾ����зǶԽ�Ԫ��
    Y(j,i)= Y(i,j);
end
for k=1:numnodes
    Y(k,k)=-sum(Y(k,:)); %���ɾ����еĶԽ�Ԫ��
end
%��ɾ����ƽ��ڵ����ڵ�������
YY=Y;
Y(slack_bus,:)=[];
Y(:,slack_bus)=[];

%% ��ʱ�̽ڵ㸺��
PL=loadcurve(numnodes+1,2:T+1);
pd=loadcurve(1:numnodes,2:T+1);
%%
%�Ż�����
p          = sdpvar(gennum,T,'full');%24ʱ���Ż��Ļ���ʵʱ����p(i,t)
u           = binvar(gennum,T,'full');%24ʱ���Ż�״̬����
costH       = sdpvar(gennum,T,'full');%24ʱ���Ż������ɱ�
costJ       = sdpvar(gennum,T,'full');%24ʱ���Ż���ͣ�ɱ�


%Լ������
st1  = [];

%�������������Լ��
for     t = 1:T
    for   i = 1:gennum
         st1 = [st1,u(i,t)*Pmin(i)<=p(i,t)];
         st1 = [st1,p(i,t)<=u(i,t)*Pmax(i)];
    end
end
%����Լ��
% for t=2:T
%     for i=1:gennum
%         st1=st1+[(p(i,t)-p(i,t-1))<=Rud(i,1)*u(i,t-1)+(u(i,t)-u(i,t-1))*Su(i)+(1-u(i,t))*Pmax(i)];%����
%         st1=st1+[(p(i,t-1)-p(i,t))<=Rud(i,1)*u(i,t)+(u(i,t-1)-u(i,t))*Sd(i)+(1-u(i,t-1))*Pmax(i)];%����
%     end
% end
for t=2:T
    for i=1:gennum
        st1=st1+[-Ru(i)*Pmax(i)<=p(i,t)-p(i,t-1)];%����
        st1=st1+[p(i,t)-p(i,t-1)<=Rd(i)*Pmax(i)];%����
    end
end
%����Լ��
for t=2:T
    for i=1:gennum
        indicator=u(i,t)-u(i,t-1);%��ͣʱ��Լ���ļ򻯱��ʽ���Լ��Ƶ��ģ�,indicatorΪ1��ʾ������Ϊ0��ʾֹͣ
        range=t:min(T,t+lasttime(i)-1);
        st1=st1+[u(i,range)>=indicator];
    end
end
%ͣ��Լ��
for t=2:T
    for i=1:gennum
        indicator=u(i,t-1)-u(i,t);%��ͣʱ��Լ��
        range=t:min(T,t+lasttime(i)-1);%�ر�����ʱ������
        st1=st1+[u(i,range)<=1-indicator];
    end
end
%��ͣ�ɱ�Լ��
for t=1:T   %��ͣ�ɱ�����Լ��
    for i=1:gennum
        st1=st1+[costH(i,t)>=0];
        st1=st1+[costJ(i,t)>=0];
    end
end
for i=1:gennum  %��ͣ�ɱ�����Լ��
    for t=2:T
        st1=st1+[costH(i,t)>=H_start(i,1)*(u(i,t)-u(i,t-1))];
        st1=st1+[costJ(i,t)>=J_stop(i,1)*(u(i,t-1)-u(i,t))];
    end
end
% ����Լ��
%    C_sto = [C_sto; 0 <= P_s <= 0; 0<= E_s <= 0]; %��ģʱ����ȥ�����Լ�������Ե�ʱ����á�
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
% ��������Դ����Լ��
w_1 = w_ref(data_r);
w_0 = w_1(1:T);
% w_0 = 0.5*w_0;
for t = 1:T
    P_r_max(:,t) = (data_r(:,2))*w_0(t);
    st1 = [st1; 0<=P_r(:,t)<=P_r_max(:,t)]; %����Դ����С��Ԥ��Ĳ�������
    cur(:,t) =  (data_r(:,2))*w_0(t)-P_r(:,t);
    %    C_r = [C_r; 0<=P_r(:,t)<=0];%���Ե�ʱ����á�
end
     st1 = [st1; sum(P_r(:,t))==beta*sum(sum(pd(:,t)))]; %����Դ��͸Լ��
     % st1 = [st1; 0<=beta<=0.5]; %����Դ����С��Ԥ��Ĳ�������
    beta_true = sum(P_r(:,t))/sum(sum(pd(:,t)));
    
% ֧·��ȫԼ��
theta1=sdpvar(numnodes,T,'full');
jili=sdpvar(branch_num,T,'full');
for t=1:T
    theta1(slack_bus,t)=0;
end
for t=1:T
    for k=1:branch_num 
        m=netpara(k,2);%�׶˽ڵ�
        n=netpara(k,3);%ĩ�˽ڵ�
        xk=netpara(k,4);%֧·k���迹ֵ����ǰ��һ���Ѿ�����˵���ֵ
        jili(k,t)=( theta1(m,t)-theta1(n,t) )*xk;
        st1=[st1,PL_min(k,1)<=( theta1(m,t)-theta1(n,t) )*xk];%֧·����������
        st1=[st1,( theta1(m,t)-theta1(n,t) )*xk<=PL_max(k,1)];
    end
end


%����ƽ��Լ��
        st1=[st1,sum(p)+P_r+P_dc-P_c==sum(pd)];

%Ŀ�꺯��
obj_1 = 0;
obj_2 = 0;
    for  t = 1:T
        for  i = 1:gennum
            obj_2=obj_2+costH(i,t)+costJ(i,t);%���ϻ�����ͣ�����Ŀ�ͣ���ɱ�
            obj_1=obj_1+para(i,2)*(BaseMVA*p(i,t));%ú�ĳɱ�
        end
    end
obj_3 = 10000*sum(cur);
obj_4 = 12000000*P_s;
obj_5 = 300*sum(P_dc+P_c);% ��Ϊ�ñ���ֵ���� ���ﶼ����100
     obj=obj_1+obj_2+obj_3+obj_4+obj_5;     


%% ���
ops1 = sdpsettings('solver', 'cplex','savesolveroutput',1);
% ops1.cplex= cplexoptimset('cplex');
% ops1.cplex.mip.tolerances.absmipgap = 0.01;
result1 = solvesdp(st1,obj,ops1);
solve1=double(obj) ;
p1_double=double(p);
value(P_s)
value(obj)
P_C = P_dc-P_c;

%% �����滮
% plp = Opt (st1, totalcost1, beta, P_s); % ����MPT3���߰���ģ
% solution = plp.solve; % ����MPT3���߰���������滮���������solution����
% 
% for i = 1:N_s
%     figure;
%     solution.xopt.fplot('primal', 'position', i);
%     xlabel('beta');
%     ylabel(sprintf('x_%d(beta)', i));
% end


%% ���Ƴ�������
p_P_r = sum(p) + P_r;
subplot(1,2,1)
bar(value(pd)','stack')%����ͼ
% legend('Unit 1','Unit 2','Unit 3','Unit 4','Unit 5','Unit 6');	%�������������ͼ��
subplot(1,2,2)
bar(value(p)','stack')%����ͼ
% legend('Unit 1','Unit 2','Unit 3','Unit 4','Unit 5','Unit 6');	%�������������ͼ��
% stairs(value(p)')
% legend('Unit 1','Unit 2','Unit 3','Unit 4','Unit 5','Unit 6');	%�������������ͼ��
cur_double = double(cur);


set(0,'ShowHiddenHandles','On')
set(gcf,'menubar','figure')
toc
disp(['����ʱ��: ',num2str(toc)]);
u_double = double(u);
save('MILP_result','u_double')