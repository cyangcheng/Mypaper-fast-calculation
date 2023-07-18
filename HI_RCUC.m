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
H = 2; %������ŵ�ʱ��

%% ������� ע��˳��
% ��������
P_g = sdpvar(N_g,T,'full'); % ���繦��
P_c = sdpvar(N_s,T,'full'); % ���ܳ��
P_dc = sdpvar(N_s,T,'full'); % ���ܷŵ�
E_init = sdpvar(N_s,1,'full'); %���ܳ�ʼ״̬
dem = sdpvar(N_b,T,'full'); % ����
P_s = sdpvar(N_s,1,'full'); % 
E_s = sdpvar(N_s,T,'full');% 
P_r = sdpvar(N_r,T,'full');% ����Դ����
costH = sdpvar(N_g,T,'full');%24ʱ���Ż������ɱ�
costJ = sdpvar(N_g,T,'full');%24ʱ���Ż���ͣ�ɱ�
cur = sdpvar(N_r,T,'full'); % ����Դ������

P_clu = sdpvar(c,T,'full');
O_c = sdpvar(c,T,'full');
SU_c = sdpvar(c,T,'full');
SD_c = sdpvar(c,T,'full');
cost_c = sdpvar(c,T,'full');
SU_all = sdpvar(c,T,'full');
SD_all = sdpvar(c,T,'full');


%% ��������-��׼
% load load_data;
% dem = bus_load;

%% ����Դ����
load P_r_max;
%       bus  P_r
P_r_max =   P_r34+ P_r52+ P_r53+ P_r78+ P_r119+ P_r225 ;  
% P_r_all = P_r;
P_r_max = P_r_max(25345:25440,1);
P_r_max = P_r_max';
 P_r_max = 10*P_r_max;

%% ��������-���졣 
load load_all;
dem = load_all(25345:25440,1);
n1 = 1:4:length(dem);
dem = dem(n1);
dem = dem';

%% ��������Դ����Լ��-��׼
% C_r = [];
% w_1 = w_ref(data_r); 
% w_0 = w_1(1:T);
% w_0 = w_0/150;
% for t = 1:T
%     C_r = [C_r; 0<=P_r(:,t)<=(data_r(:,2))*w_0(t)]; %����Դ����С��Ԥ��Ĳ�������
%     P_r_max(:,t) = (data_r(:,2))*w_0(t);
%     cur(:,t) = (data_r(:,2))*w_0(t)-P_r(:,t);
%     %    C_r = [C_r; 0<=P_r(:,t)<=0];%���Ե�ʱ����á�
% end
%    C_r = [C_r; sum(P_r(:,t))>=beta*sum(sum(dem(:,t)))]; %����Դ����С��Ԥ��Ĳ�������
% %      C_r = [C_r; sum(P_r(:,t))>=beta*sum(sum(dem(:,t)))]; %����Դ����С��Ԥ��Ĳ�������
% %      C_r = [C_r; 0<=beta<=0.5]; %����Դ����С��Ԥ��Ĳ�������
%     beta_true = sum(P_r(:,t))/sum(sum(dem(:,t)));

%% ��������Դ����Լ��-����
 C_r = [];

 for t = 1:T
     C_r = [C_r; P_r(:,t)>=0];
     C_r = [C_r; P_r(:,t)<=P_r_max(:,t)];
     cur(:,t) = P_r_max(:,t)-P_r(:,t);
     %    C_r = [C_r; 0<=P_r(:,t)<=0];%���Ե�ʱ����á�
 end
    C_r = [C_r; sum(P_r(:,t))==beta*sum(dem(:,t))]; %����Դ����С��Ԥ��Ĳ�������
% %      C_r = [C_r; sum(P_r(:,t))>=beta*sum(sum(dem(:,t)))]; %����Դ����С��Ԥ��Ĳ�������
%     C_r = [C_r; 0<=beta<=0.5]; %����Դ����С��Ԥ��Ĳ�������
     beta_true = sum(P_r(:,t))/sum(dem(:,t));
     
% %% PTDF-������Ϊû�и����ڵ�ĸ��������Ⱥ���
% C_PTDF = [];
% for t = 1:T
%     C_PTDF = [C_PTDF; -data_l(:,6) <=( (PTDF(:,data_g(:,1)) * P_g(:,t))...
%         - (PTDF * dem(:,t))...
%         + (PTDF(:,data_r(:,1)) * P_r(:,t))...
%         + (PTDF(:,data_s(:,1)) * (P_dc(:,t)-P_c(:,t)))  )...
%         <= data_l(:,6)];
% end


%% ��Ⱥ������������ޡ����¡���ͣԼ��
%% �����
C_gen = [];
% ��̨�������������Լ��
for     t = 1:T
    for   i = 1:N_g
         C_gen = [C_gen,0<=P_g(i,t)];
         C_gen = [C_gen,P_g(i,t)<=Pmax(i)];
    end
end

for t=2:T
    for i=1:N_g
        C_gen=[C_gen,-Rud(i)*Cap(i)<=P_g(i,t)-P_g(i,t-1)];%����
        C_gen=[C_gen,P_g(i,t)-P_g(i,t-1)<=Rud(i)*Cap(i)];%����
    end
end
%���鼯Ⱥ���Լ��
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

%�������������Լ��
for     t = 1:T
    for   i = 1:c
         C_gen = C_gen+[nanmeda_c(i)*O_c(i,t)<=P_clu(i,t)];
         C_gen = C_gen+[P_clu(i,t)<=O_c(i,t)];
    end
end
%��ͣ�������Ǹ�Լ��
for     t = 1:T
    for   i = 1:c
         C_gen = C_gen+[0<=SU_c(i,t)];
         C_gen = C_gen+[0<=SD_c(i,t)];
    end
end

%����Լ��
for t=2:T
    for i=1:c
        C_gen=C_gen+[-Rd_c(i)*O_c(i,t)<=P_clu(i,t)-P_clu(i,t-1)];%����
        C_gen=C_gen+[P_clu(i,t)-P_clu(i,t-1)<=Ru_c(i)*O_c(i,t)];%����
                    O_c(i,t) = O_c(i,t-1)+SU_c(i,t)-SD_c(i,t);
    end
end

%����Լ��
for t=1:T
    for i=1:c
        for tau = 1:6
            SU_all(i,t) = 0;
            SU_all(i,t) = SU_all(i,t)+SU_c(i,t-tau);
        end
        C_gen=C_gen+[SU_all(i,t)<=O_c(i,t)];
    end
end
%ͣ��Լ��
for t=1:T
    for i=1:c
        for tau = 1:6
            SD_all(i,t) = 0;
            SD_all(i,t) = SD_all(i,t)+SD_c(i,t-tau);
        end
        C_gen=C_gen+[O_c(i,t)<=Ng_c(i)*Cap_c(i)-SD_all(i,t)];
    end
end

% %��ͣ�ɱ�Լ��
for t=1:T   %��ͣ�ɱ�����Լ��
    for i=1:c
    C_gen=C_gen+[0<=cost_c(i,t)];
    cost_c(i,t) = SC_c(i)*SU_c(i,t);
    end
end


%% ����
C_sto = [];
%    C_sto = [C_sto; 0 <= P_s <= 0; 0<= E_s <= 0]; %��ģʱ����ȥ�����Լ�������Ե�ʱ����á�
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



%% ϵͳ����ƽ��
C_bal = [];
for t = 1:T
    C_bal = [C_bal; dem(:,t)>=sum(P_g(:,t))+P_dc(:,t)-P_c(:,t)+P_r(:,t) ];
    C_bal = [C_bal; dem(:,t)<=sum(P_g(:,t))+P_dc(:,t)-P_c(:,t)+P_r(:,t) ];
end

%% Լ����
C = [C_gen;C_sto;C_bal;C_r];
%% Ŀ�꺯���� obj_1: ���гɱ�+startup�ɱ�+shutdown�ɱ�, obj_:�������гɱ�
obj_1=0;%24�������Ż����ܳɱ�
for  i = 1:N_g
    for  t = 1:T
        obj_1=obj_1+35*(P_g(i,t));%ú�ĳɱ�
    end
end
obj_5 = sum(sum(cost_c));%���ϻ�����ͣ�����Ŀ�ͣ���ɱ�
obj_2 = 200*sum(cur);
obj_3 = 120000*P_s;
obj_4 = 3*sum(P_c);
obj = obj_1+obj_2+obj_3+obj_4+obj_5;




%% ���
ops = sdpsettings('verbose',0,'solver','cplex');
result = optimize(C,obj,ops)
value(P_s)
value(obj)
subplot(1,2,1)
% bar(value(P_r)','stack')%����ͼ
bar(value(dem)','stack')%����ͼ
P_C = P_dc-P_c;
subplot(1,2,2)
% bar(value(E_s)','stack')%����ͼ
bar(value(P_g)','stack')%����ͼ
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

 %% ��ʼ��������
% plp = Opt (C, obj, beta, P_clu); % ����MPT3���߰���ģ
% solution = plp.solve; % ����MPT3���߰���������滮���������solution����
% % regions = solution.xopt.Set;% ��ȡÿ��������Ϣ
% % regions.plot;% ������
% figure;
% solution.xopt.fplot('obj','linewidth',3);

toc
disp(['����ʱ��: ',num2str(toc)]);

save('HI_RCUC_result','P_fix')

