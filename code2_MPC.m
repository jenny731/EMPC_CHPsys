
clear all 
close all 
clc 
yalmip('clear')


% Model for CHP system
load('Obs_Kal_CHP2.mat') 
load('CHP_sys.mat')   
T1_data = CHP_sys.Inputdata(:,3);
T1_data = T1_data(3659:end);
load('model_CHP_CVA1.mat')  
A = model_CHP_CVA1.A;
B = model_CHP_CVA1.B;
C = model_CHP_CVA1.C;
D = 0; 
nx = size(A,1); 
nu = size(B,2);
ny = size(C,1); 

% Demand of thermal and electrical power
load('Demand.mat')
Thermal_P       = Demand(:,1); 
Electrical_P    = Demand(:,2); 
Ts_demand       = 1; 
t_demand        = 0:Ts_demand:size(Electrical_P,1)-Ts_demand;

% Prices electrical power
load('Prices_Elec.mat')
Ts_ElcP     = 15; 
t_elcP      = Prices_Elec(:,1); 
Price_Elc_T = Prices_Elec(:,2); 

% Prices gas
load('Prices_Gas.mat')
Ts_gas      = 1; 
t_gasP      = Prices_Gas(:,2);
Price_Gas   = Prices_Gas(:,1); 

% Values and profile per one day
Ts_blocking = 15;  
Pr_Gas      = Price_Gas(1:(24*4))'./1000; 
Pr_water    = 1.91;  
Pr_ope      = 5; 
Cost_Sw     = 5; 
k = 1;
for i = 1:(24*4)
    if mod(i,(60/Ts_blocking))== 0
        k = k +1; 
    end
    Pel_consumed_P(i) = Electrical_P(k); 
    Pt_consumed_P(i)  = Thermal_P(k);    
end
Pel_consumed    = 0.1.*Pel_consumed_P; 
Pt_consumed     = 0.1.*Pt_consumed_P;  
Pr_Elec_T       = Price_Elc_T(1:(24*4))'./1000;  

Pel_consumed    = [Pel_consumed Pel_consumed Pel_consumed Pel_consumed Pel_consumed];
Pt_consumed     = [Pt_consumed Pt_consumed Pt_consumed Pt_consumed Pt_consumed];
Pr_Elec_T       = [Pr_Elec_T Pr_Elec_T Pr_Elec_T Pr_Elec_T Pr_Elec_T];
Pr_Gas          = [Pr_Gas Pr_Gas Pr_Gas Pr_Gas Pr_Gas];
tt_plot_15      = 0:Ts_blocking:Ts_blocking*size(Pel_consumed,2)-Ts_blocking;
%%

% Simulation parmeters
Ts_model = 10; 
Ts_sim   = 5;  
Hp_hour  = 24;
H_p      = (60/Ts_sim)*Hp_hour; 
Hp       = (1 + (30/Ts_sim)) +  (30/15) + (Hp_hour-1);
Hs       = 1*((60/5)*Hp_hour);

% Parameters
Cp_water    = 4.180;       
Rho_w       = 1000;        
Fac_conv    = 11.384;      
h_i         = 0.5; 
d           = 0.35;
r           = d/2;
V           = pi()*(r^2)*h_i;
Ucoef       = 0.2; 
Area        = (pi()*d*h_i);
Tref        = 25 + 273.15;

% Range constraint
Fmin    = 0;    %m3/h
Fmax    = 2.4;  %m3/h
Fw_min  = 0;    %L/h
Fw_max  = 2800; %L/h
Pmin    = 0;    %W
Pmax    = 6000; %W
Tmin    = 25;   %C
Tmax    = 80;   %C
Pw_min  = 1;    %kW
Pw_max  = 1000; %kW

% Controller design 
% Variables as sdpvar
F_G     = sdpvar(repmat(1,1,Hp),repmat(1,1,Hp));  
P_sp    = sdpvar(repmat(1,1,Hp),repmat(1,1,Hp));  

% sdpvar initial conditions
x_0             = sdpvar(nx,1); 
Pt_consumed_0   = sdpvar(Hp,1); 
T1_0            = sdpvar(1,1);  
On_off_0        = sdpvar(1,1);  
PT_0            = sdpvar(1,1);  
PT_St           = sdpvar(Hp+1,1); 
PT_St_2         = sdpvar(Hp+1,1); 
P_T             = sdpvar(Hp,1); 
U               = sdpvar(3,Hp); 
Fg_0            = sdpvar(1,1); 


% Variables for constraints and cost function 
constraints = [];
objective   = 0;
objective2  = 0;
objective3  = 0; 
Sum_cost_Fg = 0;
Sum_cost_On = 0; 
Sum_cost_T  = 0; 
Sum_Tracking_P = 0; 
Sum_Benf_Pel = 0; 
Sum_cost_Fw = 0; 

% Initial state
k = 1; 
x           = x_0;
Pthermal    = Pt_consumed_0;  
T1(1)       = T1_0;           
Ts_price    = Ts_sim/60;      
F_W         = 2753.4;         
On_off_i    = On_off_0; 
tt          = 0; 
PT_St(1)    = PT_0;           
epsilon     = 1e-4; 


% Constraints and objective
 for i = 1:Hp
   
    On_Off  = binvar(2,1);
     
    if  tt == 30
        Ts_sim = 15;
        Ts_price    = Ts_sim/60; 
    elseif tt == 60
        Ts_sim = 60;    
        Ts_price    = Ts_sim/60; 
    end

    constraints = [constraints, sum(On_Off) == 1,...
                    2 <= PT_St(i) <= Pw_max,...
                    Tmin <= T1 <= Tmax,...
                    0 <= On_Off(1) <= 1,...
                    0 <= On_Off(2) <= 1,...
                    Fmin <= F_G{i} <= Fmax,...
                    0 <= P_sp{i} <= 6000,...
                    [0;0;Tmin] <= U(:,i) <= [Fmax;F_W;Tmax]];
                
   constraints = [constraints,...
                   implies(On_Off(1),[Fmax/2 <= F_G{i} <= Fmax,...
                   3000 <= P_sp{i} <= Pmax,...
                   U(:,i) == [F_G{i}; F_W; T1]]),...
                   implies(On_Off(2),[F_G{i} == 0,...
                   P_sp{i} == 0,...
                   U(:,i) == [0; 0; T1]])];    
   
    
    YY = [];
    YY =  [YY C*x];        
    for k = 1:(Ts_sim*(60/Ts_model))
           x = A*x + B*U(:,i);
           y = C*x; 
           YY = [YY y]; 
    end
    P_e(i) = YY(1,k);   
    T2(i)  = YY(2,k);   
    
    constraints = [constraints, 10 <= T2(i) <= 100,...
                   -1000 <= P_T(i) <= 1000,...
                    -1000 <= P_e(i) <= 7000];
   
    constraints = [constraints,...
                   implies(On_Off(1),[0.95*P_sp{i} <= P_e(i) <= P_sp{i},...
                   Tmax >= T2(i) >= T1+2,...
                   1000 >= P_T(i)>= 6]),...
                   implies(On_Off(2),[P_T(i)<=0,...
                   P_e(i) <= 2999])];
               
    P_T(i) = ((F_W*(1/1000)*(1/3600))*Rho_w)*Cp_water*((T2(i)+273.15)-(T1+273.15)); 
    Q_loss(i)  =  Ucoef*Area*((T1+273.15)-Tref);  
    PT_St(i+1) = PT_St(i) + Ts_sim*(max(0,P_T(i)) - Pthermal(i) - Q_loss(i));  

    
    if i == Hp
        constraints = [constraints, 2 <= PT_St(i+1) <= Pw_max];
    end
    
    % Operation cost
    Cost_On(i)    = On_Off(1)*(Pr_ope*Ts_price);  
    % Switching frequency 
    On(i) = On_Off(1); 
    if i == 1
        Delta_U(i) = On(i)-On_off_i;
    else
        Delta_U(i) = On(i)-On(i-1);
    end
    Swit(i) = abs(Delta_U(i));   
    Cost_Swit(i) = Swit(i)*(Cost_Sw);   
    
    % Cost of gas        
    Fg_kW(i)      = (F_G{i}*Fac_conv); 
    % Delta Fg
    Fg_del(i)   = F_G{i};
    if i == 1
        Delta_Fg(i) = abs(Fg_del(i)-Fg_0); 
        
        constraints = [constraints,0 <= Delta_Fg(i) <= 2.4...
                       implies((1-Swit(i)),0 <= Delta_Fg(i) <= 0.3)];

    elseif (i > 1) && (tt < 60) 
        Delta_Fg(i) = abs(Fg_del(i)-Fg_del(i-1));         
        
        constraints = [constraints,0 <= Delta_Fg(i) <= 2.4,...
                       0 <= Swit(i) <= 1,...
                       implies((1-Swit(i)),0 <= Delta_Fg(i) <= 0.3)];
    else
        Delta_Fg(i) = abs(Fg_del(i)-Fg_del(i-1));         
        
        constraints = [constraints,0 <= Delta_Fg(i) <= 2.4];

    end 
    
    % Profit for selling energy
    Benf_Pel_2(i) = P_e(i)/1000;   
    tt            = tt + Ts_sim;  
    Ts_price_F(i) = Ts_price;         
      
 end

 Ts_s 	= 5;
 it     = 0;
 h      = 1;
for n = 1:Hp
    
   if  it == 15
        h = h + 1;
        Ts_s = 5;
   elseif  it == 30
        h = h + 1;
        Ts_s = 15;
   elseif (it > 30) && (it < 60)
        h = h + 1;
        Ts_s = 15;
   elseif it == 60
        h = h + 1;
        Ts_s = 60; 
   elseif it > 60
        h = h + 4;
        Ts_s = 60;
   end
    
   Pr_Elec_T_obj(n)     = Pr_Elec_T(h);   
   Pr_Gas_obj(n)        = Pr_Gas(1);      
   Pt_consumed_obj(n)   = Pt_consumed(h); 
   it = it + Ts_s; 
   
end
 
objective5 = (1/6)*(Pr_Elec_T_obj.*Ts_price_F)*Benf_Pel_2';
objective4 = (1/8)*(Pr_Gas_obj.*Ts_price_F)*Fg_kW';
objective  = (1/240)*sum(Cost_On);
objective2 = (1/160)*sum(Cost_Swit);
Obj_EMPC_2  = -(0.1*objective5 - 0.4*objective - 0.3*objective2 - 0.2*objective4);

options     = sdpsettings('solver','cplex','verbose',0);
x_in        = zeros(nx,1);
Pt_con_in   = Pt_consumed_obj';  
T1_in       = T1_data(1);   
On_off_in   = 0; 
PT_in       = 3.5;  
Fg_in       = 0;

%%
% Initial conditions for simulation
p = 1; 
x_OPT   = x_in;
PT_St_OPT(1) = PT_in;
PT_St_pl(1)  = PT_St_OPT(1);
x_est = x_OPT;
PT_St_pl_est(1) = PT_St_pl(1); 
y_est = [];

% Variables for saving data 
Seq_Fg_opt  = [];
Seq_Fw_opt  = [];
Seq_Ps_opt  = [];
Seq_Sw_opt  = [];
Seq_T1_opt  = [];
Seq_Psb_opt = [];
Seq_Pes_opt = [];
Seq_Peb_opt = [];
Seq_Psell_opt = [];
Seq_On_s_opt = [];
Seq_x_OPT   = [];
Seq_Pe_OPT  = [];
Seq_T2_OPT  = [];
Seq_P_T_OPT = [];
Seq_L_OPT   = []; 
Seq_PT_St_OPT   = [];
Seq_Pt_in_OPT   = [];
Seq_Pt_out_OPT  = [];
Seq_PT_con    = [];
Seq_Pr_gas    = [];
Seq_Pr_Pel    = [];
Seq_T2b_OPT   = [];
Save_revenue = [];
Save_Obj = [];
Save_tc = []; 

inc = 6;
f   = 1;
m   = 1; 
Areq    = 0;
Ts_simM = 5; 
t_area  = [0:Ts_simM:15];
tt_sim  = 0; 
Area_T  = [];
it_sim = 0;


for j = 1:Hs
     
        tic
        diagnostics =  optimize([constraints, x_0 == x_in,PT_0 == PT_in,...
                                     Pt_consumed_0 == Pt_con_in, T1_0 == T1_in,Fg_0 == Fg_in,...
                                     On_off_0 == On_off_in],Obj_EMPC_2,options);
        tF = toc
        
        if diagnostics.problem ~= 0
            pause
        end
        
        % Results of optimization
        Fg_opt  = value([F_G{:}]);
        Ps_opt  = value([P_sp{:}]);
        On_opt  = round(value(On));
        Fw_opt  = F_W.*On_opt; 
        T1_opt  = value(T1);
        Pe_opt  = value(P_e);
        T2_opt  = value(T2);
        PT_opt  = (value(P_T));
        PT_sto_opt  = value(PT_St);
        Save_revenue = [Save_revenue value(objective5)];
        Save_Obj = [Save_Obj value(Obj_EMPC_2)];
        Save_tc = [Save_tc tF];
        % To save data for 1 minutes
        Fg_pl  = Fg_opt(1);
        Ps_pl  = Ps_opt(1);
        On_pl  = On_opt(1);
        Fw_pl  = Fw_opt(1);
        T1_pl  = T1_opt;
        Pe_pl  = Pe_opt(1);
        T2_pl  = T2_opt(1);
        PT_pl  = PT_opt(1);
        PT_sto_pl  = PT_sto_opt(1);
        
        % Plant simulation 
        yy_Opt_es = [];
        y_OPT       = C*x_OPT;  
        yy_Opt_es   = [yy_Opt_es y_OPT];  % x(1)
        
        for p = 1:(Ts_simM*(60/Ts_model))
               x_OPT = A*x_OPT + B*[Fg_pl; Fw_pl; T1_pl];
               y_OPT = C*x_OPT;  
               yy_Opt_es = [yy_Opt_es y_OPT]; % x(p+1)
        end
      
        Pe_OPT(j)   = yy_Opt_es(1,p);      % W
        T2_OPT(j)   = yy_Opt_es(2,p);      % C
        T1_OPT(j)   = T1_pl; 
        Fw_s_OPT(j) = Fw_pl*(1/1000)*(1/3600);  %L/h  ---> m3/s
        P_T_OPT(j)      = (Fw_s_OPT(j)*Rho_w)*Cp_water*((T2_OPT(j)+273.15)-(T1_pl+273.15)); % kW
        Q_loss_OPT(j)   =  Ucoef*Area*((T1_pl+273.15)-Tref);
        PT_St_pl(j+1)   = PT_St_pl(j) + Ts_simM*(P_T_OPT(j) - Pt_consumed_obj(1) - Q_loss_OPT(j));
        PT_St_OPT(j)    = PT_St_pl(j);
                

        % Save data
        Seq_Fg_opt  = [Seq_Fg_opt Fg_pl];
        Seq_Fw_opt  = [Seq_Fw_opt Fw_pl];
        Seq_Ps_opt  = [Seq_Ps_opt Ps_pl];
        Seq_Sw_opt  = [Seq_Sw_opt On_pl];
        Seq_T1_opt  = [Seq_T1_opt T1_pl];
        Seq_P_T_OPT   = [Seq_P_T_OPT P_T_OPT(j)];
        Seq_PT_St_OPT = [Seq_PT_St_OPT PT_St_OPT(j)]; 
        Seq_x_OPT    = [Seq_x_OPT x_OPT];
        Seq_Pe_OPT   = [Seq_Pe_OPT y_OPT(1)];
        Seq_T2_OPT   = [Seq_T2_OPT y_OPT(2)];
        Seq_T2b_OPT   = [Seq_T2b_OPT T2_pl];
        
        % Initial condition for the next iteration (optimize)
        yy_es = [];
        y_est = C*x_est;
        yy_es = [yy_es y_est];  %x(1)
        for k = 1:(Ts_simM*(60/Ts_model))
            x_est = Obs_Kal_CHP2.A*x_est + Obs_Kal_CHP2.B*[Fg_pl; Fw_pl; T1_pl] + Obs_Kal_CHP2.L*(yy_Opt_es(:,k)-y_est);
            y_est = Obs_Kal_CHP2.C*x_est;
            yy_es = [yy_es y_est];   %x(k+1)
        end
        Fw_s_est(j) = Fw_pl*(1/1000)*(1/3600);  %L/h  ---> m3/s
        P_T_est(j)       = (Fw_s_est(j)*Rho_w)*Cp_water*((yy_es(2,k)+273.15)-(T1_pl+273.15)); % kW
        Q_loss_est(j)   =  Ucoef*Area*((T1_pl+273.15)-Tref);
        PT_St_pl_est(j+1)   = PT_St_pl_est(j) + Ts_simM*(P_T_est(j) - Pt_consumed_obj(1) - Q_loss_est(j));
        PT_St_est(j)     = PT_St_pl(j);

        x_in        = x_OPT;
        x_in        = x_est;
        PT_in       = PT_St_pl(j+1); 
        PT_in       = PT_St_pl_est(j+1); 
        T1_in       = T1_data((6*5)*(j)+1);   %C
        Fg_in       = Fg_pl; 
        
        if j == 1
            t_area = [0:Ts_simM:15];
            Areq   = trapz(t_area,Pe_opt(1:4));
            Pnow   = Pe_OPT(j); 
            m      = m + 2;
            endf   = 3;   
            Area_T = [Area_T Areq]; 
        elseif j > 1
            if j == m
                t_area_2 = [0:Ts_simM:15];
                Areq_2   = Areq;
                Pnow_2 = [Pnow Pe_OPT(j)];
                t_area = [0:Ts_simM:15];
                Areq   = trapz(t_area,Pe_opt(2:5));
                Pnow   = []; 
                m      = m + 3;
                endf   = 4;
                Area_T = [Area_T Areq]; 
            else 
                endf   = endf-1;
                Pnow   = [Pnow Pe_OPT(j)];
            end
        end        
                
        Seq_PT_con    = [Seq_PT_con Pt_consumed_obj(1)];
        Seq_Pr_gas    = [Seq_Pr_gas Pr_Gas_obj(1)];
        Seq_Pr_Pel    = [Seq_Pr_Pel Pr_Elec_T_obj(1)];
        Ts_s    = 5;
        it_sim  = it_sim + 5;
        
        if mod(j,3)==0
            f   = f+1;
            it_sim  = 0;
        end
        h = f;
        it = it_sim;
        for n = 1:Hp
           if  it == 15
                h = h + 1;
                Ts_s = 5;
           elseif  it == 30
                h = h + 1;
                Ts_s = 15;
           elseif (it > 30) && (it < 60)
                h = h + 1;
                Ts_s = 15;
           elseif it == 60
                h = h + 1;
                Ts_s = 60; 
           elseif it > 60
                h = h + 4;
                Ts_s = 60;
           end
           Pr_Elec_T_obj(n) = Pr_Elec_T(h);
           Pr_Gas_obj(n) = Pr_Gas(1);
           Pt_consumed_obj(n) = Pt_consumed(h);
           it = it + Ts_s;    
        end
        Pt_con_in   = Pt_consumed_obj';
        On_off_in   = On_pl;
        

        % Index for next iteration
        Ts_sim      = 5; 
        Ts_price    = Ts_sim/60; % h
        tt          = 0; 
        objective5 = (1/6)*(Pr_Elec_T_obj.*Ts_price_F)*Benf_Pel_2';
        objective4 = (1/8)*(Pr_Gas_obj.*Ts_price_F)*Fg_kW';
        objective  = (1/240)*sum(Cost_On);
        objective2 = (1/160)*sum(Cost_Swit);
        
        if (j == m-3)
            P_cont_2 = [Pnow_2 P_e(1)];
            P_cont = P_e(1:4);
            objective6_2 = norm((Areq - ( trapz(t_area,P_cont))),2) + norm((Areq_2 - ( trapz(t_area_2,P_cont_2))),2);
            Obj_EMPC_2  = -(0.05*objective5 - 0.35*objective - 0.3*objective2 - 0.1*objective4 - 0.2*(1/90000)*objective6_2);
        else
            P_cont = [Pnow P_e(1:endf)];
            objective6 = norm((Areq - ( trapz(t_area,P_cont))),2);
            Obj_EMPC_2  = -(0.05*objective5 - 0.35*objective - 0.3*objective2 - 0.1*objective4 - 0.2*(1/90000)*objective6);
        end
        tt_sim = tt_sim + Ts_sim;
        
 end