function [ varargout ] = ParabolicBlend( FuncName, VeloMax, AccelerateMax, Steps )
%功能一：[u,t]= ParabolicBlend( FuncName, VeloMax, AccelerateMax, Steps )
%          or (u,t)=ParabolicBlend( FuncName, VeloMax, AccelerateMax, Steps )
%功能二：[u,t,T]= ParabolicBlend( FuncName, VeloMax, AccelerateMax, Steps )
%功能三：[u,t,T,L]= ParabolicBlend( FuncName, VeloMax, AccelerateMax, Steps )
%
%A function for blending the parameters of the normalized function
%  将归一化轨迹函数的自变量参数作抛物线糅合处理的函数
%  功能等效于对轨迹作梯形速度规划

%created by K.roc 2017/8/29
% 输入的参数应该是归一化后的轨迹函数名FuncName、最大允许速度VeloMax、最大加速度AccelerateMax
% 和插补点数目
% 返回的参数是被归一化处理后的抛物线糅合处理的自变量参数 u 及其对应的时间节点

% TODO:: 
% 可能需要考虑始末速度不为零的情况 
%                                 ------K.roc 2017/9/1
%
%-------------------函数名是否有效的判定----------%
if isa(FuncName,'function_handle')==0
    error('can''t find such a trajectory funtion ');
end

%--------------------测量轨迹行程-----------------%
accuracy=0.001; %设定精度
L =0; %初始化轨迹长度
u =0;
[xs,ys,zs]=FuncName(u);
u_step=0.1;
while u<1
    uTemp=u+u_step;
    [xe,ye,ze]=FuncName(uTemp);
    V=[xe,ye,ze]-[xs,ys,zs];
    deltaL=norm(V);
    if  deltaL >accuracy
        u_step=u_step/2;
        continue;
    else
        u=uTemp;
        xs=xe;
        ys=ye;
        zs=ze;
        L=L+deltaL;
    end
    
end


%--------------------计算所需的参数---------------%
Tb=VeloMax./AccelerateMax;
Lb=0.5.*AccelerateMax.*Tb.^2;
%--------------------判定行程是否足够------------%
if  L-2*Lb<=0
    warning('the velocity may be too large to reach or the acceleration was too small');
    Situa=1; %情况1 没有匀速运动的情况
    Lb= 0.5*L;
    Tb= sqrt(2*Lb/AccelerateMax);
    T=2*Tb;  %计算轨迹运行总时间
else
    Situa=2; %情况2 存在匀速运动的情况
    T= 2*Tb+(L-2*Lb)/VeloMax;
    
end

%-------------------按需求插补给出结果----------%
Lb_lamda= Lb/L;
Tb_lamda=Tb/T;
accelerate= 2*Lb_lamda/(Tb_lamda.^2);
if isa(Steps,'uint32')==0
    warning('the Steps should be an integer');
    Steps = uint32(Steps);
    Steps= double(Steps);
end
u= zeros(Steps+1,1);
t= zeros(Steps+1,1);

switch Situa
    case 1
        for i=0: 1/Steps :1
            n=uint32(Steps*i+1);
            if i>=0 && i<=Tb_lamda
                u(n,1)=0.5*accelerate*i^2;
                t(n,1)= i*T;
            elseif i>(1-Tb_lamda) && i<=1
                 u(n,1)=0.5*accelerate*Tb_lamda^2 + accelerate*Tb_lamda*(i-Tb_lamda)-0.5*accelerate*(i+ Tb_lamda -1)^2;
                 t(n,1)= i*T;
            end
        end
        
       
    
    case 2
        for i=0:1/Steps:1
            n=uint32(Steps*i+1);
            if i>=0 && i<=Tb_lamda
                u(n,1)=0.5*accelerate*i^2;
                t(n,1)= i*T;
            elseif (i>Tb_lamda) && (i <= 1-Tb_lamda)
                u(n,1)=0.5*accelerate*Tb_lamda^2 + accelerate*Tb_lamda*(i- Tb_lamda);
                t(n,1)= i*T;
            elseif i>(1-Tb_lamda) && i<=1
                u(n,1)=0.5*accelerate*Tb_lamda^2 + accelerate*Tb_lamda*(i-Tb_lamda)-0.5*accelerate*(i+ Tb_lamda -1)^2;
                t(n,1)= i*T;
            end
            
        end
end

%-----------------------结果输出--------------%
if nargout==1
    varargout{1}=[u,t];
    
elseif nargout==2
    varargout{1}=u;
    varargout{2}=t;

elseif nargout==3
    varargout{1}=u;
    varargout{2}=t;
    varargout{3}=T; %如有需要可以读取轨迹总运行时间

elseif nargout==4
    varargout{1}=u;
    varargout{2}=t;
    varargout{3}=T; %如有需要可以读取轨迹总运行时间
    varargout{3}=L; %如有需要可以读取轨迹长度
end


end

