function [ varargout ] = ParabolicBlend( FuncName, VeloMax, AccelerateMax, Steps )
%����һ��[u,t]= ParabolicBlend( FuncName, VeloMax, AccelerateMax, Steps )
%          or (u,t)=ParabolicBlend( FuncName, VeloMax, AccelerateMax, Steps )
%���ܶ���[u,t,T]= ParabolicBlend( FuncName, VeloMax, AccelerateMax, Steps )
%��������[u,t,T,L]= ParabolicBlend( FuncName, VeloMax, AccelerateMax, Steps )
%
%A function for blending the parameters of the normalized function
%  ����һ���켣�������Ա����������������ۺϴ���ĺ���
%  ���ܵ�Ч�ڶԹ켣�������ٶȹ滮

%created by K.roc 2017/8/29
% ����Ĳ���Ӧ���ǹ�һ����Ĺ켣������FuncName����������ٶ�VeloMax�������ٶ�AccelerateMax
% �Ͳ岹����Ŀ
% ���صĲ����Ǳ���һ���������������ۺϴ�����Ա������� u �����Ӧ��ʱ��ڵ�

% TODO:: 
% ������Ҫ����ʼĩ�ٶȲ�Ϊ������ 
%                                 ------K.roc 2017/9/1
%
%-------------------�������Ƿ���Ч���ж�----------%
if isa(FuncName,'function_handle')==0
    error('can''t find such a trajectory funtion ');
end

%--------------------�����켣�г�-----------------%
accuracy=0.001; %�趨����
L =0; %��ʼ���켣����
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


%--------------------��������Ĳ���---------------%
Tb=VeloMax./AccelerateMax;
Lb=0.5.*AccelerateMax.*Tb.^2;
%--------------------�ж��г��Ƿ��㹻------------%
if  L-2*Lb<=0
    warning('the velocity may be too large to reach or the acceleration was too small');
    Situa=1; %���1 û�������˶������
    Lb= 0.5*L;
    Tb= sqrt(2*Lb/AccelerateMax);
    T=2*Tb;  %����켣������ʱ��
else
    Situa=2; %���2 ���������˶������
    T= 2*Tb+(L-2*Lb)/VeloMax;
    
end

%-------------------������岹�������----------%
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

%-----------------------������--------------%
if nargout==1
    varargout{1}=[u,t];
    
elseif nargout==2
    varargout{1}=u;
    varargout{2}=t;

elseif nargout==3
    varargout{1}=u;
    varargout{2}=t;
    varargout{3}=T; %������Ҫ���Զ�ȡ�켣������ʱ��

elseif nargout==4
    varargout{1}=u;
    varargout{2}=t;
    varargout{3}=T; %������Ҫ���Զ�ȡ�켣������ʱ��
    varargout{3}=L; %������Ҫ���Զ�ȡ�켣����
end


end

