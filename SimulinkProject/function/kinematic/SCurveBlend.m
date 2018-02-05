function [ varargout ] = SCurveBlend( FuncName,vs,ve,v,amax,jmax,steps )
%A function for blending the parameters of the normalized function with S-Curve
%[ u,t ] = SCurveBlend( FuncName,vs,ve,v,amax,jmax,steps )
%  将归一化轨迹函数的自变量参数作S曲线糅合处理的函数
%  功能等效于对轨迹作S型速度规划
% 参考文献《石川_数控系统S曲线加减速规划研究》
%返回的结果是按照需要的插补步数，给出每一步对应的自变量参数uSCurve和时间t


%-------------------函数名是否有效的判定----------%
if isa(FuncName,'function_handle')==0
    error('can''t find such a trajectory funtion ');
end
%----------------------------------------------end--%
varargout{1}=zeros(steps+1,1);
varargout{2}=zeros(steps+1,1);
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
%----------------------------------end--%

%------------变量初始化---------------%
amaxAcc=amax;
amaxDec=amax;
vmax=v;
%--------------------------------------end%

%----------假定存在匀速运动段,v可行,规划S型加减速-------%
 if  ( ve<v) && (vs<v) %判断是否指令速度v 大于始末速度
[SAcc,amaxAcc,t1L,t2L,t3L]=STypeAccPlan(vs,vmax,amaxAcc,jmax);
[SDec,amaxDec,t5L,t6L,t7L]=STypeAccPlan(ve,vmax,amaxDec,jmax); %减速S型规划相当于做反向的S型加速规划, 前提条件是ve<v
 else 
    error('vs and ve shoule be small than v');  
 end
 
 if SAcc+SDec <= L
    Situa = 1;   %指令速度能够达到, 存在匀速段 
 else
    Situa = 2; %指令速度达不到，需要降低指令速度
 end
%-----------------------------------------------------end-% 

%---------------针对各种情况分别做处理分别做处理-------%
while Situa~=0
    switch Situa    
    case 1 %包含有匀速段的情况, 对应论文中的情况 a,c,e,g
        t4L= (L- SAcc-SDec)/vmax;
        [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc,amaxDec,vmax,vs); %调用子函数按照步数要求插补
        Situa =0; %将情况标志置0, 退出情况判断循环
        
        
    case 2 % 不含匀速段且匀加速段与匀减速段同时存在的情况, 对应论文中的情况b及其临界情况 
        t4L=0;
        if vs>ve  %取vs,ve中的较大值，降低最大速度vmax,作情况b及其临界情况的判断
            vmax=vs+amaxAcc^2/jmax;
            vmax_temp=ve+amaxDec^2/jmax;
        elseif vs<ve
            vmax=ve+amaxDec^2/jmax;
            vmax_temp=vs+amaxAcc^2/jmax;
        else 
            vmax=v;
            vmax_temp=ve+amaxDec^2/jmax;
            
            %若是相等的情况, 只能是情况b或h,则直接到情况h和其临界情况
            %在原来的代码中是case 3下的内容，为了能在simulink中使用，改成了直接插入case3的形式
            %Situa=3；
             t4L=0;
            %case 2 中已经做了vs,ve大小的判断,此时只需要取以vs,ve中的较小值,降低vmax值的结果即可
            [SAcc,amaxAcc1,t1L,t2L,t3L]=STypeAccPlan(vs,vmax_temp,amaxAcc,jmax);  %重新规划S段的加减速情况
            [SDec,amaxDec1,t5L,t6L,t7L]=STypeAccPlan(ve,vmax_temp,amaxDec,jmax);
            if SAcc+SDec == L %情况h临界情况
                 [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax_temp,vs);
                 Situa=0;
            elseif SAcc+SDec <L  %情况d或f  若vs=ve, 则是情况b
                [vmax,t1L,t2L,t3L,t4L,t5L,t6L,t7L,amaxAcc1,amaxDec1]=VeloSearchWithDichomize(vs,ve,vmax_temp,vmax,0.001,L,amaxAcc,amaxDec,jmax); 
                                                                                     %在 (vmax_temp,vmax)二分法求vmax  VeloSearchWithDichomize(vs,ve,vDownLimit,vUpLimit,accuracy,L,amaxAcc,amaxDec,jmax)

                [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax,vs);
                 Situa=0;
            else 
                 %情况h
                 [vmax,t1L,t2L,t3L,t4L,t5L,t6L,t7L,amaxAcc1,amaxDec1]=VeloSearchWithDichomize(vs,ve,max(vs,ve),vmax_temp,0.001,L,amaxAcc,amaxDec,jmax); 
                                                                                     %在 (0,vmax_temp)二分法求vmax  VeloSearchWithDichomize(vs,ve,vDownLimit,vUpLimit,accuracy,L,amaxAcc,amaxDec,jmax)

                [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax,vs);
                 Situa=0;
            end
            continue;
        end
        
        [SAcc,amaxAcc1,t1L,t2L,t3L]=STypeAccPlan(vs,vmax,amaxAcc,jmax);  %重新规划S段的加减速情况
        [SDec,amaxDec1,t5L,t6L,t7L]=STypeAccPlan(ve,vmax,amaxDec,jmax);
        if SAcc+SDec == L %情况b临界情况
             [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax,vs);
             Situa=0;
        elseif SAcc+SDec <L  %情况b
%             vmax=-amax^2/2*jmax + sqrt(amax^4-2*jmax*(amax^2*(vs+ve)- jmax*(vs^2+ve^2)-2*amax*jmax*L) )/2*jmax; %公式法求vmax 

           [vmax,t1L,t2L,t3L,t4L,t5L,t6L,t7L,amaxAcc1,amaxDec1]=VeloSearchWithDichomize(vs,ve,vmax,v,0.001,L,amaxAcc,amaxDec,jmax); 
                                                                                 %在 (vmax,v)二分法求vmax  VeloSearchWithDichomize(vs,ve,vDownLimit,vUpLimit,accuracy,L,amaxAcc,amaxDec,jmax)
           
            [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax,vs);
             Situa=0;
        else 
            %情况b被排除,假定情况h继续判断
            %Situa=3; 
             t4L=0;
            %case 2 中已经做了vs,ve大小的判断,此时只需要取以vs,ve中的较小值,降低vmax值的结果即可
            [SAcc,amaxAcc1,t1L,t2L,t3L]=STypeAccPlan(vs,vmax_temp,amaxAcc,jmax);  %重新规划S段的加减速情况
            [SDec,amaxDec1,t5L,t6L,t7L]=STypeAccPlan(ve,vmax_temp,amaxDec,jmax);
            if SAcc+SDec == L %情况h临界情况
                 [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax_temp,vs);
                 Situa=0;
            elseif SAcc+SDec <L  %情况d或f  若vs=ve, 则是情况b
                [vmax,t1L,t2L,t3L,t4L,t5L,t6L,t7L,amaxAcc1,amaxDec1]=VeloSearchWithDichomize(vs,ve,vmax_temp,vmax,0.001,L,amaxAcc,amaxDec,jmax); 
                                                                                     %在 (vmax_temp,vmax)二分法求vmax  VeloSearchWithDichomize(vs,ve,vDownLimit,vUpLimit,accuracy,L,amaxAcc,amaxDec,jmax)

                [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax,vs);
                 Situa=0;
            else 
                 %情况h
                 [vmax,t1L,t2L,t3L,t4L,t5L,t6L,t7L,amaxAcc1,amaxDec1]=VeloSearchWithDichomize(vs,ve,max(vs,ve),vmax_temp,0.001,L,amaxAcc,amaxDec,jmax); 
                                                                                     %在 (0,vmax_temp)二分法求vmax  VeloSearchWithDichomize(vs,ve,vDownLimit,vUpLimit,accuracy,L,amaxAcc,amaxDec,jmax)

                [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax,vs);
                 Situa=0;
            end
        end        
        
    end
    
end
%-----------------------------------------------------end%
end


function [ varargout ] = STypeAccPlan( vStart,vBalance,amax,jmax )
%  S型加速规划的子函数
% [ S, amax_real, t1L,t2L,t3L]=STypePlan(vStart,vBalance,amax,jmax)
%  在指定最大加速度和最大捷度情况下，按照S型速度增加的方式规划
%  从vStart 到vBalance 各段的加速时间长度 
%  并且能够返回此段S型加速实际达到的最大加速度和所走的行程
 if (vBalance-vStart)>(amax^2/jmax)
    t1L=amax/jmax;
    t2L=(vBalance-vStart)/amax-t1L;
    t3L=t1L;
    amax_real=amax;
 elseif (vBalance-vStart)<=(amax^2/jmax)
     t1L=sqrt((vBalance-vStart)/jmax);
     t2L=0;
     t3L=t1L;
     amax_real=sqrt((vBalance-vStart)*jmax);
 else
     error('can''t determine the relationship of vBalance-vStart and amax^2/jmax');
 end
 %-----------计算所走的行程----------%

 
 v1=vStart+0.5*jmax*t1L^2;
 s1=vStart*t1L+1/6*jmax*t1L^3;
 v2=v1+amax_real*t2L;
 s2=s1+v1*t2L+0.5*amax_real*t2L^2;
 v3=v2+amax_real*t3L-0.5*jmax*t3L^2;  %notice: v3 should be equal to vBalance
 s3=s2+v2*t3L+0.5*amax_real*t3L^2-1/6*jmax*t3L^3;
 
 %--------------按要求返回输出的结果-------%
 varargout{1}= s3;
 varargout{2}= amax_real;
 varargout{3}= t1L; 
 varargout{4}= t2L;
 varargout{5}= t3L;
 
end

function [varargout]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc,amaxDec,vmax,vs)
% 按照步数要求,使用S型速度规划插补给出归一化的轨迹自变量的子函数
% [uSCurve,t]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc,amaxDec,vmax,vs)
% if needed, the velocity, accelation at each point can be added in the future   
%       ----------------------------------------2017/9/3--------by K.roc

%------为适配simuink的使用对某些函数作coder的屏蔽-----%

coder.extrinsic('warning');
%----------------------------------------------end--%

if isa(steps,'uint32')==0
    warning('the steps should be an integer');
    steps = uint32(steps);
    steps= double(steps);
end
uSCurve= zeros(steps+1,1);
time=zeros(steps+1,1);
t1=t1L;
t2=t1+t2L;
t3=t2+t3L;
t4=t3+t4L;
t5=t4+t5L;
t6=t5+t6L;
t7=t6+t7L;
tAll= t1L+t2L+t3L+t4L+t5L+t6L+t7L;
v1=vs+0.5*jmax*(t1L^2);
v2=v1+amaxAcc*t2L;
v3=v2+amaxAcc*t3L-0.5*jmax*(t3L^2); %v3应该等于vmax,可以用于验算
% v3=v2+amaxAcc*t3L-0.5*t3L*amaxAcc;
v4=vmax;
v5=v4- 0.5*jmax*t5L^2;
v6=v5-amaxDec*t6L;
% 还可以计算v7并返回，用于验证其是否等于ve
s1=vs*t1+1/6*jmax*t1L^3;
s2=s1+v1*t2L+0.5*amaxAcc*t2L^2;
s3=s2+v2*t3L+0.5*amaxAcc*t3L^2-1/6*jmax*t3L^3;
s4=s3+v3*t4L;
s5=s4+v4*t5L-1/6*jmax*t5L^3;
s6=s5+v5*t6L-0.5*amaxDec*t6L^2;
s7=s6+v6*t7L-0.5*amaxDec*t7L^2+1/6*jmax*t7L^3;
% 还可以计算s7并返回，用于验证其是否等于轨迹长度L

for i =0:1/steps:1
    n= uint32(steps*i+1);
    time(n,1)= i*tAll;
    if time(n,1)>=0 && time(n,1)<t1
        s=vs*time(n,1)+1/6*jmax*time(n,1)^3;
    elseif time(n,1)>=t1 && time(n,1)<t2
        s=s1+v1*(time(n,1)-t1)+0.5*amaxAcc*(time(n,1)-t1)^2;
    elseif time(n,1)>=t2 && time(n,1)<t3
        s=s2+v2*(time(n,1)-t2)+0.5*amaxAcc*(time(n,1)-t2)^2-1/6*jmax*(time(n,1)-t2)^3;
    elseif time(n,1)>=t3 && time(n,1)<t4
        s=s3+v3*(time(n,1)-t3);
    elseif time(n,1)>=t4 && time(n,1)<t5
        s=s4+v4*(time(n,1)-t4)-1/6*jmax*(time(n,1)-t4)^3;
    elseif time(n,1)>=t5 && time(n,1)<t6
        s=s5+v5*(time(n,1)-t5)-0.5*amaxDec*(time(n,1)-t5)^2;
    elseif time(n,1)>=t6 && time(n,1)<t7
        s=s6+v6*(time(n,1)-t6)-0.5*amaxDec*(time(n,1)-t6)^2+1/6*jmax*(time(n,1)-t6)^3;
    else 
        s=s7;
    end
    uSCurve(n,1)=s/s7;
    
end
varargout{1}=uSCurve;
varargout{2}=time;

end
 
function  [varargout]= VeloSearchWithDichomize(vs,ve,vDownLimit,vUpLimit,accuracy,L,amaxAcc,amaxDec,jmax)
% a function for searching reasonable max-velocity with Dichomization method
% [vmax,t1L,t2L,t3L,t4L,t5L,t6L,t7L,amaxAcc,amaxDec] =VeloSearchWithDichomize(vs,ve,vDownLimit,vUpLimit,accuracy,L,amaxAcc,amaxDec)
% 用S型速度规划方法,vs到vmax和vmax到ve的加减速方式,使行程满足精度要求
% 一旦确定了vmax,就可以确定t1L,t2L...t7L,以及amaxAcc和amaxDec,从而完成速度规划
vmax= (vDownLimit+vUpLimit)/2;
t4L=0;
[SAcc,amaxAcc1,t1L,t2L,t3L]=STypeAccPlan(vs,vmax,amaxAcc,jmax);  %重新规划S段的加减速情况
[SDec,amaxDec1,t5L,t6L,t7L]=STypeAccPlan(ve,vmax,amaxDec,jmax);
while abs( (SAcc+SDec)-L) >accuracy
    if  SAcc+SDec >L %行程过多,vmax假设过大,应降低vmax的上限
        vUpLimit = vmax;
    elseif SAcc+SDec <L  %行程太少,vmax假设过小,应提高vmax的下限
       vDownLimit = vmax;
    else 
        error('SAcc+SDec=L, but unsatisfy the accuracy');         
    end
    vmax= (vDownLimit+vUpLimit)/2;
    [SAcc,amaxAcc1,t1L,t2L,t3L]=STypeAccPlan(vs,vmax,amaxAcc,jmax);  %重新规划S段的加减速情况
    [SDec,amaxDec1,t5L,t6L,t7L]=STypeAccPlan(ve,vmax,amaxDec,jmax);
end

varargout{1}=vmax;
varargout{2}=t1L;
varargout{3}=t2L;
varargout{4}=t3L;
varargout{5}=t4L;
varargout{6}=t5L;
varargout{7}=t6L;
varargout{8}=t7L;
varargout{9}=amaxAcc1;
varargout{10}=amaxDec1;


end
