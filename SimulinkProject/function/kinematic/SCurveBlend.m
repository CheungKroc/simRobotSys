function [ varargout ] = SCurveBlend( FuncName,vs,ve,v,amax,jmax,steps )
%A function for blending the parameters of the normalized function with S-Curve
%[ u,t ] = SCurveBlend( FuncName,vs,ve,v,amax,jmax,steps )
%  ����һ���켣�������Ա���������S�����ۺϴ���ĺ���
%  ���ܵ�Ч�ڶԹ켣��S���ٶȹ滮
% �ο����ס�ʯ��_����ϵͳS���߼Ӽ��ٹ滮�о���
%���صĽ���ǰ�����Ҫ�Ĳ岹����������ÿһ����Ӧ���Ա�������uSCurve��ʱ��t


%-------------------�������Ƿ���Ч���ж�----------%
if isa(FuncName,'function_handle')==0
    error('can''t find such a trajectory funtion ');
end
%----------------------------------------------end--%
varargout{1}=zeros(steps+1,1);
varargout{2}=zeros(steps+1,1);
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
%----------------------------------end--%

%------------������ʼ��---------------%
amaxAcc=amax;
amaxDec=amax;
vmax=v;
%--------------------------------------end%

%----------�ٶ����������˶���,v����,�滮S�ͼӼ���-------%
 if  ( ve<v) && (vs<v) %�ж��Ƿ�ָ���ٶ�v ����ʼĩ�ٶ�
[SAcc,amaxAcc,t1L,t2L,t3L]=STypeAccPlan(vs,vmax,amaxAcc,jmax);
[SDec,amaxDec,t5L,t6L,t7L]=STypeAccPlan(ve,vmax,amaxDec,jmax); %����S�͹滮�൱���������S�ͼ��ٹ滮, ǰ��������ve<v
 else 
    error('vs and ve shoule be small than v');  
 end
 
 if SAcc+SDec <= L
    Situa = 1;   %ָ���ٶ��ܹ��ﵽ, �������ٶ� 
 else
    Situa = 2; %ָ���ٶȴﲻ������Ҫ����ָ���ٶ�
 end
%-----------------------------------------------------end-% 

%---------------��Ը�������ֱ�������ֱ�������-------%
while Situa~=0
    switch Situa    
    case 1 %���������ٶε����, ��Ӧ�����е���� a,c,e,g
        t4L= (L- SAcc-SDec)/vmax;
        [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc,amaxDec,vmax,vs); %�����Ӻ������ղ���Ҫ��岹
        Situa =0; %�������־��0, �˳�����ж�ѭ��
        
        
    case 2 % �������ٶ����ȼ��ٶ����ȼ��ٶ�ͬʱ���ڵ����, ��Ӧ�����е����b�����ٽ���� 
        t4L=0;
        if vs>ve  %ȡvs,ve�еĽϴ�ֵ����������ٶ�vmax,�����b�����ٽ�������ж�
            vmax=vs+amaxAcc^2/jmax;
            vmax_temp=ve+amaxDec^2/jmax;
        elseif vs<ve
            vmax=ve+amaxDec^2/jmax;
            vmax_temp=vs+amaxAcc^2/jmax;
        else 
            vmax=v;
            vmax_temp=ve+amaxDec^2/jmax;
            
            %������ȵ����, ֻ�������b��h,��ֱ�ӵ����h�����ٽ����
            %��ԭ���Ĵ�������case 3�µ����ݣ�Ϊ������simulink��ʹ�ã��ĳ���ֱ�Ӳ���case3����ʽ
            %Situa=3��
             t4L=0;
            %case 2 ���Ѿ�����vs,ve��С���ж�,��ʱֻ��Ҫȡ��vs,ve�еĽ�Сֵ,����vmaxֵ�Ľ������
            [SAcc,amaxAcc1,t1L,t2L,t3L]=STypeAccPlan(vs,vmax_temp,amaxAcc,jmax);  %���¹滮S�εļӼ������
            [SDec,amaxDec1,t5L,t6L,t7L]=STypeAccPlan(ve,vmax_temp,amaxDec,jmax);
            if SAcc+SDec == L %���h�ٽ����
                 [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax_temp,vs);
                 Situa=0;
            elseif SAcc+SDec <L  %���d��f  ��vs=ve, �������b
                [vmax,t1L,t2L,t3L,t4L,t5L,t6L,t7L,amaxAcc1,amaxDec1]=VeloSearchWithDichomize(vs,ve,vmax_temp,vmax,0.001,L,amaxAcc,amaxDec,jmax); 
                                                                                     %�� (vmax_temp,vmax)���ַ���vmax  VeloSearchWithDichomize(vs,ve,vDownLimit,vUpLimit,accuracy,L,amaxAcc,amaxDec,jmax)

                [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax,vs);
                 Situa=0;
            else 
                 %���h
                 [vmax,t1L,t2L,t3L,t4L,t5L,t6L,t7L,amaxAcc1,amaxDec1]=VeloSearchWithDichomize(vs,ve,max(vs,ve),vmax_temp,0.001,L,amaxAcc,amaxDec,jmax); 
                                                                                     %�� (0,vmax_temp)���ַ���vmax  VeloSearchWithDichomize(vs,ve,vDownLimit,vUpLimit,accuracy,L,amaxAcc,amaxDec,jmax)

                [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax,vs);
                 Situa=0;
            end
            continue;
        end
        
        [SAcc,amaxAcc1,t1L,t2L,t3L]=STypeAccPlan(vs,vmax,amaxAcc,jmax);  %���¹滮S�εļӼ������
        [SDec,amaxDec1,t5L,t6L,t7L]=STypeAccPlan(ve,vmax,amaxDec,jmax);
        if SAcc+SDec == L %���b�ٽ����
             [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax,vs);
             Situa=0;
        elseif SAcc+SDec <L  %���b
%             vmax=-amax^2/2*jmax + sqrt(amax^4-2*jmax*(amax^2*(vs+ve)- jmax*(vs^2+ve^2)-2*amax*jmax*L) )/2*jmax; %��ʽ����vmax 

           [vmax,t1L,t2L,t3L,t4L,t5L,t6L,t7L,amaxAcc1,amaxDec1]=VeloSearchWithDichomize(vs,ve,vmax,v,0.001,L,amaxAcc,amaxDec,jmax); 
                                                                                 %�� (vmax,v)���ַ���vmax  VeloSearchWithDichomize(vs,ve,vDownLimit,vUpLimit,accuracy,L,amaxAcc,amaxDec,jmax)
           
            [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax,vs);
             Situa=0;
        else 
            %���b���ų�,�ٶ����h�����ж�
            %Situa=3; 
             t4L=0;
            %case 2 ���Ѿ�����vs,ve��С���ж�,��ʱֻ��Ҫȡ��vs,ve�еĽ�Сֵ,����vmaxֵ�Ľ������
            [SAcc,amaxAcc1,t1L,t2L,t3L]=STypeAccPlan(vs,vmax_temp,amaxAcc,jmax);  %���¹滮S�εļӼ������
            [SDec,amaxDec1,t5L,t6L,t7L]=STypeAccPlan(ve,vmax_temp,amaxDec,jmax);
            if SAcc+SDec == L %���h�ٽ����
                 [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax_temp,vs);
                 Situa=0;
            elseif SAcc+SDec <L  %���d��f  ��vs=ve, �������b
                [vmax,t1L,t2L,t3L,t4L,t5L,t6L,t7L,amaxAcc1,amaxDec1]=VeloSearchWithDichomize(vs,ve,vmax_temp,vmax,0.001,L,amaxAcc,amaxDec,jmax); 
                                                                                     %�� (vmax_temp,vmax)���ַ���vmax  VeloSearchWithDichomize(vs,ve,vDownLimit,vUpLimit,accuracy,L,amaxAcc,amaxDec,jmax)

                [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax,vs);
                 Situa=0;
            else 
                 %���h
                 [vmax,t1L,t2L,t3L,t4L,t5L,t6L,t7L,amaxAcc1,amaxDec1]=VeloSearchWithDichomize(vs,ve,max(vs,ve),vmax_temp,0.001,L,amaxAcc,amaxDec,jmax); 
                                                                                     %�� (0,vmax_temp)���ַ���vmax  VeloSearchWithDichomize(vs,ve,vDownLimit,vUpLimit,accuracy,L,amaxAcc,amaxDec,jmax)

                [varargout{1}, varargout{2}]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc1,amaxDec1,vmax,vs);
                 Situa=0;
            end
        end        
        
    end
    
end
%-----------------------------------------------------end%
end


function [ varargout ] = STypeAccPlan( vStart,vBalance,amax,jmax )
%  S�ͼ��ٹ滮���Ӻ���
% [ S, amax_real, t1L,t2L,t3L]=STypePlan(vStart,vBalance,amax,jmax)
%  ��ָ�������ٶȺ����ݶ�����£�����S���ٶ����ӵķ�ʽ�滮
%  ��vStart ��vBalance ���εļ���ʱ�䳤�� 
%  �����ܹ����ش˶�S�ͼ���ʵ�ʴﵽ�������ٶȺ����ߵ��г�
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
 %-----------�������ߵ��г�----------%

 
 v1=vStart+0.5*jmax*t1L^2;
 s1=vStart*t1L+1/6*jmax*t1L^3;
 v2=v1+amax_real*t2L;
 s2=s1+v1*t2L+0.5*amax_real*t2L^2;
 v3=v2+amax_real*t3L-0.5*jmax*t3L^2;  %notice: v3 should be equal to vBalance
 s3=s2+v2*t3L+0.5*amax_real*t3L^2-1/6*jmax*t3L^3;
 
 %--------------��Ҫ�󷵻�����Ľ��-------%
 varargout{1}= s3;
 varargout{2}= amax_real;
 varargout{3}= t1L; 
 varargout{4}= t2L;
 varargout{5}= t3L;
 
end

function [varargout]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc,amaxDec,vmax,vs)
% ���ղ���Ҫ��,ʹ��S���ٶȹ滮�岹������һ���Ĺ켣�Ա������Ӻ���
% [uSCurve,t]= STypeInterpolate(steps,t1L,t2L,t3L,t4L,t5L,t6L,t7L,jmax,amaxAcc,amaxDec,vmax,vs)
% if needed, the velocity, accelation at each point can be added in the future   
%       ----------------------------------------2017/9/3--------by K.roc

%------Ϊ����simuink��ʹ�ö�ĳЩ������coder������-----%

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
v3=v2+amaxAcc*t3L-0.5*jmax*(t3L^2); %v3Ӧ�õ���vmax,������������
% v3=v2+amaxAcc*t3L-0.5*t3L*amaxAcc;
v4=vmax;
v5=v4- 0.5*jmax*t5L^2;
v6=v5-amaxDec*t6L;
% �����Լ���v7�����أ�������֤���Ƿ����ve
s1=vs*t1+1/6*jmax*t1L^3;
s2=s1+v1*t2L+0.5*amaxAcc*t2L^2;
s3=s2+v2*t3L+0.5*amaxAcc*t3L^2-1/6*jmax*t3L^3;
s4=s3+v3*t4L;
s5=s4+v4*t5L-1/6*jmax*t5L^3;
s6=s5+v5*t6L-0.5*amaxDec*t6L^2;
s7=s6+v6*t7L-0.5*amaxDec*t7L^2+1/6*jmax*t7L^3;
% �����Լ���s7�����أ�������֤���Ƿ���ڹ켣����L

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
% ��S���ٶȹ滮����,vs��vmax��vmax��ve�ļӼ��ٷ�ʽ,ʹ�г����㾫��Ҫ��
% һ��ȷ����vmax,�Ϳ���ȷ��t1L,t2L...t7L,�Լ�amaxAcc��amaxDec,�Ӷ�����ٶȹ滮
vmax= (vDownLimit+vUpLimit)/2;
t4L=0;
[SAcc,amaxAcc1,t1L,t2L,t3L]=STypeAccPlan(vs,vmax,amaxAcc,jmax);  %���¹滮S�εļӼ������
[SDec,amaxDec1,t5L,t6L,t7L]=STypeAccPlan(ve,vmax,amaxDec,jmax);
while abs( (SAcc+SDec)-L) >accuracy
    if  SAcc+SDec >L %�г̹���,vmax�������,Ӧ����vmax������
        vUpLimit = vmax;
    elseif SAcc+SDec <L  %�г�̫��,vmax�����С,Ӧ���vmax������
       vDownLimit = vmax;
    else 
        error('SAcc+SDec=L, but unsatisfy the accuracy');         
    end
    vmax= (vDownLimit+vUpLimit)/2;
    [SAcc,amaxAcc1,t1L,t2L,t3L]=STypeAccPlan(vs,vmax,amaxAcc,jmax);  %���¹滮S�εļӼ������
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
