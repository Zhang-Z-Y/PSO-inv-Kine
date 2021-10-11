clear;

%基于改进粒子群优化算法的逆运动学求解

%结果应该是[0 pi/2 0 0 0 0]

pd=[-22.0000         0  928.8000];%机械臂末端执行器的期望位置
rd=[0,0,0];%机械臂末端执行器的期望姿态(欧拉角)

qn=[0 0 0 0 0 0];%初始关节变量（可让得到的解接近原关节，即得到最近解）
qd=[0 0 0 0 0 0];
% qd;%机械臂末端执行器达到期望位置的关节变量



c1=2;
c2=2;%加速因子
M=500; %最大迭代次数
D=6;%搜索空间维数
N=30;%初始化群体个体数目
pm=0.1;%变异概率
lambda=100;%目标函数的权值系数
xmax=3.14;
s=100;

%vt限制
k=0.5;
vmax=xmax*k;

%初始化种群
x=(rand(N,D)-0.5)*2*3.14;%初始化位置(关节变量)
v=(rand(N,D)-0.5)*2*vmax;%初始化速度（关节增量）
pbest=x;

%求每个粒子的适应度
for i=1:N
    [pq(i,:),rq(i,:)]=posematrix2EulerLoca(kinematic(x(i,:)));%求出各粒子的所在位置和姿态
    eadapt(i)=lambda*norm(qd-qn)+norm(pq(i,:)-pd)+s*norm(rq(i,:)-rd);%适应度公式
end


%初始化全局最优
gbest=x(N,:);%设置全局最优
eadaptmin=eadapt(N);%全局最优适应度
for i=1:N %获得种群最优适应度
    if eadapt(i)<eadaptmin
        gbest=x(i,:);
        eadaptmin=eadapt(i);
    end
end

%主循环程序
for t=1:M%迭代次数
    
    w=0.2+.07*rand(); %惯性权重
    for i=1:N%粒子个体
        v(i,:)=w*v(i,:)+c1*rand()*(pbest(i,:)-x(i,:))+c2*rand()*(gbest-x(i,:));%更新各粒子的位置和速度
        for j=1:D
            if v(i,j)>vmax||v(i,j)<(-vmax)%判断粒子中各速度是否达到极限值
                v(i,j)=(rand()-0.5)*2*vmax;%随机初始化
            end                               
        end   
        x(i,:)=x(i,:)+v(i,:);
        
        %x取值范围修正
        for j=1:D
            while x(i,j)>xmax
                x(i,j)=xmax;
            end
            while x(i,j)<(-xmax)
                x(i,j)=-xmax;
            end
        end
        
        %变异操作
        if pm>rand()
                x(i,:)=(rand(1,6)-0.5)*2*xmax;%随机初始化
        end
        
        [pq(i,:),rq(i,:)]=posematrix2EulerLoca(kinematic(x(i,:)));%求出当前粒子的所在位置和姿态
        ptemp=lambda*norm(qd-qn)+norm(pq(i,:)-pd)+s*norm(rq(i,:)-rd);%适应度公式
        
        if ptemp<eadapt(i)%适应度超过原适应度
            pbest(i,:)=x(i,:);%替代
            eadapt(i)=ptemp;
        end        
    end
    
    %更新gbest
    for i=1:N
        if eadapt(i)<eadaptmin
            gbest=pbest(i,:);
            eadaptmin=eadapt(i);
        end
    end        
    testnum(t)=eadaptmin;
end
testnum2=1:M;
plot(testnum2,testnum);

fprintf('期望位置为\n');
pd
fprintf('期望姿态为\n');
rd
fprintf('初始变量为\n');
qn
fprintf('获得的解为\n');
gbest
fprintf('适应度为\n');
eadaptmin
[pd1,rd1]=posematrix2EulerLoca(kinematic(gbest));
fprintf('解对应的位置为\n');
pd1
fprintf('解对应的姿态为\n');
rd1

% pq;%机械臂末端执行器的位置
% rq;%机械臂末端执行器的欧拉角
% qn;%当前时刻的关节变量值
