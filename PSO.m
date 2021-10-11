clear;

%���ڸĽ�����Ⱥ�Ż��㷨�����˶�ѧ���

%���Ӧ����[0 pi/2 0 0 0 0]

pd=[-22.0000         0  928.8000];%��е��ĩ��ִ����������λ��
rd=[0,0,0];%��е��ĩ��ִ������������̬(ŷ����)

qn=[0 0 0 0 0 0];%��ʼ�ؽڱ��������õõ��Ľ�ӽ�ԭ�ؽڣ����õ�����⣩
qd=[0 0 0 0 0 0];
% qd;%��е��ĩ��ִ�����ﵽ����λ�õĹؽڱ���



c1=2;
c2=2;%��������
M=500; %����������
D=6;%�����ռ�ά��
N=30;%��ʼ��Ⱥ�������Ŀ
pm=0.1;%�������
lambda=100;%Ŀ�꺯����Ȩֵϵ��
xmax=3.14;
s=100;

%vt����
k=0.5;
vmax=xmax*k;

%��ʼ����Ⱥ
x=(rand(N,D)-0.5)*2*3.14;%��ʼ��λ��(�ؽڱ���)
v=(rand(N,D)-0.5)*2*vmax;%��ʼ���ٶȣ��ؽ�������
pbest=x;

%��ÿ�����ӵ���Ӧ��
for i=1:N
    [pq(i,:),rq(i,:)]=posematrix2EulerLoca(kinematic(x(i,:)));%��������ӵ�����λ�ú���̬
    eadapt(i)=lambda*norm(qd-qn)+norm(pq(i,:)-pd)+s*norm(rq(i,:)-rd);%��Ӧ�ȹ�ʽ
end


%��ʼ��ȫ������
gbest=x(N,:);%����ȫ������
eadaptmin=eadapt(N);%ȫ��������Ӧ��
for i=1:N %�����Ⱥ������Ӧ��
    if eadapt(i)<eadaptmin
        gbest=x(i,:);
        eadaptmin=eadapt(i);
    end
end

%��ѭ������
for t=1:M%��������
    
    w=0.2+.07*rand(); %����Ȩ��
    for i=1:N%���Ӹ���
        v(i,:)=w*v(i,:)+c1*rand()*(pbest(i,:)-x(i,:))+c2*rand()*(gbest-x(i,:));%���¸����ӵ�λ�ú��ٶ�
        for j=1:D
            if v(i,j)>vmax||v(i,j)<(-vmax)%�ж������и��ٶ��Ƿ�ﵽ����ֵ
                v(i,j)=(rand()-0.5)*2*vmax;%�����ʼ��
            end                               
        end   
        x(i,:)=x(i,:)+v(i,:);
        
        %xȡֵ��Χ����
        for j=1:D
            while x(i,j)>xmax
                x(i,j)=xmax;
            end
            while x(i,j)<(-xmax)
                x(i,j)=-xmax;
            end
        end
        
        %�������
        if pm>rand()
                x(i,:)=(rand(1,6)-0.5)*2*xmax;%�����ʼ��
        end
        
        [pq(i,:),rq(i,:)]=posematrix2EulerLoca(kinematic(x(i,:)));%�����ǰ���ӵ�����λ�ú���̬
        ptemp=lambda*norm(qd-qn)+norm(pq(i,:)-pd)+s*norm(rq(i,:)-rd);%��Ӧ�ȹ�ʽ
        
        if ptemp<eadapt(i)%��Ӧ�ȳ���ԭ��Ӧ��
            pbest(i,:)=x(i,:);%���
            eadapt(i)=ptemp;
        end        
    end
    
    %����gbest
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

fprintf('����λ��Ϊ\n');
pd
fprintf('������̬Ϊ\n');
rd
fprintf('��ʼ����Ϊ\n');
qn
fprintf('��õĽ�Ϊ\n');
gbest
fprintf('��Ӧ��Ϊ\n');
eadaptmin
[pd1,rd1]=posematrix2EulerLoca(kinematic(gbest));
fprintf('���Ӧ��λ��Ϊ\n');
pd1
fprintf('���Ӧ����̬Ϊ\n');
rd1

% pq;%��е��ĩ��ִ������λ��
% rq;%��е��ĩ��ִ������ŷ����
% qn;%��ǰʱ�̵Ĺؽڱ���ֵ
