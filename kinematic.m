%ָ������ʽ����ģ������

theta=[0 0 0 0 0 0];

%���ɶ�����
n=6;
%gt0����
gst0=[1 0 0 -22
    0 1 0 0
    0 0 1 928.8
    0 0 0 1];
w{1}=[0 0 1]';
w{2}=[0 1 0]';
w{3}=[0 1 0]';
w{4}=[0 0 1]';
w{5}=[0 1 0]';
w{6}=[0 0 1]';
r{1}=[0 0 0]';
r{2}=[30 0 308.5]';
r{3}=[30 0 571]';
r{4}=[-22 0 571]';
r{5}=[-22 0 839.5]';
r{6}=[-22 0 928.8]';

zeromartix=[0 0 0 0];
for i=1:n
    % xi{i}=[w{i};cross(r{i},w{i})];%�˶���������������
    wbar{i}=[0 -w{i}(3) w{i}(2)
            w{i}(3) 0 -w{i}(1)
            -w{i}(2) w{i}(1) 0];%���ٶȵ�б�Գƾ���
    v{i}=cross(r{i},w{i});
    xibar{i}=[wbar{i},cross(r{i},w{i});zeromartix];%�˶�����
end

%��ת�������
for i=1:n
    eomigabartheta{i}=eye(3)+wbar{i}*sin(theta(i))+wbar{i}*wbar{i}*(1-cos(theta(i)));
    a{i}=(eye(3)-eomigabartheta{i})*(cross(w{i},v{i}))+theta(i)*w{i}*w{i}'*v{i};
    exibartheta{i}=[eomigabartheta{i},a{i};[0 0 0 1]];
end

gst=1;

for i=1:n
gst=gst*exibartheta{i};%���˶�ѧָ������ʽ
end
gst=gst*gst0
