%λ�˾�����Ϊŷ���Ǻ�λ������
function [Location,Euler]=posematrix2EulerLoca(posematrix)
    Location=posematrix(1:3,4)';
    Euler=rotm2eul(posematrix(1:3,1:3));
end