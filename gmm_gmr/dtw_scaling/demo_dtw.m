[Dist1,D1,k1,w1]=dtw(RER',RER1');
nbw1=size(w1,1);
for i=1:nbw1
    RERn1(i,1)=RER(w1(nbw1+1-i,1),1);
    RER1n1(i,1)=RER1(w1(nbw1+1-i,2),1);
end

[Dist2,D2,k2,w2]=dtw(RERn1',RER2');
nbw2=size(w2,1);
for i=1:nbw2
    RERn2(i,1)=RERn1(w2(nbw2+1-i,1),1);
    RER1n2(i,1)=RER1n1(w2(nbw2+1-i,1),1);
    RER2n1(i,1)=RER2(w2(nbw2+1-i,2),1);
end

[Dist3,D3,k3,w3]=dtw(RERn2',RER3');
nbw3=size(w3,1);
for i=1:nbw3
    RERn3(i,1)=RERn2(w3(nbw3+1-i,1),1);
    RER1n3(i,1)=RER1n2(w3(nbw3+1-i,1),1);
    RER2n2(i,1)=RER2n1(w3(nbw3+1-i,1),1);
    RER3n1(i,1)=RER3(w3(nbw3+1-i,2),1);
end

[Dist4,D4,k4,w4]=dtw(RERn3',RER4');
nbw4=size(w4,1);
for i=1:nbw4
    RERn4(i,1)=RERn3(w4(nbw4+1-i,1),1);
    RER1n4(i,1)=RER1n3(w4(nbw4+1-i,1),1);
    RER2n3(i,1)=RER2n2(w4(nbw4+1-i,1),1);
    RER3n2(i,1)=RER3n1(w4(nbw4+1-i,1),1);
    RER4n1(i,1)=RER4(w4(nbw4+1-i,2),1);
end

m=1:nbw4;
plot(m,RERn4); hold on;
plot(m,RER1n4); hold on;
plot(m,RER2n3); hold on;
plot(m,RER3n2); hold on;
plot(m,RER4n1);