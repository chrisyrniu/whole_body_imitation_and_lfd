x=(100/123):(100/123):100;
%%y=[0 0.9 4.8 24.3 67.6 83.5 92.8 98.5 0];  
pp=csape(x,RERn4,'second');  

X=1:1:100;     
Y=ppval(pp,X);   
plot(X,Y,'c','LineWidth',2); hold on;       

x=(100/123):(100/123):100;
%%y=[0 0.9 4.8 24.3 67.6 83.5 92.8 98.5 0];  
pp=csape(x,RER1n4,'second');     

X=1:1:100;     
Y1=ppval(pp,X);  
plot(X,Y1,'g','LineWidth',2); hold on;

x=(100/123):(100/123):100;
%%y=[0 0.9 4.8 24.3 67.6 83.5 92.8 98.5 0];  
pp=csape(x,RER2n3,'second');     

X=1:1:100;     
Y2=ppval(pp,X);   
plot(X,Y2,'y','LineWidth',2); hold on;

x=(100/123):(100/123):100;
%%y=[0 0.9 4.8 24.3 67.6 83.5 92.8 98.5 0];  
pp=csape(x,RER3n2,'second');     

X=1:1:100;     
Y3=ppval(pp,X);  
plot(X,Y3,'r','LineWidth',2); hold on;

x=(100/123):(100/123):100;
%%y=[0 0.9 4.8 24.3 67.6 83.5 92.8 98.5 0]; 
pp=csape(x,RER4n1,'second');    

X=1:1:100;     
Y4=ppval(pp,X);  
plot(X,Y4,'b','LineWidth',2);

dataDTW(1,:)=[1:1:100,1:1:100,1:1:100,1:1:100,1:1:100];
dataDTW(2,:)=[Y,Y1,Y2,Y3,Y4];

