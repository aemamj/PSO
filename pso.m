clc;
clear;
close all;

%%  تعریف مسئله

global NFE ;

NFE = 0 ;
    
    
CostFunction = @(x) Sphere(x); % تابع هزینه

nVar = 5; %تعداد متغیر تصمیم

VarSize = [1 nVar];  % اندازه ماتریکس تصمیم گیری 

Varmin = -10; % حد پایین متغییر ها
Varmax = 10; % حد بالا 

%% PSO پارامتر

MaxIt = 100 ;  % بیشترین تعداد تکرار یا شرایط توقف

nPop = 20 ;  % اندازه جمعیت 

% W = 1;   % ضریب اینرسی
% Wdamp = 0.99; 
% 
% c1 = 2; % ضریب یادگیری شخصی
% 
% c2 = 1; % ضریب یادگیری جمعی

%
phi1 = 2.05;
phi2 = 2.05;
phi = phi1+phi2;
chi = 2 / ( phi - 2 + sqrt( phi^2 - ( 4 * phi ) ) );

W = chi;   % ضریب اینرسی
Wdamp = 1; 

c1 = chi * phi1 ; % ضریب یادگیری شخصی

c2 = chi * phi2 ; % ضریب یادگیری جمعی

alpha = 0.1 ;

% محدود سرعت
Velmax = (Varmax - Varmin )/10 ;

Velmin = -Velmax ;

%% اماده سازی

empty_particle.Position=[];
empty_particle.Cost=[];
empty_particle.Velocity=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];

particles =repmat(empty_particle,nPop,1);

GlobalBest.Cost = inf ;

for i = 1:nPop 
   
    % آماده سازی موقعیت
    particles(i).Position = unifrnd(Varmin , Varmax ,VarSize) ; 
    
    % آماده سازی سرعت
    particles(i).Velocity = zeros(VarSize);
    
    % آماده سازی ارزیابی
    particles(i).Cost = CostFunction(particles(i).Position);
    
    % آماده سازی بهترین موقعبت
    particles(i).Best.Position = particles(i).Position ;
    
    % آماده سازی بهترین ارزیابی
    particles(i).Best.Cost = particles(i).Cost;
    
    % آماده سازی بهترین خاطره جمعی
    if particles(i).Best.Cost < GlobalBest.Cost
        
        % GlobalBest.Position = particles(i).Best.Position ;
        
        % GlobalBest.Cost = particles(i).Best.Cost ;
        
        GlobalBest = particles(i).Best ;
        
    end
    
end

BestCost=zeros(MaxIt,1);

nfe=zeros(MaxIt,1);

%% حلقه اصلی

for it=1:MaxIt

    for i = 1 : nPop
        
        % بروز رسانی سرعت
        
        particles(i).Velocity = W * particles(i).Velocity ... 
            + c1 * rand(VarSize).* ( particles(i).Best.Position - particles(i).Position ) ...
            + c2 * rand(VarSize).* ( GlobalBest.Position - particles(i).Position );
        
        % اعمال حدود سرعت
        
        m = rand(VarSize);
        particles(i).Velocity = max(particles(i).Velocity , Velmin);
        particles(i).Velocity = min(particles(i).Velocity , Velmax);
        
        % بروز رسانی موقعیت
        
        particles(i).Position = particles(i).Position + particles(i).Velocity ;
        
        % اثر آینه ای سرعت
        
        IsOutside = ( particles(i).Position < Varmin | particles(i).Position > Varmax ) ;
        
        particles(i).Velocity(IsOutside) = -particles(i).Velocity(IsOutside);
        
        
        % اعمال حدود موقعیت
        
        particles(i).Position = max(particles(i).Position , Varmin);
        particles(i).Position = min(particles(i).Position , Varmax);
        
        
        % ارزیابی
        
        particles(i).Cost = CostFunction(particles(i).Position);
        
        % بروزرسانی بهترین خاطره شخصی
        
        if particles(i).Cost < particles(i).Best.Cost
            
            particles(i).Best.Position = particles(i).Position ;
            particles(i).Best.Cost = particles(i).Cost ; 
            
            % بروزرسانی بهترین خاطره جمعی
            
            if particles(i).Best.Cost < GlobalBest.Cost
                
                GlobalBest = particles(i).Best;
                
            end
            
        end
        
        
    end
    
    
    BestCost(it) = GlobalBest.Cost ;
    
    nfe(it)= NFE ;
    
    disp(['It ' num2str( it) ':  Best Cost = ' num2str(BestCost(it) ) ] );
    
    
    W=W*Wdamp;
    
    
    
end


%% خروجی


figure;
%plot(nfe,BestCost,'LineWidth',2);
semilogy(nfe , BestCost,'LineWidth',2);
xlabel('number of function evalution');
ylabel('Best Cost');





