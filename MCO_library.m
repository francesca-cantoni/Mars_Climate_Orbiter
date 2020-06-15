clc; clear; close all;

%% System definition
fprintf("---ANALYSIS WITH TWO SENSORS---\n\n");
A = [0 1; 0 0 ];        % (2,2)
B = [0; 1];             % (1,1)
C = [1 1];              % (1,2)
D = [0; 1];             % (2,1)
sys_prop(A,B,C, 'Properties of OPEN LOOP SYSTEM:');

%% System definition
fprintf("---ANALYSIS WITH ONLY SUNTRACKER SENSOR---\n\n");

A = [0 1; 0 0 ];        % (2,2)
B = [0; 1];             % (1,1)
C = [0 1];              % (1,2)
D = [0; 1];             % (2,1)

sys_prop(A,B,C, 'Properties of OPEN LOOP SYSTEM:');

%% System definition
fprintf("---ANALYSIS WITH ONLY GYROSCOPE SENSOR---\n\n");

A = [0 1; 0 0 ];        % (2,2)
B = [0; 1];             % (1,1)
C = [1 0];              % (1,2)
D = [0; 1];             % (2,1)

[n,m] = size(B);
des_out = 5;    %des_out position

%% Caratheristic of disturbance
r=1; %amplitude of cosine
J=1; %momentum of inertia

%% Analysis of the system in open loop
% Stability, controllability, osservability

sys_prop(A,B,C, 'Properties of OPEN LOOP SYSTEM:');

%% Feedback control law

Q1 = eye(2);
gamma = 1;
Q = gamma * Q1;     %same dimension as the states
R = eye(1);         %same dimension as the inputs

[K,S,e] = lqr(A,B,Q,R);         %K:(1,2)

%% State space observer
[H] = gain_obs(A,C,e);          %H:(2,1)  
                                %e_CL*2--> observer faster than the system

% Q1 = eye(2);
% gamma = 1;
% Q = gamma * Q1;   %same dimension as the states
% R = eye(1);       %same dimension as the inputs
% 
% [Ht,S,egn_val_H] = lqr(A',C',Q,R);
% H = Ht';          %(2,1)

%% Reduced Order Observer

[Hb] = gain_obs(A(2,2),A(1,2),real(e(1,1)));        %Hb:(1,1)
                                                    %e_CL*2--> observer faster than the system

% Q1 = eye(1);
% gamma = 1;
% Q = gamma * Q1;   %same dimension as the states
% R = eye(1);       %same dimension as the inputs
% 
% [Hbt,S,egn_val_H] = lqr(A(2,2)',A(1,2)',Q,R);
% Hb = Hbt';          %(1,1)

%% Implementation of Nu and Nx filters
%y* = des-_out = constant
%A is not invertible (because det(A)=0), so I cannot use the standard formula

As=[A B; C 0];          %(3,3)
Bs=[0; 0; des_out];     %(3,1)         
X=lsqminnorm(As, Bs);   %(3,1)
                        %X array that solves the linear equation As*X = Bs and minimizes the value of norm(As*X-Bs)
Nx=X(1:2,1)/des_out;    %(2,1)
Nu=X(3,1)/des_out;      %(1,1)

%% Implementation of dinamic compensator Ki and K

Ak = [A zeros(2,1);C 0];    %(3,3) 
Bk = [B;0];                 %(3,1)
Ck = [C 0];                 %(1,3)

[nk,mk] = size(Bk);

Q3 = eye(nk);
Qk = gamma * Q3;
Rk = eye(mk);

sys_prop(Ak,Bk,Ck, 'Properties of AUGMENTED SYSTEM FOR DYNAMIC COMPENSATOR: (check controllability)'); %check for controllability

[Kk_big,Sk,ek] = lqr(Ak,Bk,Qk,Rk);      %Kk_big:(1,3)
Kk = Kk_big(1,1:2);                     %(1,2)
Ki = Kk_big(1,3);                       %(1,1)

[Hk] = gain_obs(A,C,ek(1:2,:));

%% Disturbance Observer

Bd = D;     %(2,1)
            %Bd is the matrix that multiplies disturbance d(t) in S (not Bd of Sd system)
            %it is independent from the type of disturbance

%% d(t) = d0 1(t)
% Sd
Ad = 0;     %(1,1)
Cd = 1;     %(1,1)

sys_prop(Ad,0,Cd, 'Properties of DISTURBANCE SYSTEM Sd: (check controllability)'); %check for controllability

% Sa
Aa=[A Bd*Cd;0 0 Ad];        %(3,3)
Ba=[B;0];                   %(3,1)
Ca=[C 0];                   %(1,3)

G = pinv(B)*Bd; %gain for feed-forward disturbance component

sys_prop(Aa,Ba,Ca, 'Properties of AUGMENTED SYSTEM FOR DISTURBANCE OBSERVER: (check observability)'); %check for observability

egn_val_Ha=[-1 -3 -5]; %eigenvalues of Aa-HaCa: they must be Re<0
Hat = place(Aa',Ca',egn_val_Ha);
Ha=Hat';

% Q1_d = eye(3);
% gamma = 1;
% Q_d = gamma * Q1_d;     
% R_d = eye(1);           
% 
% [Hat,S,egn_val_Ha] = lqr(Aa',Ca',Q_d,R_d);
% Ha=Hat';          %(1,1)

%% d(t) = d0 sin(w0t+d1)

w=5;
Ad_s=[0 1; -w^2 0];
Cd_s=[1 0];

Aa_s=[A Bd*Cd_s;zeros(2) Ad_s];
Ba_s=[B;0;0]; 
Ca_s=[C 0 0]; 

sys_prop(Aa_s,Ba_s,Ca_s, 'Properties of AUGMENTED SYSTEM FOR DISTURBANCE sin OBSERVER: (check observability)'); %check for observability

G_s = pinv(B)*Bd;

% 
% Q1_s = eye(3);
% gamma = 1;
% Q_s = gamma * Q1_s;   %same dimension as the states
% R_s = eye(3);       %same dimension as the inputs
% 
% [Ha_s,S_a,egn_val_Ha_s] = lqr(Aa_s',Ca_s',Q_s,R_s);
% Ha_s=Hat_s';          %(1,1)

egn_val_Ha_s=[-1 -3 -5 -8]; %eigenvalues of Aa-HaCa: they must be Re<0
Hat_s = place(Aa_s',Ca_s',egn_val_Ha_s);
Ha_s=Hat_s';


%% Properties of Open-loop system
% This function computes and checks all the main properties of a system, 
% namely the stability, the controllability and the observability.

function [s,c,o] = sys_prop(A,B,C, ST)
    disp (ST);
    sys = ss(A,B,C,[]); 
    
    %% Stability check
    s = isstable(sys); %determine if the system is stable 

    if s
        disp (' The system is stable.');
    else
        disp (' The system is unstable.');
    end

    [stable_part, unstable_part] = stabsep(sys); %subdivide the system into stable and unstable part

    %% Controllability check
    uncontrollable_states = length(sys.A) - rank(ctrb(sys.A,sys.B)); 
    c = (uncontrollable_states == 0); 

    if c
        disp (' The system is fully controllable.');  %Kalman controllability matrix is fully rank
    else
        disp (' The system is not controllable.');

        msg = sprintf ('  It has %d uncontrollable states.', uncontrollable_states);
        fprintf(msg);
        
        % Stabilizability check
        if rank(ctrb(unstable_part.A, unstable_part.B)) == length(unstable_part.A) %if the unstable states are controllable
            c = 1;
            disp (' However, the system is at least stabilizable.');
        else
            disp (' The system is not even stabilizable.');
        end
    end

    %% Obserability check
    unbservable_states = length(sys.A) - rank(obsv(sys.A,sys.C));
    o = (unbservable_states == 0); 

    if o
        disp (' The system is fully observable.'); %Kalman observability matrix is fully rank
    else
        disp (' The system is not observable.');
        msg = sprintf ('  It has %d unobservable states.', unbservable_states);
        fprintf(msg);
        
        % Detectability check
        if rank(obsv(unstable_part.A,unstable_part.C)) == length(unstable_part.A) %if the unstable states are observable
            o = 1;
            disp (' However, the system is at least detectable.');
        else
            disp (' The system is not even detectable.');
        end
    end
    
    fprintf(1, '\n');   
end

%% State Space Observer 
function [H] = gain_obs(A,C,e)
    eig_values = e*2; %because the observer must be faster than the system
	Ht = place(A',C',eig_values);
    H = Ht';
end