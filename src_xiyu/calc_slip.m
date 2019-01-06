function    [s, sign_Fx] = calc_slip(Vx, omega_R)
%
%   This function calculates the tyre slip which is necessary for
%   determining the tyre-forces. This function has to be adjusted to the
%   tyre-model used. 
% 
%   Vx = longitudinal vehicle velocity
%   omega_R = omega*r_w
%
%   s = tyre slip (between 0 and 1)
%   sign_Fx = sign for resulting force due to tyre slip (-1 or 1)

% error('Add equations for slip here')
%% ver 0.2
Vx_abs = abs(Vx);
omega_R_abs = abs(omega_R);

if abs(Vx) >= abs(omega_R) && Vx ~=0 % 
    
    s = 1-omega_R_abs/Vx_abs;
    
    sign_Fx = -1;
    
elseif abs(omega_R) > abs(Vx)
    
    s = 1-Vx_abs/omega_R_abs;
    
    sign_Fx = 1;
    
else % Vx = omega_R = 0
    
    s = 0;
    
    sign_Fx = 0;
    
end
% if sign(Vx*omega_R) < 0 % opposite directions, must be brake
%     s = 1;
%     sign_Fx = -sign(Vx);
% elseif Vx_abs >= omega_R_abs && Vx ~= 0 % brake
% %     s = (Vx - omega_R)/Vx ;
%     s = (Vx_abs - omega_R_abs)/Vx_abs;
%     sign_Fx = -sign(Vx);
% elseif Vx_abs < omega_R_abs % accelerate
%     s =(omega_R_abs - Vx_abs)/omega_R_abs;
%     sign_Fx = sign(omega_R);
% else % Vx = 0
%     s = 0;
%     sign_Fx = 0;
% end


%% ver 0.1
% if abs(Vx) >= abs(omega_R) && Vx ~=0 % 
%     
%     s = abs( (Vx - omega_R)/Vx );
%     % brake
%     if sign(Vx*omega_R) < 0
%         s = 1;
%     end
%     sign_Fx = -sign(Vx);
%     
% elseif abs(omega_R) > abs(Vx)
%     
%     s = abs( (Vx - omega_R)/omega_R );
%     if sign(Vx*omega_R) > 0
%         % accelerate
%         sign_Fx = sign(Vx);
%     elseif Vx == 0
%         sign_Fx = sign(omega_R);
%     else
%         % brake
%         sign_Fx = -sign(Vx);
%         s = 1;
%     end
%     
% else % Vx = omega_R = 0
%     
%     s = 0;
%     
%     sign_Fx = 1;
%     
% end

