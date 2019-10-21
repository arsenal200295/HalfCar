function [Fx,Fy,Mz] = MF5ss_eval(Fz,kappa,alpha,gamma,MF)
% Simplified Symmetric Magic Formula tyre model (based on Pacejka 1996/MF5.2)

% (c) 2015 Igo Besselink - TU Eindhoven Dynamics & Control

if MF.fittyp ~= 5
    error('incorrect Magic Formula Data: MF5ss_eval only works with FITTYP = 5')
    return
end

if Fz < 0
    Fx=0;Fy=0;Mz=0;
    return
end

Fz_scaling = 1;                      % Fz scaling factor
if Fz >= MF.limits.Fz(2);            % if Fz exceeds Fzmax, use Fzmax
    Fz = MF.limits.Fz(2);
elseif Fz <= MF.limits.Fz(1);        % if Fz is below Fzmin, use Fzmin and scale MF results
    Fz_scaling=Fz/MF.limits.Fz(1);
    Fz = MF.limits.Fz(1);
end

if alpha < MF.limits.alpha(1);       % limit side slip angle input
    alpha = MF.limits.alpha(1);
elseif alpha > MF.limits.alpha(2);
    alpha = MF.limits.alpha(2);
end

if kappa < MF.limits.kappa(1);       % limit longitudinal slip input
    kappa = MF.limits.kappa(1);
elseif kappa > MF.limits.kappa(2);
    kappa = MF.limits.kappa(2);
end

if gamma < MF.limits.gamma(1);       % limit inclination angle input
    gamma = MF.limits.gamma(1);
elseif gamma > MF.limits.gamma(2);
    gamma = MF.limits.gamma(2);
end

dfz=(Fz-MF.Fz0)/MF.Fz0; 

% LONGITUDINAL FORCE (Fx)
Cx=MF.long.pCx1;
mu_x=(MF.long.pDx1+MF.long.pDx2.*dfz)*MF.scaling.lmux;
Dx=mu_x.*Fz;
Ex=MF.long.pEx1+MF.long.pEx2*dfz+MF.long.pEx3*dfz.^2;
Ex=min(Ex,1);
Kx=(MF.long.pKx1+MF.long.pKx2*dfz).*exp(MF.long.pKx3*dfz).*Fz*MF.scaling.lKx;
Bx=Kx./(Cx*Dx);

Bxalpha=MF.long.rBx1*cos(atan(MF.long.rBx2*kappa));
Cxalpha=MF.long.rCx1;
Gxalpha=cos(Cxalpha.*atan(Bxalpha.*alpha));

Fx=Dx.*sin(Cx*atan(Bx.*kappa-Ex.*(Bx.*kappa-atan(Bx.*kappa)))).*Gxalpha ;

% LATERAL FORCE (Fy)
gamma_y=gamma*MF.scaling.lgay;
SHy=MF.lat.pHy3*gamma_y; 
alpha_y=alpha+SHy;
SVy=Fz.*(MF.lat.pVy3+MF.lat.pVy4*dfz).*gamma_y*MF.scaling.lmuy;
Cy=MF.lat.pCy1;
mu_y=(MF.lat.pDy1+MF.lat.pDy2*dfz).*(1-MF.lat.pDy3*gamma.^2).*MF.scaling.lmuy;
Dy=mu_y.*Fz;
Ey=(MF.lat.pEy1+MF.lat.pEy2*dfz).*(1-MF.lat.pEy4*gamma.*sign(alpha_y));
Ey=min(Ey,1);
Ky=MF.lat.pKy1*MF.Fz0.*sin(2*atan(Fz./(MF.lat.pKy2.*MF.Fz0))).*(1-MF.lat.pKy3*(abs(gamma)))*MF.scaling.lKy;
By=Ky./(Cy.*Dy);
Fyp=Dy.*sin(Cy.*atan(By.*alpha_y-Ey.*(By.*alpha_y-atan(By.*alpha_y))))+SVy;

Bykappa=MF.lat.rBy1*cos(atan(MF.lat.rBy2*alpha));
Cykappa=MF.lat.rCy1;
Gykappa=cos(Cykappa.*atan(Bykappa.*kappa));

Fy=Gykappa.*Fyp;

% SELF ALIGNING MOMENT (Mz)
gamma_z=gamma*MF.scaling.lgaz;

SHt=(MF.align.qHz3+MF.align.qHz4*dfz).*gamma_z;
alpha_t=alpha+SHt;
alpha_teq=atan(sqrt( tan(alpha_t).^2 + ((Kx./Ky).^2).*kappa.^2 )).*sign(alpha_t);
Bt=(MF.align.qBz1+MF.align.qBz2*dfz+MF.align.qBz3*dfz.^2).*(1+MF.align.qBz5*abs(gamma_z))*MF.scaling.lKy/MF.scaling.lmuy;
Ct=MF.align.qCz1;
Dt=Fz.*(MF.align.qDz1+MF.align.qDz2*dfz).*(1+MF.align.qDz4*gamma_z.^2).*MF.R0*MF.scaling.ltr/MF.Fz0;
Et=(MF.align.qEz1+MF.align.qEz2*dfz+MF.align.qEz3*dfz.^2).*(1+MF.align.qEz5*gamma_z*(2/pi).*atan(Bt.*Ct.*alpha_t));
Et=min(Et,1);
t=Dt.*cos(Ct.*atan(Bt.*alpha_teq-Et.*(Bt.*alpha_teq-atan(Bt.*alpha_teq)))).*cos(alpha);

SHf=SHy+SVy./Ky;
alpha_r=alpha+SHf;
alpha_req=atan(sqrt( tan(alpha_r).^2 + ((Kx./Ky).^2).*kappa.^2 )).*sign(alpha_r);
Br=MF.align.qBz9*MF.scaling.lKy/MF.scaling.lmuy + MF.align.qBz10*By.*Cy;
Dr=Fz.*(MF.align.qDz8+MF.align.qDz9*dfz).*gamma_z*MF.R0*MF.scaling.lres*MF.scaling.lmuy;
Mzr=Dr.*cos(atan(Br.*alpha_req)).*cos(alpha);

s=(MF.align.ssz2*(Fy./MF.Fz0)+(MF.align.ssz3+MF.align.ssz4*dfz).*gamma_z)*MF.R0*MF.scaling.ls;

Mz= -t.*Fy + Mzr + s.*Fx;

Fx=Fx.*Fz_scaling;
Fy=Fy.*Fz_scaling;
Mz=Mz.*Fz_scaling;
