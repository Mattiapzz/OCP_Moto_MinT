alg_constr_Fz := [Fzf(t) = -Kp__f*(L__b*sin(theta(t)+epsilon)*cos(phi(t))+
x__off*sin(theta(t)+epsilon)*cos(phi(t))+s__f__00*cos(theta(t)+epsilon)*cos(
phi(t))-s__fs*cos(theta(t)+epsilon)*cos(phi(t))+h(t)*cos(phi(t))-rf*cos(phi(t)
)+rtf*cos(phi(t))-rtf)+Cp__f*(diff(phi(t),t)*sin(phi(t))*sin(theta(t)+epsilon)
*L__b-cos(theta(t)+epsilon)*diff(theta(t),t)*cos(phi(t))*L__b+s__f__00*sin(
theta(t)+epsilon)*diff(theta(t),t)*cos(phi(t))-s__fs*sin(theta(t)+epsilon)*
diff(theta(t),t)*cos(phi(t))+x__off*sin(theta(t)+epsilon)*sin(phi(t))*diff(phi
(t),t)-x__off*cos(theta(t)+epsilon)*diff(theta(t),t)*cos(phi(t))+s__f__00*cos(
theta(t)+epsilon)*sin(phi(t))*diff(phi(t),t)-s__fs*cos(theta(t)+epsilon)*sin(
phi(t))*diff(phi(t),t)+h(t)*sin(phi(t))*diff(phi(t),t)-rf*sin(phi(t))*diff(phi
(t),t)+rtf*sin(phi(t))*diff(phi(t),t)-diff(h(t),t)*cos(phi(t)))+delta(t)*(rf*
diff(phi(t),t)*cos(phi(t))*sin(theta(t)+epsilon)*Cp__f-rtf*diff(phi(t),t)*cos(
phi(t))*sin(theta(t)+epsilon)*Cp__f+rf*cos(theta(t)+epsilon)*diff(theta(t),t)*
sin(phi(t))*Cp__f-rtf*cos(theta(t)+epsilon)*diff(theta(t),t)*sin(phi(t))*Cp__f
-x__off*diff(phi(t),t)*cos(phi(t))*Cp__f+rf*sin(phi(t))*sin(theta(t)+epsilon)*
Kp__f-rtf*sin(phi(t))*sin(theta(t)+epsilon)*Kp__f-x__off*sin(phi(t))*Kp__f)+
Cp__f*diff(delta(t),t)*(rf*sin(theta(t)+epsilon)-rtf*sin(theta(t)+epsilon)-
x__off)*sin(phi(t)), Fzr(t) = Kp__r*(cos(phi(t))*sin(theta(t)+epsilon)*cos(
eta__00)*L__swa-cos(phi(t))*cos(theta(t)+epsilon)*sin(eta__00)*L__swa-h(t)*cos
(phi(t))+cos(phi(t))*rr-rtr*cos(phi(t))+rtr)-Cp__r*(diff(phi(t),t)*sin(phi(t))
*sin(theta(t)+epsilon)*cos(eta__00)*L__swa-cos(theta(t)+epsilon)*diff(theta(t)
,t)*cos(phi(t))*cos(eta__00)*L__swa-diff(theta(t),t)*cos(phi(t))*sin(theta(t)+
epsilon)*sin(eta__00)*L__swa-cos(theta(t)+epsilon)*diff(phi(t),t)*sin(phi(t))*
sin(eta__00)*L__swa-h(t)*sin(phi(t))*diff(phi(t),t)+rr*sin(phi(t))*diff(phi(t)
,t)-rtr*sin(phi(t))*diff(phi(t),t)+diff(h(t),t)*cos(phi(t)))];
alg_constr_tyre_forces := [Fxf(t) = (pDx2__f*(Fzf(t)-Fz0)/Fz0+pDx1__f)*
lambda__mu__x__f*Fzf(t)*sin(pCx1__f*lambda__C__x__f*arctan(Fzf(t)*(pKx2__f*(
Fzf(t)-Fz0)/Fz0+pKx1__f)*exp(pKx3__f*(Fzf(t)-Fz0)/Fz0)*lambda__K__x__f/(
pCx1__f*lambda__C__x__f*(pDx2__f*(Fzf(t)-Fz0)/Fz0+pDx1__f)*lambda__mu__x__f*
Fzf(t)+epsilon__x__f)*lambda__f(t))), Fxr(t) = (pDx2__r*(Fzr(t)-Fz0)/Fz0+
pDx1__r)*lambda__mu__x__r*Fzr(t)*sin(pCx1__r*lambda__C__x__r*arctan(Fzr(t)*(
pKx2__r*(Fzr(t)-Fz0)/Fz0+pKx1__r)*exp(pKx3__r*(Fzr(t)-Fz0)/Fz0)*
lambda__K__x__r/(pCx1__r*lambda__C__x__r*(pDx2__r*(Fzr(t)-Fz0)/Fz0+pDx1__r)*
lambda__mu__x__r*Fzr(t)+epsilon__x__r)*lambda__r(t))), Fyf(t) = (Fzf(t)^2*
d4__f^2/(d7__f*(-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1)^2)^(1/2)*sin(d8*
arctan((d1__f*Fz0__f+d2__f*(Fzf(t)-Fz0__f))/(d5__f*(-sin(theta(t)+epsilon)*
delta(t)+phi(t))^2+1)/d8/d4__f/Fzf(t)*(d7__f*(-sin(theta(t)+epsilon)*delta(t)+
phi(t))^2+1)*(d4__f*Fzf(t)/(d7__f*(-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1
)/(Fzf(t)^2*d4__f^2/(d7__f*(-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1)^2)^(1
/2)*(alpha__f(t)-d3__f*Fzf(t)*(-sin(theta(t)+epsilon)*delta(t)+phi(t))/(d1__f*
Fz0__f+d2__f*(Fzf(t)-Fz0__f))*(d5__f*(-sin(theta(t)+epsilon)*delta(t)+phi(t))^
2+1))-d6__f*(-sin(theta(t)+epsilon)*delta(t)+phi(t))*(Fzf(t)^2*d4__f^2/(d7__f*
(-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1)^2)^(1/2)/d4__f*(d7__f*(-sin(
theta(t)+epsilon)*delta(t)+phi(t))^2+1)/(d1__f*Fz0__f+d2__f*(Fzf(t)-Fz0__f))*(
d5__f*(-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1))))+d6__f*(-sin(theta(t)+
epsilon)*delta(t)+phi(t))*(Fzf(t)^2*d4__f^2/(d7__f*(-sin(theta(t)+epsilon)*
delta(t)+phi(t))^2+1)^2)^(1/2)/d4__f*(d7__f*(-sin(theta(t)+epsilon)*delta(t)+
phi(t))^2+1), Fyr(t) = (Fzr(t)^2*d4__r^2/(d7__r*phi(t)^2+1)^2)^(1/2)*sin(d8*
arctan((d1__r*Fz0__r+d2__r*(Fzr(t)-Fz0__r))/(d5__r*phi(t)^2+1)/d8/d4__r/Fzr(t)
*(d7__r*phi(t)^2+1)*(d4__r*Fzr(t)/(d7__r*phi(t)^2+1)/(Fzr(t)^2*d4__r^2/(d7__r*
phi(t)^2+1)^2)^(1/2)*(alpha__r(t)-d3__r*Fzr(t)*phi(t)/(d1__r*Fz0__r+d2__r*(Fzr
(t)-Fz0__r))*(d5__r*phi(t)^2+1))-d6__r*phi(t)*(Fzr(t)^2*d4__r^2/(d7__r*phi(t)^
2+1)^2)^(1/2)/d4__r*(d7__r*phi(t)^2+1)/(d1__r*Fz0__r+d2__r*(Fzr(t)-Fz0__r))*(
d5__r*phi(t)^2+1))))+d6__r*phi(t)*(Fzr(t)^2*d4__r^2/(d7__r*phi(t)^2+1)^2)^(1/2
)/d4__r*(d7__r*phi(t)^2+1), Mzf(t) = -e1__f*Fzf(t)/(d1__f*Fz0__f+d2__f*(Fzf(t)
-Fz0__f))*cos(e10/(e5*(-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1)*arctan(e7*
d4__f*Fzf(t)/(d7__f*(-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1)/(Fzf(t)^2*
d4__f^2/(d7__f*(-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1)^2)^(1/2)*alpha__f
(t)))/(e5*(-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1)*(Fzf(t)^2*d4__f^2/(
d7__f*(-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1)^2)^(1/2)*sin(d8*arctan((
d1__f*Fz0__f+d2__f*(Fzf(t)-Fz0__f))/(d5__f*(-sin(theta(t)+epsilon)*delta(t)+
phi(t))^2+1)/d8/(Fzf(t)^2*d4__f^2/(d7__f*(-sin(theta(t)+epsilon)*delta(t)+phi(
t))^2+1)^2)^(1/2)*alpha__f(t)))+e2__f*Fzf(t)*arctan(e6*(-sin(theta(t)+epsilon)
*delta(t)+phi(t)))/e6*cos(e10/(e5*(-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1
)*arctan(e9/(e4*(-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1)*d4__f*Fzf(t)/(
d7__f*(-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1)/(Fzf(t)^2*d4__f^2/(d7__f*(
-sin(theta(t)+epsilon)*delta(t)+phi(t))^2+1)^2)^(1/2)*alpha__f(t)))+rtf*Fxf(t)
*tan(sin(theta(t)+epsilon)*delta(t)-phi(t)), Mzr(t) = -e1__r*Fzr(t)/(d1__r*
Fz0__r+d2__r*(Fzr(t)-Fz0__r))*cos(e10/(e5*phi(t)^2+1)*arctan(e7*d4__r*Fzr(t)/(
d7__r*phi(t)^2+1)/(Fzr(t)^2*d4__r^2/(d7__r*phi(t)^2+1)^2)^(1/2)*alpha__r(t)))/
(e5*phi(t)^2+1)*(Fzr(t)^2*d4__r^2/(d7__r*phi(t)^2+1)^2)^(1/2)*sin(d8*arctan((
d1__r*Fz0__r+d2__r*(Fzr(t)-Fz0__r))/(d5__r*phi(t)^2+1)/d8/(Fzr(t)^2*d4__r^2/(
d7__r*phi(t)^2+1)^2)^(1/2)*alpha__r(t)))+e2__r*Fzr(t)*arctan(e6*phi(t))/e6*cos
(e10/(e5*phi(t)^2+1)*arctan(e9/(e4*phi(t)^2+1)*d4__r*Fzr(t)/(d7__r*phi(t)^2+1)
/(Fzr(t)^2*d4__r^2/(d7__r*phi(t)^2+1)^2)^(1/2)*alpha__r(t)))-rtr*Fxr(t)*tan(
phi(t)), Mxf(t) = rtf*tan(sin(theta(t)+epsilon)*delta(t)-phi(t))*Fzf(t), Mxr(t
) = -rtr*tan(phi(t))*Fzr(t)];
alg_constr_slips := [alpha__r(t) = -arctan(1/cos(phi(t))^2*(-Omega(t)*cos(phi(
t))^2*cos(theta(t)+epsilon)*cos(eta__00)*L__swa-Omega(t)*cos(phi(t))^2*sin(
theta(t)+epsilon)*sin(eta__00)*L__swa+cos(phi(t))^2*v(t)-rtr*diff(phi(t),t)),1
/cos(phi(t))*(-cos(theta(t)+epsilon)*diff(theta(t),t)*L__swa*sin(eta__00)*cos(
phi(t))+sin(theta(t)+epsilon)*diff(theta(t),t)*cos(phi(t))*cos(eta__00)*L__swa
+rtr*sin(phi(t))*Omega(t)+cos(phi(t))*u(t))), alpha__f(t) = -arctan(1/cos(phi(
t))^3/cos(theta(t)+epsilon)*(cos(theta(t)+epsilon)^3*diff(theta(t),t)*L__swa*
sin(eta__00)*cos(phi(t))^2*delta(t)-cos(theta(t)+epsilon)^2*(sin(theta(t)+
epsilon)*diff(theta(t),t)*L__swa*cos(phi(t))*cos(eta__00)*delta(t)+diff(theta(
t),t)*(rf-rtf)*cos(phi(t))*delta(t)+L__swa*cos(eta__00)*Omega(t)*cos(phi(t))^2
+rtf*sin(phi(t))*delta(t)*Omega(t)+cos(phi(t))*delta(t)*u(t))*cos(phi(t))+cos(
theta(t)+epsilon)*(sin(theta(t)+epsilon)*(-diff(phi(t),t)*delta(t)*(cos(phi(t)
)*(rf-rtf)-rtf+rtr)*sin(phi(t))-(diff(delta(t),t)*(rf-rtf)+L__swa*sin(eta__00)
*Omega(t)*cos(phi(t)))*cos(phi(t))^2)+(diff(phi(t),t)*(x__off*sin(phi(t))*
delta(t)-rtf)+(cos(phi(t))*v(t)+x__off*diff(delta(t),t))*cos(phi(t)))*cos(phi(
t)))-(-sin(theta(t)+epsilon)*(-diff(theta(t),t)*(cos(eta__00)*L__swa+L__b+
x__off)*delta(t)+(cos(phi(t))*(-rf+rr+rtf-rtr)+x__off*sin(phi(t))*delta(t)-rtf
+rtr)*Omega(t))*cos(phi(t))-diff(theta(t),t)*delta(t)*(cos(phi(t))*(rf-rr-rtf+
rtr)+rtf-rtr)+(cos(phi(t))*(-cos(eta__00)*L__swa-L__b-x__off)+(rf-rtf)*sin(phi
(t))*delta(t))*Omega(t)*cos(phi(t)))*cos(phi(t))),1/cos(theta(t)+epsilon)^2/
cos(phi(t))^3*(-Omega(t)*cos(phi(t))^2*cos(theta(t)+epsilon)^4*cos(eta__00)*
delta(t)*L__swa+cos(theta(t)+epsilon)^3*(-Omega(t)*cos(phi(t))^2*sin(theta(t)+
epsilon)*sin(eta__00)*delta(t)*L__swa-diff(theta(t),t)*cos(phi(t))^3*sin(
eta__00)*L__swa+(cos(phi(t))^2*v(t)-rtf*diff(phi(t),t))*delta(t))+cos(theta(t)
+epsilon)^2*(sin(theta(t)+epsilon)*(diff(theta(t),t)*L__swa*cos(eta__00)*cos(
phi(t))^2+(cos(phi(t))*(rr-rtr)-rtf+rtr)*delta(t)*Omega(t))+(cos(phi(t))*u(t)+
(sin(phi(t))*rtf+(cos(eta__00)*L__swa+L__b)*delta(t))*Omega(t))*cos(phi(t)))*
cos(phi(t))-cos(theta(t)+epsilon)*(sin(theta(t)+epsilon)*(diff(phi(t),t)*(sin(
phi(t))*(rtf-rtr)-delta(t)*x__off)-x__off*diff(delta(t),t)*cos(phi(t))*sin(phi
(t)))+(diff(delta(t),t)*cos(phi(t))*sin(phi(t))+diff(phi(t),t)*delta(t))*(rf-
rtf))*cos(phi(t))-diff(theta(t),t)*cos(phi(t))^2*(sin(theta(t)+epsilon)*(cos(
phi(t))*(-cos(eta__00)*L__swa-L__b-x__off)+(rf-rtf)*sin(phi(t))*delta(t))+cos(
phi(t))*(rf-rr-rtf+rtr)-x__off*sin(phi(t))*delta(t)+rtf-rtr))), lambda__r(t) =
1/(cos(theta(t)+epsilon)*diff(theta(t),t)*L__swa*sin(eta__00)*cos(phi(t))-sin(
theta(t)+epsilon)*diff(theta(t),t)*cos(phi(t))*cos(eta__00)*L__swa-rtr*sin(phi
(t))*Omega(t)-cos(phi(t))*u(t))*(-cos(theta(t)+epsilon)*diff(theta(t),t)*
L__swa*sin(eta__00)*cos(phi(t))+sin(theta(t)+epsilon)*diff(theta(t),t)*cos(phi
(t))*cos(eta__00)*L__swa+cos(phi(t))*(-rr*omega__r(t)+u(t))+rtr*sin(phi(t))*
Omega(t)), lambda__f(t) = 1/(-Omega(t)*cos(phi(t))^2*cos(theta(t)+epsilon)^4*
cos(eta__00)*delta(t)*L__swa+cos(theta(t)+epsilon)^3*(-Omega(t)*cos(phi(t))^2*
sin(theta(t)+epsilon)*sin(eta__00)*delta(t)*L__swa-diff(theta(t),t)*cos(phi(t)
)^3*sin(eta__00)*L__swa+cos(phi(t))^2*v(t)*delta(t)-diff(phi(t),t)*delta(t)*
rtf)+cos(theta(t)+epsilon)^2*(sin(theta(t)+epsilon)*(diff(theta(t),t)*L__swa*
cos(eta__00)*cos(phi(t))^2+(cos(phi(t))*(rr-rtr)-rtf+rtr)*delta(t)*Omega(t))+(
cos(phi(t))*u(t)+(sin(phi(t))*rtf+(cos(eta__00)*L__swa+L__b)*delta(t))*Omega(t
))*cos(phi(t)))*cos(phi(t))-cos(theta(t)+epsilon)*(sin(theta(t)+epsilon)*(diff
(phi(t),t)*(sin(phi(t))*(rtf-rtr)-delta(t)*x__off)-x__off*diff(delta(t),t)*cos
(phi(t))*sin(phi(t)))+diff(phi(t),t)*(rf-rtf)*delta(t)+(rf-rtf)*sin(phi(t))*
diff(delta(t),t)*cos(phi(t)))*cos(phi(t))-diff(theta(t),t)*cos(phi(t))^2*(sin(
theta(t)+epsilon)*(cos(phi(t))*(-cos(eta__00)*L__swa-L__b-x__off)+(rf-rtf)*sin
(phi(t))*delta(t))+cos(phi(t))*(rf-rr-rtf+rtr)-x__off*sin(phi(t))*delta(t)+rtf
-rtr))*(Omega(t)*cos(phi(t))^2*cos(theta(t)+epsilon)^4*cos(eta__00)*delta(t)*
L__swa+cos(theta(t)+epsilon)^3*(Omega(t)*cos(phi(t))^2*sin(theta(t)+epsilon)*
sin(eta__00)*delta(t)*L__swa+diff(theta(t),t)*cos(phi(t))^3*sin(eta__00)*
L__swa-cos(phi(t))^2*v(t)*delta(t)+diff(phi(t),t)*delta(t)*rtf)-cos(theta(t)+
epsilon)^2*(sin(theta(t)+epsilon)*(diff(theta(t),t)*L__swa*cos(eta__00)*cos(
phi(t))^2+(cos(phi(t))*(rr-rtr)-rtf+rtr)*delta(t)*Omega(t))+(cos(phi(t))*(-rf*
omega__f(t)+u(t))+(sin(phi(t))*rtf+(cos(eta__00)*L__swa+L__b)*delta(t))*Omega(
t))*cos(phi(t)))*cos(phi(t))+cos(theta(t)+epsilon)*(sin(theta(t)+epsilon)*(
diff(phi(t),t)*(sin(phi(t))*(rtf-rtr)-delta(t)*x__off)-x__off*diff(delta(t),t)
*cos(phi(t))*sin(phi(t)))+diff(phi(t),t)*(rf-rtf)*delta(t)+(rf-rtf)*sin(phi(t)
)*diff(delta(t),t)*cos(phi(t)))*cos(phi(t))+diff(theta(t),t)*cos(phi(t))^2*(
sin(theta(t)+epsilon)*(cos(phi(t))*(-cos(eta__00)*L__swa-L__b-x__off)+(rf-rtf)
*sin(phi(t))*delta(t))+cos(phi(t))*(rf-rr-rtf+rtr)-x__off*sin(phi(t))*delta(t)
+rtf-rtr))];
