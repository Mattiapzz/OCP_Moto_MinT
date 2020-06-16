alg_constr_Fz := [Fzf(t) = -z__f(t)*Kp__f-diff(z__f(t),t)*Cp__f, Fzr(t) = -
z__r(t)*Kp__r-diff(z__r(t),t)*Cp__r];
alg_constr_tyre_forces := [Fxf(t) = 1/cos(arctan(rHx1__f/(rBx2__f^2*lambda__f(
t)^2+1)^(1/2)*(rBx3__f*phi__f(t)+rBx1__f))*rCx1__f)*cos(arctan((alpha__f(t)+
rHx1__f)/(rBx2__f^2*lambda__f(t)^2+1)^(1/2)*(rBx3__f*phi__f(t)+rBx1__f))*
rCx1__f)*sin(arctan(1/(lambda__C__x__f*pCx1__f*Fzf(t)*lambda__mu__x__f*(1/Fz0*
(Fzf(t)-Fz0)*pDx2__f+pDx1__f)+epsilon__x__f)*lambda__K__x__f*exp(1/Fz0*(Fzf(t)
-Fz0)*pKx3__f)*(1/Fz0*(Fzf(t)-Fz0)*pKx2__f+pKx1__f)*Fzf(t)*lambda__f(t))*
lambda__C__x__f*pCx1__f)*Fzf(t)*lambda__mu__x__f*(1/Fz0*(Fzf(t)-Fz0)*pDx2__f+
pDx1__f), Fxr(t) = 1/cos(arctan(rHx1__r/(rBx2__r^2*lambda__r(t)^2+1)^(1/2)*(
rBx3__r*phi(t)+rBx1__r))*rCx1__r)*cos(arctan((alpha__r(t)+rHx1__r)/(rBx2__r^2*
lambda__r(t)^2+1)^(1/2)*(rBx3__r*phi(t)+rBx1__r))*rCx1__r)*sin(arctan(1/(
lambda__C__x__r*pCx1__r*Fzr(t)*lambda__mu__x__r*(1/Fz0*(Fzr(t)-Fz0)*pDx2__r+
pDx1__r)+epsilon__x__r)*lambda__K__x__r*exp(1/Fz0*(Fzr(t)-Fz0)*pKx3__r)*(1/Fz0
*(Fzr(t)-Fz0)*pKx2__r+pKx1__r)*Fzr(t)*lambda__r(t))*lambda__C__x__r*pCx1__r)*
Fzr(t)*lambda__mu__x__r*(1/Fz0*(Fzr(t)-Fz0)*pDx2__r+pDx1__r), Fyf(t) = 1/cos(
arctan((rHy1__f+1/Fz0__f*(Fzf(t)-Fz0__f)*rHy2__f)/(1+(tan(alpha__f(t))-rBy3__f
)^2*rBy2__f^2)^(1/2)*rBy1__f)*rCy1__f)*cos(arctan((lambda__f(t)+rHy1__f+1/
Fz0__f*(Fzf(t)-Fz0__f)*rHy2__f)/(1+(tan(alpha__f(t))-rBy3__f)^2*rBy2__f^2)^(1/
2)*rBy1__f)*rCy1__f)*(sin(arctan(((alpha__f(t)-(phi__f(t)^2*d5__f+1)/(Fz0__f*
d1__f+(Fzf(t)-Fz0__f)*d2__f)*phi__f(t)*Fzf(t)*d3__f)/(1/(phi__f(t)^2*d7__f+1)^
2*Fzf(t)^2*d4__f^2)^(1/2)/(phi__f(t)^2*d7__f+1)*Fzf(t)*d4__f-(phi__f(t)^2*
d5__f+1)/(Fz0__f*d1__f+(Fzf(t)-Fz0__f)*d2__f)*(phi__f(t)^2*d7__f+1)/d4__f*(1/(
phi__f(t)^2*d7__f+1)^2*Fzf(t)^2*d4__f^2)^(1/2)*phi__f(t)*d6__f)*(phi__f(t)^2*
d7__f+1)/Fzf(t)/d4__f/d8/(phi__f(t)^2*d5__f+1)*(Fz0__f*d1__f+(Fzf(t)-Fz0__f)*
d2__f))*d8)*(1/(phi__f(t)^2*d7__f+1)^2*Fzf(t)^2*d4__f^2)^(1/2)+(phi__f(t)^2*
d7__f+1)/d4__f*(1/(phi__f(t)^2*d7__f+1)^2*Fzf(t)^2*d4__f^2)^(1/2)*phi__f(t)*
d6__f), Fyr(t) = 1/cos(arctan((rHy1__r+1/Fz0__r*(Fzr(t)-Fz0__r)*rHy2__r)/(1+(
tan(alpha__r(t))-rBy3__r)^2*rBy2__r^2)^(1/2)*rBy1__r)*rCy1__r)*cos(arctan((
lambda__r(t)+rHy1__r+1/Fz0__r*(Fzr(t)-Fz0__r)*rHy2__r)/(1+(tan(alpha__r(t))-
rBy3__r)^2*rBy2__r^2)^(1/2)*rBy1__r)*rCy1__r)*(sin(arctan(((alpha__r(t)-(phi(t
)^2*d5__r+1)/(Fz0__r*d1__r+(Fzr(t)-Fz0__r)*d2__r)*phi(t)*Fzr(t)*d3__r)/(1/(phi
(t)^2*d7__r+1)^2*Fzr(t)^2*d4__r^2)^(1/2)/(phi(t)^2*d7__r+1)*Fzr(t)*d4__r-(phi(
t)^2*d5__r+1)/(Fz0__r*d1__r+(Fzr(t)-Fz0__r)*d2__r)*(phi(t)^2*d7__r+1)/d4__r*(1
/(phi(t)^2*d7__r+1)^2*Fzr(t)^2*d4__r^2)^(1/2)*phi(t)*d6__r)*(phi(t)^2*d7__r+1)
/Fzr(t)/d4__r/d8/(phi(t)^2*d5__r+1)*(Fz0__r*d1__r+(Fzr(t)-Fz0__r)*d2__r))*d8)*
(1/(phi(t)^2*d7__r+1)^2*Fzr(t)^2*d4__r^2)^(1/2)+(phi(t)^2*d7__r+1)/d4__r*(1/(
phi(t)^2*d7__r+1)^2*Fzr(t)^2*d4__r^2)^(1/2)*phi(t)*d6__r), Mzf(t) = -sin(
arctan(alpha__f(t)/(1/(phi__f(t)^2*d7__f+1)^2*Fzf(t)^2*d4__f^2)^(1/2)/d8/(
phi__f(t)^2*d5__f+1)*(Fz0__f*d1__f+(Fzf(t)-Fz0__f)*d2__f))*d8)*(1/(phi__f(t)^2
*d7__f+1)^2*Fzf(t)^2*d4__f^2)^(1/2)/(phi__f(t)^2*e5+1)*cos(arctan(alpha__f(t)/
(1/(phi__f(t)^2*d7__f+1)^2*Fzf(t)^2*d4__f^2)^(1/2)/(phi__f(t)^2*d7__f+1)*Fzf(t
)*d4__f*e7)/(phi__f(t)^2*e5+1)*e10)/(Fz0__f*d1__f+(Fzf(t)-Fz0__f)*d2__f)*Fzf(t
)*e1__f+cos(arctan(alpha__f(t)/(1/(phi__f(t)^2*d7__f+1)^2*Fzf(t)^2*d4__f^2)^(1
/2)/(phi__f(t)^2*d7__f+1)*Fzf(t)*d4__f/(phi__f(t)^2*e4+1)*e9)/(phi__f(t)^2*e5+
1)*e10)/e6*arctan(phi__f(t)*e6)*Fzf(t)*e2__f, Mzr(t) = -sin(arctan(alpha__r(t)
/(1/(phi(t)^2*d7__r+1)^2*Fzr(t)^2*d4__r^2)^(1/2)/d8/(phi(t)^2*d5__r+1)*(Fz0__r
*d1__r+(Fzr(t)-Fz0__r)*d2__r))*d8)*(1/(phi(t)^2*d7__r+1)^2*Fzr(t)^2*d4__r^2)^(
1/2)/(phi(t)^2*e5+1)*cos(arctan(alpha__r(t)/(1/(phi(t)^2*d7__r+1)^2*Fzr(t)^2*
d4__r^2)^(1/2)/(phi(t)^2*d7__r+1)*Fzr(t)*d4__r*e7)/(phi(t)^2*e5+1)*e10)/(
Fz0__r*d1__r+(Fzr(t)-Fz0__r)*d2__r)*Fzr(t)*e1__r+cos(arctan(alpha__r(t)/(1/(
phi(t)^2*d7__r+1)^2*Fzr(t)^2*d4__r^2)^(1/2)/(phi(t)^2*d7__r+1)*Fzr(t)*d4__r/(
phi(t)^2*e4+1)*e9)/(phi(t)^2*e5+1)*e10)/e6*arctan(phi(t)*e6)*Fzr(t)*e2__r, Mxf
(t) = 0, Mxr(t) = 0];
alg_constr_slips := [alpha__r(t) = arctan((-Omega(t)*x__r(t)+v(t)+diff(y__r(t)
,t))/(Omega(t)*y__r(t)+diff(x__r(t),t)-u(t))), alpha__f(t) = -arctan(1/(-Omega
(t)*y__f(t)+u(t)+diff(x__f(t),t)+delta__f(t)*(Omega(t)*x__f(t)+v(t)+diff(y__f(
t),t)))*(-delta__f(t)*(-Omega(t)*y__f(t)+u(t)+diff(x__f(t),t))+Omega(t)*x__f(t
)+v(t)+diff(y__f(t),t))), lambda__r(t) = (-diff(x__r(t),t)-omega__r(t)*cos(phi
(t))*rtr+(-rr+rtr)*omega__r(t)-Omega(t)*y__r(t)+u(t))/(Omega(t)*y__r(t)+diff(
x__r(t),t)-u(t)), lambda__f(t) = (-diff(x__f(t),t)-diff(y__f(t),t)*delta__f(t)
+omega__f(t)*cos(phi__f(t))*rtf+(-Omega(t)*x__f(t)-v(t))*delta__f(t)+Omega(t)*
y__f(t)+(rf-rtf)*omega__f(t)-u(t))/(diff(x__f(t),t)+diff(y__f(t),t)*delta__f(t
)+(Omega(t)*x__f(t)+v(t))*delta__f(t)-Omega(t)*y__f(t)+u(t))];
alg_constr_slips := [alpha__r(t) = arctan((-Omega(t)*x__r(t)+v(t)+diff(y__r(t)
,t))/(Omega(t)*y__r(t)+diff(x__r(t),t)-u(t))), alpha__f(t) = -arctan(1/(-Omega
(t)*y__f(t)+u(t)+diff(x__f(t),t)+delta__f(t)*(Omega(t)*x__f(t)+v(t)+diff(y__f(
t),t)))*(-delta__f(t)*(-Omega(t)*y__f(t)+u(t)+diff(x__f(t),t))+Omega(t)*x__f(t
)+v(t)+diff(y__f(t),t))), lambda__r(t) = (-diff(x__r(t),t)-omega__r(t)*cos(phi
(t))*rtr+(-rr+rtr)*omega__r(t)-Omega(t)*y__r(t)+u(t))/(Omega(t)*y__r(t)+diff(
x__r(t),t)-u(t)), lambda__f(t) = (-diff(x__f(t),t)-diff(y__f(t),t)*delta__f(t)
+omega__f(t)*cos(phi__f(t))*rtf+(-Omega(t)*x__f(t)-v(t))*delta__f(t)+Omega(t)*
y__f(t)+(rf-rtf)*omega__f(t)-u(t))/(diff(x__f(t),t)+diff(y__f(t),t)*delta__f(t
)+(Omega(t)*x__f(t)+v(t))*delta__f(t)-Omega(t)*y__f(t)+u(t))];
alg_constr_mInertia := [M__tot = m__m+m__rdr+m__delta+m__swa+m__wf+m__wr, XG =
((m__swa*(-L__swa+x__Swing)-L__swa*m__wr)*cos(eta__00)+sin(theta__d__00)*
h__rdr*m__rdr+z__Swing*m__swa*sin(eta__00)+(L__b+x__delta)*m__delta+m__rdr*
x__rdr+(L__b+x__off)*m__wf+x__Rear*m__m)/(m__m+m__rdr+m__delta+m__swa+m__wf+
m__wr), YG = 0, ZG = (((L__swa-x__Swing)*m__swa+L__swa*m__wr)*sin(eta__00)+cos
(theta__d__00)*h__rdr*m__rdr+z__Swing*m__swa*cos(eta__00)+z__rdr*m__rdr+(-
s__fs+s__f__00)*m__wf+z__Rear*m__m+z__delta*m__delta)/(m__m+m__rdr+m__delta+
m__swa+m__wf+m__wr), IX = (((-(x__Swing-z__Swing-L__swa)*(x__Swing+z__Swing-
L__swa)*m__rdr-(x__Swing-z__Swing-L__swa)*(x__Swing+z__Swing-L__swa)*m__wf-(
x__Swing-z__Swing-L__swa)*(x__Swing+z__Swing-L__swa)*m__delta-(x__Swing-
z__Swing-L__swa)*(x__Swing+z__Swing-L__swa)*m__m+(-x__Swing^2+z__Swing^2)*
m__wr+Ix__swa-Iz__swa)*m__swa+(-L__swa^2*m__wr+Ix__swa-Iz__swa)*m__rdr+(-
L__swa^2*m__wr+Ix__swa-Iz__swa)*m__wf+(-L__swa^2*m__wr+Ix__swa-Iz__swa)*
m__delta+(-L__swa^2*m__wr+Ix__swa-Iz__swa)*m__m+m__wr*(Ix__swa-Iz__swa))*cos(
eta__00)^2+(((-2*z__Swing*(-L__swa+x__Swing)*m__rdr-2*z__Swing*(-L__swa+
x__Swing)*m__wf-2*z__Swing*(-L__swa+x__Swing)*m__delta-2*z__Swing*(-L__swa+
x__Swing)*m__m-2*x__Swing*z__Swing*m__wr-2*Cxz__swa)*m__swa-2*Cxz__swa*(m__rdr
+m__delta+m__m+m__wf+m__wr))*sin(eta__00)-2*z__Swing*m__swa*(cos(theta__d__00)
*h__rdr*m__rdr+z__rdr*m__rdr+(-s__fs+s__f__00)*m__wf+z__Rear*m__m+z__delta*
m__delta))*cos(eta__00)+2*(m__swa*(-L__swa+x__Swing)-L__swa*m__wr)*(cos(
theta__d__00)*h__rdr*m__rdr+z__rdr*m__rdr+(-s__fs+s__f__00)*m__wf+z__Rear*m__m
+z__delta*m__delta)*sin(eta__00)+((h__rdr^2*m__rdr+Ix__rdr-Iz__rdr)*m__swa+(
h__rdr^2*m__m+h__rdr^2*m__wf+h__rdr^2*m__wr+h__rdr^2*m__delta+Ix__rdr-Iz__rdr)
*m__rdr+(m__m+m__delta+m__wf+m__wr)*(Ix__rdr-Iz__rdr))*cos(theta__d__00)^2+2*
m__rdr*(z__rdr*m__swa+(z__rdr-s__f__00+s__fs)*m__wf+(z__rdr-z__delta)*m__delta
+(z__rdr-z__Rear)*m__m+z__rdr*m__wr)*h__rdr*cos(theta__d__00)+((L__swa^2-2*
L__swa*x__Swing+z__rdr^2+x__Swing^2)*m__rdr+(L__swa^2-2*x__Swing*L__swa+
x__Swing^2+(-s__fs+s__f__00)^2)*m__wf+(L__swa^2-2*L__swa*x__Swing+x__Swing^2+
z__delta^2)*m__delta+(L__swa^2-2*L__swa*x__Swing+z__Rear^2+x__Swing^2)*m__m+
x__Swing^2*m__wr+Iz__rdr+Iz__swa+Ix__m+Ix__delta+Id__wf+Id__wr)*m__swa+((
z__rdr-s__f__00+s__fs)^2*m__wf+(z__rdr-z__delta)^2*m__delta+(z__rdr-z__Rear)^2
*m__m+z__rdr^2*m__wr+m__wr*L__swa^2+Iz__rdr+Iz__swa+Ix__m+Ix__delta+Id__wf+
Id__wr)*m__rdr+((s__f__00-z__delta-s__fs)^2*m__delta+(z__Rear-s__f__00+s__fs)^
2*m__m+m__wr*L__swa^2+(-s__fs+s__f__00)^2*m__wr+Iz__rdr+Iz__swa+Ix__m+
Ix__delta+Id__wf+Id__wr)*m__wf+((z__Rear-z__delta)^2*m__m+z__delta^2*m__wr+
m__wr*L__swa^2+Iz__rdr+Iz__swa+Ix__m+Ix__delta+Id__wf+Id__wr)*m__delta+(L__swa
^2*m__wr+m__wr*z__Rear^2+Id__wf+Id__wr+Ix__m+Iz__rdr+Iz__swa+Ix__delta)*m__m+
m__wr*(Iz__rdr+Iz__swa+Ix__m+Ix__delta+Id__wf+Id__wr))/(m__m+m__rdr+m__delta+
m__swa+m__wf+m__wr), IY = ((-2*m__rdr*(m__swa*(-L__swa+x__Swing)-L__swa*m__wr)
*h__rdr*sin(theta__d__00)-2*m__swa*z__Swing*h__rdr*m__rdr*cos(theta__d__00)+((
2*L__swa*x__rdr-2*x__rdr*x__Swing-2*z__rdr*z__Swing)*m__swa+2*x__rdr*m__wr*
L__swa)*m__rdr+(((2*L__b+2*x__off)*L__swa-2*x__Swing*L__b-2*x__Swing*x__off-2*
(-s__fs+s__f__00)*z__Swing)*m__wf+((2*L__b+2*x__delta)*L__swa-2*x__Swing*L__b-\
2*x__Swing*x__delta-2*z__Swing*z__delta)*m__delta-2*m__m*(-L__swa*x__Rear+
x__Rear*x__Swing+z__Rear*z__Swing))*m__swa+2*L__swa*m__wr*((L__b+x__off)*m__wf
+(L__b+x__delta)*m__delta+x__Rear*m__m))*cos(eta__00)+(-2*sin(theta__d__00)*
h__rdr*m__rdr*m__swa*z__Swing+2*m__rdr*(m__swa*(-L__swa+x__Swing)-L__swa*m__wr
)*h__rdr*cos(theta__d__00)+((-2*L__swa*z__rdr-2*x__rdr*z__Swing+2*z__rdr*
x__Swing)*m__swa-2*z__rdr*m__wr*L__swa)*m__rdr+(((-2*s__f__00+2*s__fs)*L__swa-\
2*z__Swing*L__b+(2*s__f__00-2*s__fs)*x__Swing-2*z__Swing*x__off)*m__wf+(-2*
L__b*z__Swing-2*L__swa*z__delta+2*x__Swing*z__delta-2*x__delta*z__Swing)*
m__delta-2*m__m*(L__swa*z__Rear+x__Rear*z__Swing-z__Rear*x__Swing))*m__swa-2*
L__swa*m__wr*((-s__fs+s__f__00)*m__wf+z__Rear*m__m+z__delta*m__delta))*sin(
eta__00)-2*h__rdr*(-x__rdr*m__swa+(L__b-x__rdr+x__delta)*m__delta+(L__b+x__off
-x__rdr)*m__wf+(-x__rdr+x__Rear)*m__m-x__rdr*m__wr)*m__rdr*sin(theta__d__00)+2
*m__rdr*(z__rdr*m__swa+(z__rdr-s__f__00+s__fs)*m__wf+(z__rdr-z__delta)*
m__delta+(z__rdr-z__Rear)*m__m+z__rdr*m__wr)*h__rdr*cos(theta__d__00)+((L__swa
^2-2*L__swa*x__Swing+h__rdr^2+x__rdr^2+z__rdr^2+x__Swing^2+z__Swing^2)*m__swa+
(h__rdr^2+L__b^2+(2*x__off-2*x__rdr)*L__b+x__rdr^2-2*x__off*x__rdr+s__f__00^2+
(-2*z__rdr-2*s__fs)*s__f__00+x__off^2+z__rdr^2+2*z__rdr*s__fs+s__fs^2)*m__wf+(
h__rdr^2+L__b^2+(-2*x__rdr+2*x__delta)*L__b+x__rdr^2-2*x__rdr*x__delta+z__rdr^
2-2*z__delta*z__rdr+x__delta^2+z__delta^2)*m__delta+(h__rdr^2+x__Rear^2-2*
x__Rear*x__rdr+x__rdr^2+z__Rear^2-2*z__Rear*z__rdr+z__rdr^2)*m__m+(L__swa^2+
h__rdr^2+x__rdr^2+z__rdr^2)*m__wr+Iy__rdr+Iy__swa+Iy__delta+Iy__m+Iy__wf+
Iy__wr)*m__rdr+((L__b^2+2*L__b*x__off+L__swa^2-2*L__swa*x__Swing+s__fs^2-2*
s__fs*s__f__00+x__off^2+s__f__00^2+x__Swing^2+z__Swing^2)*m__wf+(L__b^2+2*L__b
*x__delta+L__swa^2-2*L__swa*x__Swing+x__Swing^2+x__delta^2+z__Swing^2+z__delta
^2)*m__delta+(L__swa^2-2*L__swa*x__Swing+x__Rear^2+z__Rear^2+x__Swing^2+
z__Swing^2)*m__m+(x__Swing^2+z__Swing^2)*m__wr+Iy__rdr+Iy__swa+Iy__delta+Iy__m
+Iy__wf+Iy__wr)*m__swa+((s__f__00^2+(-2*z__delta-2*s__fs)*s__f__00+x__off^2-2*
x__off*x__delta+x__delta^2+z__delta^2+2*z__delta*s__fs+s__fs^2)*m__delta+(L__b
^2+(2*x__off-2*x__Rear)*L__b+s__f__00^2+(-2*z__Rear-2*s__fs)*s__f__00+x__off^2
-2*x__off*x__Rear+x__Rear^2+z__Rear^2+2*z__Rear*s__fs+s__fs^2)*m__m+(L__b^2+2*
L__b*x__off+L__swa^2+s__fs^2-2*s__fs*s__f__00+x__off^2+s__f__00^2)*m__wr+
Iy__rdr+Iy__swa+Iy__delta+Iy__m+Iy__wf+Iy__wr)*m__wf+((L__b^2+(-2*x__Rear+2*
x__delta)*L__b+x__delta^2-2*x__Rear*x__delta+x__Rear^2+(z__Rear-z__delta)^2)*
m__m+(L__b^2+2*L__b*x__delta+L__swa^2+x__delta^2+z__delta^2)*m__wr+Iy__rdr+
Iy__swa+Iy__delta+Iy__m+Iy__wf+Iy__wr)*m__delta+((L__swa^2+x__Rear^2+z__Rear^2
)*m__wr+Iy__rdr+Iy__swa+Iy__delta+Iy__m+Iy__wf+Iy__wr)*m__m+m__wr*(Iy__rdr+
Iy__swa+Iy__delta+Iy__m+Iy__wf+Iy__wr))/(m__m+m__rdr+m__delta+m__swa+m__wf+
m__wr), IZ = ((((x__Swing-z__Swing-L__swa)*(x__Swing+z__Swing-L__swa)*m__rdr+(
x__Swing-z__Swing-L__swa)*(x__Swing+z__Swing-L__swa)*m__delta+(x__Swing-
z__Swing-L__swa)*(x__Swing+z__Swing-L__swa)*m__wf+(x__Swing-z__Swing-L__swa)*(
x__Swing+z__Swing-L__swa)*m__m+(x__Swing^2-z__Swing^2)*m__wr-Ix__swa+Iz__swa)*
m__swa+(L__swa^2*m__wr-Ix__swa+Iz__swa)*m__rdr+(L__swa^2*m__wr-Ix__swa+Iz__swa
)*m__delta+(L__swa^2*m__wr-Ix__swa+Iz__swa)*m__wf+(L__swa^2*m__wr-Ix__swa+
Iz__swa)*m__m-m__wr*(Ix__swa-Iz__swa))*cos(eta__00)^2+(((2*z__Swing*(-L__swa+
x__Swing)*m__rdr+2*z__Swing*(-L__swa+x__Swing)*m__delta+2*z__Swing*(-L__swa+
x__Swing)*m__wf+2*z__Swing*(-L__swa+x__Swing)*m__m+2*x__Swing*z__Swing*m__wr+2
*Cxz__swa)*m__swa+2*Cxz__swa*(m__rdr+m__delta+m__m+m__wf+m__wr))*sin(eta__00)-\
2*(m__swa*(-L__swa+x__Swing)-L__swa*m__wr)*(sin(theta__d__00)*h__rdr*m__rdr+
m__rdr*x__rdr+(L__b+x__delta)*m__delta+(L__b+x__off)*m__wf+x__Rear*m__m))*cos(
eta__00)-2*m__swa*z__Swing*(sin(theta__d__00)*h__rdr*m__rdr+m__rdr*x__rdr+(
L__b+x__delta)*m__delta+(L__b+x__off)*m__wf+x__Rear*m__m)*sin(eta__00)+((-
h__rdr^2*m__rdr-Ix__rdr+Iz__rdr)*m__swa+(-h__rdr^2*m__m-h__rdr^2*m__wf-h__rdr^
2*m__wr-h__rdr^2*m__delta-Ix__rdr+Iz__rdr)*m__rdr-(m__m+m__delta+m__wf+m__wr)*
(Ix__rdr-Iz__rdr))*cos(theta__d__00)^2-2*h__rdr*(-x__rdr*m__swa+(L__b-x__rdr+
x__delta)*m__delta+(L__b+x__off-x__rdr)*m__wf+(-x__rdr+x__Rear)*m__m-x__rdr*
m__wr)*m__rdr*sin(theta__d__00)+((h__rdr^2+x__rdr^2+z__Swing^2)*m__rdr+(L__b^2
+2*L__b*x__delta+x__delta^2+z__Swing^2)*m__delta+(L__b^2+2*L__b*x__off+x__off^
2+z__Swing^2)*m__wf+(x__Rear^2+z__Swing^2)*m__m+m__wr*z__Swing^2+Ix__rdr+
Ix__swa+Iz__delta+Iz__m+Id__wf+Id__wr)*m__swa+((L__b^2+(-2*x__rdr+2*x__delta)*
L__b+h__rdr^2+(x__rdr-x__delta)^2)*m__delta+(L__b^2+(2*x__off-2*x__rdr)*L__b+
h__rdr^2+(x__off-x__rdr)^2)*m__wf+(h__rdr^2+(x__rdr-x__Rear)^2)*m__m+(h__rdr^2
+x__rdr^2)*m__wr+Ix__rdr+Ix__swa+Iz__delta+Iz__m+Id__wf+Id__wr)*m__rdr+((
x__off-x__delta)^2*m__wf+(L__b-x__Rear+x__delta)^2*m__m+(L__b+x__delta)^2*
m__wr+Ix__rdr+Ix__swa+Iz__delta+Iz__m+Id__wf+Id__wr)*m__delta+((L__b+x__off-
x__Rear)^2*m__m+(L__b+x__off)^2*m__wr+Ix__rdr+Ix__swa+Iz__delta+Iz__m+Id__wf+
Id__wr)*m__wf+(m__wr*x__Rear^2+Id__wf+Id__wr+Ix__rdr+Ix__swa+Iz__m+Iz__delta)*
m__m+m__wr*(Ix__rdr+Ix__swa+Iz__delta+Iz__m+Id__wf+Id__wr))/(m__m+m__rdr+
m__delta+m__swa+m__wf+m__wr), CXZ = (((2*z__Swing*(-L__swa+x__Swing)*m__rdr+2*
z__Swing*(-L__swa+x__Swing)*m__delta+2*z__Swing*(-L__swa+x__Swing)*m__wf+2*
z__Swing*(-L__swa+x__Swing)*m__m+2*x__Swing*z__Swing*m__wr+2*Cxz__swa)*m__swa+
2*Cxz__swa*(m__rdr+m__delta+m__m+m__wf+m__wr))*cos(eta__00)^2+(((-(x__Swing-
z__Swing-L__swa)*(x__Swing+z__Swing-L__swa)*m__rdr-(x__Swing-z__Swing-L__swa)*
(x__Swing+z__Swing-L__swa)*m__wf-(x__Swing-z__Swing-L__swa)*(x__Swing+z__Swing
-L__swa)*m__delta-(x__Swing-z__Swing-L__swa)*(x__Swing+z__Swing-L__swa)*m__m+(
-x__Swing^2+z__Swing^2)*m__wr+Ix__swa-Iz__swa)*m__swa+(-L__swa^2*m__wr+Ix__swa
-Iz__swa)*m__rdr+(-L__swa^2*m__wr+Ix__swa-Iz__swa)*m__wf+(-L__swa^2*m__wr+
Ix__swa-Iz__swa)*m__delta+(-L__swa^2*m__wr+Ix__swa-Iz__swa)*m__m+m__wr*(
Ix__swa-Iz__swa))*sin(eta__00)-m__rdr*(m__swa*(-L__swa+x__Swing)-L__swa*m__wr)
*h__rdr*cos(theta__d__00)-sin(theta__d__00)*h__rdr*m__rdr*m__swa*z__Swing+((
L__swa*z__rdr-x__rdr*z__Swing-z__rdr*x__Swing)*m__rdr+((-s__fs+s__f__00)*
L__swa+(-L__b-x__off)*z__Swing-x__Swing*(-s__fs+s__f__00))*m__wf+(z__delta*
L__swa+(-L__b-x__delta)*z__Swing-x__Swing*z__delta)*m__delta-m__m*(-L__swa*
z__Rear+x__Rear*z__Swing+z__Rear*x__Swing))*m__swa+L__swa*m__wr*(z__rdr*m__rdr
+(-s__fs+s__f__00)*m__wf+z__Rear*m__m+z__delta*m__delta))*cos(eta__00)+(-
m__swa*z__Swing*h__rdr*m__rdr*cos(theta__d__00)+m__rdr*(m__swa*(-L__swa+
x__Swing)-L__swa*m__wr)*h__rdr*sin(theta__d__00)+((-L__swa*x__rdr+x__rdr*
x__Swing-z__rdr*z__Swing)*m__rdr+((-L__b-x__off)*L__swa+(-s__f__00+s__fs)*
z__Swing+x__Swing*(L__b+x__off))*m__wf+((-L__b-x__delta)*L__swa-z__Swing*
z__delta+x__Swing*(L__b+x__delta))*m__delta+m__m*(-L__swa*x__Rear+x__Rear*
x__Swing-z__Rear*z__Swing))*m__swa-(m__rdr*x__rdr+(L__b+x__off)*m__wf+(L__b+
x__delta)*m__delta+x__Rear*m__m)*L__swa*m__wr)*sin(eta__00)+(((h__rdr^2*m__rdr
+Ix__rdr-Iz__rdr)*m__swa+(h__rdr^2*m__m+h__rdr^2*m__wf+h__rdr^2*m__wr+h__rdr^2
*m__delta+Ix__rdr-Iz__rdr)*m__rdr+(m__m+m__delta+m__wf+m__wr)*(Ix__rdr-Iz__rdr
))*sin(theta__d__00)-m__rdr*(-x__rdr*m__swa+(L__b-x__rdr+x__delta)*m__delta+(
L__b+x__off-x__rdr)*m__wf+(-x__rdr+x__Rear)*m__m-x__rdr*m__wr)*h__rdr)*cos(
theta__d__00)+m__rdr*(z__rdr*m__swa+(z__rdr-s__f__00+s__fs)*m__wf+(z__rdr-
z__delta)*m__delta+(z__rdr-z__Rear)*m__m+z__rdr*m__wr)*h__rdr*sin(theta__d__00
)+((L__swa*z__Swing+x__rdr*z__rdr-x__Swing*z__Swing)*m__rdr+(L__swa*z__Swing-
x__Swing*z__Swing+(-s__fs+s__f__00)*(L__b+x__off))*m__wf+(L__swa*z__Swing-
x__Swing*z__Swing+z__delta*(L__b+x__delta))*m__delta+(L__swa*z__Swing+x__Rear*
z__Rear-x__Swing*z__Swing)*m__m-x__Swing*z__Swing*m__wr-Cxz__swa+Cxz__delta+
Cxz__m)*m__swa+(-(z__rdr-s__f__00+s__fs)*(L__b+x__off-x__rdr)*m__wf-(z__rdr-
z__delta)*(L__b-x__rdr+x__delta)*m__delta+(z__rdr-z__Rear)*(x__rdr-x__Rear)*
m__m+x__rdr*z__rdr*m__wr-Cxz__swa+Cxz__delta+Cxz__m)*m__rdr+((s__f__00-
z__delta-s__fs)*(x__off-x__delta)*m__delta-(z__Rear-s__f__00+s__fs)*(L__b+
x__off-x__Rear)*m__m+(-s__fs+s__f__00)*(L__b+x__off)*m__wr-Cxz__swa+Cxz__delta
+Cxz__m)*m__wf+(-(z__Rear-z__delta)*(L__b-x__Rear+x__delta)*m__m+z__delta*(
L__b+x__delta)*m__wr-Cxz__swa+Cxz__delta+Cxz__m)*m__delta+(m__wr*x__Rear*
z__Rear+Cxz__m+Cxz__delta-Cxz__swa)*m__m-m__wr*(Cxz__swa-Cxz__delta-Cxz__m))/(
m__m+m__rdr+m__delta+m__swa+m__wf+m__wr)];
alg_kine_constr := [];
penetration := [p__r(t) = -z__r(t), p__f(t) = -z__f(t)];
