
- [1. Tensione della catena](#1-tensione-della-catena)
- [2. Equazioni del moto del rider](#2-equazioni-del-moto-del-rider)
- [3. Trascurare dinamica della catena?](#3-trascurare-dinamica-della-catena)
  - [3.1. Idee aggiuntive](#31-idee-aggiuntive)
  - [3.2. Trascurare dinamica dell'albero motore?](#32-trascurare-dinamica-dellalbero-motore)
- [4. Problemi di convergenza](#4-problemi-di-convergenza)
  - [4.1. Parametri di BAUMGARTE](#41-parametri-di-baumgarte)
  - [4.2. Altro](#42-altro)
- [5. Problemi vari con PINS](#5-problemi-vari-con-pins)
  - [5.1. Le funzioni regolarizzare sono state modificate?](#51-le-funzioni-regolarizzare-sono-state-modificate)


# 1. Tensione della catena

Angolo $\alpha_{crw}(t)$ ha una definizione vaga al momento (prima opzione).

- raggi di pignone e corona differenti con pignone centrato nella cerniera della forcella posteriore (soluzione semplice e in forma chiusa)
- considerare la forza agente sulla corona come se avesse sempre lo stesso angolo dela sospensione posteriore( soluzione più semplice ma trascura diversi effetti)
- raggi uguali e posizione del pignone libera (+ complesso ma comunque risolvibile)
- raggi diversi e posizione del pignone libera (fatico a risolvere analiticamente)

# 2. Equazioni del moto del rider

Come impongo nel problema di ottimo la legge del moto?

1. Impongo angoli fissi momentaneamente
2. dimentico le equazioni del moto e impongo qualcosa del tipo $\tau \dot{\phi}_d + \phi_d = \phi_{d_{des}}$ e $\tau \dot{\theta}_d + \theta_d = \theta_{d_{des}}$  dove $\phi_{d_{des}}$ e $\theta_{d_{des}}$ diventano funzioni di rollio e accelerazioni/decellerazioni
3. utilizzo le equazioni del moto per calcolare le coppie che vengono generate al torso e all'interno del termine di laggrange minimizzo un termine del tipo $(\phi_d-\phi_{d_{des}})^2 + (\theta_d-\theta_{d_{des}})^2$ 


# 3. Trascurare dinamica della catena?

Considero solo legame cinematico tr la rotazione della corona (e quindi ruota) ed il pignone?

$$\omega_r=\tau_{c} \omega_p$$

La forza di trazione $Ftr$ posso considerarla dunque come la coppia al pignone fratto il raggio del pignone

$$F_{tr}  =  \frac{M_{pin}}{r_{pin}}$$

al momento per sfruttare il pacchetto engine di xoptima scrivo 

$$F_{tr}  =  \frac{M_{crw}}{r_{crw}}$$

per rendere più agevole definire $M_{crw}$

## 3.1. Idee aggiuntive

chiamando $\omega_p$ la velocità di rotazione del pignone, $\omega_e$ quella del motore allora potrei scrivere

$$M_{pin}=\tau_{G} M_e$$

dove $\tau_G$ è di fatto una funzione continua a tratti dipendende dalla velocità del pignone

$$
\tau_G (\omega_p) =
\begin{cases}
    \tau_1,  &          0&< \omega_p &\le &\omega_{12}\\
    \tau_2,  &\omega_{12}&< \omega_p &\le &\omega_{23}\\
    \dots 
\end{cases}
$$

Poi $M_e$ è funzione della velocità di rotazione dell'albero motore

$$M_e=M_e(\omega_e)$$

dove $\omega_e$ dipende da $\omega_p$ attraverso il cambio

## 3.2. Trascurare dinamica dell'albero motore?


# 4. Problemi di convergenza

## 4.1. Parametri di BAUMGARTE

- cosa usare come fattore di damping
- cosa usare come frequenza naturale

## 4.2. Altro

- coppie vengono positive anche se il controllo è vincolato tra 0 e 1
- sembra non tenere il centro strada
- slip parte la $\lambda=1$ (idk)

# 5. Problemi vari con PINS

## 5.1. Le funzioni regolarizzare sono state modificate?

Mi dava errore il fase di compilazione.



