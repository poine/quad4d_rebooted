---
title: Quad4D rebooted
layout: default
---

<script src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML" type="text/javascript"></script>

<!-- <script src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML" type="text/javascript"></script> -->

<!-- <script type="math/tex; mode=display"> -->
<!--    \newcommand{\vect}[1]{\underline{#1}} -->
<!--    \newcommand{\mat}[1]{\mathbf{#1}} -->
<!--    \newcommand{\eye}[0]{\mathbf{I}} -->
<!--    \newcommand{\est}[1]{\hat{#1}} -->
<!--    \newcommand{\err}[1]{\tilde{#1}} -->
<!--    \newcommand{\pd}[2]{\frac{\partial{#1}}{\partial{#2}}} -->
<!--    \newcommand{\transp}[1]{#1^{T}} -->
<!--    \newcommand{\inv}[1]{#1^{-1}} -->
<!--    \newcommand{\norm}[1]{|{#1}|} -->
<!--    \def\rddots{\cdot^{\cdot^{\cdot}}} -->
<!--    \newcommand{\jac}[3]{\frac{\partial{#1}}{\partial{#2}}|_{#3}} % Jacobian -->
<!-- </script> -->


## 1: Trajectoires

Nous allons ici définir une trajectoire (de sortie) d'un quadrirotor comme une fonction (dérivable) du temps de dimension 4

$$
Y(t) = \begin{pmatrix}x(t) \\ y(t) \\ z(t) \\ \psi(t) \end{pmatrix} \quad t \in [t_0, t_1[
$$

avec $$x, y, z$$ les coordonnées du centre de gravité du véhicule et $$\psi$$ son lacet.


### 1.1 Trajectoires analytiques

#### 1.1.1 Segment de Droite

<details>
  <summary>Details</summary>

  <img src="graphics/demo_line.png" width="800">

</details>

<br> 
 
  * classe `Line` dans <a href="https://github.com/poine/pat/blob/master/src/pat3/vehicles/rotorcraft/multirotor_trajectory.py">pat3/vehicles/rotorcraft/multirotor_trajectory.py</a>
<br>


 * exemple:  <a href="https://github.com/poine/quad4d_rebooted/blob/main/src/demo_pat_trajectories.py">exemple</a>
 
 
{% highlight python %}
import numpy as np
import pat3.vehicles.rotorcraft.multirotor_trajectory as trj
def demo_line():
    p1, p2, v, psi = np.array([0, 0, 0]), np.array([1, 0, 0]), 2., np.deg2rad(45.)
    demo(trj.Line(p1, p2, v, psi), 'Line', '/tmp/demo_line.png')

{% endhighlight %}





#### 1.1.2 Cercle

$$
Y(t) = \begin{pmatrix}x(t)= c_x + r \cos(\omega t) \\
                      y(t)= c_x + r \sin(\omega t) \\ \\ z(t) \\ \psi(t) \end{pmatrix} \quad t \in [t_0, t_1[
$$
 
<details>
	<summary>Details</summary> 
		<img src="graphics/demo_circle.png" width="800">
		
</details>


#### 1.1.3 Polynômiale (min snap trajectories)

On se donne $$Y_0$$ (éventuellement $$\dot{Y}_0 \ddot{Y}_0 \dddot{Y}_0 \dots$$),  $$Y_1$$ (éventuellement $$\dot{Y}_1 \ddot{Y}_1 \dddot{Y}_1 \dots$$) et on fabrique le polynôme 

$$Y(t) = \Sigma a_k t^k$$

tel que 

$$ Y(t_0) = Y_0 \quad \dot{Y}(t_0) = \dot{Y}_0 \dots$$

et 

$$ Y(t_1) = Y_1 \quad \dot{Y}(t_1) = \dot{Y}_1 \dots$$



SmoothLine

#### 1.1.4 Composites

$$
Y(t)=Y1(t) \quad t \in [t_0 t_1[, Y(t)=Y2(t) \quad t \in [t_1 t_2[, 
$$


Oval





## 2: Space Indexed Trajectories

Il s'agit ici de découpler la géometrie et la dynamique d'une trajectoire.

$$
Y(t) = G(\lambda(t)) \quad t \in [t_0; t_1[ -> \lambda(t) \in [0; 1[
$$



## 3: Platitude différentielle 

{%comment%}
La propriété de `platitude differentielle` permet d'exprimer l'état et l'entrée d'un système dynamique comme fonction d'un nombre de dérivées temporelles de sa sortie.
{%endcomment%}


$$
Y(t) = \begin{pmatrix}x(t) \\ y(t) \\ z(t) \\ \psi(t) \end{pmatrix}
$$

$$
X(t) = \begin{pmatrix}x(t) \\ y(t) \\ z(t) \\ \dot{x}(t) \\ \dot{y}(t) \\ \dot{z}(t) \\ \phi(t) \\ \theta(t) \\ \psi(t) \\ p \\ q \\ r \end{pmatrix}
\quad
U(t) = \begin{pmatrix} F \\ \tau_p \\ \tau_q \\ \tau_r \end{pmatrix}
$$

### Définition
Un certain nombre de systèmes possèdent en réalité moins de degrés de liberté que la dimension de leur vecteur d'état. Par exemple, un avion doit s'incliner afin d'être en mesure d'accélérer dans le plan horizontal.
Lorsqu'un système est tel que ses vecteurs d'état et de commande peuvent être exprimés en fonction d'une sortie et d'un nombre fini des dérivées temporelles de cette sortie, le système est dit `différentiellement plat` et la sortie correspondante est appelée `sortie plate`.


Soit un système dynamique $$S$$ défini par une représentation d'état

$$
\begin{align}
\dot{\vect{X}} = f(\vect{X},\vect{U}) \qquad \vect{X} \in \mathbb{R}^{n} \qquad \vect{U} \in \mathbb{R}^{m}
\end{align}
$$

$S$ est dit `différentiellement plat` si et seulement si il existe une sortie $$\vect{Y}=g(\vect{X},\vect{U})$$ de dimension $$m$$, appelée `sortie plate`, deux entiers $$r$$ et $$s$$ et des applications $$\Psi : X \times (\mathbb{R}^n)^{s+1} \rightarrow \mathbb{R}^m$$, de rang $$m$$ dans un ouvert convenable, et $$(\phi_0, \phi_1) : \mathbb{R}^{(m+2)r} \rightarrow \mathbb{R}^n \times \mathbb{R}^m$$, de rang $$n+m$$ dans un ouvert convenable, tel que $$\vect{Y} = \psi(\vect{X}, \vect{U}, \dots, \vect{U}^{(s)})$$
 $$\vect{X} = \phi_0(\vect{Y},\dots, \vect{Y}^{(r)})$$ et $$\vect{U} = \phi_1(\vect{Y},\dots, \vect{Y}^{(r+1)})$$, l'équation $$\dot{\phi}_0 = f(\phi_0, \phi_1)$$ étant identiquement vérifiée.


Le système $S$ est dit `Lie-Bäcklund` équivalent au système trivial suivant, où $$\vect{v}$$ est une nouvelle entrée :

$$
\begin{equation}
\vect{Y}^{(r+1)} = \vect{v}
\end{equation}
$$

La propriété de platitude différentielle exprime que l'on est en mesure d'obtenir toutes les variables du système, c'est-à-dire le vecteur d'état ainsi que le vecteur de commande, en fonction de la sortie plate et d'un nombre fini de ses dérivées temporelles. Par conséquent, toute trajectoire $$t \rightarrow (\vect{X}(t), \vect{U}(t))$$ du système $$(S)$$ est l'image d'une trajectoire $$t \rightarrow (\vect{Y}(t), \dots, \vect{Y}^{(r+1)}(t))$$ engendrée par la sortie plate et un nombre fini de ses dérivées temporelles.

Il n'y a pas unicité des sorties plates. Par exemple, \cite{levine_2004} montre que si $$(y_1, y_2)$$ est une sortie plate d'un système à deux entrées, alors $$(z_1, z_2) = (y_1+y_2^k, y_2)$$ est aussi une sortie plate pour tout entier $$k$$. En revanche, certaines sorties plates pourront avoir un sens physique et un objectif de commande pourra être de faire suivre aux sortie plates une certaine trajectoire, alors que d'autres sorties plates pourront n'avoir aucun sens physiquement interprétable. 




## Configuration

flight_zone: 

saturations:
  vitesses
  acceleration?
  attitude
  vitesses angulaires
  poussée moteur
  




