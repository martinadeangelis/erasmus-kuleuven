# Ship Motion


## Description    
With the development of intelligent systems for unmanned surface vehicles (USVs), ship dynamics has become increasingly important in the iteration and renewing processes of algorithms in the systems, such as planning and control algorithms. Next, you will learn about the relevant ship maneuvering models through this material and attempt to control your ship to go far in 3DoF.


<div style="text-align:center;">
    <img src="fig1.png" alt="6dof model" height="400">
    <figcaption> 6-DoF Ship Model </figcaption>
</div>
<br>


For simplicity,we consider its planar motion in 3 degree of freedom: **Surge; Sway, and Yaw**.

<div style="text-align:center;">
    <img src="fig2.png" alt="2 coordinate system" height="300">
    <figcaption> The earth-fixed coordinate and the body-fixed coordinate</figcaption>
</div>
<br>


To mathematically study ship maneuverability, we define two coordinate systems in Fig.2. The earth-fixed coordinate system $o_o-x_oy_oz_o$ represents the coordinate system at the moment when the ship maneuvering motion begins. The body-fixed coordinate system $o-xyz$ represents the current coordinate system of the ship.

The heading angle $\psi$ determines the orientation of the ship.

 $\bar{V}$ is the ship speed. 

 $r = \dot{\psi}$ is the yaw rate of rotational motion about the z-axis. 

 $u, v$ are the speeds in the direction of the x-axis and y-axis based on the $o-xyz$ coordinate system. 

 $\beta$ is the drift angle.


<div style="text-align:center;">
    <img src="fig3.png" alt="2 coordinate system" height="300">
    <figcaption> The earth-fixed coordinate and the body-fixed coordinate</figcaption>
</div>
<br>


Based on those two coordinate systems，assuming that an arbitrary point P has coordinate value ( x_0 , y_0 , z_0 ) in the earth-fixed coordinate system and coordinate value ( x , y , z ) in the body-fixed coordinate system, with the original of the body-fixed coordinate system lying on the center of gravity, then we have:

$$ X_0=X_{OG}+x\cos \psi -y\sin \psi $$
$$ Y_0=Y_{OG}+x\sin \psi +y\cos \psi $$
$$ Z_0=Z $$

Next, we can obtain:
$$x_0\prime=x\cos \psi -y\sin \psi $$
$$y_0\prime=x\sin \psi +y\cos \psi $$
$$z_0\prime=z$$
denoting $x_0\prime = x_0 - x_{0G}$ , $y_0\prime = y_0 - y_{0G}$ and $z_0\prime = z_0$

At the same time, we can also obtain the velocity correspondence between the two coordinate systems:
$$\bar{V}_x = u \cos(\psi) - v \sin(\psi);$$
$$\bar{V}_y = u \sin(\psi) + v \cos(\psi);$$
The relationship between these two coordinate systems will help us to better analyze the ship model.

## Equations

### Basic Equation
In the earth-fixed coordinate system we have the equations of motion:

$$
X_0 = m \ddot{x}_{0G}\\
Y_0 = m \ddot{y}_{0G}\\
N_0 = I_G \dot{\psi}
$$

Assuming that the ship is symmetrical about its longitudinal centerplane, the center of gravity has the coordinates ( $x_G$, 0, $z_G$) in the body-fixed coordinate system with the original lying on the midship point. From this, we can derive the equations of motion in the body-fixed coordinate system.
$$
X=m(\dot{u}-vr-x_Gr^2),\\
Y=m(\dot{v}+ur+x_G\dot{r}),\\
N=I_z\dot{r}+mx_G(\dot{v}+ur).\\
$$

X , Y and N are the components of external force and moment acting on the ship in the body-fixed coordinate system.

Expressing the aforementioned the equations of motion in matrix form will be helpful to analyze Fossen model in the next step:
$$
\left[ \begin{matrix}
	m&		0&		0\\
	0&		m&		mx_G\\
	0&		mx_G&		I_z\\
\end{matrix} \right] \left[ \begin{array}{c}
	\dot{u}\\
	\dot{v}\\
	\dot{r}\\
\end{array} \right] +\left[ \begin{array}{c}
	-mr(x_Gr+v)\\
	mur\\
	mx_Gur\\
\end{array} \right] =\left[ \begin{array}{c}
	X\\
	Y\\
	N\\
\end{array} \right] 
$$


### Hydrodynamic Derivatives:
To better describe the hydrodynamic force and moment, Abkowitz then expanded them in a Taylor series about the initial steady state of forward motion with constant speed.($u_0=U,\quad v_0=0,\quad r_0=0,\quad \dot{u}_0=0,\quad \dot{v}_0=0,\quad \dot{r}_0=0,\quad \delta _0=0$).
$$
	X=X_0+\frac{\partial X}{\partial u}(u-U)+\frac{\partial X}{\partial v}v+\frac{\partial X}{\partial r}r+\frac{\partial X}{\partial \dot{u}}\dot{u}+\frac{\partial X}{\partial \dot{v}}\dot{v}+\frac{\partial X}{\partial \dot{r}}\dot{r}+\frac{\partial X}{\partial \delta}\delta\\
	\quad +\frac{1}{2!}\left[ \frac{\partial}{\partial u}(u-U)+\frac{\partial}{\partial v}v+\frac{\partial}{\partial r}r+\frac{\partial}{\partial \dot{u}}\dot{u}+\frac{\partial}{\partial \dot{v}}\dot{v}+\frac{\partial}{\partial \dot{r}}\dot{r}+\cdots \right] ^2X\\
	\quad +\frac{1}{n!}\left[ \frac{\partial}{\partial u}(u-U)+\frac{\partial}{\partial v}v+\cdots \right] ^nX+\cdots
$$

For simplicity these derivatives are usually expressed as：
$$
\frac{\partial X}{\partial u}=X_u,\quad \frac{\partial X}{\partial v}=X_v,\quad \frac{\partial X}{\partial r}=X_r,\quad \frac{\partial X}{\partial \dot{u}}=X_{\dot{u}},\quad \frac{\partial X}{\partial \dot{v}}=X_{\dot{v}},\quad \frac{\partial X}{\partial \dot{r}}=X_{\dot{r}},\quad \frac{\partial X}{\partial \delta}=X_{\delta}\dots
$$
Furthermore, Y and N will also be expressed in the same form in the following text.


### Fossen Model

There are various main models for ship heading control, including the Nomoto model, MMG model, and Fossen model, etc. Here, we will primarily discuss the Fossen model and use this model to control your ship.

In maneuvering theory, frequency-dependent added mass and potential damping are approximated by constant values and thus it is not necessary to compute the fluidmemory effects.  The main results are based on the assumption that the hydrodynamic forces and moments can be approximated at one frequency of oscillation such that the fluid-memory effects can be neglected.  The result is a nonlinear mass–damper–spring system with constant coefficients.  In the following sections, it is shown that the maneuvering equations of motion can be represented by：

<div style="text-align:center;">
    <img src="fig4.png" alt="Fossen Model" height="150">
    <figcaption> Fossen Maneuvering equations of motion </figcaption>
</div>
<br>

$$
\mathbf{M} = \mathbf{M}_{RB} + \mathbf{M}_A \text{- system inertia matrix (including added mass)} \\
\mathbf{C}(v_r) = \mathbf{C}_{RB}(v_r) + \mathbf{C}_A(v_r) \text{- Coriolis–centripetal matrix (including added mass)} \\
\mathbf{D}(v_r) \text{- damping matrix} \\
\mathbf{g}(\eta) \text{- vector of gravitational/buoyancy forces and moments} \\
\mathbf{g}_0 \text{- vector used for pretrimming (ballast control)} \\
\boldsymbol{\tau} \text{- vector of control inputs} \\
\boldsymbol{\tau}_{\text{wind}} \text{- vector of wind forces} \\
\boldsymbol{\tau}_{\text{wave}} \text{- vector of wave-induced forces}\\
\mathbf{v}_r=\mathbf{v}-\mathbf{v}_c,\quad \mathbf{v}_c=[u_c,v_c,w_c,0,0,0],\mathbf{v}_r\text{- the relative velocity vector},\mathbf{v}_c\text{- the current velocity vector}\\
$$

For a detailed explanation of each parameter, refer to [lecture note](./ch6.pdf)

The Fossen model fully explains the equations of motion stated at the beginning of the article using mathematical language.

For the aforementioned Fossen model, we primarily focus on Nonlinear Maneuvering Models based on Second-order Modulus Functions, hence we can obtain the following equation:
$$
\mathbf{M}\dot{\mathbf{v}}_r+\mathbf{C}(\mathbf{v}_r)\mathbf{v}_r+\mathbf{D}(\mathbf{v}_r)\mathbf{v}_r=\boldsymbol{\tau }+\boldsymbol{\tau }_{\mathrm{wind}}+\boldsymbol{\tau }_{\mathrm{wave}},
$$


Where:
$$
\quad \mathbf{N}(\mathbf{v}_r,\mathbf{v}_r)=\mathbf{C}(\mathbf{v}_r)\mathbf{v}_r+\mathbf{D}(\mathbf{v}_r)\mathbf{v}_r\\
\mathbf{M}=\mathbf{M}_A+\mathbf{M}_{RB}
\\
\mathbf{C}(\mathbf{v}_r)=\mathbf{C}_A(\mathbf{v}_r)+\mathbf{C}_{RB}(\mathbf{v}_r)
\\
\mathbf{D}(\mathbf{v}_r)=\mathbf{D}+\mathbf{D}_n(\mathbf{v}_r)
$$

Therefore, we can list the expression:
$$
\left[ \begin{matrix}
	m-X_{\dot{u}}&		0&		0\\
	0&		m-Y_{\dot{v}}&		mx_G-Y_{\dot{r}}\\
	0&		mx_G-Y_{\dot{r}}&		I_z-N_{\dot{r}}\\
\end{matrix} \right] \left[ \begin{array}{c}
	\dot{u}\\
	\dot{v}\\
	\dot{r}\\
\end{array} \right] +\left[ \begin{matrix}
	-X_{|u|u}|u_r|&		0&		-m(x_Gr+v)+Y_{\dot{v}}v+Y_{\dot{r}}r\\
	0&		-Y_{|v|v}|v_r|&		mu-X_{\dot{u}}u-Y_{|v|r}|v_r|\\
	m(x_Gr+v)-Y_{\dot{v}}v-Y_{\dot{r}}r&		-mu+X_{\dot{u}}u-N_{|v|v}|v_r|&		-N_{v_r}|v_r|\\
\end{matrix} \right] \left[ \begin{array}{c}
	u\\
	v\\
	r\\
\end{array} \right] =\left[ \begin{array}{c}
	\tau _1\\
	\tau _2\\
	\tau _6\\
\end{array} \right]
$$


Under the conditions of the ship driving at low speed and the center of gravity of the ship coinciding with the center of the ship ($x_G=0$), the expression can be simplified and rewritten as:
$$
\left[ \begin{matrix}
	m-X_{\dot{u}}&		0&		0\\
	0&		m-Y_{\dot{v}}&		-Y_{\dot{r}}\\
	0&		-Y_{\dot{r}}&		I_z-N_{\dot{r}}\\
\end{matrix} \right] \left[ \begin{array}{c}
	\dot{u}\\
	\dot{v}\\
	\dot{r}\\
\end{array} \right] +\left[ \begin{matrix}
	-X_{|u|u}|u_r|&		0&		0\\
	0&		-Y_{|v|v}|v_r|&		mu-X_{\dot{u}}u-Y_{|v|r}|v_r|\\
	0&		-mu+X_{\dot{u}}u-N_{|v|v}|v_r|&		-N_{v_r}|v_r|\\
\end{matrix} \right] \left[ \begin{array}{c}
	u\\
	v\\
	r\\
\end{array} \right] =\left[ \begin{array}{c}
	\tau _1\\
	\tau _2\\
	\tau _6\\
\end{array} \right]
$$
For a detailed explanation of each parameter in these matrixes, refer to [lecture note 2](./ch7.pdf)

In addition, the relationship between the propeller rotation rate and the thrust force is expressed as follows, for a ship with a single propeller:

$$T_p=\rho n_{p}^{2}D_{p}^{4}K_T$$
$$
T_p \quad\text{is thrust of propellor}\\
𝑛_p \quad\text{is propeller rotation rate}\\
𝐷_p \quad\text{is propeller diameter}\\
K_T \quad\text{is thrust coefficent}\\
\rho \quad\text{is density of water}\\
$$


## Constants

Converting the calculations for ships into non-dimensional form is a common practice in fluid dynamics and ship design. It helps designers use the principle of similarity, allowing for the inference of full-scale ship performance from model scale and making numerical analysis and analytical solutions easier to handle. Therefore, we will provide non-dimensional data for the relevant constants. Definitions of the non-dimensional parameters and coefficients can be referred to [lecture note 3](./LectureNotesofShipManoeuvring.pdf).

For the basic units, the parameters are as follows. 
$$
\rho = 1.025 \text{kg/m}^3\\
L = 10 \text{m}\\
U = 2 \text{m/s}
$$

If necessary, you can convert the obtained non-dimensional values into corresponding values containing physical quantities, but you can only use non-dimensional parameters for the subsequent simulation:
$$
m\prime = 40 \\
K_T\prime = 0.161\\
D_{p}\prime = 1.53\\
L\prime = 5\\
X_{\dot{u}}\prime = -1.42 \\
X_u\prime = 0.1 \\
X_{uu}\prime = 8.2 \\
Y_{\dot{v}}\prime = -38.4 \\
Y_v\prime = 10 \\
Y_{vv}\prime = 200 \\
Y_{\dot{r}}\prime = -2.5 \\
Y_r\prime = 5 \\
N_{\dot{r}}\prime = -8.9 \\
N_{\dot{v}}\prime = 2.2 \\
N_v\prime = 36 \\
N_r\prime = 5 \\
N_{rr}\prime = 15 \\
I_z\prime = 8.0 \\
Y_{uv}\prime = 0 \\
Y_{ur}\prime = u \cdot X_{\dot{u}} \\
N_{uv}\prime = u \cdot (Y_{\dot{v}} - X_{\dot{u}}) \\
N_{ur}\prime = u \cdot Y_{\dot{r}}

$$



## Analysis
**Q1.** Given the material above, write down the ODE function of the system and simulate it from the stationary state of the ship using the constants provided in the example. Define your own control inputs to make the system function reasonably.

**Q2** Based on the linearized maneuvering equations, linearize the system around the equilibrium point and simulate the system in the state space or transfer function. You may refer to the [lecture note 4](./ch6.pdf).

**Q3** Add environmental effects to the system (i.e. ocean current or wind ). You can also make these effects time-variant. You may refer to the [lecture note 5](./ch8.pdf).

**Q4** Based on the nonlinear model, design a controller that is able to perform a zig-zag maneuver. You can find the definition of these maneuvers on the internet or in the [lecture note 3](./LectureNotesofShipManoeuvring.pdf).

**Q5** You are free to do whatever you want now. You can go back to analyze the complex model further or use the linearized ones.

**Extra** You can learn about other models, such as the MMG model, Nomoto model, etc., based on the references below or other materials. And consider the advantages and disadvantages of the Fossen model compared to them. 
([MMG](./user_guide.pdf) and [Nomoto](./Nomoto_Model.pdf))