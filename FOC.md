# FOC
Before entering FOC closed loop control, we need to know how to 
control th motor in open loop case, which we will use SPWM.

---
## SPWM
SPWM is done by applying three sine pwm to three ports known as UVW,
and these three ports has phase difference 120 degree between each
other.

For a 24V condition, we can have each of our UVW node with volatge:

U: $12+12m \sin(\theta)$   
V: $12+12m \sin(\theta +120)$  
W: $12+12m \sin(\theta -120)$  
The m here is modulation.  
The reason that we need to add 12 before each modulation term since
we want to apply the sine wave on three sides from 0 to 24V and 
amplitude for the modulation term is just 12V so it can vibrate with
the bus voltage range (never produce a voltage that exceeds bus 
voltage or below 0V at application side).  
Once we have the three application side voltage, we can find the
central neutral voltage: $U_{N}= \frac{U+V+W}{3}$ which is 12V, this 
was proved by Kirchhoff current law.  
After figuring out the neutral point voltage, we can find the three 
phases voltage:  
U phase: $12m \sin(\theta)$  
V phase: $12m \sin(\theta +120)$  
W phase: $12m \sin(\theta -120)$  
These phase voltage will oscillate in between -12V to 12V, and these
phase voltage implies how current flow, if we plot on diagram and 
substitute difference $\theta$ from 0 to 360 deg, we can find a 
continuous rotating magnetic field about the roter, which is how
SPWM work.  
In conclude, SPWM doesn't care where the roter is, it only need to 
provide the roter with a continuous rotationary, withe changable 
frequency and changeable strength of magnetic field.  

Application using STM32 timers:  
For application side, we know we can only apply voltage like:
U: $12+12m \sin(\theta)$, and we know that, 
$V_{out} = V_{bus}D$, here D represents the Duty cycle, so if we
have our duty count for a timer from 0 to 4520, then 0 means 0 volts,
4520 means 24 volts, we can remap the duty to the application side 
voltage and get new equation:  
$U_{out}=24 \cdot (0.5+0.5m \sin(\theta))$, so 
$D = (0.5+0.5m \sin(\theta))$, remap in terms of MCU will get:
$D = (2125+2125m \sin(\theta))$.

This is for upper side MOSFET control, for lower side MOSFET, we
take complement of upper duty cycle this provides lower voltage, so
the entire application side will become:  
$24 \cdot D + 0 \cdot \hat{D}$. The complement pwm for lower side 
MOSFET also provide a complete return path for current.  

We must need to have low MOSFET to be opened, otherwise, that 
application node will have floating voltage which is no longer PWM
form.
