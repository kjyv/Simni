mode(-1)
clear

function y = clip(x)
  y = max(-1,min(1,x))  
  //y = x
endfunction


function y = gauss(v, G, vs)
//    y = Fc  + (1 - Fc).*exp( -(v./vs)^2) 
    y = min(1, G  + exp( -(v./vs)^2))
endfunction



T = 15 //seconds
TStoerAN = 10
TStoerAUS = 10.2
dt = 0.001 // stepsize
dt_R = 0.01 // controller at 100Hz 


show_graphics = %T
controller_on = %T
if dt > dt_R
  error('too large stepsize')
  break
end

D = dt_R/dt

N = T/dt // time steps
NStoerAN = TStoerAN/dt
NStoerAUS = TStoerAUS/dt
epsilon = 0.01
TAU = 100
sigma = 5000

// physical system parameters
g = -9.81   // const. of gravity
m = 1       // mass 
l = 0.5  // length of lever
J = m*l^2   // momentum of inertia
w0 = sqrt(abs(g)/l) // eigenfrequency

//b = 2*J*w0
//b = 1/3*b
b = 0

edge = 1.0*l
edges = [-edge,-edge,edge,edge]

//data stores for plotting
aT = zeros(1,N)
wT = zeros(1,N)
pT = zeros(1,N)
uT = zeros(1,N)
hT = zeros(1,N)
vT = zeros(1,N)
gT = zeros(1,N)

function pendulum()
  if show_graphics 
    xset("window",0)
    f = gcf()
    f.pixmap = 'on'
  end
 
  global aT,wT,pT,uT,hT,vT,gT
  
  // initial conditions
   
  a = 0     // angular acceleration
  w = 0     // angular velocity
  p = -0.15 // startposition  // phi (angle)
  z = p  // init delay unit (angular)
  z2 = 0  //delay unit (feedback)
  R_gleit = 0     // Gleitreibung
  R_haft = 0
  Mreib = 0
  FStoer = 0.0
  ub = 0    //output
  alpha = 0.3    //R_haft gain
  bet = 20      //R_gleit gain
  negphidot = 0  //velocity
  gi = 80
  
  //gaussian parameters (multiplied with feedback)
  G = 0.2
  vs = 0.01

  for n=1:N
    if (n > NStoerAN & n<NStoerAUS)
        FStoer = -5
    else
        FStoer = 0
    end
    
    phi = p //+ 0.0002*rand(1)-0.0001

    // calculate friction
    negphidot = -gi*phi + z    //v
    z = gi*phi
        
    ub = clip(negphidot + z2)
    z2 = ub * gauss(-negphidot, G, vs)
    R_haft = alpha*ub
    R_gleit = bet * negphidot
    
    M = -m*g*sin(p)*l + R_gleit + R_haft + FStoer     // torque
    a = M/J                         // angular acc.
    w = w + a*dt                    // angular vel.
    p = p + w*dt                    // angle
    
    if show_graphics 
      aT(n) = a
      wT(n) = w
      pT(n) = p
      uT(n) = R_gleit
      hT(n) = R_haft
      vT(n) = negphidot
      gT(n) = gauss(n/100000, G, vs)
      
      if modulo(n*dt,.1) == 0
        clf
        isoview(-edge,edge,-edge,edge)
        plot2d([0,l*sin(p)],[0,l*cos(p)],style=2)
        plot2d(l*sin(p),l*cos(p),style=-9)
        show_pixmap
      end
    end
  end
  
  if show_graphics
    show_pixmap
    f.pixmap = 'off'
    xset("window",1)
    clf
    t = [1:N]
    subplot(221)
    plot2d(t*dt,pT/(%pi),style=5,rect=[0,-2.05,N*dt,2.05])
    legend(['winkel']);
    subplot(223)

    plot2d(t*dt,uT,style=2) 
    plot2d(t*dt,hT,style=5) //,axesflag=5) 
    plot2d(t*dt,vT,style=3) 
    legend(['gleit';'reib';'velo']);

    subplot(224)
    //plot2d(pT,vT,style=2) 
    plot2d(vT,uT+hT,style=2) //,axesflag=1) 

    subplot(222)
    plot2d(t*dt/100000, gT, style=2) //,axesflag=1) 
  end
endfunction



pendulum()







