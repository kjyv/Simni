mode(-1)
clear

Fc = 0.0
vs = 0.01

function y = gp(v)
    y = min(1, Fc  + 1.1*exp( -(v./vs)^2))
endfunction

function y = sat(x)
  y = max(-1,min(1,x))  
endfunction

T = 15 //seconds
TStoerAN = 10
TStoerAUS = 10.2
dt = 0.001 // stepsize

show_graphics = %T

//D = dt_R/dt

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

b = 2*J*w0

edge = 1.0*l
edges = [-edge,-edge,edge,edge]

aT = zeros(1,N)
wT = zeros(1,N)
pT = zeros(1,N)
uT = zeros(1,N)
hT = zeros(1,N)
vT = zeros(1,N)
 
 
function pendulum()
  if show_graphics 
    xset("window",0)
    f = gcf()
    f.pixmap = 'on'
  end
 
  global aT,wT,pT,uT,hT,vT
  
  // initial conditions
   
  a = 0     // angular acceleration
  w = 0     // angular velocity
  p = -0.2 // startposition  // phi (angle)
  z = p/dt     // init delay unit
  R_gleit = 0     // Reibung
  R_haft = 0     // Reibung
  tp = 0
  E = 0
  
  FStoer = 0.0
  z2 = 0
  ub = 0
  v= 0
  
  for n=1:N
    
    if (n > NStoerAN & n<NStoerAUS)
       //FStoer = -5
    else
       FStoer = 0
    end

    alpha = 1
    
    // calculate friction
    v = -p/dt + z
    z = p/dt 
    // v = -w   
        
    ub = sat(v + z2)
    z2 = ub * gp(-v)
     
    R_haft = alpha * z2 + 50*alpha*gp(-v)*v
    R_gleit = b * v
    
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
      vT(n) = v
      
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
    subplot(223)

    plot2d(t*dt,uT,style=2) 
     plot2d(t*dt,hT,style=5,axesflag=5) 
    plot2d(t*dt,vT,style=3) 
    legend(['gleit';'haft';'velo']);

    subplot(222)
    xv = linspace(-1,1,1000);
    plot2d(xv,gp(xv),style=2) 

    subplot(224)
    //plot2d(pT,vT,style=2) 
    plot2d(vT,uT+hT,style=2,axesflag=5) 
  end
endfunction



pendulum()







