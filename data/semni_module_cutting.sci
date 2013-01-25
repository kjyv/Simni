mode(-1);
// ==================================================
// ========== S E M N I - M O D U L E S ==========
// ==================================================
//separate semni cutting data, triangulate for convex shapes
//for delaunay, you need to run atomsInstall('cglab') and install libCGAL

Semni = fscanfMat("/Users/stefan/Dropbox/diplomarbeit/2d-simulation/data/SemniOutline.txt");

function decompose(x,y, filename)
    nump = size(x,2);
    //C = [x(1:(nump-1))' y(2:nump)'; nump 1];
    //tri = constrained_delaunay_2(x, y, C)
    tri = delaunay_2(x, y)
    [nbtri,nb] = size(tri);
    tri = [tri tri(:,1)];

    F = mopen(filename,mode="w")
    for k = 1:nbtri
        plot2d(x(tri(k,:)),y(tri(k,:)), style = 2)
        fixture = [x(tri(k,:)); y(tri(k,:))]
        mfprintf(F, format="new Array(\n")
        for v = 1:size(fixture, 2)-1
            mfprintf(F, format="\t new b2Vec2(%f, %f), \n", fixture(1,v), fixture(2,v))
        end
        mfprintf(F, format="),\n\n")
        //fprintfMat(F, [x(tri(k,:)); y(tri(k,:))])
    end
    mclose(F)
endfunction

normfactor = 1 / 333.3 / 100
resolution = 4 // every 4th contour point is taken into account

contour = normfactor*[Semni(121:resolution:241, :); Semni(763:resolution:1004, :)]
plot2d(contour(:,1), contour(:,2),frameflag=4)
fprintfMat("/Users/stefan/Dropbox/diplomarbeit/2d-simulation/data/SemniContourCCW.txt", contour)

// contour of each module from contour for milling machine -> SemniOutline.txt

// The data might contain points of more than one module.
// Each of the N modules, given by a list of points, should start with a preamble containing properties of the module.
// At the end, there should be a list of points of all solid contours.
//
// SI - System!
//
// structure of preamble
// pre = [ M  j  jamin  c  L ;
//         p  j  jamax  c  m ] ...
//
//      M     = module number
//      p     = parent: which module number
//      j     = joint (coordinate)
//      jamin = minimum of joint angle (Min: -%pi)
//      jamax = maximum of joint angle (Max:  %pi)
//      c     = center of mass (coordinate)
//      m     = mass
//      L     = length of following list of points
//
// preamble for solid contours
// pre0 = [ -M ;
//           L ]

R1 = [Semni(122:241, :); Semni(763:1004, :)]'
R1 = normfactor * R1(:, 1:resolution:361)
R1i = [Semni(1276:-1:1005, :); Semni(1365:-1:1277, :)]'
temp = 1276 - 1005 + 1 + 1365 - 1277 + 1 // = 361;     (19^2 = 361; 19*18 = 342)
resolution1i = round(sqrt(temp))
bins = 1:resolution1i:temp
R1i = normfactor * R1i(:, bins)
R1i(1, 1) = R1(1, 83)
R1i(1, resolution1i) = R1(1, 84)
R1 = [ [ 1  0.0 -%pi         0.184  200 ;
         0  0.0  %pi         0.068  0.111 ] R1(:, 1:83) R1i R1(:, 84:$) ]

R2 = [Semni(763:-resolution:441, :); Semni(1492:resolution:1644, :)]'

arm1Contour = normfactor*[R2(1,1:resolution:size(R2, 2)); R2(2,1:resolution:size(R2, 2))]'
plot2d(arm1Contour(:,1), arm1Contour(:,2))
//decompose(arm1Contour(:,1)',arm1Contour(:,2)', "/Users/stefan/Dropbox/diplomarbeit/2d-simulation/data/SemniArm1ContourCCW.txt")

//R2 = [Semni(763:-1:441, :); Semni(1492:1644, :)]'
//temp = 763 - 441 + 1 + 1644 - 1492 + 1 // = 476
//R2 = [ [ 2  0.175 -0.89*%pi  0.158  238 ;
//	1  0.070 0.375*%pi  0.057  0.125 ] normfactor*R2(:, 1:resolution:(temp-1)) ]

R3 = [Semni(1:resolution:439, :)]'
// old version: SEMNI without LiPo batteries
//R3 = [ [ 3  0.094 -0.6*%pi   0.080  220 ;
//	2  0.033      %pi   0.037  0.092 ] normfactor*R3 ]

// new: SEMNI with LiPo batteries in lower leg
R3_ = [ [ 3  0.094 -0.6*%pi   0.058  220 ;
         2  0.033      %pi   0.046  0.18 ]]

plot2d(R3_(1,4), R3_(2,4))

//min(-1*R3(2,1:resolution:size(R3, 2)))
arm2Contour = normfactor*[R3(1,1:resolution:size(R3, 2)); R3(2,1:resolution:size(R3, 2))]'

plot2d(arm2Contour(:,1), arm2Contour(:,2))
//decompose(arm2Contour(:,1)',arm2Contour(:,2)', "/Users/stefan/Dropbox/diplomarbeit/2d-simulation/data/SemniArm2ContourCCW.txt")


// contour of solid part: radius 0.04 around [0.255; 0.070]
resolution0 = 1/100;
R01 = [ 0.04*sin(0:resolution0*%pi:2*%pi) + 0.255 ; 0.04*cos(0:resolution0*%pi:2*%pi) + 0.070]
R01 = [ [                    -1 ;
          2 * 1/resolution0 + 1 ] R01 ]

R02 = [ [ -2 ;
           0 ] [] ]
R03 = [ [ -3 ;
           0 ] [] ]

R =  [R3 R2 R1]
R0 = [R03 R02 R01]

//clear Semni
//clear normfactor
//clear resolution
//clear R1
//clear R1i
//clear temp
//clear resolution1i
//clear bins
//clear R2
//clear R3
//clear R01
